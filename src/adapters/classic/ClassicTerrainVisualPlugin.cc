#include "dynamic_terrain/adapters/classic/ClassicTerrainCodec.hh"
#include "dynamic_terrain/adapters/classic/ClassicTerrainRenderer.hh"

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/transport/transport.hh>
#include <sdf/sdf.hh>

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace dynamic_terrain
{
class ClassicTerrainVisualPlugin : public gazebo::VisualPlugin
{
public:
    ~ClassicTerrainVisualPlugin() override
    {
        requestConnection_.reset();
        manifestSubscription_.reset(); pageSubscription_.reset();
        if (node_) node_->Fini();
        renderer_.reset();
    }
    void Load(gazebo::rendering::VisualPtr visual, sdf::ElementPtr sdf) override
    {
        if (!visual || !visual->GetScene() || !sdf || !sdf->HasElement("topic"))
        {
            logError("[DynamicTerrain][CLASSIC] visual plugin requires a render scene and topic");
            return;
        }
        const auto topic = sdf->Get<std::string>("topic");
        renderer_ = std::make_unique<ClassicTerrainRenderer>(visual);
        node_.reset(new gazebo::transport::Node);
        node_->Init(visual->GetScene()->Name());
        manifestRequest_ = node_->Advertise<gazebo::msgs::Empty>(topic + "/request", 1);
        pageRequest_ = node_->Advertise<classic_msgs::PageRequest>(topic + "/page_request", 1);
        manifestSubscription_ = node_->Subscribe(topic + "/manifest",
            &ClassicTerrainVisualPlugin::OnManifest, this, true);
        pageSubscription_ = node_->Subscribe(topic + "/page",
            &ClassicTerrainVisualPlugin::OnPage, this);
        requestConnection_ = gazebo::event::Events::ConnectPreRender(
            std::bind(&ClassicTerrainVisualPlugin::PumpRequests, this));
    }
private:
    bool GeometryReady() const
    {
        if (!assembly_ || assembly_->pages.empty()) return false;
        for (const auto &page : assembly_->pages)
            if (!page.mesh || !page.texture) return false;
        return true;
    }
    bool TexturesCurrent() const
    {
        if (!GeometryReady()) return false;
        for (std::size_t i = 0; i < assembly_->pages.size(); ++i)
            if (assembly_->pages[i].textureName != expectedTextures_[i]) return false;
        return true;
    }
    void PublishAssembledSnapshot()
    {
        renderer_->QueueSnapshot(std::make_shared<const TerrainSnapshot>(*assembly_), config_);
        queuedGeneration_ = assembly_->generation;
        lastSnapshotQueue_ = std::chrono::steady_clock::now();
    }
    void PumpRequests()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto now = std::chrono::steady_clock::now();
        // Replay also repairs missed notifications, failed allocations, and
        // publishers that became available after a render scene was created.
        if (manifestRequest_ && manifestRequest_->HasConnections() &&
            now - lastManifestRequest_ >= std::chrono::seconds(5))
        {
            lastManifestRequest_ = now;
            manifestRequest_->Publish(gazebo::msgs::Empty{});
        }
        if (!assembly_) return;
        // Publish a complete geometry generation as soon as its bootstrap
        // images arrive; a moving refinement target must not postpone it.
        if (GeometryReady() && (queuedGeneration_ != assembly_->generation ||
            (renderer_->ActiveGeneration() != assembly_->generation &&
             now - lastSnapshotQueue_ >= std::chrono::seconds(5))))
            PublishAssembledSnapshot();
        if (TexturesCurrent()) return;
        if (!pageRequest_ || !pageRequest_->HasConnections()) return;
        if (inFlight_ && now - lastPageRequest_ < std::chrono::seconds(1)) return;
        for (std::size_t i = 0; i < assembly_->pages.size(); ++i)
        {
            const auto &page = assembly_->pages[i];
            if (page.mesh && page.texture && page.textureName == expectedTextures_[i]) continue;
            classic_msgs::PageRequest message;
            message.set_generation(assembly_->generation); message.set_index(i);
            message.set_include_mesh(!page.mesh);
            inFlight_ = i; lastPageRequest_ = now;
            pageRequest_->Publish(message);
            return;
        }
    }
    void OnManifest(const boost::shared_ptr<const classic_msgs::Manifest> &message)
    {
        if (!message->IsInitialized() || message->ByteSizeLong() > kClassicManifestMessageLimit ||
            message->pages_size() < 1 || message->pages_size() > 65 * 65 ||
            message->metadata().pages_size() != 0) return;
        Config config;
        std::string error;
        auto metadata = DecodeClassicSnapshot(message->metadata(), config, error, true);
        if (!metadata)
        { logError("[DynamicTerrain][CLASSIC] invalid manifest: ", error); return; }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (assembly_ && (metadata->generation < assembly_->generation ||
                (metadata->generation == assembly_->generation && message->revision() < revision_))) return;
            std::vector<TerrainPage> pages;
            std::vector<std::string> expected;
            pages.reserve(message->pages_size()); expected.reserve(message->pages_size());
            for (int i = 0; i < message->pages_size(); ++i)
            {
                const auto &identity = message->pages(i);
                if (identity.index() != static_cast<std::uint64_t>(i) || identity.texture_name().empty()) return;
                const TileKey key{identity.key().x(), identity.key().y(), identity.key().z()};
                if (key.z < 0 || key.z > 20 || key.x < 0 || key.y < 0 ||
                    key.x >= (1 << key.z) || key.y >= (1 << key.z)) return;
                TerrainPage page; page.key = key; page.index = i;
                if (assembly_ && metadata->generation == assembly_->generation &&
                    static_cast<std::size_t>(i) < assembly_->pages.size() && assembly_->pages[i].key == key)
                    page = assembly_->pages[i];
                pages.push_back(std::move(page)); expected.push_back(identity.texture_name());
            }
            if (!assembly_ || assembly_->generation != metadata->generation) inFlight_.reset();
            assembly_ = std::make_shared<TerrainSnapshot>(*metadata);
            assembly_->pages = std::move(pages); expectedTextures_ = std::move(expected);
            config_ = std::move(config); revision_ = message->revision();
        }
        PumpRequests();
    }
    void OnPage(const boost::shared_ptr<const classic_msgs::PageResponse> &message)
    {
        if (!message->IsInitialized() || message->ByteSizeLong() > kClassicPageMessageLimit) return;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!assembly_ || message->generation() != assembly_->generation ||
                message->page().index() >= assembly_->pages.size()) return;
        }
        std::string error;
        auto page = DecodeClassicPage(message->page(), error);
        if (!page) { logError("[DynamicTerrain][CLASSIC] invalid page: ", error); return; }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!assembly_ || message->generation() != assembly_->generation || page->index >= assembly_->pages.size()) return;
            auto &previous = assembly_->pages[page->index];
            if (previous.key != page->key || previous.imageryZoom > page->imageryZoom) return;
            if (!page->mesh) page->mesh = previous.mesh;
            if (!page->mesh) return;
            const bool changedTexture = previous.textureName != page->textureName;
            previous = std::move(*page);
            if (inFlight_ && *inFlight_ == previous.index) inFlight_.reset();
            if (queuedGeneration_ == assembly_->generation && changedTexture)
            {
                TextureUpdate update; update.generation = assembly_->generation; update.changedPageCount = 1;
                update.pages.push_back({previous.index, previous.key, previous.submeshName, previous.imageryZoom,
                    previous.textureSize, previous.texture, previous.textureName});
                renderer_->QueueTexture(std::move(update));
            }
            if (GeometryReady() && queuedGeneration_ != assembly_->generation) PublishAssembledSnapshot();
        }
        PumpRequests();
    }
    std::unique_ptr<ClassicTerrainRenderer> renderer_;
    gazebo::transport::NodePtr node_;
    gazebo::transport::SubscriberPtr manifestSubscription_, pageSubscription_;
    gazebo::transport::PublisherPtr manifestRequest_, pageRequest_;
    gazebo::event::ConnectionPtr requestConnection_;
    std::mutex mutex_;
    std::shared_ptr<TerrainSnapshot> assembly_;
    std::vector<std::string> expectedTextures_;
    Config config_;
    std::uint64_t revision_{0}, queuedGeneration_{0};
    std::optional<std::size_t> inFlight_;
    std::chrono::steady_clock::time_point lastManifestRequest_{}, lastPageRequest_{}, lastSnapshotQueue_{};
};
GZ_REGISTER_VISUAL_PLUGIN(ClassicTerrainVisualPlugin)
}
