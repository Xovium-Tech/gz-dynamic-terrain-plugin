#include "dynamic_terrain/adapters/classic/ClassicTerrainTransport.hh"
#include "dynamic_terrain/adapters/classic/ClassicTerrainCodec.hh"

#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <algorithm>
#include <mutex>

namespace dynamic_terrain
{
class ClassicTerrainSource::Impl
{
public:
    Impl(Config config, const std::string &world, const std::string &topic)
        : config(std::move(config))
    {
        node.reset(new gazebo::transport::Node);
        node->Init(world);
        manifests = node->Advertise<classic_msgs::Manifest>(topic + "/manifest", 1);
        pages = node->Advertise<classic_msgs::PageResponse>(topic + "/page", 1);
        requests = node->Subscribe(topic + "/request", &Impl::Replay, this);
        pageRequests = node->Subscribe(topic + "/page_request", &Impl::SendPage, this);
    }
    ~Impl()
    {
        requests.reset(); pageRequests.reset();
        if (node) node->Fini();
    }
    void PublishManifest()
    {
        if (!snapshot) return;
        classic_msgs::Manifest message;
        *message.mutable_metadata() = EncodeClassicSnapshot(*snapshot, config, false);
        message.set_revision(revision);
        for (const auto &page : snapshot->pages)
        {
            auto *identity = message.add_pages();
            identity->set_index(page.index);
            identity->mutable_key()->set_x(page.key.x);
            identity->mutable_key()->set_y(page.key.y);
            identity->mutable_key()->set_z(page.key.z);
            identity->set_texture_name(page.textureName);
        }
        if (message.ByteSizeLong() > kClassicManifestMessageLimit)
        {
            logError("[DynamicTerrain][CLASSIC] terrain manifest exceeds 4 MiB; shorten configured names");
            return;
        }
        manifests->Publish(message);
    }
    void Replay(const boost::shared_ptr<const gazebo::msgs::Empty> &)
    {
        std::lock_guard<std::mutex> lock(mutex);
        PublishManifest();
    }
    void SendPage(const boost::shared_ptr<const classic_msgs::PageRequest> &request)
    {
        std::shared_ptr<const TerrainSnapshot> current;
        std::uint64_t currentRevision;
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (!snapshot || request->generation() != snapshot->generation ||
                request->index() >= snapshot->pages.size()) return;
            current = snapshot; currentRevision = revision;
        }
        classic_msgs::PageResponse message;
        message.set_generation(current->generation); message.set_revision(currentRevision);
        *message.mutable_page() = EncodeClassicPage(current->pages[request->index()], request->include_mesh());
        if (message.ByteSizeLong() > kClassicPageMessageLimit)
        {
            logError("[DynamicTerrain][CLASSIC] terrain page exceeds 51 MiB message limit");
            return;
        }
        pages->Publish(static_cast<const google::protobuf::Message &>(message));
    }
    Config config;
    mutable std::mutex mutex;
    std::shared_ptr<const TerrainSnapshot> snapshot;
    std::uint64_t revision{0};
    gazebo::transport::NodePtr node;
    gazebo::transport::PublisherPtr manifests, pages;
    gazebo::transport::SubscriberPtr requests, pageRequests;
};
ClassicTerrainSource::ClassicTerrainSource(Config config, const std::string &world,
                                         const std::string &topic)
    : data_(std::make_unique<Impl>(std::move(config), world, topic)) {}
ClassicTerrainSource::~ClassicTerrainSource() = default;
void ClassicTerrainSource::QueueSnapshot(std::shared_ptr<const TerrainSnapshot> snapshot)
{
    if (!snapshot || snapshot->pages.empty()) return;
    std::lock_guard<std::mutex> lock(data_->mutex);
    if (data_->snapshot && data_->snapshot->generation > snapshot->generation) return;
    data_->snapshot = std::move(snapshot); ++data_->revision;
    data_->PublishManifest();
}
void ClassicTerrainSource::QueueTexture(TextureUpdate update)
{
    std::lock_guard<std::mutex> lock(data_->mutex);
    if (!data_->snapshot || data_->snapshot->generation != update.generation) return;
    auto next = std::make_shared<TerrainSnapshot>(*data_->snapshot);
    bool changed = false;
    for (const auto &incoming : update.pages)
    {
        if (incoming.pageIndex >= next->pages.size() || !incoming.texture || !incoming.texture->Valid()) continue;
        auto &page = next->pages[incoming.pageIndex];
        if (page.key != incoming.pageKey || page.submeshName != incoming.submeshName ||
            page.textureSize != incoming.textureSize || incoming.imageryZoom < page.imageryZoom ||
            incoming.textureName == page.textureName) continue;
        page.texture = incoming.texture; page.textureName = incoming.textureName;
        page.imageryZoom = incoming.imageryZoom; changed = true;
    }
    if (!changed) return;
    data_->snapshot = std::move(next); ++data_->revision;
    data_->PublishManifest();
}
std::uint64_t ClassicTerrainSource::ActiveGeneration() const
{
    std::lock_guard<std::mutex> lock(data_->mutex);
    return data_->snapshot ? data_->snapshot->generation : 0;
}
bool ClassicTerrainSource::HasActiveTerrain() const
{
    std::lock_guard<std::mutex> lock(data_->mutex);
    return static_cast<bool>(data_->snapshot);
}
std::optional<TileKey> ClassicTerrainSource::ActiveCenterTile() const
{
    std::lock_guard<std::mutex> lock(data_->mutex);
    return data_->snapshot ? std::make_optional(data_->snapshot->centerTile) : std::nullopt;
}
}
