#include "dynamic_terrain/adapters/classic/ClassicTerrainRenderer.hh"

#include <gazebo/common/Events.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/rendering/GpuLaser.hh>
#include <gazebo/rendering/WideAngleCamera.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/UserCamera.hh>
#include <gazebo/rendering/Visual.hh>

#include <OgreAxisAlignedBox.h>
#include <OgreCamera.h>
#include <OgreImage.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgrePass.h>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <OgreTextureUnitState.h>


#include <algorithm>
#include <atomic>
#include <cmath>
#include <functional>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <utility>

namespace dynamic_terrain
{
namespace
{
std::atomic<std::uint64_t> nextRenderer{0};
bool nameMatches(const std::string &candidate, const std::string &requested)
{
    if (candidate == requested) return true;
    if (requested.empty() || candidate.size() <= requested.size()) return false;
    const auto offset = candidate.size() - requested.size();
    return candidate.compare(offset, requested.size(), requested) == 0 &&
        (candidate[offset - 1] == ':' || candidate[offset - 1] == '/');
}
}
class ClassicTerrainRenderer::Impl
{
public:
    struct Page
    {
        TerrainPage data;
        Ogre::ManualObject *geometry{nullptr};
        Ogre::SceneNode *node{nullptr};
        Ogre::MaterialPtr material;
        std::string texture;
        bool textureHeld{false};
        Ogre::AxisAlignedBox bounds;
        int offscreenFrames{0};
    };
    struct Slot
    {
        std::uint64_t generation{0};
        TileKey center;
        Config config;
        int warmup{0};
        std::vector<Page> pages;
    };
    explicit Impl(gazebo::rendering::VisualPtr visual)
        : parent(std::move(visual)), instance(++nextRenderer)
    {
        preRender = gazebo::event::Events::ConnectPreRender(std::bind(&Impl::PreRender, this));
        postRender = gazebo::event::Events::ConnectPostRender(std::bind(&Impl::PostRender, this));
    }
    ~Impl()
    {
        stopping.store(true);
        preRender.reset(); postRender.reset();
        // Visual plugins are owned by the render scene. Their destruction is
        // the final render-thread opportunity to detach resources from it.
        if (Ogre::Root::getSingletonPtr() && manager)
        {
            if (active) DestroySlot(*active);
            if (staging) DestroySlot(*staging);
            if (retired) DestroySlot(*retired);
            DrainDeferred();
        }
    }
    std::string Unique(const std::string &kind)
    {
        return "dynamic_terrain_classic_" + std::to_string(instance) + "_" + kind + "_" + std::to_string(++serial);
    }
    void Queue(std::shared_ptr<const TerrainSnapshot> snapshot, Config config)
    {
        if (!snapshot || stopping.load()) return;
        std::lock_guard<std::mutex> lock(mutex);
        if (snapshot->generation < activeGeneration ||
            (pending && pending->generation > snapshot->generation)) return;
        pending = std::move(snapshot); pendingConfig = std::move(config);
    }
    void Queue(TextureUpdate update)
    {
        if (update.pages.empty() || stopping.load()) return;
        std::lock_guard<std::mutex> lock(mutex);
        if (update.generation < activeGeneration) return;
        if (!pendingTexture || pendingTexture->generation < update.generation)
        { pendingTexture = std::move(update); return; }
        if (pendingTexture->generation != update.generation) return;
        for (auto &incoming : update.pages)
        {
            auto &pages = pendingTexture->pages;
            auto found = std::find_if(pages.begin(), pages.end(),
                [&](const TexturePageUpdate &p) { return p.pageKey == incoming.pageKey; });
            if (found == pages.end()) pages.push_back(std::move(incoming));
            else if (found->imageryZoom <= incoming.imageryZoom) *found = std::move(incoming);
        }
        pendingTexture->changedPageCount = pendingTexture->pages.size();
    }
    void PrepareMaterial(Page &page, const Config &config)
    {
        page.texture = page.data.textureName + "_classic_r" + std::to_string(instance);
        auto &textures = Ogre::TextureManager::getSingleton();
        // Track ownership before upload: Ogre may create the named resource
        // before a later allocation step throws.
        textureReferences.Acquire(page.texture); page.textureHeld = true;
        if (!textures.resourceExists(page.texture))
        {
            const auto &data = *page.data.texture;
            Ogre::Image image;
            image.loadDynamicImage(const_cast<std::uint8_t *>(data.rgb.data()),
                data.width, data.height, 1, Ogre::PF_BYTE_RGB, false);
            textures.loadImage(page.texture, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                               image, Ogre::TEX_TYPE_2D, Ogre::MIP_UNLIMITED);
        }
        page.material = Ogre::MaterialManager::getSingleton().create(Unique("material"),
            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        page.material->setReceiveShadows(config.visualLightingEnabled && config.visualReceiveShadows);
        auto *pass = page.material->getTechnique(0)->getPass(0);
        pass->setLightingEnabled(config.visualLightingEnabled);
        pass->setDiffuse(1, 1, 1, 1); pass->setAmbient(1, 1, 1); pass->setSpecular(0, 0, 0, 1);
        auto *texture = pass->createTextureUnitState(page.texture);
        texture->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);
        texture->setTextureFiltering(Ogre::TFO_TRILINEAR);
        page.material->load();
    }
    void ReleaseMaterial(Page &page)
    {
        if (!page.material.isNull())
        {
            const auto name = page.material->getName();
            page.material->removeAllTechniques();
            page.material.setNull();
            Ogre::MaterialManager::getSingleton().remove(name);
        }
        if (page.textureHeld)
        {
            const auto name = page.texture;
            page.textureHeld = false;
            if (textureReferences.Release(name)) Ogre::TextureManager::getSingleton().remove(name);
        }
        page.texture.clear();
    }
    bool Unload(Page &page)
    {
        try
        {
            if (page.geometry)
            {
                if (page.geometry->isAttached()) page.geometry->detachFromParent();
                manager->destroyManualObject(page.geometry); page.geometry = nullptr;
            }
            if (page.node)
            { manager->destroySceneNode(page.node); page.node = nullptr; }
            ReleaseMaterial(page);
            return true;
        }
        catch (const std::exception &e)
        {
            logError("[DynamicTerrain][CLASSIC] deferred resource cleanup: ", e.what());
            return false;
        }
    }
    bool MakeResident(Page &page, const Config &config, bool visible)
    {
        if (page.geometry) { page.geometry->setVisible(visible); return true; }
        try
        {
            if (!page.data.mesh || !page.data.texture || !page.data.texture->Valid()) return false;
            if (!page.material.isNull() || page.textureHeld) if (!Unload(page)) return false;
            PrepareMaterial(page, config);
            page.geometry = manager->createManualObject(Unique("mesh"));
            const auto &mesh = *page.data.mesh;
            page.geometry->estimateVertexCount(mesh.positions.size());
            page.geometry->estimateIndexCount(mesh.indices.size());
            page.geometry->begin(page.material->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
            for (std::size_t i = 0; i < mesh.positions.size(); ++i)
            {
                const auto &v = mesh.positions[i]; const auto &n = mesh.normals[i]; const auto &uv = mesh.texCoords[i];
                page.geometry->position(v.X(), v.Y(), v.Z());
                page.geometry->normal(n.X(), n.Y(), n.Z());
                page.geometry->textureCoord(uv.X(), uv.Y());
            }
            for (const auto index : mesh.indices) page.geometry->index(index);
            page.geometry->end();
            page.geometry->setCastShadows(config.visualLightingEnabled && config.visualCastShadows);
            const auto visual = parent.lock();
            if (!visual || !visual->GetSceneNode()) throw std::runtime_error("terrain visual was removed");
            page.node = visual->GetSceneNode()->createChildSceneNode(Unique("page"));
            // The origin-fixed anchor is a 1 mm box, so Classic gives its
            // visual node a 0.001 scale. Terrain vertices and culling bounds
            // already use world metres and must not inherit that scale.
            page.node->setInheritScale(false);
            page.node->attachObject(page.geometry);
            page.geometry->setVisible(visible);
            page.offscreenFrames = 0;
            return true;
        }
        catch (const std::exception &e)
        {
            Unload(page);
            logError("[DynamicTerrain][CLASSIC] page upload failed: ", e.what());
            return false;
        }
    }
    void DestroySlot(Slot &slot)
    {
        for (auto &page : slot.pages) if (!Unload(page)) deferred.push_back(std::move(page));
        slot.pages.clear();
    }
    void DrainDeferred()
    {
        for (auto it = deferred.begin(); it != deferred.end();)
            if (Unload(*it)) it = deferred.erase(it); else ++it;
    }
    std::optional<Slot> CreateSlot(const TerrainSnapshot &snapshot, const Config &config)
    {
        Slot slot;
        try
        {
            slot.generation = snapshot.generation; slot.center = snapshot.centerTile;
            slot.config = config; slot.warmup = config.visualWarmupFrames;
            slot.pages.reserve(snapshot.pages.size());
            for (const auto &source : snapshot.pages)
            {
                Page page; page.data = source;
                if (!source.mesh || source.mesh->positions.empty() ||
                    source.mesh->normals.size() != source.mesh->positions.size() ||
                    source.mesh->texCoords.size() != source.mesh->positions.size())
                    throw std::runtime_error("invalid terrain page mesh");
                for (const auto &v : source.mesh->positions)
                    page.bounds.merge(Ogre::Vector3(v.X(), v.Y(), v.Z()));
                if (!MakeResident(page, config, false))
                {
                    if (!Unload(page)) deferred.push_back(std::move(page));
                    DestroySlot(slot); return std::nullopt;
                }
                slot.pages.push_back(std::move(page));
            }
            return slot;
        }
        catch (const std::exception &e)
        {
            DestroySlot(slot);
            logError("[DynamicTerrain][CLASSIC] staging failed: ", e.what());
            return std::nullopt;
        }
    }
    void ClearActiveState()
    {
        std::lock_guard<std::mutex> lock(mutex);
        activeGeneration = 0; center.reset();
    }
    void Stage(const TerrainSnapshot &snapshot, const Config &config)
    {
        staging = CreateSlot(snapshot, config);
        if (!staging && active)
        {
            logError("[DynamicTerrain][CLASSIC] allocation failed; releasing active terrain for one retry");
            DestroySlot(*active); active.reset(); ClearActiveState(); DrainDeferred();
            staging = CreateSlot(snapshot, config);
            if (staging) staging->warmup = 0;
        }
    }
    void Apply(const TextureUpdate &update, Slot &slot)
    {
        for (const auto &incoming : update.pages)
        {
            auto it = std::find_if(slot.pages.begin(), slot.pages.end(),
                [&](const Page &page) { return page.data.key == incoming.pageKey; });
            if (it == slot.pages.end() || incoming.submeshName != it->data.submeshName ||
                incoming.imageryZoom < it->data.imageryZoom || incoming.textureName == it->data.textureName ||
                !incoming.texture || !incoming.texture->Valid() || incoming.textureSize != it->data.textureSize) continue;
            TerrainPage next = it->data;
            next.texture = incoming.texture; next.textureName = incoming.textureName; next.imageryZoom = incoming.imageryZoom;
            if (!it->geometry) { it->data = std::move(next); continue; }
            Page replacement; replacement.data = next;
            try
            {
                PrepareMaterial(replacement, slot.config);
                it->geometry->setMaterialName(0, replacement.material->getName());
                std::swap(it->material, replacement.material); std::swap(it->texture, replacement.texture);
                std::swap(it->textureHeld, replacement.textureHeld); it->data = std::move(next);
                if (!Unload(replacement)) deferred.push_back(std::move(replacement));
            }
            catch (const std::exception &e)
            {
                if (!Unload(replacement)) deferred.push_back(std::move(replacement));
                logError("[DynamicTerrain][CLASSIC] texture update failed: ", e.what());
            }
        }
    }
    void Residency()
    {
        if (!active || !active->config.visualFrustumEviction) return;
        const auto &cfg = active->config;
        std::vector<Ogre::Camera *> cameras;
        bool uncertainCoverage = false;
        std::vector<bool> matched(cfg.cameraNames.size(), false);
        const auto visual = parent.lock();
        if (!visual) return;
        auto scene = visual->GetScene();
        auto select = [&](const gazebo::rendering::CameraPtr &camera)
        {
            if (!camera || !camera->OgreCamera()) return;
            bool wanted = cfg.cameraNames.empty();
            for (std::size_t i = 0; i < cfg.cameraNames.size(); ++i)
                if (nameMatches(camera->Name(), cfg.cameraNames[i]) || nameMatches(camera->ScopedName(), cfg.cameraNames[i]))
                { wanted = true; matched[i] = true; }
            if (wanted)
            {
                cameras.push_back(camera->OgreCamera());
                if (boost::dynamic_pointer_cast<gazebo::rendering::GpuLaser>(camera) ||
                    boost::dynamic_pointer_cast<gazebo::rendering::WideAngleCamera>(camera))
                    uncertainCoverage = true;
            }
        };
        for (unsigned int i = 0; i < scene->CameraCount(); ++i) select(scene->GetCamera(i));
        for (unsigned int i = 0; i < scene->UserCameraCount(); ++i) select(scene->GetUserCamera(i));
        const bool complete = !cameras.empty() && std::all_of(matched.begin(), matched.end(), [](bool v) { return v; });
        for (auto &page : active->pages)
        {
            bool visible = !complete || uncertainCoverage;
            const auto margin = Ogre::Vector3(cfg.visualTextureGuardM, cfg.visualTextureGuardM, cfg.visualTextureGuardM);
            Ogre::AxisAlignedBox bounds(page.bounds.getMinimum() - margin, page.bounds.getMaximum() + margin);
            for (const auto *camera : cameras)
            {
                // Projection types with unknown coverage conservatively retain
                // pages; this also keeps orthographic scenes free of holes.
                if (camera->getProjectionType() != Ogre::PT_PERSPECTIVE || camera->isVisible(bounds))
                { visible = true; break; }
            }
            if (visible)
            {
                page.offscreenFrames = 0;
                MakeResident(page, cfg, true);
            }
            else if (++page.offscreenFrames >= cfg.visualOffscreenFrames) Unload(page);
        }
    }
    void PreRender()
    {
        const auto visual = parent.lock();
        if (stopping.load() || !visual || !visual->GetScene() || !visual->GetSceneNode()) return;
        manager = visual->GetScene()->OgreSceneManager();
        if (!manager) return;
        DrainDeferred();
        if (retired) { DestroySlot(*retired); retired.reset(); }
        std::shared_ptr<const TerrainSnapshot> snapshot;
        Config config;
        {
            std::lock_guard<std::mutex> lock(mutex);
            snapshot = std::move(pending); config = pendingConfig;
        }
        if (snapshot)
        {
            Slot *existing = staging && staging->generation == snapshot->generation ? &*staging :
                (active && active->generation == snapshot->generation ? &*active : nullptr);
            if (existing)
            {
                TextureUpdate update; update.generation = snapshot->generation;
                for (const auto &p : snapshot->pages)
                    update.pages.push_back({p.index, p.key, p.submeshName, p.imageryZoom, p.textureSize, p.texture, p.textureName});
                Apply(update, *existing);
            }
            else if ((!active || snapshot->generation > active->generation) &&
                     (!staging || snapshot->generation > staging->generation))
            {
                if (staging) { DestroySlot(*staging); staging.reset(); }
                Stage(*snapshot, config);
            }
        }
        std::optional<TextureUpdate> update;
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (pendingTexture && ((staging && pendingTexture->generation == staging->generation) ||
                (active && pendingTexture->generation == active->generation)))
            { update = std::move(pendingTexture); pendingTexture.reset(); }
            else if (pendingTexture && pendingTexture->generation < activeGeneration) pendingTexture.reset();
        }
        if (update)
        {
            if (staging && staging->generation == update->generation) Apply(*update, *staging);
            else if (active && active->generation == update->generation) Apply(*update, *active);
        }
        if (staging)
        {
            if (staging->warmup > 0) --staging->warmup;
            else
            {
                if (active) for (auto &p : active->pages) if (p.geometry) p.geometry->setVisible(false);
                for (auto &p : staging->pages) if (p.geometry) p.geometry->setVisible(true);
                retired = std::move(active); active = std::move(staging); staging.reset();
                std::lock_guard<std::mutex> lock(mutex);
                activeGeneration = active->generation; center = active->center;
            }
        }
        Residency();
    }
    void PostRender()
    {
        if (stopping.load() || !manager) return;
        if (retired) { DestroySlot(*retired); retired.reset(); }
        DrainDeferred();
    }
    std::weak_ptr<gazebo::rendering::Visual> parent;
    Ogre::SceneManager *manager{nullptr};
    gazebo::event::ConnectionPtr preRender, postRender;
    std::atomic<bool> stopping{false};
    std::uint64_t instance, serial{0};
    mutable std::mutex mutex;
    std::shared_ptr<const TerrainSnapshot> pending;
    Config pendingConfig;
    std::optional<TextureUpdate> pendingTexture;
    std::uint64_t activeGeneration{0};
    std::optional<TileKey> center;
    std::optional<Slot> active, staging, retired;
    ResourceReferenceCounter textureReferences;
    std::vector<Page> deferred;
};
ClassicTerrainRenderer::ClassicTerrainRenderer(gazebo::rendering::VisualPtr parent)
    : data_(std::make_unique<Impl>(std::move(parent))) {}
ClassicTerrainRenderer::~ClassicTerrainRenderer() = default;
void ClassicTerrainRenderer::QueueSnapshot(std::shared_ptr<const TerrainSnapshot> snapshot)
{ data_->Queue(std::move(snapshot), Config{}); }
void ClassicTerrainRenderer::QueueSnapshot(std::shared_ptr<const TerrainSnapshot> snapshot, Config config)
{ data_->Queue(std::move(snapshot), std::move(config)); }
void ClassicTerrainRenderer::QueueTexture(TextureUpdate update) { data_->Queue(std::move(update)); }
std::uint64_t ClassicTerrainRenderer::ActiveGeneration() const
{ std::lock_guard<std::mutex> lock(data_->mutex); return data_->activeGeneration; }
bool ClassicTerrainRenderer::HasActiveTerrain() const
{ std::lock_guard<std::mutex> lock(data_->mutex); return data_->center.has_value(); }
std::optional<TileKey> ClassicTerrainRenderer::ActiveCenterTile() const
{ std::lock_guard<std::mutex> lock(data_->mutex); return data_->center; }
}
