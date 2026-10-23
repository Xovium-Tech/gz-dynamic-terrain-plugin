#include "dynamic_terrain/adapters/gzsim/GzTerrainRenderer.hh"
#include "Ogre2ResourceCleanup.hh"

#include <gz/common/SubMesh.hh>
#include <gz/math/AxisAlignedBox.hh>
#include <gz/math/Color.hh>
#include <gz/math/Frustum.hh>
#include <gz/rendering/GpuRays.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/WideAngleCamera.hh>
#include <gz/sim/rendering/Events.hh>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace dynamic_terrain
{
namespace
{
std::shared_ptr<gz::common::Mesh> renderMesh(const MeshData &data)
{
    auto mesh = std::make_shared<gz::common::Mesh>();
    mesh->SetName(data.name);
    auto surface = std::make_unique<gz::common::SubMesh>(data.submeshName);
    surface->SetPrimitiveType(gz::common::SubMesh::TRIANGLES);
    for (const auto &position : data.positions)
        surface->AddVertex(position.X(), position.Y(), position.Z());
    for (const auto &normal : data.normals)
        surface->AddNormal(normal.X(), normal.Y(), normal.Z());
    for (const auto &uv : data.texCoords)
        surface->AddTexCoord(uv.X(), uv.Y());
    for (const auto index : data.indices)
        surface->AddIndex(index);
    mesh->AddSubMesh(std::move(surface));
    return mesh;
}

std::atomic<std::uint64_t> &rendererInstanceCounter()
{
    static std::atomic<std::uint64_t> counter{0};
    return counter;
}

bool scopedNameMatches(const std::string &candidate,
                       const std::string &requested)
{
    if (requested.empty() || candidate == requested)
        return candidate == requested;
    if (candidate.size() < requested.size() ||
        candidate.compare(candidate.size() - requested.size(),
                          requested.size(), requested) != 0)
        return false;
    const std::size_t prefixEnd = candidate.size() - requested.size();
    return prefixEnd == 0u || candidate[prefixEnd - 1u] == ':' ||
           candidate[prefixEnd - 1u] == '/';
}

bool finiteVector(const gz::math::Vector3d &value)
{
    return std::isfinite(value.X()) && std::isfinite(value.Y()) &&
           std::isfinite(value.Z());
}
}

std::shared_ptr<gz::common::Image> PersistentTerrainRenderer::RenderImage(
    const std::string &name, const std::shared_ptr<const ImageData> &data)
{
    if (!data || !data->Valid())
        return {};
    const auto cached = imageCache_.find(name);
    if (cached != imageCache_.end())
        if (auto image = cached->second.lock())
            return image;
    auto image = std::make_shared<gz::common::Image>();
    image->SetFromData(data->rgb.data(), data->width, data->height,
                       gz::common::Image::RGB_INT8);
    imageCache_[name] = image;
    return image;
}

void PersistentTerrainRenderer::PruneRenderImageCache()
{
    for (auto it = imageCache_.begin(); it != imageCache_.end();)
        if (it->second.expired())
            it = imageCache_.erase(it);
        else
            ++it;
}

PersistentTerrainRenderer::PersistentTerrainRenderer(
    Config config, gz::sim::EventManager &events)
    : cfg_(std::move(config)),
      rendererId_(rendererInstanceCounter().fetch_add(
          1, std::memory_order_relaxed) + 1)
{
    preRenderConnection_ = events.Connect<gz::sim::events::PreRender>(
        std::bind(&PersistentTerrainRenderer::OnPreRender, this));
    postRenderConnection_ = events.Connect<gz::sim::events::PostRender>(
        std::bind(&PersistentTerrainRenderer::OnPostRender, this));
    teardownConnection_ = events.Connect<gz::sim::events::RenderTeardown>(
        std::bind(&PersistentTerrainRenderer::OnRenderTeardown, this));
}

PersistentTerrainRenderer::~PersistentTerrainRenderer()
{
    renderShuttingDown_.store(true, std::memory_order_release);
    preRenderConnection_.reset();
    postRenderConnection_.reset();
    teardownConnection_.reset();
}

void PersistentTerrainRenderer::QueueSnapshot(
    std::shared_ptr<const TerrainSnapshot> snapshot)
{
    if (!snapshot || renderShuttingDown_.load(std::memory_order_acquire))
        return;
    std::lock_guard<std::mutex> lock(queueMutex_);
    if (renderShuttingDown_.load(std::memory_order_relaxed))
        return;
    if (pendingTexture_ && pendingTexture_->generation < snapshot->generation)
        pendingTexture_.reset();
    if (!pendingSnapshot_ || snapshot->generation > pendingSnapshot_->generation ||
        (snapshot->generation == pendingSnapshot_->generation &&
         snapshot->coverageRevision >= pendingSnapshot_->coverageRevision))
        pendingSnapshot_ = std::move(snapshot);
}

void PersistentTerrainRenderer::QueueTexture(TextureUpdate update)
{
    MergePendingTexture(std::move(update), true);
}

void PersistentTerrainRenderer::MergePendingTexture(
    TextureUpdate update, bool replaceEqual)
{
    if (update.pages.empty() ||
        renderShuttingDown_.load(std::memory_order_acquire))
        return;
    std::lock_guard<std::mutex> lock(queueMutex_);
    if (renderShuttingDown_.load(std::memory_order_relaxed))
        return;
    if (!pendingTexture_ || update.generation > pendingTexture_->generation)
    {
        pendingTexture_ = std::move(update);
        return;
    }
    if (update.generation < pendingTexture_->generation)
        return;

    for (auto &incoming : update.pages)
    {
        auto it = std::find_if(
            pendingTexture_->pages.begin(), pendingTexture_->pages.end(),
            [&](const TexturePageUpdate &queued)
            {
                return queued.pageKey == incoming.pageKey;
            });
        if (it == pendingTexture_->pages.end())
            pendingTexture_->pages.push_back(std::move(incoming));
        else if (incoming.imageryZoom > it->imageryZoom ||
                 (replaceEqual && incoming.imageryZoom == it->imageryZoom))
            *it = std::move(incoming);
    }
    pendingTexture_->changedPageCount = pendingTexture_->pages.size();
}

std::uint64_t PersistentTerrainRenderer::ActiveGeneration() const
{
    return activeGeneration_.load(std::memory_order_relaxed);
}

std::uint64_t PersistentTerrainRenderer::ActiveCoverageRevision() const
{
    return activeCoverageRevision_.load(std::memory_order_acquire);
}

bool PersistentTerrainRenderer::HasActiveTerrain() const
{
    return hasActive_.load(std::memory_order_relaxed);
}

std::optional<TileKey> PersistentTerrainRenderer::ActiveCenterTile() const
{
    if (!hasActive_.load(std::memory_order_acquire))
        return std::nullopt;
    return TileKey{activeCenterX_.load(std::memory_order_relaxed),
                   activeCenterY_.load(std::memory_order_relaxed),
                   activeCenterZ_.load(std::memory_order_relaxed)};
}

void PersistentTerrainRenderer::FindScene()
{
    auto scene = gz::rendering::sceneFromFirstRenderEngine();
    if (!scene || !scene->IsInitialized() || !scene->RootVisual())
        return;

    scene_ = scene;
    const std::string rootName = "dynamic_terrain_persistent_root_i" +
        std::to_string(rendererId_);
    root_ = scene_->CreateVisual(rootName);
    if (!root_)
    {
        auto node = scene_->RootVisual()->ChildByName(rootName);
        root_ = std::dynamic_pointer_cast<gz::rendering::Visual>(node);
    }
    if (root_ && !root_->HasParent())
        scene_->RootVisual()->AddChild(root_);
    if (root_)
    {
        root_->SetVisible(true);
        logInfo("[DynamicTerrain][RENDER] persistent server-scene root ready scene=",
                scene_->Name(), " mode=evictable-geographic-pages");
    }
}

void PersistentTerrainRenderer::ConfigureMaterial(
    const gz::rendering::MaterialPtr &material) const
{
    if (!material)
        return;
    material->SetDiffuse(1.0, 1.0, 1.0, 1.0);
    material->SetAmbient(1.0, 1.0, 1.0, 1.0);
    material->SetSpecular(0.0, 0.0, 0.0, 1.0);
    material->SetLightingEnabled(cfg_.visualLightingEnabled);
    material->SetCastShadows(cfg_.visualLightingEnabled &&
                             cfg_.visualCastShadows);
    material->SetReceiveShadows(cfg_.visualLightingEnabled &&
                                cfg_.visualReceiveShadows);
}

std::optional<PersistentTerrainRenderer::PageSlot>
PersistentTerrainRenderer::CreatePage(
    const TerrainSnapshot &snapshot, const TerrainPage &page)
{
    if (!page.mesh || !page.texture || !page.texture->Valid() ||
        page.textureName.empty())
        return std::nullopt;

    PageSlot result;
    result.key = page.key;
    result.pageIndex = page.index;
    result.geographicName = page.submeshName;
    result.sourceMesh = renderMesh(*page.mesh);
    result.sourceTexture = RenderImage(page.textureName, page.texture);
    result.sourceTextureName = page.textureName;
    result.sourceTextureSize = page.textureSize;
    result.sourceImageryZoom = page.imageryZoom;
    // The near view can refine while the worker builds an expanded horizon.
    // Keep finer resident imagery for the same geographic page across both
    // coverage appends and recenter generations. A different dataset/runtime
    // has its own prefix and must never inherit these pixels.
    if (active_ && active_->resourcePrefix == snapshot.resourcePrefix)
    {
        const auto previous = std::find_if(active_->pages.begin(), active_->pages.end(),
            [&](const PageSlot &candidate) { return candidate.key == page.key; });
        if (previous != active_->pages.end() &&
            previous->sourceImageryZoom > result.sourceImageryZoom)
        {
            result.sourceTexture = previous->sourceTexture;
            result.sourceTextureName = previous->sourceTextureName;
            result.sourceTextureSize = previous->sourceTextureSize;
            result.sourceImageryZoom = previous->sourceImageryZoom;
        }
    }
    result.boundsMin = result.sourceMesh->Min();
    result.boundsMax = result.sourceMesh->Max();
    if (!MakePageResident(snapshot.resourcePrefix, snapshot.generation,
                          result, false))
    {
        DestroyPage(result);
        return std::nullopt;
    }
    return result;
}

bool PersistentTerrainRenderer::MakePageResident(
    const std::string &resourcePrefix, std::uint64_t generation,
    PageSlot &page, bool visible)
{
    if (page.gpuResident && page.geometry && page.visual && page.submesh &&
        page.material && page.meshReferenceHeld &&
        page.textureReferenceHeld)
    {
        page.visual->SetVisible(visible);
        return true;
    }

    if (page.geometry || page.visual || page.submesh || page.material ||
        page.meshReferenceHeld || page.textureReferenceHeld)
    {
        if (!UnloadPageGpu(page))
            return false;
    }
    if (!page.sourceMesh || !page.sourceTexture ||
        !page.sourceTexture->Valid() || page.sourceTextureName.empty())
        return false;

    const auto serial = ++renderSerial_;
    page.meshName = page.sourceMesh->Name();
    page.textureName = page.sourceTextureName;
    page.textureBytes = mipmappedRgbaBytes(
        page.sourceTextureSize, page.sourceTextureSize);
    try
    {
        if (!prepareOgre2TerrainMesh(scene_, page.sourceMesh.get()))
            throw std::runtime_error("terrain mesh import failed for " + tileText(page.key));
        page.geometry = scene_->CreateMesh(page.sourceMesh.get());
        if (!page.geometry)
            throw std::runtime_error(
                "CreateMesh failed for " + tileText(page.key));
        meshReferences_.Acquire(page.meshName);
        page.meshReferenceHeld = true;
        deferredMeshDeletes_.erase(page.meshName);
        if (page.geometry->SubMeshCount() != 1u)
            throw std::runtime_error(
                "CreateMesh did not return one geographic submesh for " +
                tileText(page.key));
        page.submesh = page.geometry->SubMeshByIndex(0);
        if (!page.submesh)
            throw std::runtime_error(
                "SubMeshByIndex failed for " + tileText(page.key));

        const std::string suffix = "_g" + std::to_string(generation) +
            "_p" + std::to_string(page.pageIndex) + "_r" +
            std::to_string(serial);
        page.visual = scene_->CreateVisual(
            resourcePrefix + "_visual" + suffix);
        if (!page.visual)
            throw std::runtime_error(
                "CreateVisual failed for " + tileText(page.key));
        page.visual->SetVisible(false);

        page.material = scene_->CreateMaterial(
            resourcePrefix + "_material" + suffix);
        if (!page.material)
            throw std::runtime_error(
                "CreateMaterial failed for " + tileText(page.key));

        textureReferences_.Acquire(page.textureName);
        page.textureReferenceHeld = true;
        deferredTextureDeletes_.erase(page.textureName);
        page.material->SetTexture(page.textureName, page.sourceTexture);
        ConfigureMaterial(page.material);
        page.submesh->SetMaterial(page.material, false);
        page.visual->AddGeometry(page.geometry);
        root_->AddChild(page.visual);
        page.visual->SetVisible(visible);
        page.offscreenFrames = 0;
        page.gpuResident = true;
        return true;
    }
    catch (const std::exception &e)
    {
        UnloadPageGpu(page);
        logError("[DynamicTerrain][RENDER] failed to make page resident page=",
                 tileText(page.key), " error=", e.what());
        return false;
    }
    catch (...)
    {
        UnloadPageGpu(page);
        logError("[DynamicTerrain][RENDER] failed to make page resident page=",
                 tileText(page.key), " unknown rendering exception");
        return false;
    }
}

PersistentTerrainRenderer::ResidencyCameraSet
PersistentTerrainRenderer::ResidencyCameras() const
{
    ResidencyCameraSet result;
    if (!scene_)
        return result;

    std::vector<bool> matched(cfg_.cameraNames.size(), false);
    std::unordered_set<const gz::rendering::Camera *> seen;
    try
    {
        for (unsigned int i = 0; i < scene_->SensorCount(); ++i)
        {
            auto camera = std::dynamic_pointer_cast<gz::rendering::Camera>(
                scene_->SensorByIndex(i));
            if (!camera)
                continue;
            bool selected = cfg_.cameraNames.empty();
            for (std::size_t requestedIndex = 0;
                 requestedIndex < cfg_.cameraNames.size(); ++requestedIndex)
            {
                if (!scopedNameMatches(camera->Name(),
                                       cfg_.cameraNames[requestedIndex]))
                    continue;
                matched[requestedIndex] = true;
                selected = true;
            }
            if (selected && seen.insert(camera.get()).second)
                result.cameras.push_back(std::move(camera));
        }
    }
    catch (...)
    {
        result.cameras.clear();
        return result;
    }

    result.complete = !result.cameras.empty() &&
        (cfg_.cameraNames.empty() ||
         std::all_of(matched.begin(), matched.end(),
                     [](bool value) { return value; }));
    return result;
}

PersistentTerrainRenderer::CameraObservation
PersistentTerrainRenderer::ObservePageFromCamera(
    const PageSlot &page, const gz::rendering::CameraPtr &camera) const
{
    if (!camera)
        return CameraObservation::Unknown;

    if (std::dynamic_pointer_cast<gz::rendering::GpuRays>(camera) ||
        std::dynamic_pointer_cast<gz::rendering::WideAngleCamera>(camera) ||
        camera->ProjectionType() != gz::rendering::CPT_PERSPECTIVE)
        return CameraObservation::Unknown;

    const double nearClip = camera->NearClipPlane();
    const double farClip = camera->FarClipPlane();
    const double hfov = camera->HFOV().Radian();
    double aspect = camera->AspectRatio();
    if ((!std::isfinite(aspect) || aspect <= 0.0) &&
        camera->ImageHeight() != 0u)
    {
        aspect = static_cast<double>(camera->ImageWidth()) /
                 static_cast<double>(camera->ImageHeight());
    }
    const auto pose = camera->WorldPose();
    const auto rotation = pose.Rot();
    if (!std::isfinite(nearClip) || !std::isfinite(farClip) ||
        !std::isfinite(hfov) || !std::isfinite(aspect) ||
        nearClip < 0.0 || farClip <= nearClip || aspect <= 0.0 ||
        hfov <= 0.0 || hfov >= kPi || !finiteVector(pose.Pos()) ||
        !std::isfinite(rotation.W()) || !std::isfinite(rotation.X()) ||
        !std::isfinite(rotation.Y()) || !std::isfinite(rotation.Z()) ||
        !finiteVector(page.boundsMin) || !finiteVector(page.boundsMax))
        return CameraObservation::Unknown;

    const double margin = cfg_.visualTextureGuardM;
    const gz::math::Vector3d minimum = page.boundsMin -
        gz::math::Vector3d(margin, margin, margin);
    const gz::math::Vector3d maximum = page.boundsMax +
        gz::math::Vector3d(margin, margin, margin);
    try
    {
        const gz::math::Frustum frustum(
            nearClip, farClip, camera->HFOV(), aspect, pose);
        return frustum.Contains(gz::math::AxisAlignedBox(minimum, maximum))
            ? CameraObservation::Visible
            : CameraObservation::Offscreen;
    }
    catch (...)
    {
        return CameraObservation::Unknown;
    }
}

void PersistentTerrainRenderer::UpdateActivePageResidency()
{
    if (!cfg_.visualFrustumEviction || !active_)
        return;
    const auto cameraSet = ResidencyCameras();
    if (!cameraSet.complete)
    {
        for (auto &page : active_->pages)
        {
            if (page.startupWarmupFrames >= 0) continue;
            page.offscreenFrames = 0;
            // Camera discovery can temporarily be incomplete during sensor
            // reloads. Merely stopping eviction leaves previously evicted
            // pages missing from the remaining camera's view.
            MakePageResident(active_->resourcePrefix,
                             active_->generation, page, true);
        }
        if (!warnedNoResidencyCameras_)
        {
            logInfo("[DynamicTerrain][GPU-RESIDENCY] selected camera set is empty or incomplete; ",
                    "restoring the complete page working set");
            warnedNoResidencyCameras_ = true;
        }
        return;
    }
    warnedNoResidencyCameras_ = false;

    std::size_t loaded = 0;
    std::size_t unloaded = 0;
    for (auto &page : active_->pages)
    {
        if (page.startupWarmupFrames >= 0) continue;
        bool visible = false;
        bool unknown = false;
        for (const auto &camera : cameraSet.cameras)
        {
            const auto observation = ObservePageFromCamera(page, camera);
            visible = visible || observation == CameraObservation::Visible;
            unknown = unknown || observation == CameraObservation::Unknown;
        }
        const bool resident = page.gpuResident && page.geometry &&
                              page.visual && page.submesh && page.material &&
                              page.meshReferenceHeld &&
                              page.textureReferenceHeld;
        const bool hasGpuState = page.geometry || page.visual || page.submesh ||
                                 page.material || page.meshReferenceHeld ||
                                 page.textureReferenceHeld;
        if (visible)
        {
            page.offscreenFrames = 0;
            if (!resident)
            {
                if (MakePageResident(active_->resourcePrefix,
                                     active_->generation, page, true))
                    ++loaded;
            }
            else if (page.visual)
            {
                page.visual->SetVisible(true);
            }
            continue;
        }

        if (unknown)
        {
            page.offscreenFrames = 0;
            // Unsupported projections must be conservative in both
            // directions: restore pages as well as preventing eviction.
            if (!resident && MakePageResident(active_->resourcePrefix,
                                              active_->generation, page, true))
                ++loaded;
            continue;
        }

        page.offscreenFrames = std::min(
            cfg_.visualOffscreenFrames, page.offscreenFrames + 1);
        if (hasGpuState &&
            page.offscreenFrames >= cfg_.visualOffscreenFrames)
        {
            if (UnloadPageGpu(page))
                ++unloaded;
        }
    }

    if (loaded != 0u || unloaded != 0u)
    {
        const auto residentPages = static_cast<std::size_t>(std::count_if(
            active_->pages.begin(), active_->pages.end(),
            [](const PageSlot &page) { return page.gpuResident; }));
        logInfo("[DynamicTerrain][GPU-RESIDENCY] generation=",
                active_->generation, " loaded=", loaded,
                " unloaded=", unloaded,
                " resident_pages=", residentPages,
                " total_pages=", active_->pages.size(),
                " named_textures=", textureReferences_.ResourceCount());
    }
}

std::optional<PersistentTerrainRenderer::Slot>
PersistentTerrainRenderer::CreateSlot(
    const std::shared_ptr<const TerrainSnapshot> &snapshot,
    bool *resourcePressureCandidate)
{
    if (resourcePressureCandidate)
        *resourcePressureCandidate = false;
    if (!scene_ || !root_ || !snapshot || snapshot->pages.empty())
        return std::nullopt;

    Slot slot;
    slot.generation = snapshot->generation;
    slot.coverageRevision = snapshot->coverageRevision;
    slot.centerTile = snapshot->centerTile;
    slot.resourcePrefix = snapshot->resourcePrefix;
    slot.warmupFrames = cfg_.visualWarmupFrames;
    slot.estimatedTextureBytes = snapshot->estimatedTextureBytes;
    slot.pages.reserve(snapshot->pages.size());

    slot.uploading = snapshot;
    return slot;
}

void PersistentTerrainRenderer::AdvanceStagingUpload()
{
    if (!staging_ || !staging_->uploading) return;
    const auto began = std::chrono::steady_clock::now();
    std::size_t uploaded = 0;
    try
    {
        auto &slot = *staging_;
        const auto &snapshot = *slot.uploading;
        while (slot.pages.size() < snapshot.pages.size())
        {
            auto page = CreatePage(snapshot, snapshot.pages[slot.pages.size()]);
            if (!page) throw std::runtime_error("invalid geographic page during staging");
            slot.pages.push_back(std::move(*page));
            ++uploaded;
            const double elapsedMs = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - began).count();
            if (uploaded >= static_cast<std::size_t>(cfg_.visualUploadPagesPerFrame) ||
                elapsedMs >= cfg_.visualUploadBudgetMs) break;
        }
        if (slot.pages.size() == snapshot.pages.size())
        {
            slot.uploading.reset();
            if (cfg_.diagnostics)
                logInfo("[DynamicTerrain][RENDER] staging complete generation=", slot.generation,
                        " pages=", slot.pages.size(), " warmup_frames=", slot.warmupFrames);
        }
    }
    catch (const std::exception &error)
    {
        logError("[DynamicTerrain][RENDER] staging failed; active terrain retained: ", error.what());
        DestroySlot(*staging_); staging_.reset();
    }
    catch (...)
    {
        logError("[DynamicTerrain][RENDER] staging failed; active terrain retained");
        DestroySlot(*staging_); staging_.reset();
    }
}

bool PersistentTerrainRenderer::AdoptProgressiveSnapshot(
    const std::shared_ptr<const TerrainSnapshot> &snapshot)
{
    if (!snapshot || !snapshot->progressiveInitial || snapshot->pages.empty()) return false;
    if (active_ && (!active_->progressiveInitial || active_->generation != snapshot->generation ||
             active_->resourcePrefix != snapshot->resourcePrefix ||
             snapshot->coverageRevision < active_->coverageRevision))
        return false;

    std::unordered_map<TileKey, const TerrainPage *, TileKeyHash> incoming;
    for (const auto &page : snapshot->pages)
        if (!incoming.emplace(page.key, &page).second)
        {
            logError("[DynamicTerrain][STARTUP] rejected duplicate cumulative page");
            return false;
        }
    if (!active_)
    {
        active_ = CreateSlot(snapshot);
        if (!active_) return false;
        active_->progressiveInitial = true;
    }
    for (const auto &page : active_->pages)
    {
        const auto found = incoming.find(page.key);
        if (found == incoming.end() || !found->second->mesh || !page.sourceMesh ||
            found->second->mesh->name != page.sourceMesh->Name())
        {
            logError("[DynamicTerrain][STARTUP] rejected non-cumulative snapshot; visible pages retained");
            return false;
        }
    }
    active_->coverageRevision = snapshot->coverageRevision;
    active_->progressiveComplete = snapshot->progressiveComplete;
    active_->estimatedTextureBytes = snapshot->estimatedTextureBytes;
    active_->uploading = snapshot;
    active_->uploadCursor = 0;
    active_->pages.reserve(snapshot->pages.size());
    return true;
}

void PersistentTerrainRenderer::AdvanceProgressiveUpload()
{
    if (!active_ || !active_->progressiveInitial) return;
    auto &slot = *active_;
    const auto began = std::chrono::steady_clock::now();
    std::size_t uploaded = 0;
    try
    {
        if (slot.uploading && !staging_)
        {
            const auto &snapshot = *slot.uploading;
            while (slot.uploadCursor < snapshot.pages.size())
            {
                const auto &incoming = snapshot.pages[slot.uploadCursor];
                const auto existing = std::find_if(slot.pages.begin(), slot.pages.end(),
                    [&](const PageSlot &page) { return page.key == incoming.key; });
                if (existing != slot.pages.end())
                {
                    existing->pageIndex = incoming.index;
                    ++slot.uploadCursor;
                    continue;
                }
                auto page = CreatePage(snapshot, incoming);
                if (!page) throw std::runtime_error("invalid page during initial streaming");
                page->startupWarmupFrames = cfg_.visualWarmupFrames;
                slot.pages.push_back(std::move(*page));
                ++slot.uploadCursor;
                ++uploaded;
                if (uploaded >= static_cast<std::size_t>(cfg_.visualUploadPagesPerFrame) ||
                    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - began).count()
                        >= cfg_.visualUploadBudgetMs) break;
            }
        }
    }
    catch (const std::exception &error)
    {
        logError("[DynamicTerrain][STARTUP] page upload failed; completed pages retained: ", error.what());
        slot.uploading.reset();
    }
    catch (...)
    {
        logError("[DynamicTerrain][STARTUP] page upload failed; completed pages retained");
        slot.uploading.reset();
    }

    std::size_t visiblePages = 0;
    for (const auto &page : slot.pages) visiblePages += page.startupWarmupFrames < 0;
    for (auto &page : slot.pages)
    {
        if (page.startupWarmupFrames < 0) continue;
        if (page.startupWarmupFrames > 0)
        {
            --page.startupWarmupFrames;
            continue;
        }
        if (page.visual) page.visual->SetVisible(true);
        page.startupWarmupFrames = -1;
        ++visiblePages;
        activeCenterX_.store(slot.centerTile.x, std::memory_order_relaxed);
        activeCenterY_.store(slot.centerTile.y, std::memory_order_relaxed);
        activeCenterZ_.store(slot.centerTile.z, std::memory_order_relaxed);
        activeGeneration_.store(slot.generation, std::memory_order_release);
        hasActive_.store(true, std::memory_order_release);
        if (cfg_.diagnostics)
            logInfo("[DynamicTerrain][STARTUP] page visible generation=", slot.generation,
                " revision=", slot.coverageRevision, " page=", tileText(page.key),
                " visible_pages=", visiblePages, " uploaded_pages=", slot.pages.size(),
                " target_pages=", slot.uploading ? slot.uploading->pages.size() : slot.pages.size());
    }
    if (slot.uploading && slot.uploadCursor == slot.uploading->pages.size() &&
        visiblePages == slot.pages.size())
    {
        activeCoverageRevision_.store(slot.coverageRevision, std::memory_order_release);
        if (cfg_.diagnostics)
            logInfo("[DynamicTerrain][STARTUP] revision active generation=", slot.generation,
                " revision=", slot.coverageRevision, " pages=", slot.pages.size(),
                " complete=", slot.progressiveComplete);
        slot.uploading.reset();
    }
}

void PersistentTerrainRenderer::ClearActiveState()
{
    hasActive_.store(false, std::memory_order_release);
    activeGeneration_.store(0, std::memory_order_release);
    activeCoverageRevision_.store(0, std::memory_order_release);
    activeCenterX_.store(0, std::memory_order_relaxed);
    activeCenterY_.store(0, std::memory_order_relaxed);
    activeCenterZ_.store(0, std::memory_order_relaxed);
}

std::optional<PersistentTerrainRenderer::Slot>
PersistentTerrainRenderer::CreateSlotWithRecovery(
    const std::shared_ptr<const TerrainSnapshot> &snapshot)
{
    // Failed staging never releases the last complete active generation.
    // The runtime retries later; its renderer acknowledgement remains unchanged.
    return CreateSlot(snapshot);
}

void PersistentTerrainRenderer::ApplyPendingTexture()
{
    std::optional<TextureUpdate> update;
    {
        std::lock_guard<std::mutex> lock(queueMutex_);
        if (pendingTexture_)
        {
            update = std::move(pendingTexture_);
            pendingTexture_.reset();
        }
    }
    if (!update || update->pages.empty())
        return;

    Slot *target = nullptr;
    if (staging_ && staging_->generation == update->generation)
        target = &*staging_;
    else if (active_ && active_->generation == update->generation)
    {
        target = &*active_;
        if (staging_ && staging_->resourcePrefix == active_->resourcePrefix)
        {
            // A final old-generation refinement can arrive after its page was
            // copied into staging. Apply it to the replacement instead of
            // sharpening the old view for one frame and then swapping it out
            // for the older staged image. Geographic keys validate the mapping.
            target = &*staging_;
            update->generation = staging_->generation;
        }
    }
    if (!target)
    {
        if (cfg_.diagnostics)
            logInfo("[DynamicTerrain][RENDER] dropped stale page update generation=",
                    update->generation,
                    " active=", active_ ? active_->generation : 0,
                    " staging=", staging_ ? staging_->generation : 0);
        return;
    }

    std::size_t applied = 0;
    std::size_t rejected = 0;
    std::size_t processed = 0;
    const auto began = std::chrono::steady_clock::now();
    for (const auto &pageUpdate : update->pages)
    {
        if (processed != 0u &&
            (processed >= static_cast<std::size_t>(cfg_.visualUploadPagesPerFrame) ||
             std::chrono::duration<double, std::milli>(
                 std::chrono::steady_clock::now() - began).count() >=
                 cfg_.visualUploadBudgetMs))
            break;
        ++processed;
        auto pageIt = std::find_if(
            target->pages.begin(), target->pages.end(),
            [&](const PageSlot &page) { return page.key == pageUpdate.pageKey; });
        if (pageIt == target->pages.end() ||
            pageIt->geographicName != pageUpdate.submeshName ||
            !pageUpdate.texture || !pageUpdate.texture->Valid() ||
            pageUpdate.textureName.empty())
        {
            ++rejected;
            continue;
        }
        if (pageIt->sourceTextureName == pageUpdate.textureName)
            continue;
        if (pageUpdate.imageryZoom < pageIt->sourceImageryZoom)
            continue;

        if (!pageIt->gpuResident || !pageIt->geometry || !pageIt->submesh ||
            !pageIt->material || !pageIt->textureReferenceHeld)
        {
            pageIt->sourceTexture = RenderImage(pageUpdate.textureName, pageUpdate.texture);
            pageIt->sourceTextureName = pageUpdate.textureName;
            pageIt->sourceTextureSize = pageUpdate.textureSize;
            pageIt->sourceImageryZoom = pageUpdate.imageryZoom;
            ++applied;
            continue;
        }

        std::string preparedSourceName = pageUpdate.textureName;
        auto preparedSourceTexture = RenderImage(pageUpdate.textureName, pageUpdate.texture);
        PageSlot replacementResource;
        replacementResource.textureName = pageUpdate.textureName;
        replacementResource.textureBytes = mipmappedRgbaBytes(
            pageUpdate.textureSize, pageUpdate.textureSize);
        replacementResource.material = scene_->CreateMaterial(
            target->resourcePrefix + "_material_g" +
            std::to_string(update->generation) + "_u" +
            std::to_string(++materialSerial_));
        if (!replacementResource.material)
        {
            ++rejected;
            continue;
        }

        try
        {
            textureReferences_.Acquire(replacementResource.textureName);
            replacementResource.textureReferenceHeld = true;
            deferredTextureDeletes_.erase(replacementResource.textureName);
            replacementResource.material->SetTexture(
                replacementResource.textureName, preparedSourceTexture);
            ConfigureMaterial(replacementResource.material);
            pageIt->submesh->SetMaterial(replacementResource.material, false);
            pageIt->material.swap(replacementResource.material);
            pageIt->textureName.swap(replacementResource.textureName);
            std::swap(pageIt->textureBytes,
                      replacementResource.textureBytes);
            std::swap(pageIt->textureReferenceHeld,
                      replacementResource.textureReferenceHeld);
            pageIt->sourceTextureName.swap(preparedSourceName);
            pageIt->sourceTexture.swap(preparedSourceTexture);
            pageIt->sourceTextureSize = pageUpdate.textureSize;
            pageIt->sourceImageryZoom = pageUpdate.imageryZoom;

            DestroyPageMaterial(replacementResource);
            ++applied;
        }
        catch (const std::exception &e)
        {
            DestroyPage(replacementResource);
            ++rejected;
            logError("[DynamicTerrain][GPU-GC] page replacement failed page=",
                     tileText(pageUpdate.pageKey), " error=", e.what());
        }
        catch (...)
        {
            DestroyPage(replacementResource);
            ++rejected;
            logError("[DynamicTerrain][GPU-GC] page replacement failed page=",
                     tileText(pageUpdate.pageKey), " unknown rendering exception");
        }
    }

    const auto generation = update->generation;
    const auto deferred = update->pages.size() - processed;
    if (deferred != 0u)
    {
        update->pages.erase(update->pages.begin(),
                            update->pages.begin() + processed);
        update->changedPageCount = update->pages.size();
        // New worker updates may have arrived while the render thread was
        // uploading. An older deferred page cannot replace an equally fine
        // newer update, and unrelated pages must remain queued.
        MergePendingTexture(std::move(*update), false);
    }
    logInfo("[DynamicTerrain][RENDER] progressive page textures applied generation=",
            generation, " applied=", applied,
            " rejected=", rejected,
            " deferred=", deferred,
            " resident_named_textures=", textureReferences_.ResourceCount(),
            " destroyed_textures=", destroyedTextures_);
}

void PersistentTerrainRenderer::OnPreRender()
{
    if (renderShuttingDown_.load(std::memory_order_acquire))
        return;
    if (!scene_ || !root_)
        FindScene();
    if (!scene_ || !root_)
        return;

    DrainDeferredPageReleases();
    DrainDeferredTextureReleases();
    DrainDeferredTextureDeletes();
    DrainDeferredMeshDeletes();

    if (retired_)
    {
        const auto generation = retired_->generation;
        DestroySlot(*retired_);
        retired_.reset();
        if (cfg_.diagnostics)
            logInfo("[DynamicTerrain][GPU-GC] pre-render safety cleanup generation=",
                    generation);
    }

    // Only the first generation may grow in place. The worker publishes
    // cumulative immutable snapshots, so a bounded latest-only queue can
    // supersede a target without discarding already uploaded or visible pages.
    std::shared_ptr<const TerrainSnapshot> progressive;
    {
        std::lock_guard<std::mutex> lock(queueMutex_);
        if (!staging_ && pendingSnapshot_ && pendingSnapshot_->progressiveInitial &&
            (!active_ || (active_->progressiveInitial &&
                active_->generation == pendingSnapshot_->generation &&
                pendingSnapshot_->coverageRevision >= active_->coverageRevision)))
        {
            progressive = std::move(pendingSnapshot_);
            pendingSnapshot_.reset();
        }
    }
    if (progressive)
    {
        try { AdoptProgressiveSnapshot(progressive); }
        catch (const std::exception &error)
        {
            if (active_ && active_->progressiveInitial) active_->uploading.reset();
            logError("[DynamicTerrain][STARTUP] cumulative target failed; visible pages retained: ", error.what());
        }
        catch (...)
        {
            if (active_ && active_->progressiveInitial) active_->uploading.reset();
            logError("[DynamicTerrain][STARTUP] cumulative target failed; visible pages retained");
        }
    }

    if (staging_)
    {
        std::shared_ptr<const TerrainSnapshot> newer;
        {
            std::lock_guard<std::mutex> lock(queueMutex_);
            if (pendingSnapshot_ &&
                (pendingSnapshot_->generation > staging_->generation ||
                 (pendingSnapshot_->generation == staging_->generation &&
                  pendingSnapshot_->coverageRevision > staging_->coverageRevision)))
            {
                newer = std::move(pendingSnapshot_);
                pendingSnapshot_.reset();
            }
        }
        if (newer)
        {
            const auto oldGeneration = staging_->generation;
            DestroySlot(*staging_);
            staging_.reset();
            staging_ = CreateSlotWithRecovery(newer);
            if (cfg_.diagnostics)
                logInfo("[DynamicTerrain][RENDER] superseded hidden staging generation=",
                        oldGeneration, " by generation=", newer->generation);
        }
    }

    if (!staging_)
    {
        std::shared_ptr<const TerrainSnapshot> pending;
        {
            std::lock_guard<std::mutex> lock(queueMutex_);
            if (pendingSnapshot_ &&
                // A worker can publish between this tick's append check and
                // this check. Never route that cumulative target into hidden
                // replacement staging; it will append on the next tick.
                !(pendingSnapshot_->progressiveInitial &&
                  (!active_ || (active_->progressiveInitial &&
                   pendingSnapshot_->generation == active_->generation))) &&
                (!active_ || pendingSnapshot_->generation > active_->generation ||
                 (pendingSnapshot_->generation == active_->generation &&
                  pendingSnapshot_->coverageRevision > active_->coverageRevision)))
            {
                pending = std::move(pendingSnapshot_);
                pendingSnapshot_.reset();
            }
        }
        if (pending)
            staging_ = CreateSlotWithRecovery(pending);
    }

    AdvanceStagingUpload();
    AdvanceProgressiveUpload();
    if ((!staging_ || !staging_->uploading) &&
        (!active_ || !active_->progressiveInitial || !active_->uploading))
        ApplyPendingTexture();

    if (staging_ && !staging_->uploading)
    {
        if (staging_->warmupFrames > 0)
        {
            --staging_->warmupFrames;
        }
        else
        {
            if (active_)
                for (auto &page : active_->pages)
                    if (page.visual)
                        page.visual->SetVisible(false);
            for (auto &page : staging_->pages)
                if (page.visual)
                    page.visual->SetVisible(true);
            if (active_)
                retired_ = std::move(active_);
            active_ = std::move(staging_);
            staging_.reset();
            activeCenterX_.store(active_->centerTile.x, std::memory_order_relaxed);
            activeCenterY_.store(active_->centerTile.y, std::memory_order_relaxed);
            activeCenterZ_.store(active_->centerTile.z, std::memory_order_relaxed);
            activeGeneration_.store(active_->generation, std::memory_order_release);
            activeCoverageRevision_.store(active_->coverageRevision, std::memory_order_release);
            hasActive_.store(true, std::memory_order_release);
            if (cfg_.diagnostics)
                logInfo("[DynamicTerrain][RENDER] atomic A/B swap generation=",
                        active_->generation, " pages=", active_->pages.size(),
                        " resident_named_textures=", textureReferences_.ResourceCount());
        }
    }

    UpdateActivePageResidency();
}

void PersistentTerrainRenderer::ReleaseTextureReference(PageSlot &page)
{
    if (!page.textureReferenceHeld || page.textureName.empty())
        return;
    const std::string textureName = page.textureName;
    page.textureReferenceHeld = false;
    if (!textureReferences_.Release(textureName))
        return;
    if (destroyUnreferencedOgre2Texture(scene_, textureName))
    {
        ++destroyedTextures_;
        deferredTextureDeletes_.erase(textureName);
    }
    else if (ogre2TextureExists(scene_, textureName))
    {
        deferredTextureDeletes_.insert(textureName);
        logError("[DynamicTerrain][GPU-GC] Ogre2 texture deletion deferred name=",
                 textureName);
    }
}

bool PersistentTerrainRenderer::DetachAndDestroyMaterial(
    gz::rendering::MaterialPtr &material)
{
    if (!material)
        return true;

    try
    {
        material->ClearTexture();
    }
    catch (...)
    {
        return false;
    }

    try
    {
        scene_->DestroyMaterial(material);
        ++destroyedMaterials_;
    }
    catch (...)
    {
    }
    material.reset();
    return true;
}

void PersistentTerrainRenderer::DestroyPageMaterial(PageSlot &page)
{
    if (DetachAndDestroyMaterial(page.material))
    {
        ReleaseTextureReference(page);
        return;
    }

    PageSlot deferred;
    deferred.material = std::move(page.material);
    deferred.textureName = std::move(page.textureName);
    deferred.textureReferenceHeld = page.textureReferenceHeld;
    page.textureReferenceHeld = false;
    deferredTextureReleases_.push_back(std::move(deferred));
    logError("[DynamicTerrain][GPU-GC] ClearTexture failed; cleanup deferred");
}

void PersistentTerrainRenderer::DrainDeferredTextureReleases()
{
    auto it = deferredTextureReleases_.begin();
    while (it != deferredTextureReleases_.end())
    {
        if (!DetachAndDestroyMaterial(it->material))
        {
            ++it;
            continue;
        }
        ReleaseTextureReference(*it);
        it = deferredTextureReleases_.erase(it);
    }
}

void PersistentTerrainRenderer::DrainDeferredTextureDeletes()
{
    auto it = deferredTextureDeletes_.begin();
    while (it != deferredTextureDeletes_.end())
    {
        if (textureReferences_.References(*it) != 0u)
        {
            ++it;
            continue;
        }
        if (!ogre2TextureExists(scene_, *it))
        {
            it = deferredTextureDeletes_.erase(it);
            continue;
        }
        if (destroyUnreferencedOgre2Texture(scene_, *it))
        {
            ++destroyedTextures_;
            it = deferredTextureDeletes_.erase(it);
            continue;
        }
        ++it;
    }
}

void PersistentTerrainRenderer::ReleaseMeshResource(PageSlot &page)
{
    if (page.meshName.empty())
        return;
    const std::string meshName = std::move(page.meshName);
    if (page.meshReferenceHeld)
    {
        page.meshReferenceHeld = false;
        if (!meshReferences_.Release(meshName))
            return;
    }
    else if (meshReferences_.References(meshName) != 0u)
    {
        return;
    }
    if (destroyUnreferencedOgre2Mesh(scene_, meshName))
    {
        ++destroyedMeshes_;
        deferredMeshDeletes_.erase(meshName);
    }
    else if (ogre2MeshExists(scene_, meshName))
    {
        deferredMeshDeletes_.insert(meshName);
        logError("[DynamicTerrain][GPU-GC] Ogre2 mesh deletion deferred name=",
                 meshName);
    }
}

void PersistentTerrainRenderer::DrainDeferredMeshDeletes()
{
    auto it = deferredMeshDeletes_.begin();
    while (it != deferredMeshDeletes_.end())
    {
        if (meshReferences_.References(*it) != 0u)
        {
            ++it;
            continue;
        }
        if (!ogre2MeshExists(scene_, *it))
        {
            it = deferredMeshDeletes_.erase(it);
            continue;
        }
        if (destroyUnreferencedOgre2Mesh(scene_, *it))
        {
            ++destroyedMeshes_;
            it = deferredMeshDeletes_.erase(it);
            continue;
        }
        ++it;
    }
}

bool PersistentTerrainRenderer::UnloadPageGpu(PageSlot &page)
{
    page.gpuResident = false;
    if (!scene_)
        return !page.geometry && !page.visual && !page.material;

    if (page.visual)
    {
        try
        {
            page.visual->SetVisible(false);
            page.visual->RemoveGeometries();
        }
        catch (...) {}
    }

    bool geometryDestroyed = !page.geometry;
    if (page.geometry)
    {
        try
        {
            page.geometry->Destroy();
            geometryDestroyed = true;
        }
        catch (...) {}
    }

    if (!geometryDestroyed)
        return false;
    page.submesh.reset();
    page.geometry.reset();
    if (page.visual)
    {
        try { scene_->DestroyVisual(page.visual, true); } catch (...) {}
        page.visual.reset();
    }

    ReleaseMeshResource(page);

    DestroyPageMaterial(page);
    page.textureName.clear();
    page.textureBytes = 0u;
    return true;
}

void PersistentTerrainRenderer::DestroyPage(PageSlot &page)
{
    if (!UnloadPageGpu(page))
    {
        deferredPageReleases_.push_back(std::move(page));
        logError("[DynamicTerrain][GPU-GC] geometry destruction failed; page cleanup deferred");
        return;
    }
    page.sourceMesh.reset();
    page.sourceTexture.reset();
    page.sourceTextureName.clear();
    page.geographicName.clear();
}

void PersistentTerrainRenderer::DrainDeferredPageReleases()
{
    if (deferredPageReleases_.empty())
        return;
    auto deferred = std::move(deferredPageReleases_);
    deferredPageReleases_.clear();
    for (auto &page : deferred)
        DestroyPage(page);
}

void PersistentTerrainRenderer::DestroySlot(Slot &slot)
{
    if (!scene_)
        return;

    const auto materialCountBefore = destroyedMaterials_;
    const auto textureCountBefore = destroyedTextures_;
    const auto meshCountBefore = destroyedMeshes_;
    for (auto &page : slot.pages)
        DestroyPage(page);
    slot.pages.clear();

    if (cfg_.diagnostics)
        logInfo("[DynamicTerrain][GPU-GC] destroyed terrain slot generation=",
                slot.generation,
                " materials=", destroyedMaterials_ - materialCountBefore,
                " textures=", destroyedTextures_ - textureCountBefore,
                " meshes=", destroyedMeshes_ - meshCountBefore,
                " remaining_named_textures=",
                textureReferences_.ResourceCount(),
                " total_textures_destroyed=", destroyedTextures_);
}

void PersistentTerrainRenderer::OnPostRender()
{
    PruneRenderImageCache();
    if (retired_)
    {
        const auto generation = retired_->generation;
        DestroySlot(*retired_);
        retired_.reset();
        if (cfg_.diagnostics)
            logInfo("[DynamicTerrain][RENDER] retired old mesh after completed frame generation=",
                    generation);
    }
}

void PersistentTerrainRenderer::OnRenderTeardown()
{
    if (renderShuttingDown_.exchange(true, std::memory_order_acq_rel))
        return;
    {
        std::lock_guard<std::mutex> lock(queueMutex_);
        pendingSnapshot_.reset();
        pendingTexture_.reset();
    }
    if (retired_)
        DestroySlot(*retired_);
    if (staging_)
        DestroySlot(*staging_);
    if (active_)
        DestroySlot(*active_);
    DrainDeferredPageReleases();
    DrainDeferredTextureReleases();
    DrainDeferredTextureDeletes();
    DrainDeferredMeshDeletes();
    retired_.reset();
    staging_.reset();
    active_.reset();
    if (!deferredPageReleases_.empty())
        logError("[DynamicTerrain][GPU-GC] render teardown retained ",
                 deferredPageReleases_.size(),
                 " pages whose native geometry could not be destroyed");
    if (!deferredTextureReleases_.empty())
        logError("[DynamicTerrain][GPU-GC] render teardown retained ",
                 deferredTextureReleases_.size(),
                 " materials whose texture binding could not be cleared");
    if (!deferredTextureDeletes_.empty())
        logError("[DynamicTerrain][GPU-GC] render teardown retained ",
                 deferredTextureDeletes_.size(),
                 " Ogre2 texture deletions for backend teardown");
    if (!deferredMeshDeletes_.empty())
        logError("[DynamicTerrain][GPU-GC] render teardown retained ",
                 deferredMeshDeletes_.size(),
                 " Ogre2 mesh deletions for backend teardown");
    ClearActiveState();
    if (textureReferences_.ResourceCount() != 0u)
        logError("[DynamicTerrain][GPU-GC] render teardown left ",
                 textureReferences_.ResourceCount(),
                 " named texture references pending");
    if (meshReferences_.ResourceCount() != 0u)
        logError("[DynamicTerrain][GPU-GC] render teardown left ",
                 meshReferences_.ResourceCount(),
                 " named mesh references pending");
    if (scene_ && root_)
    {
        try { scene_->DestroyVisual(root_, true); } catch (...) {}
    }
    root_.reset();
    scene_.reset();
}

}
