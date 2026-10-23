#include "dynamic_terrain/core/TerrainRuntime.hh"
#include "dynamic_terrain/core/TerrainBuilder.hh"
#include <curl/curl.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <thread>
#include <unordered_map>
namespace dynamic_terrain
{
namespace {
struct VisualRequest
{
    TileKey center;
    std::uint64_t generation{0};
};

struct RefinementRequest
{
    std::shared_ptr<TerrainSnapshot> snapshot;
    std::uint64_t generation{0};
    unsigned retryAttempt{0};
};

std::int64_t steadyMilliseconds()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}


}
struct TerrainRuntime::Impl
{
    Config cfg_;
    std::shared_ptr<const GeographicTransform> spherical_;
    std::shared_ptr<TileStore> store_;
    std::unique_ptr<PersistentTerrainBuilder> visualBuilder_;
    std::unique_ptr<CollisionTerrainBuilder> collisionBuilder_;
    TerrainRenderSink *renderer_;
    SnapshotCallback onSnapshot_;
    TextureCallback onTexture_;
std::atomic<bool> stop_{false};

std::thread visualThread_;
std::thread refineThread_;
std::mutex visualMutex_;
std::condition_variable visualCv_;
std::optional<VisualRequest> visualRequest_;
std::atomic<bool> visualBuilding_{false};
std::atomic<std::uint64_t> latestVisualGeneration_{0};
std::atomic<std::int64_t> visualRetryAfterMs_{0};
std::uint64_t visualGeneration_{0};
std::optional<TileKey> lastVisualRequestedCenter_;

std::mutex refineMutex_;
std::condition_variable refineCv_;
std::optional<RefinementRequest> refineRequest_;
std::atomic<bool> refinementActive_{false};

std::thread collisionThread_;
std::mutex collisionMutex_;
std::condition_variable collisionCv_;
std::optional<CollisionRequest> collisionRequest_;
std::atomic<std::uint64_t> latestCollisionGeneration_{0};
std::atomic<std::int64_t> collisionRetryAfterMs_{0};
std::uint64_t collisionGeneration_{0};
std::optional<TileKey> lastCollisionRequestedCenter_;

std::mutex collisionResultMutex_;
std::optional<CollisionResult> collisionResult_;

    Impl(Config config, std::shared_ptr<const GeographicTransform> geographic,
         TerrainRenderSink &renderer, SnapshotCallback snapshot, TextureCallback texture)
        : cfg_(std::move(config)), spherical_(std::move(geographic)), renderer_(&renderer),
          onSnapshot_(std::move(snapshot)), onTexture_(std::move(texture))
    {
        store_ = std::make_shared<TileStore>(cfg_, spherical_);
        visualBuilder_ = std::make_unique<PersistentTerrainBuilder>(store_);
        collisionBuilder_ = std::make_unique<CollisionTerrainBuilder>(store_);
        try
        {
            visualThread_ = std::thread([this] { visualWorker(); });
            refineThread_ = std::thread([this] { refinementWorker(); });
            collisionThread_ = std::thread([this] { collisionWorker(); });
        }
        catch (...) { Stop(); throw; }
    }
    void Stop()
    {
        {
            // Synchronise the predicate transition with each waiter's mutex;
            // an atomic alone does not prevent a notify-before-wait race.
            std::scoped_lock lock(visualMutex_, refineMutex_, collisionMutex_);
            stop_.store(true, std::memory_order_relaxed);
        }
        visualCv_.notify_all(); refineCv_.notify_all(); collisionCv_.notify_all();
        if (store_) store_->StopDownloads();
        if (visualThread_.joinable()) visualThread_.join();
        if (refineThread_.joinable()) refineThread_.join();
        if (collisionThread_.joinable()) collisionThread_.join();
    }
    ~Impl() { Stop(); }
void updateVisualRequest(const Vec3 &vehicleWorld,
                         double latDeg, double lonDeg,
                         bool force = false)
{
    if (!visualBuilder_)
        return;
    const auto retryAfter = visualRetryAfterMs_.load(std::memory_order_relaxed);
    if (!force && retryAfter > 0 && steadyMilliseconds() >= retryAfter)
    {
        force = true;
        visualRetryAfterMs_.store(0, std::memory_order_relaxed);
    }
    const TileKey center = visualBuilder_->SnappedCenter(latDeg, lonDeg);
    bool need = force;
    if (!need)
    {
        const auto activeCenter = renderer_ ? renderer_->ActiveCenterTile() : std::nullopt;
        if (!activeCenter)
        {
            need = !lastVisualRequestedCenter_.has_value();
        }
        else
        {
            const double activeLat = tileYToLat(activeCenter->y + 0.5, activeCenter->z);
            const double activeLon = tileXToLon(activeCenter->x + 0.5, activeCenter->z);
            const auto activeLocal = localFromGeodetic(
                *spherical_, activeLat, activeLon,
                spherical_->ElevationReference());
            const double dx = vehicleWorld.X() - activeLocal.X();
            const double dy = vehicleWorld.Y() - activeLocal.Y();
            need = std::sqrt(dx * dx + dy * dy) >= cfg_.visualRecenterDistanceM;
        }
    }
    if (!need)
        return;
    if (lastVisualRequestedCenter_ && *lastVisualRequestedCenter_ == center && !force)
        return;

    VisualRequest request{center, ++visualGeneration_};
    {
        std::lock_guard<std::mutex> lock(visualMutex_);
        visualRequest_ = request;
    }
    latestVisualGeneration_.store(request.generation, std::memory_order_relaxed);
    lastVisualRequestedCenter_ = center;
    visualCv_.notify_one();
    logInfo("[DynamicTerrain][VISUAL] requested generation=", request.generation,
            " center=", tileText(center),
            " vehicle_xy=", vehicleWorld.X(), ",", vehicleWorld.Y());
}

void queueRefinement(const std::shared_ptr<TerrainSnapshot> &snapshot)
{
    if (!snapshot || !cfg_.visualRefineTexture)
        return;
    std::lock_guard<std::mutex> lock(refineMutex_);
    if (!refineRequest_ || snapshot->generation >= refineRequest_->generation)
        refineRequest_ = RefinementRequest{snapshot, snapshot->generation};
    refineCv_.notify_one();
}

void visualWorker()
{
    while (!stop_.load(std::memory_order_relaxed))
    {
        VisualRequest request;
        {
            std::unique_lock<std::mutex> lock(visualMutex_);
            visualCv_.wait(lock, [&]
                           { return stop_.load(std::memory_order_relaxed) || visualRequest_.has_value(); });
            if (stop_.load(std::memory_order_relaxed))
                return;
            request = *visualRequest_;
            visualRequest_.reset();
            visualBuilding_.store(true, std::memory_order_relaxed);
        }

        auto isStale = [&]()
        {
            return stop_.load(std::memory_order_relaxed) || request.generation <
                   latestVisualGeneration_.load(std::memory_order_relaxed);
        };

        ScopedDownloadContext downloads(isStale, 10);
        // Only an empty Gazebo view opts into incremental activation. A failed
        // replacement must continue to retain the complete old generation.
        const bool hadActiveTerrain = renderer_->HasActiveTerrain();
        PersistentTerrainBuilder::ProgressCallback progress;
        if (!hadActiveTerrain && renderer_->SupportsProgressiveInitial())
            progress = [&](const TerrainSnapshot &partial) {
                if (isStale()) return false;
                auto published = std::make_shared<const TerrainSnapshot>(partial);
                renderer_->QueueSnapshot(published);
                if (onSnapshot_) onSnapshot_(published);
                if (cfg_.diagnostics)
                    logInfo("[DynamicTerrain][VISUAL] initial progress generation=", request.generation,
                            " revision=", partial.coverageRevision, " pages=", partial.pages.size(),
                            " complete=", partial.progressiveComplete);
                return true;
            };
        std::string error;
        auto snapshot = visualBuilder_->BuildBootstrap(
            request.center, request.generation, error, progress);
        if (!snapshot)
        {
            logError("[DynamicTerrain][VISUAL] generation=", request.generation,
                     " failed: ", error,
                     "; currently active terrain remains untouched");
            visualRetryAfterMs_.store(
                steadyMilliseconds() + static_cast<std::int64_t>(cfg_.retryDelaySec * 1000.0),
                std::memory_order_relaxed);
            visualBuilding_.store(false, std::memory_order_relaxed);
            continue;
        }

        if (isStale())
        {
            logInfo("[DynamicTerrain][VISUAL] generation=", request.generation,
                    " superseded during bootstrap; cache kept, mesh not queued");
            visualBuilding_.store(false, std::memory_order_relaxed);
            continue;
        }

        if (hadActiveTerrain && cfg_.visualRefineTexture)
        {
            const int geometryZoom = snapshot->geometryRect.zoom;
            const int readyZoom = std::max(
                geometryZoom, cfg_.visualRecenterReadyZoom);
            std::vector<int> pageZoom(snapshot->pages.size(),
                                      cfg_.visualBootstrapImageryZoom);
            for (std::size_t i = 0; i < snapshot->pages.size(); ++i)
                pageZoom[i] = snapshot->pages[i].imageryZoom;

            auto pageOrder = visualBuilder_->ProgressivePageOrder(*snapshot);
            for (int zoom = geometryZoom; zoom <= readyZoom && !isStale(); ++zoom)
            {
                std::vector<std::size_t> candidates;
                for (const auto i : pageOrder)
                {
                    if (i >= pageZoom.size() || pageZoom[i] >= zoom)
                        continue;
                    if (visualBuilder_->TargetZoomForPage(*snapshot, i) < zoom)
                        continue;
                    candidates.push_back(i);
                }

                const std::size_t batchSize = std::max<std::size_t>(
                    1u, static_cast<std::size_t>(cfg_.visualRefineMaxSourceTilesPerBatch));
                for (std::size_t offset = 0;
                     offset < candidates.size() && !isStale();
                     offset += batchSize)
                {
                    const auto last = std::min(candidates.size(), offset + batchSize);
                    std::vector<std::size_t> batch(candidates.begin() + offset,
                                                   candidates.begin() + last);
                    std::string stageError;
                    auto update = visualBuilder_->BuildTextureStage(
                        *snapshot, batch, zoom, stageError);
                    if (update)
                    {
                        visualBuilder_->ApplyTextureUpdate(*snapshot, *update);
                        for (const auto &u : update->pages)
                            if (u.pageIndex < pageZoom.size())
                                pageZoom[u.pageIndex] = std::max(
                                    pageZoom[u.pageIndex], u.imageryZoom);
                    }
                    if (!stageError.empty())
                        logInfo("[DynamicTerrain][TEXTURE] recenter base generation=",
                                request.generation, " z=", zoom,
                                " pages=", batch.size(), " partial: ", stageError);
                }
            }

            if (isStale())
            {
                logInfo("[DynamicTerrain][VISUAL] generation=", request.generation,
                        " superseded while preparing base imagery; not queued");
                visualBuilding_.store(false, std::memory_order_relaxed);
                continue;
            }

            if (cfg_.visualDetailMode == "bottom_camera_only")
            {
                std::vector<std::size_t> detailPages;
                for (const auto i : pageOrder)
                {
                    if (i >= pageZoom.size())
                        continue;
                    const int target = visualBuilder_->TargetZoomForPage(*snapshot, i);
                    if (target > geometryZoom && pageZoom[i] < target)
                        detailPages.push_back(i);
                }

                for (const auto pageIndex : detailPages)
                {
                    if (isStale())
                        break;
                    const int target = visualBuilder_->TargetZoomForPage(
                        *snapshot, pageIndex);
                    std::string detailError;
                    auto update = visualBuilder_->BuildTextureStage(
                        *snapshot, {pageIndex}, target, detailError);
                    if (update)
                    {
                        visualBuilder_->ApplyTextureUpdate(*snapshot, *update);
                        for (const auto &u : update->pages)
                            if (u.pageIndex < pageZoom.size())
                                pageZoom[u.pageIndex] = std::max(
                                    pageZoom[u.pageIndex], u.imageryZoom);
                    }
                    if (!detailError.empty())
                        logInfo("[DynamicTerrain][TEXTURE] recenter preload generation=",
                                request.generation, " page=", pageIndex,
                                " z=", target, " partial: ", detailError);
                }
            }

            if (isStale())
            {
                logInfo("[DynamicTerrain][VISUAL] generation=", request.generation,
                        " superseded during detail preload; not queued");
                visualBuilding_.store(false, std::memory_order_relaxed);
                continue;
            }

            bool baseReady = true;
            for (std::size_t i = 0; i < pageZoom.size(); ++i)
            {
                const int target = visualBuilder_->TargetZoomForPage(*snapshot, i);
                const int required = cfg_.visualDetailMode == "bottom_camera_only"
                                         ? target
                                         : std::min(readyZoom, target);
                if (pageZoom[i] < required)
                {
                    baseReady = false;
                    break;
                }
            }
            if (!baseReady)
            {
                logInfo("[DynamicTerrain][VISUAL] generation=", request.generation,
                        " base imagery incomplete; keeping current active terrain and retrying");
                visualRetryAfterMs_.store(
                    steadyMilliseconds() + static_cast<std::int64_t>(
                                               cfg_.retryDelaySec * 1000.0),
                    std::memory_order_relaxed);
                visualBuilding_.store(false, std::memory_order_relaxed);
                continue;
            }
        }

        auto scheduleRetry = [&] {
            visualRetryAfterMs_.store(steadyMilliseconds() + static_cast<std::int64_t>(
                std::max(1.0, cfg_.retryDelaySec) * 1000.0), std::memory_order_relaxed);
        };
        auto extendCoverage = [&] {
            // During progressive startup, missing ground must not wait behind
            // hundreds of higher-resolution images for already visible pages.
            // Collision (20) and initial near ground (10) still take precedence.
            ScopedDownloadContext outerDownloads(isStale, progress ? 8 : 2);
            std::string outerError;
            auto extended = visualBuilder_->BuildOuterCoverage(*snapshot, outerError, progress);
            if (!extended)
            {
                logInfo("[DynamicTerrain][COVERAGE] generation=", request.generation,
                        " incomplete: ", outerError, "; active terrain retained");
                if (!isStale()) scheduleRetry();
                return false;
            }
            snapshot = std::move(extended);
            return true;
        };
        auto publish = [&] {
            if (isStale()) return false;
            auto published = std::make_shared<const TerrainSnapshot>(*snapshot);
            renderer_->QueueSnapshot(published);
            if (onSnapshot_) onSnapshot_(published);
            logInfo("[DynamicTerrain][VISUAL] coverage queued generation=", request.generation,
                    " revision=", snapshot->coverageRevision,
                    " center=", tileText(request.center));
            const auto deadline = std::chrono::steady_clock::now() +
                std::chrono::duration<double>(cfg_.visualActivationTimeoutSec);
            auto active = [&] {
                return renderer_->ActiveGeneration() == request.generation &&
                    renderer_->ActiveCoverageRevision() >= snapshot->coverageRevision;
            };
            while (!isStale() && !active() && std::chrono::steady_clock::now() < deadline)
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            if (isStale()) return false;
            if (!active())
            {
                logInfo("[DynamicTerrain][VISUAL] activation timeout generation=",
                        request.generation, " revision=", snapshot->coverageRevision,
                        "; keeping active terrain and retrying");
                scheduleRetry();
                return false;
            }
            visualBuilder_->PinActiveTextures(*snapshot);
            // Separate page metadata from the visual worker's outer build;
            // their large immutable image and mesh buffers remain shared.
            queueRefinement(std::make_shared<TerrainSnapshot>(*snapshot));
            logInfo("[DynamicTerrain][VISUAL] active generation=", request.generation,
                    " revision=", snapshot->coverageRevision,
                    " center=", tileText(request.center), " refinement_worker=queued");
            return true;
        };
        // Bootstrap local ground immediately on startup. Every replacement
        // thereafter includes the distant layer before replacing usable coverage.
        if (hadActiveTerrain && cfg_.visualOuterRadiusM > 0 && !extendCoverage())
        {
            visualBuilding_.store(false, std::memory_order_relaxed);
            continue;
        }
        if (hadActiveTerrain && cfg_.visualRefineTexture)
        {
            // Recentering changes the outer quadtree's page keys. A resident
            // texture keyed by the old parent/children cannot protect the new
            // pages from a bootstrap-quality flash. Prepare their final LOD
            // before activation, keeping the complete old view during downloads.
            // Initial startup still publishes each available bootstrap page.
            ScopedDownloadContext outerDownloads(isStale, 2);
            const auto order = visualBuilder_->ProgressivePageOrder(*snapshot);
            for (int zoom = 0; zoom <= 20 && !isStale(); ++zoom)
            {
                std::vector<std::size_t> batch;
                std::size_t sourceTiles = 0;
                const auto limit = static_cast<std::size_t>(
                    std::max(1, cfg_.visualRefineMaxSourceTilesPerBatch));
                auto prepareBatch = [&] {
                    if (batch.empty() || isStale()) return;
                    std::string stageError;
                    auto update = visualBuilder_->BuildTextureStage(*snapshot, batch, zoom, stageError);
                    if (update) visualBuilder_->ApplyTextureUpdate(*snapshot, *update);
                    if (!stageError.empty())
                        logInfo("[DynamicTerrain][TEXTURE] recenter outer generation=",
                                request.generation, " z=", zoom, " partial: ", stageError);
                    batch.clear();
                    sourceTiles = 0;
                };
                for (const auto i : order)
                {
                    if (isStale()) break;
                    const auto &page = snapshot->pages[i];
                    if (!page.visualOnly || page.imageryZoom >= zoom ||
                        visualBuilder_->TargetZoomForPage(*snapshot, i) != zoom)
                        continue;
                    const std::size_t cost = std::size_t{1} << (2 * std::max(0, zoom - page.key.z));
                    if (!batch.empty() && sourceTiles + cost > limit) prepareBatch();
                    batch.push_back(i);
                    sourceTiles += cost;
                }
                prepareBatch();
            }
            const bool ready = std::all_of(snapshot->pages.begin(), snapshot->pages.end(),
                [](const TerrainPage &page) {
                    return !page.visualOnly || page.imageryZoom >= page.targetImageryZoom;
                });
            if (isStale() || !ready)
            {
                if (!isStale())
                {
                    logInfo("[DynamicTerrain][VISUAL] generation=", request.generation,
                            " outer imagery incomplete; keeping current active terrain and retrying");
                    scheduleRetry();
                }
                visualBuilding_.store(false, std::memory_order_relaxed);
                continue;
            }
        }
        if (publish() && !hadActiveTerrain && cfg_.visualOuterRadiusM > 0 &&
            !isStale() && extendCoverage())
            publish();

        visualBuilding_.store(false, std::memory_order_relaxed);
    }
}

void refinementWorker()
{
    while (!stop_.load(std::memory_order_relaxed))
    {
        RefinementRequest request;
        {
            std::unique_lock<std::mutex> lock(refineMutex_);
            refineCv_.wait(lock, [&]
                           { return stop_.load(std::memory_order_relaxed) || refineRequest_.has_value(); });
            if (stop_.load(std::memory_order_relaxed))
                return;
            request = *refineRequest_;
            refineRequest_.reset();
            refinementActive_.store(true, std::memory_order_relaxed);
        }

        auto stale = [&]()
        {
            return stop_.load(std::memory_order_relaxed) || request.generation !=
                       latestVisualGeneration_.load(std::memory_order_relaxed) ||
                   renderer_->ActiveGeneration() != request.generation ||
                   (request.snapshot && !request.snapshot->progressiveInitial &&
                    renderer_->ActiveCoverageRevision() != request.snapshot->coverageRevision);
        };
        const auto activationDeadline =
            std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (!stop_.load(std::memory_order_relaxed) &&
               request.generation == latestVisualGeneration_.load(std::memory_order_relaxed) &&
               renderer_->ActiveGeneration() < request.generation &&
               std::chrono::steady_clock::now() < activationDeadline)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        if (!request.snapshot || stale())
        {
            refinementActive_.store(false, std::memory_order_relaxed);
            continue;
        }

        // Keep near-detail priority over background replacements. Progressive
        // startup gives missing coverage priority 8 in visualWorker; after full
        // coverage is resident, further refinements use background priority 0.
        ScopedDownloadContext downloads(stale,
            cfg_.visualOuterRadiusM > 0 &&
            std::none_of(request.snapshot->pages.begin(), request.snapshot->pages.end(),
                [](const TerrainPage &page) { return page.visualOnly; }) ? 5 : 0);
        auto &snapshot = *request.snapshot;
        const int geometryZoom = snapshot.geometryRect.zoom;
        auto pageOrder = visualBuilder_->ProgressivePageOrder(snapshot);
        std::vector<int> pageZoom(snapshot.pages.size(), cfg_.visualBootstrapImageryZoom);
        std::size_t changedPages = 0;
        std::unordered_map<std::size_t, TexturePageUpdate> finalPageUpdates;
        for (std::size_t i = 0; i < snapshot.pages.size(); ++i)
            pageZoom[i] = snapshot.pages[i].imageryZoom;

        int minimumZoom = geometryZoom;
        for (const auto &page : snapshot.pages) minimumZoom = std::min(minimumZoom, page.key.z);
        int maxTargetZoom = geometryZoom;
        for (std::size_t i = 0; i < snapshot.pages.size(); ++i)
            maxTargetZoom = std::max(
                maxTargetZoom, visualBuilder_->TargetZoomForPage(snapshot, i));

        std::vector<int> refinementLevels;
        for (int zoom = minimumZoom; zoom <= maxTargetZoom; ++zoom)
            refinementLevels.push_back(zoom);

        for (const int zoom : refinementLevels)
        {
            if (stale())
                break;
            std::vector<std::size_t> candidates;
            for (const auto pageIndex : pageOrder)
            {
                if (pageIndex >= pageZoom.size() || pageZoom[pageIndex] >= zoom)
                    continue;
                if (visualBuilder_->TargetZoomForPage(snapshot, pageIndex) < zoom)
                    continue;
                candidates.push_back(pageIndex);
            }
            if (candidates.empty())
                continue;

            int delta = 0;
            for (const auto index : candidates)
                delta = std::max(delta, zoom - snapshot.pages[index].key.z);
            std::size_t tilesPerPage = 1u;
            for (int i = 0; i < delta; ++i)
                tilesPerPage *= 4u;
            const std::size_t batchSize = std::max<std::size_t>(
                1u, static_cast<std::size_t>(cfg_.visualRefineMaxSourceTilesPerBatch) /
                        std::max<std::size_t>(1u, tilesPerPage));

            logInfo("[DynamicTerrain][TEXTURE] low-priority level generation=",
                    request.generation, " z=", zoom,
                    " pages=", candidates.size(),
                    " pages_per_batch=", batchSize);

            for (std::size_t offset = 0;
                 offset < candidates.size() && !stale();
                 offset += batchSize)
            {
                const auto last = std::min(candidates.size(), offset + batchSize);
                std::vector<std::size_t> batch(candidates.begin() + offset,
                                               candidates.begin() + last);
                std::string stageError;
                auto update = visualBuilder_->BuildTextureStage(
                    snapshot, batch, zoom, stageError);
                if (stale())
                    break;
                if (update)
                {
                    visualBuilder_->ApplyTextureUpdate(snapshot, *update);
                    visualBuilder_->PinActiveTextures(snapshot);
                    changedPages += update->pages.size();
                    for (auto &u : update->pages)
                    {
                        if (u.pageIndex < pageZoom.size())
                            pageZoom[u.pageIndex] = std::max(
                                pageZoom[u.pageIndex], u.imageryZoom);
                        if (cfg_.visualProgressiveUploads)
                            continue;
                        auto existing = finalPageUpdates.find(u.pageIndex);
                        if (existing == finalPageUpdates.end() ||
                            u.imageryZoom >= existing->second.imageryZoom)
                            finalPageUpdates[u.pageIndex] = std::move(u);
                    }
                    if (cfg_.visualProgressiveUploads)
                    {
                        if (onTexture_)
                            onTexture_(*update);
                        renderer_->QueueTexture(std::move(*update));
                    }
                }
                if (!stageError.empty())
                    logInfo("[DynamicTerrain][TEXTURE] low-priority generation=",
                            request.generation, " z=", zoom,
                            " pages=", batch.size(), " partial: ", stageError);
            }
        }

        if (stale())
        {
            logInfo("[DynamicTerrain][TEXTURE] refinement preempted generation=",
                    request.generation, " by latest_generation=",
                    latestVisualGeneration_.load(std::memory_order_relaxed));
        }
        else
        {
            if (!finalPageUpdates.empty())
            {
                TextureUpdate finalUpdate;
                finalUpdate.generation = request.generation;
                finalUpdate.pages.reserve(finalPageUpdates.size());
                for (auto &entry : finalPageUpdates)
                    finalUpdate.pages.push_back(std::move(entry.second));
                finalUpdate.changedPageCount = finalUpdate.pages.size();
                if (!stale())
                {
                    if (onTexture_)
                        onTexture_(finalUpdate);
                    renderer_->QueueTexture(std::move(finalUpdate));
                    logInfo("[DynamicTerrain][TEXTURE] coalesced page uploads queued generation=",
                            request.generation, " changed_pages=",
                            finalPageUpdates.size());
                }
            }
            logInfo("[DynamicTerrain][TEXTURE] refinement complete generation=",
                    request.generation, " changed_pages=", changedPages,
                    " gpu_page_uploads=", cfg_.visualProgressiveUploads ? changedPages : finalPageUpdates.size());
        }
        refinementActive_.store(false, std::memory_order_relaxed);
        // A stationary aircraft must recover after an outage too. Previously a
        // failed refinement was abandoned forever until the next recenter.
        bool incomplete = false;
        for (std::size_t i = 0; i < pageZoom.size(); ++i)
            incomplete |= pageZoom[i] < visualBuilder_->TargetZoomForPage(snapshot, i);
        if (incomplete && !stale())
        {
            const double delay = std::min(30.0, cfg_.retryDelaySec *
                static_cast<double>(1u << std::min(request.retryAttempt, 5u)));
            std::unique_lock<std::mutex> lock(refineMutex_);
            refineCv_.wait_for(lock, std::chrono::duration<double>(delay), [&] {
                return stop_.load(std::memory_order_relaxed) || refineRequest_.has_value();
            });
            if (!refineRequest_ && !stale())
                refineRequest_ = RefinementRequest{request.snapshot, request.generation,
                                                   request.retryAttempt + 1u};
        }
    }
}

void updateCollisionRequest(double altitude, double latDeg, double lonDeg,
                            bool force = false)
{
    if (!cfg_.enableCollision || !collisionBuilder_)
        return;
    const auto retryAfter = collisionRetryAfterMs_.load(std::memory_order_relaxed);
    if (!force && retryAfter > 0 && steadyMilliseconds() >= retryAfter)
    {
        force = true;
        collisionRetryAfterMs_.store(0, std::memory_order_relaxed);
    }
    const int zoom = zoomForAltitude(cfg_, altitude);
    const TileKey aircraft = latLonToTile(latDeg, lonDeg, zoom);

    bool need = force || !lastCollisionRequestedCenter_ ||
                lastCollisionRequestedCenter_->z != zoom;
    if (!need)
    {
        const int threshold = std::max(1, cfg_.radiusTiles);
        const int dx = std::abs(aircraft.x - lastCollisionRequestedCenter_->x);
        const int dy = std::abs(aircraft.y - lastCollisionRequestedCenter_->y);
        need = dx >= threshold || dy >= threshold;
    }
    if (!need)
        return;

    CollisionRequest request{aircraft, ++collisionGeneration_};
    {
        std::lock_guard<std::mutex> lock(collisionMutex_);
        collisionRequest_ = request;
    }
    latestCollisionGeneration_.store(request.generation, std::memory_order_relaxed);
    lastCollisionRequestedCenter_ = aircraft;
    collisionCv_.notify_one();
    if (cfg_.diagnostics)
        logInfo("[DynamicTerrain][COLLISION] requested generation=", request.generation,
                " center=", tileText(aircraft), " altitude=", altitude);
}

void collisionWorker()
{
    while (!stop_.load(std::memory_order_relaxed))
    {
        CollisionRequest request;
        {
            std::unique_lock<std::mutex> lock(collisionMutex_);
            collisionCv_.wait(lock, [&]
                              { return stop_.load(std::memory_order_relaxed) || collisionRequest_.has_value(); });
            if (stop_.load(std::memory_order_relaxed))
                return;
            request = *collisionRequest_;
            collisionRequest_.reset();
        }
        ScopedDownloadContext downloads([&] {
            return stop_.load(std::memory_order_relaxed) || request.generation <
                latestCollisionGeneration_.load(std::memory_order_relaxed);
        }, 20);
        CollisionResult result;
        result.request = request;
        result.patch = collisionBuilder_->Build(request.center, result.error);
        {
            std::lock_guard<std::mutex> lock(collisionResultMutex_);
            if (!collisionResult_ || request.generation >= collisionResult_->request.generation)
                collisionResult_ = std::move(result);
        }
    }
}


};
TerrainRuntime::TerrainRuntime(Config config, std::shared_ptr<const GeographicTransform> geographic,
    TerrainRenderSink &renderer, SnapshotCallback snapshot, TextureCallback texture)
    : impl_(std::make_unique<Impl>(std::move(config), std::move(geographic), renderer,
                                 std::move(snapshot), std::move(texture))) {}
TerrainRuntime::~TerrainRuntime() = default;
void TerrainRuntime::UpdateVisual(const Vec3 &position, double latitude, double longitude, bool force)
{ impl_->updateVisualRequest(position, latitude, longitude, force); }
void TerrainRuntime::UpdateCollision(double altitude, double latitude, double longitude, bool force)
{ impl_->updateCollisionRequest(altitude, latitude, longitude, force); }
std::optional<CollisionResult> TerrainRuntime::PollCollision(bool hasActiveCollision)
{
    std::optional<CollisionResult> result;
    { std::lock_guard<std::mutex> lock(impl_->collisionResultMutex_);
      result = std::move(impl_->collisionResult_); impl_->collisionResult_.reset(); }
    if (!result) return std::nullopt;
    if (!result->patch)
    {
        logError("[DynamicTerrain][COLLISION] generation=", result->request.generation,
                 " failed: ", result->error, "; old collision remains active");
        impl_->collisionRetryAfterMs_.store(steadyMilliseconds() +
            static_cast<std::int64_t>(impl_->cfg_.retryDelaySec * 1000.0), std::memory_order_relaxed);
        return std::nullopt;
    }
    if (hasActiveCollision && result->request.generation <
        impl_->latestCollisionGeneration_.load(std::memory_order_relaxed)) return std::nullopt;
    return result;
}
bool TerrainRuntime::VisualBuilding() const { return impl_->visualBuilding_.load(); }
bool TerrainRuntime::RefinementActive() const { return impl_->refinementActive_.load(); }
std::uint64_t TerrainRuntime::LatestVisualGeneration() const { return impl_->latestVisualGeneration_.load(); }
TerrainRuntimeDiagnostics TerrainRuntime::Diagnostics() const
{
    const auto store = impl_->store_->Diagnostics();
    const auto pages = impl_->visualBuilder_->CachedPageStats();
    const auto meshes = impl_->visualBuilder_->CachedOuterMeshStats();
    return {store.downloads, store.decodedDemBytes, store.decodedDemEntries,
        store.decodedImageryBytes, store.decodedImageryEntries, pages.bytes, pages.pages,
        meshes.bytes, meshes.pages, meshes.limitBytes, meshes.hits, meshes.misses};
}
}
