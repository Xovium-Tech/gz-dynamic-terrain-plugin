#include "dynamic_terrain/core/TerrainRuntime.hh"
#include "../tests/core/TestGeographicTransform.hh"
#include <atomic>
#include <chrono>
#include <fstream>
#include <iostream>
#include <mutex>
#include <thread>
#include <unistd.h>

using namespace dynamic_terrain;
using Clock = std::chrono::steady_clock;
namespace
{
// Immediate acknowledgement deliberately isolates the real runtime/download/
// builder/collision pipeline from rendering. Run the camera soak alongside it.
class Sink final : public TerrainRenderSink
{
public:
    void QueueSnapshot(std::shared_ptr<const TerrainSnapshot> value) override
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (snapshot && value->generation <= snapshot->generation) ++invalid;
        for (const auto &page : value->pages)
        {
            if (!page.mesh || !page.texture || !page.texture->Valid()) { ++invalid; continue; }
            for (const auto &p : page.mesh->positions)
                if (!std::isfinite(p.X()) || !std::isfinite(p.Y()) || !std::isfinite(p.Z())) ++invalid;
            for (const auto i : page.mesh->indices)
                if (i >= page.mesh->positions.size()) ++invalid;
        }
        snapshot = std::move(value); ++generations;
    }
    void QueueTexture(TextureUpdate update) override
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (!snapshot || update.generation != snapshot->generation) return;
        auto copy = std::make_shared<TerrainSnapshot>(*snapshot);
        for (const auto &page : update.pages)
        {
            if (page.pageIndex >= copy->pages.size() || !page.texture || !page.texture->Valid()) { ++invalid; continue; }
            auto &target = copy->pages[page.pageIndex];
            if (target.key != page.pageKey || page.imageryZoom < target.imageryZoom) { ++invalid; continue; }
            target.texture = page.texture; target.imageryZoom = page.imageryZoom; ++textures;
        }
        snapshot = std::move(copy);
    }
    std::uint64_t ActiveGeneration() const override
    { std::lock_guard<std::mutex> lock(mutex); return snapshot ? snapshot->generation : 0; }
    bool HasActiveTerrain() const override { return ActiveGeneration() != 0; }
    std::optional<TileKey> ActiveCenterTile() const override
    { std::lock_guard<std::mutex> lock(mutex); return snapshot ? std::optional<TileKey>(snapshot->centerTile) : std::nullopt; }
    std::atomic<std::uint64_t> generations{0}, textures{0}, invalid{0};
private:
    mutable std::mutex mutex;
    std::shared_ptr<const TerrainSnapshot> snapshot;
};
std::uint64_t residentBytes()
{
    std::ifstream input("/proc/self/statm"); std::uint64_t size = 0, resident = 0;
    input >> size >> resident; return resident * static_cast<std::uint64_t>(sysconf(_SC_PAGESIZE));
}
}
int main(int argc, char **argv)
{
    if (argc != 5) { std::cerr << "usage: TerrainStreamingSoak URL SECONDS CACHE REPORT\n"; return 2; }
    try
    {
        const std::string base = argv[1]; const double seconds = std::stod(argv[2]);
        Config cfg; cfg.modelName = "streaming-soak"; cfg.cacheDir = argv[3]; cfg.diagnostics = false;
        cfg.imageryProvider = "soak"; cfg.imageryUrl = base + "/imagery/{z}/{x}/{y}.png"; cfg.imageryExtension = "png";
        cfg.elevationProvider = "terrarium"; cfg.elevationUrl = base + "/dem/{z}/{x}/{y}.png";
        cfg.elevationMaxZoom = 13; cfg.visualElevationZoom = 13; cfg.visualGeometryZoom = 14;
        cfg.visualRadiusM = 1000; cfg.visualRecenterDistanceM = 500;
        cfg.visualMeshCellsPerTile = 16; cfg.visualMaxMeshCells = 128;
        cfg.visualBootstrapImageryZoom = 12; cfg.visualPageTextureMaxSize = 512;
        cfg.visualImageryLodTable = {{1000000, 15}}; cfg.alignOriginToGround = false;
        cfg.dynamicZoom = false; cfg.staticZoom = 14; cfg.downloadConcurrency = 4; cfg.downloadPerHost = 2;
        cfg.httpTimeoutMs = 2000; cfg.downloadRetries = 3; cfg.retryDelaySec = 0.5;
        cfg.decodedDemCacheMb = 32; cfg.decodedImageryCacheMb = 16; cfg.visualPageCacheMb = 16;
        normalizeConfig(cfg);
        const auto geo = std::make_shared<TestGeographicTransform>(52.2297, 21.0122, 0.0, 0.0);
        Sink sink; std::uint64_t collisions = 0, maxResident = 0, samples = 0;
        TerrainRuntimeDiagnostics metrics;
        const auto start = Clock::now(); auto nextReport = start;
        Clock::time_point beforeStop;
        {
            TerrainRuntime runtime(cfg, geo, sink);
            while (std::chrono::duration<double>(Clock::now() - start).count() < seconds)
            {
                const auto elapsed = std::chrono::duration<double>(Clock::now() - start).count();
                const Vec3 position{elapsed * 120.0, 180.0 * std::sin(elapsed / 20), 250};
                const auto location = geo->GeodeticFromLocal(position);
                runtime.UpdateVisual(position, location.X(), location.Y());
                runtime.UpdateCollision(250, location.X(), location.Y());
                if (auto result = runtime.PollCollision(collisions != 0))
                {
                    if (!result->patch || !fs::is_regular_file(result->patch->heightmap)) throw std::runtime_error("invalid collision handoff");
                    ++collisions;
                }
                if (sink.invalid.load()) throw std::runtime_error("invalid or regressing streamed assets");
                if (elapsed > 30 && !sink.HasActiveTerrain()) throw std::runtime_error("terrain did not activate");
                maxResident = std::max(maxResident, residentBytes());
                if (Clock::now() >= nextReport)
                {
                    ++samples; nextReport = Clock::now() + std::chrono::seconds(30);
                    metrics = runtime.Diagnostics();
                    if (metrics.downloads.peakActive > static_cast<std::size_t>(cfg.downloadConcurrency) ||
                        metrics.downloads.peakQueued > 4096 ||
                        metrics.decodedDemBytes > static_cast<std::size_t>(cfg.decodedDemCacheMb) * 1024 * 1024 ||
                        metrics.decodedImageryBytes > static_cast<std::size_t>(cfg.decodedImageryCacheMb) * 1024 * 1024)
                        throw std::runtime_error("download queue or decoded cache exceeded configured bounds");
                    std::cout << "{\"elapsed\":" << elapsed << ",\"generation\":" << sink.ActiveGeneration()
                              << ",\"snapshots\":" << sink.generations << ",\"texture_pages\":" << sink.textures
                              << ",\"collision_patches\":" << collisions << ",\"rss_bytes\":" << residentBytes()
                              << ",\"download_queued\":" << metrics.downloads.queued
                              << ",\"download_active\":" << metrics.downloads.active
                              << ",\"download_retries\":" << metrics.downloads.retries
                              << ",\"download_cache_hits\":" << metrics.downloads.cacheHits
                              << ",\"decoded_dem_bytes\":" << metrics.decodedDemBytes
                              << ",\"decoded_imagery_bytes\":" << metrics.decodedImageryBytes
                              << ",\"page_texture_bytes\":" << metrics.pageTextureBytes << "}" << std::endl;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            metrics = runtime.Diagnostics();
            beforeStop = Clock::now();
        }
        const double shutdown = std::chrono::duration<double>(Clock::now() - beforeStop).count();
        if (!sink.generations || !collisions || shutdown > 2.0) throw std::runtime_error("streaming did not progress or shutdown stalled");
        std::ofstream output(argv[4]);
        output << "{\"seconds\":" << seconds << ",\"snapshots\":" << sink.generations
               << ",\"texture_pages\":" << sink.textures << ",\"collision_patches\":" << collisions
               << ",\"max_rss_bytes\":" << maxResident << ",\"final_rss_bytes\":" << residentBytes()
               << ",\"shutdown_seconds\":" << shutdown << ",\"progress_samples\":" << samples
               << ",\"download_peak_queued\":" << metrics.downloads.peakQueued
               << ",\"download_peak_active\":" << metrics.downloads.peakActive
               << ",\"download_cache_hits\":" << metrics.downloads.cacheHits
               << ",\"downloaded\":" << metrics.downloads.downloaded
               << ",\"download_failed\":" << metrics.downloads.failed
               << ",\"download_cancelled\":" << metrics.downloads.cancelled
               << ",\"download_bytes\":" << metrics.downloads.bytes
               << ",\"download_connections\":" << metrics.downloads.connections
               << ",\"download_retries\":" << metrics.downloads.retries
               << ",\"decoded_dem_bytes\":" << metrics.decodedDemBytes
               << ",\"decoded_imagery_bytes\":" << metrics.decodedImageryBytes
               << ",\"page_texture_bytes\":" << metrics.pageTextureBytes
               << ",\"invalid_assets\":" << sink.invalid << "}\n";
        return 0;
    }
    catch (const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
}
