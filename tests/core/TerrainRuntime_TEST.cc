#include "dynamic_terrain/core/TerrainRuntime.hh"
#include "TestGeographicTransform.hh"
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <iostream>
#include <thread>
#include <stdexcept>

using namespace dynamic_terrain;
#define CHECK(x) do { if (!(x)) throw std::runtime_error(#x); } while (false)

namespace
{
class RecordingRenderer final : public TerrainRenderSink
{
public:
    void QueueSnapshot(std::shared_ptr<const TerrainSnapshot> value) override
    {
        std::lock_guard<std::mutex> lock(mutex);
        snapshot = std::move(value);
    }
    void QueueTexture(TextureUpdate value) override
    {
        std::lock_guard<std::mutex> lock(mutex);
        textures = std::move(value);
    }
    std::uint64_t ActiveGeneration() const override
    { std::lock_guard<std::mutex> lock(mutex); return snapshot ? snapshot->generation : 0; }
    bool HasActiveTerrain() const override { return ActiveGeneration() != 0; }
    std::optional<TileKey> ActiveCenterTile() const override
    {
        std::lock_guard<std::mutex> lock(mutex);
        return snapshot ? std::optional<TileKey>(snapshot->centerTile) : std::nullopt;
    }
    mutable std::mutex mutex;
    std::shared_ptr<const TerrainSnapshot> snapshot;
    std::optional<TextureUpdate> textures;
};

template<class Predicate> void waitUntil(Predicate predicate)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (!predicate())
    {
        CHECK(std::chrono::steady_clock::now() < deadline);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}
}

int main()
{
    const auto path = fs::temp_directory_path() / ("terrain_runtime_test_" +
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    try
    {
        Config cfg;
        cfg.cacheDir = path.string(); cfg.modelName = "runtime_test";
        cfg.imageryProvider = "synthetic"; cfg.imageryExtension = "png";
        cfg.imageryUrl = "file:///no-network-in-runtime-test/{z}/{x}/{y}.png";
        cfg.elevationProvider = "flat"; cfg.diagnostics = false;
        cfg.downloadRetries = 0; cfg.visualGeometryZoom = 14;
        cfg.visualBootstrapImageryZoom = 14; cfg.visualRadiusM = 1000;
        cfg.visualMeshCellsPerTile = 8; cfg.visualPageTextureMaxSize = 512;
        cfg.visualImageryLodTable = {{1000000, 15}};
        cfg.radiusTiles = 0; cfg.heightmapSize = 129;
        cfg.dynamicZoom = false; cfg.staticZoom = 17;
        normalizeConfig(cfg);
        auto geographic = std::make_shared<TestGeographicTransform>(52.2297, 21.0122, 123.4, 37.0);
        TileStore store(cfg, geographic);
        const auto center = latLonToTile(52.2297, 21.0122, 14);
        for (int zoom : {14, 15})
        {
            const int scale = 1 << (zoom - 14);
            for (int y = (center.y-3)*scale; y <= (center.y+4)*scale; ++y)
                for (int x = (center.x-3)*scale; x <= (center.x+4)*scale; ++x)
                {
                    const auto file = store.ImageryPath({x,y,zoom});
                    fs::create_directories(file.parent_path());
                    CHECK(cv::imwrite(file.string(), cv::Mat(16,16,CV_8UC3,cv::Scalar(30,70,110))));
                }
        }
        RecordingRenderer renderer;
        {
            TerrainRuntime runtime(cfg, geographic, renderer);
            // Collision generation must work before any renderer has a scene.
            runtime.UpdateCollision(0,52.2297,21.0122,true);
            std::optional<CollisionResult> collision;
            waitUntil([&] { collision = runtime.PollCollision(false); return collision.has_value(); });
            CHECK(collision->patch && fs::exists(collision->patch->heightmap));
            CHECK(!renderer.HasActiveTerrain());
            runtime.UpdateVisual({},52.2297,21.0122,true);
            waitUntil([&] { return renderer.HasActiveTerrain(); });
            waitUntil([&] { std::lock_guard<std::mutex> lock(renderer.mutex); return renderer.textures.has_value(); });
            std::lock_guard<std::mutex> lock(renderer.mutex);
            CHECK(renderer.snapshot->generation == 1);
            CHECK(renderer.textures->generation == 1);
            CHECK(!renderer.textures->pages.empty());
            // Refinement changes its private page list, never the published snapshot.
            for (const auto &page : renderer.snapshot->pages) CHECK(page.imageryZoom == 14);
            for (const auto &page : renderer.textures->pages) CHECK(page.imageryZoom == 15);
        }
        fs::remove_all(path);
        std::cout << "Headless collision, worker handoff, refinement and immutable snapshots passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        fs::remove_all(path);
        std::cerr << error.what() << '\n';
        return 1;
    }
}
