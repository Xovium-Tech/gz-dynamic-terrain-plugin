#include "dynamic_terrain/core/TerrainRuntime.hh"
#include "dynamic_terrain/core/TerrainBuilder.hh"
#include "TestGeographicTransform.hh"
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <iostream>
#include <thread>
#include <stdexcept>
#include <unordered_map>

using namespace dynamic_terrain;
#define CHECK(x) do { if (!(x)) throw std::runtime_error(#x); } while (false)

namespace
{
class RecordingRenderer final : public TerrainRenderSink
{
public:
    explicit RecordingRenderer(bool progressive = false) : progressive(progressive) {}
    void QueueSnapshot(std::shared_ptr<const TerrainSnapshot> value) override
    {
        std::lock_guard<std::mutex> lock(mutex);
        snapshot = std::move(value);
        snapshots.push_back(snapshot);
    }
    void QueueTexture(TextureUpdate value) override
    {
        std::lock_guard<std::mutex> lock(mutex);
        textures = std::move(value);
    }
    std::uint64_t ActiveGeneration() const override
    { std::lock_guard<std::mutex> lock(mutex); return snapshot ? snapshot->generation : 0; }
    std::uint64_t ActiveCoverageRevision() const override
    { std::lock_guard<std::mutex> lock(mutex); return snapshot ? snapshot->coverageRevision : 0; }
    bool SupportsProgressiveInitial() const override { return progressive; }
    bool HasActiveTerrain() const override { return ActiveGeneration() != 0; }
    std::optional<TileKey> ActiveCenterTile() const override
    {
        std::lock_guard<std::mutex> lock(mutex);
        return snapshot ? std::optional<TileKey>(snapshot->centerTile) : std::nullopt;
    }
    mutable std::mutex mutex;
    std::shared_ptr<const TerrainSnapshot> snapshot;
    std::vector<std::shared_ptr<const TerrainSnapshot>> snapshots;
    std::optional<TextureUpdate> textures;
    const bool progressive;
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

void comparePayload(const TerrainSnapshot &actual, const TerrainSnapshot &expected)
{
    std::unordered_map<TileKey, const TerrainPage *, TileKeyHash> expectedPages;
    for (const auto &page : expected.pages) expectedPages.emplace(page.key, &page);
    CHECK(actual.pages.size() == expected.pages.size());
    CHECK(actual.estimatedTextureBytes == expected.estimatedTextureBytes);
    for (const auto &page : actual.pages)
    {
        CHECK(expectedPages.count(page.key));
        const auto &other = *expectedPages.at(page.key);
        CHECK(page.visualOnly == other.visualOnly);
        CHECK(page.imageryZoom == other.imageryZoom);
        CHECK(page.textureSize == other.textureSize);
        CHECK(page.texture->width == other.texture->width);
        CHECK(page.texture->height == other.texture->height);
        CHECK(page.texture->rgb == other.texture->rgb);
        CHECK(page.mesh->indices == other.mesh->indices);
        CHECK(page.mesh->positions.size() == other.mesh->positions.size());
        CHECK(page.mesh->normals.size() == other.mesh->normals.size());
        CHECK(page.mesh->texCoords.size() == other.mesh->texCoords.size());
        for (std::size_t i = 0; i < page.mesh->positions.size(); ++i)
        {
            CHECK(page.mesh->positions[i].X() == other.mesh->positions[i].X());
            CHECK(page.mesh->positions[i].Y() == other.mesh->positions[i].Y());
            CHECK(page.mesh->positions[i].Z() == other.mesh->positions[i].Z());
            CHECK(page.mesh->normals[i].X() == other.mesh->normals[i].X());
            CHECK(page.mesh->normals[i].Y() == other.mesh->normals[i].Y());
            CHECK(page.mesh->normals[i].Z() == other.mesh->normals[i].Z());
            CHECK(page.mesh->texCoords[i].X() == other.mesh->texCoords[i].X());
            CHECK(page.mesh->texCoords[i].Y() == other.mesh->texCoords[i].Y());
        }
    }
}

void testReplacementImagery(Config cfg,
    const std::shared_ptr<const GeographicTransform> &geographic, const fs::path &path)
{
    cfg.cacheDir = (path / "continuity").string();
    cfg.imageryUrl = (path / "continuity-source.png").string();
    cfg.visualRadiusM = 1000;
    cfg.visualOuterRadiusM = 30000;
    cfg.visualRefineTexture = true;
    cfg.visualPageTextureMaxSize = 512;
    cfg.visualImageryLodTable = {{1000000, 14}};
    cfg.visualPageCacheMb = 0; // Resident detail must survive disposable-cache eviction.
    cfg.visualOuterMeshCacheMb = 0;
    cfg.visualActivationTimeoutSec = 1;
    cfg.retryDelaySec = .1;
    normalizeConfig(cfg);
    auto writeSource = [&] {
        CHECK(cv::imwrite(cfg.imageryUrl, cv::Mat(32,32,CV_8UC3,cv::Scalar(35,95,155))));
    };
    writeSource();
    auto store = std::make_shared<TileStore>(cfg, geographic);
    PersistentTerrainBuilder builder(store);
    const auto center = builder.SnappedCenter(geographic->LatitudeDeg(), geographic->LongitudeDeg());
    RecordingRenderer renderer;
    TerrainRuntime runtime(cfg, geographic, renderer);
    runtime.UpdateVisual({}, geographic->LatitudeDeg(), geographic->LongitudeDeg(), true);
    waitUntil([&] {
        std::lock_guard<std::mutex> lock(renderer.mutex);
        return renderer.snapshot && renderer.snapshot->coverageRevision > 0;
    });
    waitUntil([&] { return !runtime.VisualBuilding() && !runtime.RefinementActive(); });

    // Prime only bootstrap images for a repartitioned outer layer, then simulate
    // an outage during its refinement. The complete old view must stay active.
    const TileKey next{center.x + 40, center.y + 30, center.z};
    std::string error;
    auto near = builder.BuildBootstrap(next, 2, error);
    CHECK(near);
    auto expected = builder.BuildOuterCoverage(*near, error);
    CHECK(expected);
    std::size_t missing = 0;
    for (const auto &page : expected->pages)
        if (page.visualOnly && page.imageryZoom < page.targetImageryZoom)
        {
            const int scale = 1 << (page.targetImageryZoom - page.key.z);
            for (int y = page.key.y * scale; y < (page.key.y + 1) * scale; ++y)
                for (int x = page.key.x * scale; x < (page.key.x + 1) * scale; ++x)
                    missing += !fs::exists(store->ImageryPath({x, y, page.targetImageryZoom}));
        }
    CHECK(missing > 0); // Fixture must actually require uncached refinement.
    fs::remove(cfg.imageryUrl);
    auto recenter = [&] {
        runtime.UpdateVisual({}, tileYToLat(next.y + .5, next.z),
            tileXToLon(next.x + .5, next.z), true);
    };
    recenter();
    waitUntil([&] { return runtime.Diagnostics().downloads.failed > 0 || renderer.ActiveGeneration() > 1; });
    waitUntil([&] { return !runtime.VisualBuilding(); });
    CHECK(renderer.ActiveGeneration() == 1);
    writeSource();
    recenter();
    waitUntil([&] { return renderer.ActiveGeneration() == 3; });
    waitUntil([&] { return !runtime.VisualBuilding(); });
    std::shared_ptr<const TerrainSnapshot> published;
    {
        std::lock_guard<std::mutex> lock(renderer.mutex);
        published = renderer.snapshot;
        for (const auto &snapshot : renderer.snapshots)
            if (snapshot->generation > 1)
                for (const auto &page : snapshot->pages)
                    if (page.visualOnly) CHECK(page.imageryZoom >= page.targetImageryZoom);
    }
    // Same geometry, decoded pixels and final quality as the existing builder.
    for (std::size_t i = 0; i < expected->pages.size(); ++i)
    {
        const auto target = builder.TargetZoomForPage(*expected, i);
        if (expected->pages[i].imageryZoom >= target) continue;
        auto update = builder.BuildTextureStage(*expected, {i}, target, error);
        CHECK(update); builder.ApplyTextureUpdate(*expected, *update);
    }
    comparePayload(*published, *expected);
    std::cout << "Replacement imagery: outage retained active terrain; all outer pages ready before activation; exact final payload passed\n";
}

void testProgressiveRuntime(Config cfg,
    const std::shared_ptr<const GeographicTransform> &geographic, const fs::path &path)
{
    cfg.cacheDir = (path / "progressive").string();
    cfg.imageryUrl = (path / "progressive-source.png").string();
    cfg.visualRadiusM = 1000;
    cfg.visualOuterRadiusM = 100000;
    cfg.visualRefineTexture = false;
    cfg.visualPageTextureMaxSize = 256;
    cfg.visualImageryLodTable = {{1000000, 14}};
    cfg.visualOuterMeshCacheMb = 0;
    cfg.visualActivationTimeoutSec = 1;
    CHECK(cv::imwrite(cfg.imageryUrl, cv::Mat(32,32,CV_8UC3,cv::Scalar(35,95,155))));
    normalizeConfig(cfg);
    auto baselineConfig = cfg;
    baselineConfig.cacheDir = (path / "progressive-baseline").string();
    auto store = std::make_shared<TileStore>(baselineConfig, geographic);
    PersistentTerrainBuilder builder(store);
    const auto center = builder.SnappedCenter(geographic->LatitudeDeg(), geographic->LongitudeDeg());
    std::string error;
    auto near = builder.BuildBootstrap(center, 1, error);
    CHECK(near);
    auto complete = builder.BuildOuterCoverage(*near, error);
    CHECK(complete && complete->pages.size() > near->pages.size());
    for (bool optIn : {false, true})
    {
        auto runtimeConfig = cfg;
        runtimeConfig.cacheDir = (path / (optIn ? "progressive-enabled" : "progressive-disabled")).string();
        RecordingRenderer renderer(optIn);
        TerrainRuntime runtime(runtimeConfig, geographic, renderer);
        runtime.UpdateVisual({}, geographic->LatitudeDeg(), geographic->LongitudeDeg(), true);
        waitUntil([&] {
            std::lock_guard<std::mutex> lock(renderer.mutex);
            return renderer.snapshot && renderer.snapshot->pages.size() == complete->pages.size();
        });
        waitUntil([&] { return !runtime.VisualBuilding(); });
        std::size_t initialCount = 0;
        {
            std::lock_guard<std::mutex> lock(renderer.mutex);
            initialCount = renderer.snapshots.size();
            comparePayload(*renderer.snapshot, *complete);
            if (optIn)
            {
                CHECK(initialCount > complete->pages.size());
                CHECK(renderer.snapshots.front()->pages.size() == 1);
                CHECK(renderer.snapshots.front()->pages.front().key == center);
                CHECK(!renderer.snapshots.front()->progressiveComplete);
                CHECK(renderer.snapshot->progressiveComplete);
            }
            else
            {
                CHECK(initialCount == 2); // Existing near publication plus complete outer layer.
                CHECK(renderer.snapshots.front()->pages.size() == near->pages.size());
            }
            std::size_t previousCount = 0;
            std::uint64_t previousRevision = 0;
            for (const auto &published : renderer.snapshots)
            {
                CHECK(published->generation == 1);
                CHECK(published->progressiveInitial == optIn);
                CHECK(published->pages.size() >= previousCount);
                CHECK(published->coverageRevision >= previousRevision);
                previousCount = published->pages.size();
                previousRevision = published->coverageRevision;
                // Later appends cannot mutate any published prefix or buffers.
                for (std::size_t page = 0; page < published->pages.size(); ++page)
                {
                    CHECK(published->pages[page].key == renderer.snapshot->pages[page].key);
                    CHECK(published->pages[page].mesh == renderer.snapshot->pages[page].mesh);
                    CHECK(published->pages[page].texture == renderer.snapshot->pages[page].texture);
                }
            }
        }
        const TileKey nextCenter{center.x + 1, center.y, center.z};
        auto nextNear = builder.BuildBootstrap(nextCenter, 2, error);
        CHECK(nextNear);
        auto nextComplete = builder.BuildOuterCoverage(*nextNear, error);
        CHECK(nextComplete);
        runtime.UpdateVisual({}, tileYToLat(nextCenter.y + .5, nextCenter.z),
            tileXToLon(nextCenter.x + .5, nextCenter.z), true);
        waitUntil([&] { return renderer.ActiveGeneration() == 2; });
        waitUntil([&] { return !runtime.VisualBuilding(); });
        {
            std::lock_guard<std::mutex> lock(renderer.mutex);
            CHECK(renderer.snapshots.size() == initialCount + 1);
            CHECK(!renderer.snapshot->progressiveInitial);
            CHECK(renderer.snapshot->progressiveComplete);
            CHECK(renderer.snapshot->centerTile == nextCenter);
            comparePayload(*renderer.snapshot, *nextComplete);
        }
    }
    std::cout << "Runtime progressive opt-in, immutable prefixes, exact payload and atomic replacements passed\n";
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
        // Immediate teardown used to race the workers' initial condition waits.
        for (int i = 0; i < 100; ++i) { TerrainRuntime idle(cfg, geographic, renderer); }
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
        {
            // Recover a failed level without any vehicle movement/recenter.
            cfg.visualImageryLodTable = {{1000000, 16}};
            cfg.visualPageTextureMaxSize = 1024;
            cfg.retryDelaySec = .2;
            RecordingRenderer recovery;
            TerrainRuntime runtime(cfg, geographic, recovery);
            runtime.UpdateVisual({},52.2297,21.0122,true);
            waitUntil([&] { return runtime.Diagnostics().downloads.failed > 0; });
            for (int y=(center.y-2)*4;y<=(center.y+3)*4;++y)
                for (int x=(center.x-2)*4;x<=(center.x+3)*4;++x)
                {
                    const auto file=store.ImageryPath({x,y,16});
                    fs::create_directories(file.parent_path());
                    CHECK(cv::imwrite(file.string(),cv::Mat(16,16,CV_8UC3,cv::Scalar(90,130,170))));
                }
            waitUntil([&] {
                std::lock_guard<std::mutex> lock(recovery.mutex);
                return recovery.textures && !recovery.textures->pages.empty() &&
                    recovery.textures->pages.front().imageryZoom == 16;
            });
            CHECK(recovery.ActiveGeneration() == 1);
        }
        {
            // Backoff is a condition wait, so stopping never waits for the timer.
            cfg.visualImageryLodTable = {{1000000,17}};
            cfg.visualPageTextureMaxSize = 2048;
            cfg.retryDelaySec = 30;
            RecordingRenderer waiting;
            auto runtime=std::make_unique<TerrainRuntime>(cfg,geographic,waiting);
            runtime->UpdateVisual({},52.2297,21.0122,true);
            waitUntil([&] { return runtime->Diagnostics().downloads.failed > 0; });
            const auto begin=std::chrono::steady_clock::now();
            runtime.reset();
            CHECK(std::chrono::steady_clock::now()-begin < std::chrono::seconds(1));
        }
        testProgressiveRuntime(cfg, geographic, path);
        testReplacementImagery(cfg, geographic, path);
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
