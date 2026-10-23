#include "Ogre2ResourceCleanup.hh"
#include "dynamic_terrain/adapters/gzsim/GzTerrainRenderer.hh"
#include "dynamic_terrain/core/TerrainTypes.hh"

#include <gz/common/Image.hh>
#include <gz/common/Mesh.hh>
#include <gz/common/SubMesh.hh>
#include <gz/rendering/Material.hh>
#include <gz/rendering/Image.hh>
#include <gz/rendering/Mesh.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
#include <gz/rendering/ogre2/Ogre2RenderEngine.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/rendering/Events.hh>

#include <OgreRenderSystem.h>
#include <OgreRoot.h>
#include <OgreMeshManager.h>
#include <OgreMesh.h>
#include <OgreSubMesh.h>
#include <OgreTextureGpu.h>
#include <OgreTextureGpuManager.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <fstream>
#include <algorithm>
#include <unistd.h>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{
double soakSeconds = 0.0;
std::string reportPath;
class TestFailure : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};

void check(bool condition, const char *expression, const char *file, int line)
{
    if (condition)
        return;
    std::ostringstream out;
    out << file << ':' << line << ": CHECK failed: " << expression;
    throw TestFailure(out.str());
}

#define CHECK(expression) \
    check(static_cast<bool>(expression), #expression, __FILE__, __LINE__)

void destroyMaterial(const gz::rendering::ScenePtr &scene,
                     gz::rendering::MaterialPtr &material)
{
    if (scene && material)
    {
        material->ClearTexture();
        scene->DestroyMaterial(material);
    }
    material.reset();
}

struct RenderCleanup
{
    ~RenderCleanup()
    {
        try
        {
            if (visual)
                visual->RemoveGeometries();
            if (geometry)
                geometry->Destroy();
            geometry.reset();
            if (scene && visual)
                scene->DestroyVisual(visual, true);
            visual.reset();
            if (scene && !meshName.empty())
                dynamic_terrain::destroyUnreferencedOgre2Mesh(
                    scene, meshName);
        }
        catch (...)
        {
            geometry.reset();
            visual.reset();
        }

        try
        {
            destroyMaterial(scene, materialA);
            destroyMaterial(scene, materialB);
        }
        catch (...)
        {
            materialA.reset();
            materialB.reset();
        }

        if (textureManager && !textureName.empty())
        {
            try
            {
                if (auto *texture =
                        textureManager->findTextureNoThrow(textureName))
                    textureManager->destroyTexture(texture);
            }
            catch (...)
            {
            }
        }

        if (engine && scene)
        {
            try
            {
                engine->DestroyScene(scene);
            }
            catch (...)
            {
            }
        }
        scene.reset();
    }

    gz::rendering::RenderEngine *engine{nullptr};
    gz::rendering::ScenePtr scene;
    gz::rendering::MaterialPtr materialA;
    gz::rendering::MaterialPtr materialB;
    gz::rendering::MeshPtr geometry;
    gz::rendering::VisualPtr visual;
    Ogre::TextureGpuManager *textureManager{nullptr};
    std::string textureName;
    std::string meshName;
};

std::shared_ptr<const dynamic_terrain::ImageData> solidImage(std::uint8_t value)
{
    constexpr unsigned int side = 64;
    auto image = std::make_shared<dynamic_terrain::ImageData>();
    image->width = side;
    image->height = side;
    image->rgb.assign(side * side * 3u, value);
    CHECK(image->Valid());
    return image;
}

dynamic_terrain::TerrainPage makeTerrainPage(
    const std::string &prefix, const dynamic_terrain::TileKey &key,
    std::size_t index, double minimumX, double maximumX,
    std::uint8_t imageValue, std::uint64_t generation = 1u)
{
    dynamic_terrain::TerrainPage page;
    page.key = key;
    page.index = index;
    page.submeshName = "terrain_page_z" + std::to_string(key.z) +
        "_x" + std::to_string(key.x) + "_y" + std::to_string(key.y);
    page.textureSize = 64;
    page.texture = solidImage(imageValue);
    page.textureName = prefix + "_page_z" + std::to_string(key.z) +
        "_x" + std::to_string(key.x) + "_y" + std::to_string(key.y) +
        "_q14_s64";
    auto mesh = std::make_shared<dynamic_terrain::MeshData>();
    mesh->name = prefix + "_mesh_g" + std::to_string(generation) +
        "_z" + std::to_string(key.z) + "_x" + std::to_string(key.x) +
        "_y" + std::to_string(key.y);
    mesh->submeshName = page.submeshName;
    mesh->positions = {{minimumX, -4.0, 0.0}, {maximumX, -4.0, 0.0},
                       {minimumX, 4.0, 0.0}, {maximumX, 4.0, 0.0}};
    mesh->normals.assign(4u, {0.0, 0.0, 1.0});
    mesh->texCoords = {{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}, {1.0, 1.0}};
    mesh->indices = {0u, 2u, 1u, 1u, 2u, 3u};
    page.mesh = std::move(mesh);
    return page;
}

void testRendererRecenters(const gz::rendering::ScenePtr &scene,
                           std::int64_t nonce)
{
    dynamic_terrain::Config config;
    config.visualWarmupFrames = 1;
    config.visualFrustumEviction = false;
    config.diagnostics = false;

    const std::string prefix =
        "dynamic_terrain_recenter_" + std::to_string(nonce);
    const dynamic_terrain::TileKey key{100, 100, 14};
    std::string previousMesh;
    std::string sharedTexture;
    std::string previousImportedMaterial;
    std::weak_ptr<const dynamic_terrain::ImageData> previousCpuImage;
    std::weak_ptr<const dynamic_terrain::MeshData> previousCpuMesh;

    gz::sim::EventManager events;
    {
        dynamic_terrain::PersistentTerrainRenderer renderer(config, events);
        for (std::uint64_t generation = 1u; generation <= 20u; ++generation)
        {
            auto snapshot =
                std::make_shared<dynamic_terrain::TerrainSnapshot>();
            snapshot->generation = generation;
            snapshot->resourcePrefix = prefix;
            snapshot->centerTile = key;
            auto page = makeTerrainPage(prefix, key, 0u, 10.0, 20.0, 60u,
                                        generation);
            sharedTexture = page.textureName;
            const std::string nextMesh = page.mesh->name;
            snapshot->estimatedTextureBytes =
                dynamic_terrain::mipmappedRgbaBytes(
                    page.textureSize, page.textureSize);
            snapshot->pages.push_back(std::move(page));

            renderer.QueueSnapshot(snapshot);
            events.Emit<gz::sim::events::PreRender>();
            CHECK(dynamic_terrain::ogre2MeshExists(scene, nextMesh));
            const auto imported = Ogre::v1::MeshManager::getSingleton().getByName(
                nextMesh + "::::ORIGINAL");
            CHECK(imported);
            const auto importedMaterial = imported->getSubMesh(0)->getMaterialName();
            CHECK(scene->Material(importedMaterial));
            CHECK(dynamic_terrain::ogre2TextureExists(scene, sharedTexture));
            if (!previousMesh.empty())
                CHECK(dynamic_terrain::ogre2MeshExists(scene, previousMesh));

            events.Emit<gz::sim::events::PreRender>();
            events.Emit<gz::sim::events::PostRender>();
            CHECK(renderer.ActiveGeneration() == generation);
            CHECK(dynamic_terrain::ogre2MeshExists(scene, nextMesh));
            CHECK(dynamic_terrain::ogre2TextureExists(scene, sharedTexture));
            if (!previousMesh.empty())
            {
                CHECK(!dynamic_terrain::ogre2MeshExists(scene, previousMesh));
                CHECK(previousCpuImage.expired());
                CHECK(previousCpuMesh.expired());
                CHECK(!scene->Material(previousImportedMaterial));
            }
            previousMesh = nextMesh;
            previousImportedMaterial = importedMaterial;
            previousCpuImage = snapshot->pages[0].texture;
            previousCpuMesh = snapshot->pages[0].mesh;
        }

        events.Emit<gz::sim::events::RenderTeardown>();
        CHECK(!dynamic_terrain::ogre2MeshExists(scene, previousMesh));
        CHECK(!dynamic_terrain::ogre2TextureExists(scene, sharedTexture));
        CHECK(previousCpuImage.expired());
        CHECK(previousCpuMesh.expired());
        CHECK(!scene->Material(previousImportedMaterial));
    }
}

#ifndef TERRAIN_BENCHMARK_SKIP_UPLOAD_BUDGET
void testRendererUploadBudget(const gz::rendering::ScenePtr &scene,
                              std::int64_t nonce)
{
    dynamic_terrain::Config config;
    config.visualWarmupFrames = 1;
    config.visualFrustumEviction = false;
    config.visualUploadPagesPerFrame = 1;
    config.visualUploadBudgetMs = 1000;
    config.diagnostics = false;
    gz::sim::EventManager events;
    dynamic_terrain::PersistentTerrainRenderer renderer(config, events);
    auto snapshot = std::make_shared<dynamic_terrain::TerrainSnapshot>();
    snapshot->generation = 1;
    snapshot->resourcePrefix = "budget_" + std::to_string(nonce);
    snapshot->centerTile = {100, 100, 14};
    for (unsigned int page = 0; page < 3; ++page)
        snapshot->pages.push_back(makeTerrainPage(snapshot->resourcePrefix,
            {100 + static_cast<int>(page), 100, 14}, page, page * 10, page * 10 + 8, 100));
    renderer.QueueSnapshot(snapshot);
    for (std::size_t frame = 0; frame < 3; ++frame)
    {
        events.Emit<gz::sim::events::PreRender>();
        CHECK(!renderer.HasActiveTerrain());
        for (std::size_t page = 0; page < 3; ++page)
            CHECK(dynamic_terrain::ogre2MeshExists(scene, snapshot->pages[page].mesh->name) == (page <= frame));
    }
    events.Emit<gz::sim::events::PreRender>();
    CHECK(renderer.ActiveGeneration() == 1);
    dynamic_terrain::TextureUpdate refinement;
    refinement.generation = 1;
    for (const auto &page : snapshot->pages)
    {
        dynamic_terrain::TexturePageUpdate update;
        update.pageIndex = page.index;
        update.pageKey = page.key;
        update.submeshName = page.submeshName;
        update.imageryZoom = 15;
        update.textureSize = 64;
        update.texture = solidImage(150);
        update.textureName = page.textureName + "_refined";
        refinement.pages.push_back(std::move(update));
    }
    refinement.changedPageCount = refinement.pages.size();
    renderer.QueueTexture(refinement);
    events.Emit<gz::sim::events::PreRender>();
    CHECK(dynamic_terrain::ogre2TextureExists(scene, refinement.pages[0].textureName));
    CHECK(!dynamic_terrain::ogre2TextureExists(scene, refinement.pages[1].textureName));
    CHECK(!dynamic_terrain::ogre2TextureExists(scene, refinement.pages[2].textureName));
    CHECK(dynamic_terrain::ogre2TextureExists(scene, snapshot->pages[1].textureName));
    CHECK(dynamic_terrain::ogre2TextureExists(scene, snapshot->pages[2].textureName));
    auto finer = refinement;
    finer.pages = {refinement.pages[2]};
    finer.pages[0].imageryZoom = 16;
    finer.pages[0].textureName += "_finer";
    renderer.QueueTexture(finer);
    events.Emit<gz::sim::events::PreRender>();
    CHECK(dynamic_terrain::ogre2TextureExists(scene, refinement.pages[1].textureName));
    CHECK(!dynamic_terrain::ogre2TextureExists(scene, finer.pages[0].textureName));
    events.Emit<gz::sim::events::PreRender>();
    CHECK(dynamic_terrain::ogre2TextureExists(scene, finer.pages[0].textureName));
    CHECK(!dynamic_terrain::ogre2TextureExists(scene, refinement.pages[2].textureName));
    CHECK(!dynamic_terrain::ogre2TextureExists(scene, snapshot->pages[2].textureName));
    auto expanded = std::make_shared<dynamic_terrain::TerrainSnapshot>(*snapshot);
    expanded->coverageRevision = 1;
    expanded->pages.push_back(makeTerrainPage(snapshot->resourcePrefix,
        {103, 100, 14}, 3, 30, 38, 100));
    renderer.QueueSnapshot(expanded);
    for (int frame = 0; frame < 4; ++frame)
    {
        events.Emit<gz::sim::events::PreRender>();
        CHECK(renderer.ActiveCoverageRevision() == 0);
        CHECK(dynamic_terrain::ogre2TextureExists(scene, finer.pages[0].textureName));
    }
    events.Emit<gz::sim::events::PreRender>();
    events.Emit<gz::sim::events::PostRender>();
    CHECK(renderer.ActiveGeneration() == 1);
    CHECK(renderer.ActiveCoverageRevision() == 1);
    CHECK(dynamic_terrain::ogre2TextureExists(scene, finer.pages[0].textureName));
    CHECK(!dynamic_terrain::ogre2TextureExists(scene, snapshot->pages[2].textureName));
    renderer.QueueSnapshot(snapshot); // An older near-only revision is stale.
    events.Emit<gz::sim::events::PreRender>();
    CHECK(renderer.ActiveCoverageRevision() == 1);
    events.Emit<gz::sim::events::RenderTeardown>();
    for (const auto &page : snapshot->pages)
        CHECK(!dynamic_terrain::ogre2MeshExists(scene, page.mesh->name));
}
#endif

void testProgressiveInitial(const gz::rendering::ScenePtr &scene, std::int64_t nonce)
{
    const auto background = scene->BackgroundColor(), ambient = scene->AmbientLight();
    scene->SetBackgroundColor(gz::math::Color(0, 0, 1));
    scene->SetAmbientLight(gz::math::Color(1, 1, 1));
    const std::string prefix = "initial_stream_" + std::to_string(nonce);
    auto camera = scene->CreateCamera(prefix + "_camera");
    scene->RootVisual()->AddChild(camera);
    camera->SetImageWidth(80); camera->SetImageHeight(80); camera->SetAspectRatio(1);
    camera->SetHFOV(gz::math::Angle(1)); camera->SetNearClipPlane(.1); camera->SetFarClipPlane(1000);
    camera->SetWorldPose(gz::math::Pose3d(15, 0, 5, 0, GZ_PI / 2, 0));
    auto image = camera->CreateImage();
    dynamic_terrain::Config config;
    config.visualUploadPagesPerFrame = 1;
    config.visualUploadBudgetMs = 1000;
    config.visualWarmupFrames = 1;
    config.visualFrustumEviction = false;
    config.visualLightingEnabled = false;
    config.diagnostics = false;
    gz::sim::EventManager events;
    dynamic_terrain::PersistentTerrainRenderer renderer(config, events);
    CHECK(renderer.SupportsProgressiveInitial());
    const auto initialVisualCount = scene->VisualCount();
    std::vector<dynamic_terrain::TerrainPage> pages;
    for (int index = 0; index < 8; ++index)
    {
        auto page = makeTerrainPage(prefix, {100 + index, 100, 14}, index,
            10 + index * 10, 20 + index * 10, 100);
        auto mesh = std::make_shared<dynamic_terrain::MeshData>(*page.mesh);
        mesh->indices = {0, 1, 2, 1, 3, 2}; page.mesh = std::move(mesh);
        pages.push_back(std::move(page));
    }
    auto cumulative = [&](std::size_t count, std::uint64_t revision, bool complete = false)
    {
        auto snapshot = std::make_shared<dynamic_terrain::TerrainSnapshot>();
        snapshot->generation = 1; snapshot->coverageRevision = revision;
        snapshot->progressiveInitial = true; snapshot->progressiveComplete = complete;
        snapshot->resourcePrefix = prefix; snapshot->centerTile = pages.front().key;
        snapshot->pages.assign(pages.begin(), pages.begin() + count);
        return snapshot;
    };
    auto tick = [&](bool expectGround)
    {
        events.Emit<gz::sim::events::PreRender>();
        scene->PreRender(); camera->PreRender(); camera->Render(); camera->PostRender();
        scene->PostRender(); camera->Copy(image); events.Emit<gz::sim::events::PostRender>();
        const auto *pixels = image.Data<unsigned char>();
        for (std::size_t pixel = 0; pixel < image.Width() * image.Height(); ++pixel)
        {
            if (expectGround)
            {
                CHECK(pixels[pixel * 3] > 20);
                CHECK(std::abs(int(pixels[pixel * 3]) - pixels[pixel * 3 + 2]) <= 2);
            }
            else CHECK(pixels[pixel * 3 + 2] > pixels[pixel * 3] + 20);
        }
    };
    renderer.QueueSnapshot(cumulative(4, 1));
    tick(false); CHECK(!renderer.HasActiveTerrain());
    tick(true); CHECK(renderer.HasActiveTerrain()); CHECK(renderer.ActiveCoverageRevision() == 0);
    CHECK(!dynamic_terrain::ogre2MeshExists(scene, pages[2].mesh->name));
    renderer.QueueSnapshot(cumulative(5, 2)); // Supersede during upload without restarting it.
    for (int i = 0; i < 4; ++i) tick(true);
    CHECK(renderer.ActiveCoverageRevision() == 2);
    CHECK(scene->VisualCount() == initialVisualCount + 6); // One root and five pages.
    renderer.QueueSnapshot(cumulative(6, 3));
    renderer.QueueSnapshot(cumulative(7, 4)); // Latest-only queue must retain both appended pages.
    for (int i = 0; i < 3; ++i) tick(true);
    CHECK(renderer.ActiveCoverageRevision() == 4);
    CHECK(scene->VisualCount() == initialVisualCount + 8);
    auto failed = cumulative(8, 5); failed->pages.back().texture.reset();
    renderer.QueueSnapshot(failed); tick(true);
    CHECK(renderer.ActiveCoverageRevision() == 4);
    CHECK(scene->VisualCount() == initialVisualCount + 8);
    renderer.QueueSnapshot(cumulative(8, 5)); tick(true); tick(true);
    CHECK(renderer.ActiveCoverageRevision() == 5);
    auto invalid = cumulative(8, 6); invalid->pages.erase(invalid->pages.begin());
    renderer.QueueSnapshot(invalid); tick(true);
    CHECK(renderer.ActiveCoverageRevision() == 5);
    CHECK(scene->VisualCount() == initialVisualCount + 9);
    dynamic_terrain::TextureUpdate refinement;
    refinement.generation = 1;
    dynamic_terrain::TexturePageUpdate finer;
    finer.pageKey = pages.front().key; finer.submeshName = pages.front().submeshName;
    finer.imageryZoom = 19; finer.textureSize = 64; finer.texture = solidImage(150);
    finer.textureName = pages.front().textureName + "_fine";
    refinement.pages.push_back(finer); renderer.QueueTexture(refinement); tick(true);
    renderer.QueueSnapshot(cumulative(8, 7, true)); tick(true);
    CHECK(renderer.ActiveCoverageRevision() == 7);
    CHECK(dynamic_terrain::ogre2TextureExists(scene, finer.textureName));
    CHECK(scene->VisualCount() == initialVisualCount + 9);
    const std::vector<unsigned char> sharpPixels(image.Data<unsigned char>(),
        image.Data<unsigned char>() + image.Width() * image.Height() * 3);
    auto checkSharp = [&] {
        CHECK(std::equal(sharpPixels.begin(), sharpPixels.end(), image.Data<unsigned char>()));
    };
    auto replacement = cumulative(3, 0, true);
    replacement->generation = 2; replacement->progressiveInitial = false;
    for (auto &page : replacement->pages)
    {
        auto mesh = std::make_shared<dynamic_terrain::MeshData>(*page.mesh);
        mesh->name += "_replacement"; page.mesh = std::move(mesh);
        page.textureName += "_replacement";
    }
    renderer.QueueSnapshot(replacement);
    for (int i = 0; i < 3; ++i)
    { tick(true); CHECK(renderer.ActiveGeneration() == 1); checkSharp(); }
    tick(true); CHECK(renderer.ActiveGeneration() == 2); checkSharp();
    // The delayed refinement must never cause an intervening blurry frame.
    for (int i = 0; i < 30; ++i) { tick(true); checkSharp(); }
    CHECK(scene->VisualCount() == initialVisualCount + 4);
    auto third = std::make_shared<dynamic_terrain::TerrainSnapshot>(*replacement);
    third->generation = 3;
    for (auto &page : third->pages)
    {
        auto mesh = std::make_shared<dynamic_terrain::MeshData>(*page.mesh);
        mesh->name += "_third"; page.mesh = std::move(mesh);
    }
    renderer.QueueSnapshot(third); tick(true); checkSharp();
    // Arrives after the first staging page has already copied the resident image.
    refinement.generation = 2;
    refinement.pages[0].imageryZoom = 20;
    refinement.pages[0].texture = solidImage(180);
    refinement.pages[0].textureName += "_late";
    renderer.QueueTexture(refinement);
    tick(true); checkSharp(); tick(true); checkSharp();
    tick(true); CHECK(renderer.ActiveGeneration() == 3);
    CHECK(image.Data<unsigned char>()[0] > sharpPixels[0]);
    const std::vector<unsigned char> latePixels(image.Data<unsigned char>(),
        image.Data<unsigned char>() + sharpPixels.size());
    for (int i = 0; i < 30; ++i)
    {
        tick(true);
        CHECK(std::equal(latePixels.begin(), latePixels.end(), image.Data<unsigned char>()));
    }
    renderer.QueueSnapshot(cumulative(8, 99)); tick(true);
    CHECK(renderer.ActiveGeneration() == 3);
    events.Emit<gz::sim::events::RenderTeardown>();
    CHECK(scene->VisualCount() == initialVisualCount);
    scene->DestroySensor(camera, true);
    scene->SetBackgroundColor(background); scene->SetAmbientLight(ambient);
    std::cout << "progressive-initial early_pixels=true cumulative_no_restart=true bounded_uploads=true "
                 "failed_append_retained=true replacement_atomic=true sharp_recenter_pixels=true\n";
}

void testConcurrentInitialSnapshots(const gz::rendering::ScenePtr &scene, std::int64_t nonce)
{
    const std::string prefix = "initial_queue_race_" + std::to_string(nonce);
    dynamic_terrain::Config config;
    config.visualUploadPagesPerFrame = 1; config.visualUploadBudgetMs = 1000;
    config.visualWarmupFrames = 0; config.visualFrustumEviction = false; config.diagnostics = false;
    gz::sim::EventManager events;
    dynamic_terrain::PersistentTerrainRenderer renderer(config, events);
    auto full = std::make_shared<dynamic_terrain::TerrainSnapshot>();
    full->generation = 1; full->coverageRevision = 1; full->resourcePrefix = prefix;
    full->progressiveInitial = true; full->progressiveComplete = false;
    for (int i = 0; i < 128; ++i)
        full->pages.push_back(makeTerrainPage(prefix, {100 + i, 100, 14}, i, i * 10, i * 10 + 8, 100));
    auto first = std::make_shared<dynamic_terrain::TerrainSnapshot>(*full);
    first->pages.resize(1);
    renderer.QueueSnapshot(first); events.Emit<gz::sim::events::PreRender>();
    const auto firstVisual = scene->VisualByName(prefix + "_visual_g1_p0_r1");
    CHECK(firstVisual && scene->HasVisual(firstVisual));
    const auto baseVisualCount = scene->VisualCount();
    std::atomic<bool> stop{false};
    std::atomic<std::uint64_t> revision{1};
    std::thread publisher([&]
    {
        while (!stop.load(std::memory_order_relaxed))
        {
            auto next = std::make_shared<dynamic_terrain::TerrainSnapshot>(*full);
            next->coverageRevision = ++revision;
            renderer.QueueSnapshot(std::move(next));
            std::this_thread::yield();
        }
    });
    struct Join
    {
        std::atomic<bool> &stop; std::thread &thread;
        ~Join() { stop.store(true); if (thread.joinable()) thread.join(); }
    } join{stop, publisher};
    while (revision.load(std::memory_order_acquire) < 10) std::this_thread::yield();
    for (int tick = 0; tick < 48; ++tick)
    {
        events.Emit<gz::sim::events::PreRender>(); events.Emit<gz::sim::events::PostRender>();
        CHECK(scene->HasVisual(firstVisual));
        CHECK(scene->VisualCount() <= baseVisualCount + tick + 1);
    }
    stop.store(true); publisher.join();
    auto completed = std::make_shared<dynamic_terrain::TerrainSnapshot>(*full);
    completed->coverageRevision = revision + 1; completed->progressiveComplete = true;
    renderer.QueueSnapshot(completed);
    for (int tick = 0; renderer.ActiveCoverageRevision() != completed->coverageRevision; ++tick)
    {
        CHECK(tick < 130);
        events.Emit<gz::sim::events::PreRender>(); events.Emit<gz::sim::events::PostRender>();
        CHECK(scene->HasVisual(firstVisual)); // No hidden replacement may steal a cumulative target.
        CHECK(scene->VisualCount() <= baseVisualCount + 127);
    }
    CHECK(scene->VisualCount() == baseVisualCount + 127);
    events.Emit<gz::sim::events::RenderTeardown>();
    CHECK(!scene->HasVisual(firstVisual));
    std::cout << "concurrent-initial publications=" << revision << " pages=128 initial_visual_preserved=true\n";
}

void testMountainCameraFrames(const gz::rendering::ScenePtr &scene,
                              std::int64_t nonce)
{
    const auto background = scene->BackgroundColor();
    const auto ambient = scene->AmbientLight();
    scene->SetAmbientLight(gz::math::Color(1, 1, 1));
    scene->SetBackgroundColor(gz::math::Color(0, 0, 1));
    auto camera = scene->CreateCamera("mountain_camera_" + std::to_string(nonce));
    CHECK(camera);
    scene->RootVisual()->AddChild(camera);
    camera->SetImageWidth(160);
    camera->SetImageHeight(90);
    camera->SetAspectRatio(16.0 / 9.0);
    camera->SetHFOV(gz::math::Angle(1.0));
    camera->SetNearClipPlane(0.05);
    camera->SetFarClipPlane(6000);
    camera->SetWorldPose(gz::math::Pose3d(0, 0, 1800, 0, GZ_PI / 2, 0));
    auto image = camera->CreateImage();
    dynamic_terrain::Config config;
    config.visualWarmupFrames = 1;
    config.visualFrustumEviction = true;
    config.cameraNames = {camera->Name()};
    config.visualLightingEnabled = false;
    config.diagnostics = false;
    gz::sim::EventManager events;
    dynamic_terrain::PersistentTerrainRenderer renderer(config, events);
    const std::string prefix = "mountain_pixels_" + std::to_string(nonce);
    const auto started = std::chrono::steady_clock::now();
    std::array<std::uint64_t, 10000> histogram{};
    std::uint64_t frames = 0, generations = 0, maxRssBytes = 0, maxTextureEntries = 0;
    auto *textureManager = gz::rendering::Ogre2RenderEngine::Instance()->OgreRoot()
        ->getRenderSystem()->getTextureGpuManager();
    for (std::uint64_t generation = 1; generation <= 20 ||
         std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count() < soakSeconds;
         ++generation)
    {
        auto snapshot = std::make_shared<dynamic_terrain::TerrainSnapshot>();
        snapshot->generation = generation;
        snapshot->resourcePrefix = prefix;
        snapshot->centerTile = {100, 100, 14};
        for (int side = 0; side < 2; ++side)
        {
            auto page = makeTerrainPage(prefix, {100 + side, 100, 14}, side,
                                        0, 1, 100);
            auto mesh = std::make_shared<dynamic_terrain::MeshData>();
            mesh->name = prefix + "_g" + std::to_string(generation) +
                "_p" + std::to_string(side);
            mesh->submeshName = page.submeshName;
            constexpr int cells = 32;
            for (int y = 0; y <= cells; ++y)
                for (int x = 0; x <= cells; ++x)
                {
                    const double wx = (side - 1) * 3000.0 + 3000.0 * x / cells;
                    const double wy = -3000.0 + 6000.0 * y / cells;
                    const double wz = 400 + 200 * std::sin(wx / 300) *
                                                std::cos(wy / 400);
                    mesh->positions.emplace_back(wx, wy, wz);
                    mesh->normals.emplace_back(0, 0, 1);
                    mesh->texCoords.emplace_back(double(x) / cells, double(y) / cells);
                }
            for (int y = 0; y < cells; ++y)
                for (int x = 0; x < cells; ++x)
                {
                    const unsigned int a = y * (cells + 1) + x;
                    const unsigned int b = a + 1, c = a + cells + 1, d = c + 1;
                    for (auto i : {a, b, c, b, d, c})
                        mesh->indices.push_back(i);
                }
            page.mesh = std::move(mesh);
            snapshot->pages.push_back(std::move(page));
        }
        // Repeated route changes keep both pages in view and exercise texture,
        // mesh, residency and generation retirement through real camera renders.
        camera->SetWorldPose(gz::math::Pose3d(180 * std::sin(generation * 0.07),
            180 * std::cos(generation * 0.07), 1800, 0, GZ_PI / 2, 0));
        renderer.QueueSnapshot(snapshot);
        int frame = 0;
        do
        {
            const auto frameStart = std::chrono::steady_clock::now();
            events.Emit<gz::sim::events::PreRender>();
            scene->PreRender(); camera->PreRender(); camera->Render(); camera->PostRender();
            scene->PostRender(); camera->Copy(image);
            events.Emit<gz::sim::events::PostRender>();
            const double ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - frameStart).count();
            ++histogram[std::min<std::size_t>(histogram.size() - 1, static_cast<std::size_t>(ms * 10))];
            ++frames; ++frame;
            CHECK(frame < 100);
            if (!renderer.HasActiveTerrain()) continue;
            const auto *pixels = image.Data<unsigned char>();
            CHECK(image.Depth() == 3);
            for (unsigned int i = 0; i < image.Width() * image.Height(); ++i)
            {
                CHECK(pixels[i * 3] > 20);
                CHECK(std::abs(int(pixels[i * 3]) - pixels[i * 3 + 2]) <= 2);
            }
        } while (renderer.ActiveGeneration() != generation || frame < 2);
        generations = generation;
#ifndef TERRAIN_BENCHMARK_SKIP_UPLOAD_BUDGET
        if (generation == 5)
        {
            auto invalid = std::make_shared<dynamic_terrain::TerrainSnapshot>(*snapshot);
            invalid->generation = generation + 100000000;
            invalid->pages[1].texture.reset();
            renderer.QueueSnapshot(invalid);
            for (int i = 0; i < 4; ++i) events.Emit<gz::sim::events::PreRender>();
            CHECK(renderer.ActiveGeneration() == generation);
            CHECK(renderer.HasActiveTerrain());
        }
#endif
        std::ifstream memory("/proc/self/statm");
        std::uint64_t totalPages = 0, residentPages = 0; memory >> totalPages >> residentPages;
        maxRssBytes = std::max(maxRssBytes, residentPages * static_cast<std::uint64_t>(sysconf(_SC_PAGESIZE)));
        maxTextureEntries = std::max(maxTextureEntries, static_cast<std::uint64_t>(textureManager->getEntries().size()));
        CHECK(textureManager->getEntries().size() < 100);
        if (generation % 500 == 0)
            std::cout << "terrain-soak generations=" << generation << " frames=" << frames
                      << " rss_bytes=" << residentPages * static_cast<std::uint64_t>(sysconf(_SC_PAGESIZE))
                      << " texture_entries=" << textureManager->getEntries().size() << std::endl;
    }
    auto percentile = [&](double fraction) {
        std::uint64_t count = 0;
        for (std::size_t i = 0; i < histogram.size(); ++i)
        { count += histogram[i]; if (count >= frames * fraction) return static_cast<double>(i + 1) / 10; }
        return 1000.0;
    };
    std::ostringstream report;
    report << "{\"seconds\":" << std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count()
           << ",\"generations\":" << generations << ",\"frames\":" << frames
           << ",\"frame_p50_ms\":" << percentile(0.50) << ",\"frame_p95_ms\":" << percentile(0.95)
           << ",\"frame_p99_ms\":" << percentile(0.99) << ",\"max_rss_bytes\":" << maxRssBytes
           << ",\"max_texture_entries\":" << maxTextureEntries << ",\"pixel_checks_passed\":true}";
    std::cout << report.str() << std::endl;
    if (!reportPath.empty()) { std::ofstream output(reportPath); output << report.str() << '\n'; }
    events.Emit<gz::sim::events::RenderTeardown>();
    scene->DestroySensor(camera, true);
    scene->SetBackgroundColor(background);
    scene->SetAmbientLight(ambient);
}

void testSharedMeshRetirement(const gz::rendering::ScenePtr &scene,
                              std::int64_t nonce)
{
    const std::string prefix = "shared_meshes_" + std::to_string(nonce);
    std::vector<dynamic_terrain::TerrainPage> pages;
    for (int i = 0; i < 12; ++i)
    {
        auto page = makeTerrainPage(prefix, {100 + i, 100, 14}, i,
            10 + i * 10, 20 + i * 10, 100);
        auto mesh = std::make_shared<dynamic_terrain::MeshData>(*page.mesh);
        mesh->indices = {0u, 1u, 2u, 1u, 3u, 2u};
        page.mesh = std::move(mesh);
        pages.push_back(std::move(page));
    }
    const auto background = scene->BackgroundColor();
    const auto ambient = scene->AmbientLight();
    scene->SetAmbientLight(gz::math::Color(1, 1, 1));
    scene->SetBackgroundColor(gz::math::Color(0, 0, 1));
    auto camera = scene->CreateCamera(prefix + "_camera");
    CHECK(camera);
    scene->RootVisual()->AddChild(camera);
    camera->SetImageWidth(80); camera->SetImageHeight(80); camera->SetAspectRatio(1);
    camera->SetHFOV(gz::math::Angle(1)); camera->SetNearClipPlane(0.1); camera->SetFarClipPlane(1000);
    camera->SetWorldPose(gz::math::Pose3d(15, 0, 5, 0, GZ_PI / 2, 0));
    auto image = camera->CreateImage();
    dynamic_terrain::Config config;
    config.visualFrustumEviction = false;
    config.visualLightingEnabled = false;
    config.diagnostics = false;
    gz::sim::EventManager events;
    dynamic_terrain::PersistentTerrainRenderer renderer(config, events);
    std::size_t residentTextures = 0;
    unsigned int residentVisuals = 0;
    auto *textureManager = gz::rendering::Ogre2RenderEngine::Instance()->OgreRoot()
        ->getRenderSystem()->getTextureGpuManager();
    for (std::uint64_t generation = 1; generation <= 30; ++generation)
    {
        auto snapshot = std::make_shared<dynamic_terrain::TerrainSnapshot>();
        snapshot->generation = generation;
        snapshot->coverageRevision = 1;
        snapshot->resourcePrefix = prefix;
        snapshot->pages = pages; // Immutable cached meshes and names stay shared.
        renderer.QueueSnapshot(snapshot);
        for (int tick = 0; renderer.ActiveGeneration() != generation; ++tick)
        {
            CHECK(tick < 30);
            events.Emit<gz::sim::events::PreRender>();
            scene->PreRender(); camera->PreRender(); camera->Render(); camera->PostRender();
            scene->PostRender(); camera->Copy(image);
            events.Emit<gz::sim::events::PostRender>();
            if (renderer.HasActiveTerrain())
            {
                const auto *pixels = image.Data<unsigned char>();
                for (std::size_t pixel = 0; pixel < image.Width() * image.Height(); ++pixel)
                {
                    if (pixels[pixel * 3] <= 20)
                        std::cerr << "shared-mesh unexpected pixel generation=" << generation
                                  << " tick=" << tick << " pixel=" << pixel << " rgb="
                                  << int(pixels[pixel * 3]) << ',' << int(pixels[pixel * 3 + 1])
                                  << ',' << int(pixels[pixel * 3 + 2]) << '\n';
                    CHECK(pixels[pixel * 3] > 20);
                    CHECK(std::abs(int(pixels[pixel * 3]) - pixels[pixel * 3 + 2]) <= 2);
                }
            }
        }
        for (const auto &page : pages)
        {
            CHECK(dynamic_terrain::ogre2MeshExists(scene, page.mesh->name));
            CHECK(dynamic_terrain::ogre2TextureExists(scene, page.textureName));
        }
        if (generation == 1)
        {
            residentTextures = textureManager->getEntries().size();
            residentVisuals = scene->VisualCount();
        }
        CHECK(textureManager->getEntries().size() == residentTextures);
        CHECK(scene->VisualCount() == residentVisuals);
    }
    events.Emit<gz::sim::events::RenderTeardown>();
    for (const auto &page : pages)
    {
        CHECK(!dynamic_terrain::ogre2MeshExists(scene, page.mesh->name));
        CHECK(!dynamic_terrain::ogre2TextureExists(scene, page.textureName));
    }
    scene->DestroySensor(camera, true);
    scene->SetBackgroundColor(background);
    scene->SetAmbientLight(ambient);
    std::cout << "shared-mesh-retirement generations=30 pages=12 stable_texture_and_visual_counts=true\n";
}

void testCameraResidency(const gz::rendering::ScenePtr &scene,
                         std::int64_t nonce)
{
    const std::string cameraLeaf =
        "dynamic_terrain_residency_camera_" + std::to_string(nonce);
    const std::string scopedCameraName =
        "vehicle::link::" + cameraLeaf;
    auto similarlyNamedCamera = scene->CreateCamera(cameraLeaf + "_aux");
    CHECK(similarlyNamedCamera);

    dynamic_terrain::Config config;
    config.visualWarmupFrames = 1;
    config.visualFrustumEviction = true;
    config.visualOffscreenFrames = 2;
    config.visualTextureGuardM = 0.0;
    config.cameraNames = {cameraLeaf};
    config.diagnostics = false;

    auto snapshot = std::make_shared<dynamic_terrain::TerrainSnapshot>();
    snapshot->generation = 1u;
    snapshot->resourcePrefix =
        "dynamic_terrain_residency_" + std::to_string(nonce);
    snapshot->centerTile = {100, 100, 14};
    snapshot->pages.push_back(makeTerrainPage(
        snapshot->resourcePrefix, {100, 100, 14}, 0u, 10.0, 20.0, 40u));
    snapshot->pages.push_back(makeTerrainPage(
        snapshot->resourcePrefix, {99, 100, 14}, 1u, -20.0, -10.0, 80u));
    for (const auto &page : snapshot->pages)
        snapshot->estimatedTextureBytes +=
            dynamic_terrain::mipmappedRgbaBytes(
                page.textureSize, page.textureSize);

    const std::string frontMesh = snapshot->pages[0].mesh->name;
    const std::string backMesh = snapshot->pages[1].mesh->name;
    const std::string frontTexture = snapshot->pages[0].textureName;
    const std::string backTexture = snapshot->pages[1].textureName;

    gz::sim::EventManager events;
    {
        dynamic_terrain::PersistentTerrainRenderer renderer(config, events);
        renderer.QueueSnapshot(snapshot);
        events.Emit<gz::sim::events::PreRender>();
        events.Emit<gz::sim::events::PreRender>();
        CHECK(renderer.HasActiveTerrain());
        CHECK(dynamic_terrain::ogre2MeshExists(scene, frontMesh));
        CHECK(dynamic_terrain::ogre2TextureExists(scene, frontTexture));
        CHECK(dynamic_terrain::ogre2MeshExists(scene, backMesh));
        CHECK(dynamic_terrain::ogre2TextureExists(scene, backTexture));

        auto camera = scene->CreateCamera(scopedCameraName);
        CHECK(camera);
        CHECK(camera->Name() == scopedCameraName);
        camera->SetImageWidth(640u);
        camera->SetImageHeight(480u);
        camera->SetAspectRatio(4.0 / 3.0);
        gz::math::Angle hfov;
        hfov.SetDegree(90.0);
        camera->SetHFOV(hfov);
        camera->SetNearClipPlane(0.1);
        camera->SetFarClipPlane(100.0);
        camera->SetWorldPose(gz::math::Pose3d(0.0, 0.0, 5.0,
                                              0.0, 0.0, 0.0));

        camera->SetProjectionType(gz::rendering::CPT_ORTHOGRAPHIC);
        events.Emit<gz::sim::events::PreRender>();
        events.Emit<gz::sim::events::PreRender>();
        CHECK(dynamic_terrain::ogre2MeshExists(scene, frontMesh));
        CHECK(dynamic_terrain::ogre2MeshExists(scene, backMesh));

        camera->SetProjectionType(gz::rendering::CPT_PERSPECTIVE);
        events.Emit<gz::sim::events::PreRender>();
        CHECK(dynamic_terrain::ogre2MeshExists(scene, backMesh));

        try { scene->DestroySensor(camera, true); } catch (...) {}
        camera.reset();
        events.Emit<gz::sim::events::PreRender>();
        CHECK(dynamic_terrain::ogre2MeshExists(scene, backMesh));

        camera = scene->CreateCamera(scopedCameraName);
        CHECK(camera);
        camera->SetImageWidth(640u);
        camera->SetImageHeight(480u);
        camera->SetAspectRatio(4.0 / 3.0);
        camera->SetHFOV(hfov);
        camera->SetNearClipPlane(0.1);
        camera->SetFarClipPlane(100.0);
        camera->SetWorldPose(gz::math::Pose3d(0.0, 0.0, 5.0,
                                              0.0, 0.0, 0.0));
        events.Emit<gz::sim::events::PreRender>();
        CHECK(dynamic_terrain::ogre2MeshExists(scene, backMesh));
        events.Emit<gz::sim::events::PreRender>();
        CHECK(!dynamic_terrain::ogre2MeshExists(scene, backMesh));
        CHECK(!dynamic_terrain::ogre2TextureExists(scene, backTexture));

        camera->SetWorldPose(gz::math::Pose3d(
            15.0, 0.0, 10.0, 0.0, dynamic_terrain::kPi * 0.5, 0.0));
        events.Emit<gz::sim::events::PreRender>();
        CHECK(dynamic_terrain::ogre2MeshExists(scene, frontMesh));
        CHECK(dynamic_terrain::ogre2TextureExists(scene, frontTexture));
        CHECK(!dynamic_terrain::ogre2MeshExists(scene, backMesh));
        CHECK(!dynamic_terrain::ogre2TextureExists(scene, backTexture));

        for (int turn = 0; turn < 20; ++turn)
        {
            const bool frontVisible = (turn % 2) != 0;
            camera->SetWorldPose(gz::math::Pose3d(
                0.0, 0.0, 5.0, 0.0, 0.0,
                frontVisible ? 0.0 : dynamic_terrain::kPi));
            events.Emit<gz::sim::events::PreRender>();
            CHECK(dynamic_terrain::ogre2MeshExists(scene, frontMesh));
            CHECK(dynamic_terrain::ogre2TextureExists(scene, frontTexture));
            CHECK(dynamic_terrain::ogre2MeshExists(scene, backMesh));
            CHECK(dynamic_terrain::ogre2TextureExists(scene, backTexture));

            events.Emit<gz::sim::events::PreRender>();
            CHECK(dynamic_terrain::ogre2MeshExists(scene, frontMesh) ==
                  frontVisible);
            CHECK(dynamic_terrain::ogre2TextureExists(scene, frontTexture) ==
                  frontVisible);
            CHECK(dynamic_terrain::ogre2MeshExists(scene, backMesh) ==
                  !frontVisible);
            CHECK(dynamic_terrain::ogre2TextureExists(scene, backTexture) ==
                  !frontVisible);
        }

        // An unsupported projection or a missing selected camera cannot prove
        // that an evicted page is still offscreen. Restore it conservatively.
        camera->SetProjectionType(gz::rendering::CPT_ORTHOGRAPHIC);
        events.Emit<gz::sim::events::PreRender>();
        CHECK(dynamic_terrain::ogre2MeshExists(scene, frontMesh));
        CHECK(dynamic_terrain::ogre2MeshExists(scene, backMesh));
        camera->SetProjectionType(gz::rendering::CPT_PERSPECTIVE);
        events.Emit<gz::sim::events::PreRender>();
        events.Emit<gz::sim::events::PreRender>();
        CHECK(!dynamic_terrain::ogre2MeshExists(scene, backMesh));
        scene->DestroySensor(camera, true);
        camera.reset();
        events.Emit<gz::sim::events::PreRender>();
        CHECK(dynamic_terrain::ogre2MeshExists(scene, frontMesh));
        CHECK(dynamic_terrain::ogre2MeshExists(scene, backMesh));

        events.Emit<gz::sim::events::RenderTeardown>();
        CHECK(!dynamic_terrain::ogre2MeshExists(scene, frontMesh));
        CHECK(!dynamic_terrain::ogre2MeshExists(scene, backMesh));
        CHECK(!dynamic_terrain::ogre2TextureExists(scene, frontTexture));
        CHECK(!dynamic_terrain::ogre2TextureExists(scene, backTexture));

        try { scene->DestroySensor(camera, true); } catch (...) {}
    }

    try { scene->DestroySensor(similarlyNamedCamera, true); } catch (...) {}
}

void testDualCameraTurnPixels(const gz::rendering::ScenePtr &scene,
                             std::int64_t nonce)
{
    const auto background = scene->BackgroundColor();
    const auto ambient = scene->AmbientLight();
    scene->SetBackgroundColor(gz::math::Color(0, 0, 1));
    scene->SetAmbientLight(gz::math::Color(1, 1, 1));
    std::array<gz::rendering::CameraPtr, 2> cameras;
    for (std::size_t index = 0; index < cameras.size(); ++index)
    {
        cameras[index] = scene->CreateCamera("turn_camera_" +
            std::to_string(nonce) + "_" + std::to_string(index));
        auto &camera = cameras[index];
        CHECK(camera);
        scene->RootVisual()->AddChild(camera);
        camera->SetImageWidth(160);
        camera->SetImageHeight(120);
        camera->SetAspectRatio(4.0 / 3.0);
        camera->SetHFOV(gz::math::Angle(1.74));
        camera->SetNearClipPlane(0.1);
        camera->SetFarClipPlane(100);
    }

    std::vector<std::vector<unsigned char>> reference;
    std::uint64_t checkedPixels = 0;
    for (const bool eviction : {false, true})
    {
        dynamic_terrain::Config config;
        config.visualWarmupFrames = 1;
        config.visualFrustumEviction = eviction;
        config.visualUploadPagesPerFrame = 32;
        config.visualUploadBudgetMs = 1000;
        config.visualOffscreenFrames = 2;
        config.visualTextureGuardM = 0;
        config.visualLightingEnabled = false;
        config.cameraNames = {cameras[0]->Name(), cameras[1]->Name()};
        config.diagnostics = false;
        const std::string prefix = "turn_pixels_" + std::to_string(nonce) +
            (eviction ? "_evict" : "_reference");
        auto snapshot = std::make_shared<dynamic_terrain::TerrainSnapshot>();
        snapshot->generation = 1;
        snapshot->resourcePrefix = prefix;
        snapshot->centerTile = {100, 100, 14};
        for (int y = -4; y <= 4; ++y)
            for (int x = -4; x <= 4; ++x)
            {
                auto page = makeTerrainPage(prefix, {100 + x, 100 + y, 14},
                    snapshot->pages.size(), x * 20 - 10, x * 20 + 10,
                    static_cast<std::uint8_t>(70 + ((x + 4) * 7 + y + 4) % 120));
                auto mesh = std::make_shared<dynamic_terrain::MeshData>(*page.mesh);
                for (std::size_t vertex = 0; vertex < mesh->positions.size(); ++vertex)
                    mesh->positions[vertex].Y(y * 20 + (vertex < 2 ? -10 : 10));
                mesh->indices = {0, 1, 2, 1, 3, 2};
                page.mesh = std::move(mesh);
                page.imageryZoom = 14;
                snapshot->pages.push_back(std::move(page));
            }
        gz::sim::EventManager events;
        dynamic_terrain::PersistentTerrainRenderer renderer(config, events);
        renderer.QueueSnapshot(snapshot);
        for (int warmup = 0; !renderer.HasActiveTerrain(); ++warmup)
        {
            CHECK(warmup < 150);
            events.Emit<gz::sim::events::PreRender>();
        }
        for (int frame = 0; frame < 80; ++frame)
        {
            if (eviction && frame > 0 && frame % 20 == 0)
            {
                auto replacement = std::make_shared<dynamic_terrain::TerrainSnapshot>(*snapshot);
                replacement->generation = 1 + frame / 20;
                for (auto &page : replacement->pages)
                {
                    auto mesh = std::make_shared<dynamic_terrain::MeshData>(*page.mesh);
                    mesh->name += "_g" + std::to_string(replacement->generation);
                    page.mesh = std::move(mesh);
                    page.texture = solidImage(25); page.imageryZoom = 10;
                    page.textureName += "_coarse";
                }
                renderer.QueueSnapshot(replacement);
            }
            // Hold both directions long enough to evict, jump 180 degrees,
            // then roll and yaw continuously while both cameras keep updating.
            const double yaw = frame < 10 ? 0 : frame < 20 ? GZ_PI :
                (frame - 20) * 0.31;
            const double roll = frame < 20 ? 0 : 0.9 * std::sin(frame * 0.7);
            const gz::math::Pose3d vehicle(0, 0, 20, roll, 0.2, yaw);
            cameras[0]->SetWorldPose(vehicle * gz::math::Pose3d(0.85, 0, 0.15, 0, 0, 0));
            cameras[1]->SetWorldPose(vehicle * gz::math::Pose3d(0, 0, -0.25, 0, GZ_PI / 2, 0));
            events.Emit<gz::sim::events::PreRender>();
            scene->PreRender();
            for (auto &camera : cameras)
            {
                camera->PreRender(); camera->Render(); camera->PostRender();
                auto image = camera->CreateImage();
                camera->Copy(image);
                CHECK(image.Depth() == 3);
                const auto *pixels = image.Data<unsigned char>();
                const std::size_t bytes = image.Width() * image.Height() * image.Depth();
                if (!eviction)
                    reference.emplace_back(pixels, pixels + bytes);
                else
                {
                    const std::size_t index = frame * 2 + (&camera - cameras.data());
                    CHECK(reference[index].size() == bytes);
                    CHECK(std::equal(reference[index].begin(), reference[index].end(), pixels));
                    checkedPixels += image.Width() * image.Height();
                }
            }
            scene->PostRender();
            events.Emit<gz::sim::events::PostRender>();
        }
        events.Emit<gz::sim::events::RenderTeardown>();
    }
    for (auto &camera : cameras) scene->DestroySensor(camera, true);
    scene->SetBackgroundColor(background);
    scene->SetAmbientLight(ambient);
    std::cout << "dual-camera-turns frames=160 compared_pixels=" << checkedPixels
              << " exact_pixel_parity=true recenter_quality_retained=true" << std::endl;
}

int runTest()
{
    std::map<std::string, std::string> parameters;
    auto *engine = gz::rendering::engine(TERRAIN_RENDER_ENGINE_LIBRARY, parameters);
    if (!engine)
    {
        std::cerr << "SKIP: Ogre2 render engine is unavailable\n";
        return 77;
    }

    const auto nonce = std::chrono::high_resolution_clock::now()
        .time_since_epoch().count();
    RenderCleanup cleanup;
    cleanup.engine = engine;
    cleanup.scene = engine->CreateScene(
        "dynamic_terrain_texture_lifecycle_scene_" +
        std::to_string(nonce));
    if (!cleanup.scene)
    {
        std::cerr << "SKIP: Ogre2 could not create a headless test scene\n";
        return 77;
    }

    auto *ogreEngine = gz::rendering::Ogre2RenderEngine::Instance();
    CHECK(ogreEngine);
    CHECK(ogreEngine->OgreRoot());
    CHECK(ogreEngine->OgreRoot()->getRenderSystem());
    std::cout << "ogre_reverse_depth=" <<
        ogreEngine->OgreRoot()->getRenderSystem()->isReverseDepth() << std::endl;
    cleanup.textureManager = ogreEngine->OgreRoot()
        ->getRenderSystem()->getTextureGpuManager();
    CHECK(cleanup.textureManager);

    constexpr unsigned int width = 64;
    constexpr unsigned int height = 64;
    std::vector<std::uint8_t> rgb(width * height * 3u, 127u);
    auto image = std::make_shared<gz::common::Image>();
    image->SetFromData(rgb.data(), width, height,
                       gz::common::Image::RGB_INT8);
    CHECK(image->Valid());

    cleanup.textureName =
        "dynamic_terrain_ctest_unique_texture_" + std::to_string(nonce);
    CHECK(!cleanup.textureManager->findTextureNoThrow(cleanup.textureName));

    cleanup.materialA = cleanup.scene->CreateMaterial(
        "dynamic_terrain_ctest_material_a_" + std::to_string(nonce));
    cleanup.materialB = cleanup.scene->CreateMaterial(
        "dynamic_terrain_ctest_material_b_" + std::to_string(nonce));
    CHECK(cleanup.materialA);
    CHECK(cleanup.materialB);

    dynamic_terrain::ResourceReferenceCounter references;
    cleanup.materialA->SetTexture(cleanup.textureName, image);
    CHECK(references.Acquire(cleanup.textureName) == 1u);
    cleanup.materialB->SetTexture(cleanup.textureName, image);
    CHECK(references.Acquire(cleanup.textureName) == 2u);

    auto *texture = cleanup.textureManager->findTextureNoThrow(
        cleanup.textureName);
    CHECK(texture);
    CHECK(texture->getSizeBytes() ==
          dynamic_terrain::mipmappedRgbaBytes(width, height));

    destroyMaterial(cleanup.scene, cleanup.materialA);
    CHECK(!references.Release(cleanup.textureName));
    CHECK(references.References(cleanup.textureName) == 1u);
    CHECK(cleanup.textureManager->findTextureNoThrow(cleanup.textureName));

    destroyMaterial(cleanup.scene, cleanup.materialB);
    CHECK(references.Release(cleanup.textureName));
    CHECK(references.ResourceCount() == 0u);
    texture = cleanup.textureManager->findTextureNoThrow(cleanup.textureName);
    CHECK(texture);
    CHECK(dynamic_terrain::ogre2TextureExists(
        cleanup.scene, cleanup.textureName));
    CHECK(dynamic_terrain::ogre2TextureBytes(
        cleanup.scene, cleanup.textureName) ==
        dynamic_terrain::mipmappedRgbaBytes(width, height));
    CHECK(dynamic_terrain::destroyUnreferencedOgre2Texture(
        cleanup.scene, cleanup.textureName));
    CHECK(!cleanup.textureManager->findTextureNoThrow(cleanup.textureName));

    cleanup.meshName =
        "dynamic_terrain_ctest_unique_mesh_" + std::to_string(nonce);
    gz::common::Mesh commonMesh;
    commonMesh.SetName(cleanup.meshName);
    auto surface = std::make_unique<gz::common::SubMesh>("terrain_test_page");
    surface->SetPrimitiveType(gz::common::SubMesh::TRIANGLES);
    surface->AddVertex(0.0, 0.0, 0.0);
    surface->AddNormal(0.0, 0.0, 1.0);
    surface->AddTexCoord(0.0, 0.0);
    surface->AddVertex(1.0, 0.0, 0.0);
    surface->AddNormal(0.0, 0.0, 1.0);
    surface->AddTexCoord(1.0, 0.0);
    surface->AddVertex(0.0, 1.0, 0.0);
    surface->AddNormal(0.0, 0.0, 1.0);
    surface->AddTexCoord(0.0, 1.0);
    surface->AddIndex(0u);
    surface->AddIndex(1u);
    surface->AddIndex(2u);
    commonMesh.AddSubMesh(std::move(surface));

    cleanup.geometry = cleanup.scene->CreateMesh(&commonMesh);
    cleanup.visual = cleanup.scene->CreateVisual(
        "dynamic_terrain_ctest_mesh_visual_" + std::to_string(nonce));
    CHECK(cleanup.geometry);
    CHECK(cleanup.visual);
    cleanup.visual->AddGeometry(cleanup.geometry);
    cleanup.scene->RootVisual()->AddChild(cleanup.visual);
    CHECK(dynamic_terrain::ogre2MeshExists(
        cleanup.scene, cleanup.meshName));
    const auto importedMesh = Ogre::v1::MeshManager::getSingleton().getByName(
        cleanup.meshName + "::::ORIGINAL");
    CHECK(importedMesh);
    const std::string importedMaterialName =
        importedMesh->getSubMesh(0)->getMaterialName();
    CHECK(cleanup.scene->Material(importedMaterialName));

    cleanup.visual->RemoveGeometries();
    cleanup.geometry->Destroy();
    cleanup.geometry.reset();
    cleanup.scene->DestroyVisual(cleanup.visual, true);
    cleanup.visual.reset();
    CHECK(dynamic_terrain::ogre2MeshExists(
        cleanup.scene, cleanup.meshName));
    CHECK(dynamic_terrain::destroyUnreferencedOgre2Mesh(
        cleanup.scene, cleanup.meshName));
    CHECK(!dynamic_terrain::ogre2MeshExists(
        cleanup.scene, cleanup.meshName));
    CHECK(!cleanup.scene->Material(importedMaterialName));

    testRendererRecenters(cleanup.scene, nonce);
#ifndef TERRAIN_BENCHMARK_SKIP_UPLOAD_BUDGET
    testRendererUploadBudget(cleanup.scene, nonce);
#endif
    testProgressiveInitial(cleanup.scene, nonce);
    testConcurrentInitialSnapshots(cleanup.scene, nonce);
    testDualCameraTurnPixels(cleanup.scene, nonce);
    testSharedMeshRetirement(cleanup.scene, nonce);
    testCameraResidency(cleanup.scene, nonce);
    testMountainCameraFrames(cleanup.scene, nonce);

    std::cout << "Ogre2 texture, mesh, recenter and camera-residency tests passed\n";
    return 0;
}
}

int main(int argc, char **argv)
{
    try
    {
        for (int i = 1; i < argc; ++i)
        {
            if (std::string(argv[i]) == "--seconds" && i + 1 < argc) soakSeconds = std::stod(argv[++i]);
            else if (std::string(argv[i]) == "--report" && i + 1 < argc) reportPath = argv[++i];
            else throw std::runtime_error("usage: Ogre2TextureLifecycle_TEST [--seconds N] [--report PATH]");
        }
        return runTest();
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
