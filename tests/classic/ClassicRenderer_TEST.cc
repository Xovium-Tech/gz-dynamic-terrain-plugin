#include "dynamic_terrain/adapters/classic/ClassicTerrainRenderer.hh"
#include "dynamic_terrain/adapters/classic/ClassicTerrainTransport.hh"
#include "dynamic_terrain/adapters/classic/ClassicTerrainCodec.hh"
#include <gazebo/transport/transport.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/rendering/Camera.hh>
#include <OgreMaterialManager.h>
#include <OgreTextureManager.h>
#include <OgreSceneManager.h>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <sdf/sdf.hh>
#include <unistd.h>

using namespace dynamic_terrain;
#define CHECK(x) do { if (!(x)) throw std::runtime_error(#x); } while (false)

namespace
{
std::shared_ptr<ImageData> solid(unsigned char r, unsigned char g, unsigned char b)
{
    auto image = std::make_shared<ImageData>();
    image->width = image->height = 64;
    for (int i=0; i<64*64; ++i) image->rgb.insert(image->rgb.end(), {r,g,b});
    return image;
}
std::shared_ptr<TerrainSnapshot> snapshot(std::uint64_t generation)
{
    auto out = std::make_shared<TerrainSnapshot>();
    out->generation = generation; out->centerTile = {1,1,14};
    out->resourcePrefix = "classic_renderer_test";
    auto mesh = std::make_shared<MeshData>();
    mesh->name = "classic_test_mesh_" + std::to_string(generation); mesh->submeshName = "surface";
    mesh->positions = {{-10,-10,0},{10,-10,0},{-10,10,0},{10,10,0}};
    mesh->normals.assign(4, Vec3{0,0,1});
    mesh->texCoords = {{0,0},{1,0},{0,1},{1,1}};
    mesh->indices = {0,1,2,1,3,2};
    TerrainPage page;
    page.key = out->centerTile; page.submeshName = "surface"; page.mesh = mesh;
    page.textureSize = 64; page.imageryZoom = 14;
    page.textureName = "classic_test_texture_" + std::to_string(generation);
    page.texture = solid(200,40,30); out->pages.push_back(page);
    return out;
}
std::size_t resources(Ogre::ResourceManager &manager)
{
    std::size_t result = 0;
    auto iterator = manager.getResourceIterator();
    while (iterator.hasMoreElements()) { iterator.getNext(); ++result; }
    return result;
}
}
int main()
{
    bool started = false;
    gazebo::physics::WorldPtr world;
    const auto worldFile = std::filesystem::temp_directory_path() /
        ("classic_terrain_render_" + std::to_string(getpid()) + ".world");
    try
    {
        const auto master = "http://127.0.0.1:" + std::to_string(20000 + getpid()%30000);
        setenv("GAZEBO_MASTER_URI",master.c_str(),1); setenv("GAZEBO_MODEL_DATABASE_URI","",1);
        CHECK(gazebo::setupServer()); started = true;
        // Scene initialization consumes the matching world's scene-info
        // response. A standalone render scene has no physics publisher.
        {
            std::ofstream file(worldFile);
            file << "<sdf version='1.6'><world name='classic_terrain_render_test'>"
                "<physics type='ode'><real_time_update_rate>0</real_time_update_rate></physics>"
                "</world></sdf>";
        }
        world = gazebo::loadWorld(worldFile.string());
        CHECK(world);
        world->_SetSensorsInitialized(true);
        world->SetPaused(false);
        // setupServer initializes rendering; loading it twice duplicates Ogre's
        // shader factories.
        auto scene = gazebo::rendering::create_scene("classic_terrain_render_test",false,true);
        CHECK(scene);
        const auto sceneDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
        while (!scene->Initialized() && std::chrono::steady_clock::now() < sceneDeadline)
        {
            gazebo::runWorld(world, 1);
            gazebo::event::Events::preRender();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        CHECK(scene->Initialized());
        scene->SetAmbientColor(ignition::math::Color(1,1,1,1));
        scene->SetBackgroundColor(ignition::math::Color(0,0,0,1));
        gazebo::rendering::VisualPtr anchor(new gazebo::rendering::Visual("terrain_anchor",scene->WorldVisual()));
        anchor->Load();
        auto camera = scene->CreateCamera("terrain_camera",false);
        camera->Load(); camera->SetImageSize(64,64); camera->Init();
        camera->CreateRenderTexture("terrain_regression_camera"); camera->SetCaptureData(true);
        camera->SetWorldPose(ignition::math::Pose3d(0,0,10,0,1.5707963267948966,0));
        auto frame = [&] {
            gazebo::event::Events::preRender();
            camera->Render(true); camera->PostRender();
            gazebo::event::Events::postRender();
        };
        frame();
        const auto baselineTextures = resources(Ogre::TextureManager::getSingleton());
        const auto baselineMaterials = resources(Ogre::MaterialManager::getSingleton());
        Config cfg; cfg.visualFrustumEviction = false; cfg.visualWarmupFrames = 1;
        cfg.visualLightingEnabled = false; cfg.diagnostics = false;
        {
            ClassicTerrainRenderer renderer(anchor);
            for (std::uint64_t generation=1; generation<=5; ++generation)
            {
                auto terrain = snapshot(generation);
                renderer.QueueSnapshot(terrain,cfg);
                for (int i=0;i<4;++i) frame();
                CHECK(renderer.ActiveGeneration()==generation);
                const auto *pixels = camera->ImageData();
                CHECK(pixels);
                const auto center = (32*64+32)*3;
                CHECK(pixels[center]>pixels[center+1]+40);
                TextureUpdate update; update.generation=generation;
                const auto &page=terrain->pages[0];
                update.pages.push_back({0,page.key,page.submeshName,15,64,solid(30,200,40),
                                        "classic_refined_"+std::to_string(generation)});
                renderer.QueueTexture(std::move(update));
                frame();frame();
                pixels=camera->ImageData();
                CHECK(pixels[center+1]>pixels[center]+40);
                CHECK(resources(Ogre::TextureManager::getSingleton())<=baselineTextures+1);
                CHECK(resources(Ogre::MaterialManager::getSingleton())<=baselineMaterials+1);
            }
        }
        CHECK(resources(Ogre::TextureManager::getSingleton())==baselineTextures);
        CHECK(resources(Ogre::MaterialManager::getSingleton())==baselineMaterials);
        // Exercise the actual visual plugin and late-subscriber page transport.
        {
            ClassicTerrainSource source(cfg, scene->Name(), "~/terrain_visual_smoke");
            const auto terrain = snapshot(7);
            source.QueueSnapshot(terrain);
            auto plugin = gazebo::VisualPlugin::Create(TERRAIN_CLASSIC_VISUAL_LIBRARY, "terrain_visual_test");
            CHECK(plugin);
            sdf::SDFPtr config(new sdf::SDF);
            sdf::init(config);
            CHECK(sdf::readString("<sdf version='1.6'><model name='test'><link name='link'>"
                "<visual name='anchor'><geometry><box><size>1 1 1</size></box></geometry>"
                "<plugin name='terrain_visual_test' filename='unused'>"
                "<topic>~/terrain_visual_smoke</topic></plugin></visual></link></model></sdf>", config));
            plugin->Load(anchor, config->Root()->GetElement("model")->GetElement("link")
                ->GetElement("visual")->GetElement("plugin"));
            auto waitForColor = [&](bool green) {
                const auto deadline = std::chrono::steady_clock::now()+std::chrono::seconds(10);
                while (std::chrono::steady_clock::now()<deadline)
                {
                    frame();
                    const auto *pixels = camera->ImageData();
                    const auto center = (32*64+32)*3;
                    if (pixels && (green ? pixels[center+1]>pixels[center]+40
                                         : pixels[center]>pixels[center+1]+40)) return;
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                throw std::runtime_error("Classic visual plugin did not render the transported page");
            };
            waitForColor(false);
            TextureUpdate update; update.generation=7;
            const auto &page=terrain->pages[0];
            update.pages.push_back({0,page.key,page.submeshName,15,64,solid(30,200,40),"wire_refinement"});
            source.QueueTexture(std::move(update));
            waitForColor(true);
            plugin.reset();
        }
        CHECK(resources(Ogre::TextureManager::getSingleton())==baselineTextures);
        CHECK(resources(Ogre::MaterialManager::getSingleton())==baselineMaterials);
        // A refinement manifest can overtake an in-flight bootstrap page.
        // Complete geometry must become visible while the newer image is still
        // unavailable; otherwise continuous refinement can starve activation.
        {
            gazebo::transport::NodePtr node(new gazebo::transport::Node);
            node->Init(scene->Name());
            auto manifests = node->Advertise<classic_msgs::Manifest>("~/terrain_bootstrap_race/manifest", 1);
            auto pages = node->Advertise<classic_msgs::PageResponse>("~/terrain_bootstrap_race/page", 1);
            auto terrain = snapshot(8);
            classic_msgs::Manifest manifest;
            *manifest.mutable_metadata() = EncodeClassicSnapshot(*terrain, cfg, false);
            manifest.set_revision(2);
            auto *identity = manifest.add_pages();
            identity->set_index(0);
            identity->mutable_key()->set_x(1); identity->mutable_key()->set_y(1); identity->mutable_key()->set_z(14);
            identity->set_texture_name("refinement_still_in_flight");
            classic_msgs::PageResponse bootstrap;
            bootstrap.set_generation(8); bootstrap.set_revision(1);
            *bootstrap.mutable_page() = EncodeClassicPage(terrain->pages[0], true);
            auto plugin = gazebo::VisualPlugin::Create(TERRAIN_CLASSIC_VISUAL_LIBRARY, "bootstrap_race_test");
            CHECK(plugin);
            sdf::SDFPtr config(new sdf::SDF); sdf::init(config);
            CHECK(sdf::readString("<sdf version='1.6'><model name='test'><link name='link'>"
                "<visual name='anchor'><geometry><box><size>1 1 1</size></box></geometry>"
                "<plugin name='bootstrap_race_test' filename='unused'>"
                "<topic>~/terrain_bootstrap_race</topic></plugin></visual></link></model></sdf>", config));
            plugin->Load(anchor, config->Root()->GetElement("model")->GetElement("link")
                ->GetElement("visual")->GetElement("plugin"));
            bool visible = false;
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
            while (std::chrono::steady_clock::now() < deadline)
            {
                manifests->Publish(manifest); pages->Publish(bootstrap);
                frame();
                const auto *pixels = camera->ImageData();
                const auto center = (32*64+32)*3;
                if (pixels && pixels[center] > pixels[center+1]+40) { visible = true; break; }
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            CHECK(visible);
            plugin.reset(); node->Fini();
        }
        CHECK(resources(Ogre::TextureManager::getSingleton())==baselineTextures);
        CHECK(resources(Ogre::MaterialManager::getSingleton())==baselineMaterials);
        anchor->Fini(); anchor.reset(); camera.reset(); scene.reset();
        gazebo::rendering::remove_scene("classic_terrain_render_test");
        world.reset();
        gazebo::shutdown(); started = false;
        std::filesystem::remove(worldFile);
        std::cout << "Classic Ogre1 terrain pixels, refinement, generation swaps and cleanup passed\n";
        return 0;
    }
    catch(const std::exception &error)
    {
        std::cerr<<error.what()<<'\n';
        world.reset();
        if(started) gazebo::shutdown();
        std::filesystem::remove(worldFile);
        return 1;
    }
}
