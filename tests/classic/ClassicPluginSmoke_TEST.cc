#include "dynamic_terrain/core/TileStore.hh"
#include "dynamic_terrain/adapters/classic/ClassicGeographicTransform.hh"
#include "ClassicWorldStepper.hh"

#include <gazebo/gazebo.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/util/LogRecord.hh>
#include <opencv2/imgcodecs.hpp>

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <set>
#include <stdexcept>
#include <thread>
#include <unistd.h>

#define CHECK(value) do { if (!(value)) throw std::runtime_error(#value); } while (false)

namespace
{
std::set<std::string> livePatches(const gazebo::physics::WorldPtr &world)
{
    std::set<std::string> result;
    for (const auto &model : world->Models())
    {
        if (model->GetName().find("dynamic_terrain_classic_") != 0 ||
            model->GetName().find("_collision_") == std::string::npos) continue;
        const auto link = model->GetLink("ground");
        const auto collision = link ? link->GetCollision("collision") : gazebo::physics::CollisionPtr{};
        if (collision && collision->GetShape()) result.insert(model->GetName());
    }
    return result;
}

bool safetyPresent(const gazebo::physics::WorldPtr &world)
{
    for (const auto &model : world->Models())
        if (model->GetName().find("_startup_safety") != std::string::npos) return true;
    return false;
}
}

int main()
{
    using namespace dynamic_terrain;
    const auto cache = fs::temp_directory_path() / ("terrain_classic_smoke_" +
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    bool serverStarted = false;
    gazebo::physics::WorldPtr world;
    std::unique_ptr<ClassicWorldStepper> stepper;
    try
    {
        Config cfg;
        cfg.cacheDir = cache.string();
        cfg.imageryProvider = "synthetic";
        cfg.imageryExtension = "png";
        cfg.imageryUrl = "file:///no-network-in-smoke/{z}/{x}/{y}.png";
        gazebo::common::SphericalCoordinates native(
            gazebo::common::SphericalCoordinates::EARTH_WGS84,
            ignition::math::Angle(52.2297 * kPi / 180.0),
            ignition::math::Angle(21.0122 * kPi / 180.0), 123.4,
            ignition::math::Angle(37.0 * kPi / 180.0));
        TileStore store(cfg, std::make_shared<ClassicGeographicTransform>(native));
        const auto center = latLonToTile(52.2297, 21.0122, 14);
        for (int x = center.x - 3; x <= center.x + 3; ++x)
            for (int y = center.y - 3; y <= center.y + 3; ++y)
            {
                const auto file = store.ImageryPath({x, y, 14});
                fs::create_directories(file.parent_path());
                CHECK(cv::imwrite(file.string(), cv::Mat(8, 8, CV_8UC3, cv::Scalar(40, 90, 120))));
            }
        const auto filename = cache / "smoke.world";
        std::ofstream file(filename);
        file << "<sdf version='1.6'><world name='terrain_classic_smoke'>"
            "<physics type='ode'><max_step_size>0.005</max_step_size><real_time_update_rate>0</real_time_update_rate></physics>"
            "<gravity>0 0 -9.8</gravity><spherical_coordinates><surface_model>EARTH_WGS84</surface_model>"
            "<latitude_deg>52.2297</latitude_deg><longitude_deg>21.0122</longitude_deg>"
            "<elevation>123.4</elevation><heading_deg>37</heading_deg></spherical_coordinates>"
            "<model name='aircraft'><static>true</static><pose>0 0 10 0 0 0</pose><link name='body'/></model>"
            "<model name='probe'><pose>0 0 2 0 0 0</pose><link name='body'>"
            "<inertial><mass>1</mass><inertia><ixx>0.4</ixx><iyy>0.4</iyy><izz>0.4</izz></inertia></inertial>"
            "<collision name='sphere'><geometry><sphere><radius>1</radius></sphere></geometry></collision></link></model>"
            "<plugin name='dynamic_terrain' filename='" << xmlEscape(TERRAIN_CLASSIC_LIBRARY) << "'>"
            "<tracked_model>aircraft</tracked_model><cache_dir>" << xmlEscape(cache.string()) << "</cache_dir>"
            "<imagery_provider>synthetic</imagery_provider><imagery_extension>png</imagery_extension>"
            "<imagery_url>file:///no-network-in-smoke/{z}/{x}/{y}.png</imagery_url>"
            "<elevation_provider>flat</elevation_provider><download_retries>0</download_retries>"
            "<visual_radius_m>1000</visual_radius_m><visual_mesh_cells_per_tile>8</visual_mesh_cells_per_tile>"
            "<visual_geometry_zoom>14</visual_geometry_zoom><visual_bootstrap_imagery_zoom>14</visual_bootstrap_imagery_zoom>"
            "<visual_page_texture_max_size>256</visual_page_texture_max_size><visual_refine_texture>false</visual_refine_texture>"
            "<radius_tiles>0</radius_tiles><heightmap_size>129</heightmap_size><dynamic_zoom>false</dynamic_zoom>"
            "<static_zoom>17</static_zoom><startup_preload>true</startup_preload><diagnostics>false</diagnostics>"
            "<startup_safety_remove_delay_sec>0.02</startup_safety_remove_delay_sec><update_period_sec>0.02</update_period_sec>"
            "</plugin></world></sdf>";
        file.close();

        // Isolate this test's transport master from other Gazebo tests/processes.
        const std::string master = "http://127.0.0.1:" + std::to_string(20000 + getpid() % 30000);
        setenv("GAZEBO_MASTER_URI", master.c_str(), 1);
        setenv("GAZEBO_MODEL_DATABASE_URI", "", 1);
        // Collision must run with neither a GUI nor an X display attached.
        unsetenv("DISPLAY");
        CHECK(gazebo::setupServer());
        serverStarted = true;
        CHECK(gazebo::util::LogRecord::Instance()->Init("terrain_classic_smoke"));
        world = gazebo::loadWorld(filename.string());
        CHECK(world);
        // The standalone library caller replaces gzserver's sensor loop; this
        // fixture contains no sensors and can release the world-plugin barrier.
        world->_SetSensorsInitialized(true);
        world->SetPaused(false);
        // Keep native state snapshots active throughout retirement, rather than
        // exercising only LogWorker's initial snapshot at world startup.
        gazebo::util::LogRecord::Instance()->SetPeriod(0.005);
        CHECK(gazebo::util::LogRecord::Instance()->Start("txt", (cache / "log").string()));
        stepper = std::make_unique<ClassicWorldStepper>(world);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
        std::set<std::string> first;
        while (std::chrono::steady_clock::now() < deadline)
        {
            stepper->Step(10);
            first = livePatches(world);
            if (first.size() == 1 && !safetyPresent(world)) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        CHECK(first.size() == 1 && !safetyPresent(world));
        const auto probe = world->ModelByName("probe");
        CHECK(probe);
        // Async startup may take arbitrary simulation time. Begin the contact
        // check from a known drop after terrain is live and safety has retired.
        probe->SetWorldPose(ignition::math::Pose3d(0, 0, 2, 0, 0, 0));
        probe->SetLinearVel(ignition::math::Vector3d::Zero);
        probe->SetAngularVel(ignition::math::Vector3d::Zero);
        stepper->Step(400);
        // This probe is supported by the generated terrain mesh after startup
        // ground has gone, exercising ODE contact rather than model names alone.
        CHECK(probe->WorldPose().Pos().Z() > 0.8 && probe->WorldPose().Pos().Z() < 1.2);

        const auto aircraft = world->ModelByName("aircraft");
        CHECK(aircraft);
        // Repeated replacement exercises native LogWorker snapshots concurrently
        // with retirement, and cycles through the bounded collision mesh slots.
        auto previous = livePatches(world);
        CHECK(!previous.empty());
        for (const double x : {500.0, -500.0, 500.0, -500.0})
        {
            aircraft->SetWorldPose(ignition::math::Pose3d(x, 0, 10, 0, 0, 0));
            bool replaced = false;
            bool overlapObserved = false;
            const auto recenterDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
            while (std::chrono::steady_clock::now() < recenterDeadline)
            {
                stepper->Step(10);
                const auto patches = livePatches(world);
                overlapObserved = overlapObserved || patches.size() >= 2;
                if (patches.size() == 1 && patches != previous)
                {
                    previous = patches;
                    replaced = true;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            CHECK(replaced && overlapObserved);
        }

        // The simulation thread is parked, but the world remains live. Unload
        // exercises plugin cleanup separately from World::Fini's own teardown.
        world->RemovePlugin("dynamic_terrain");
        const auto cleanupDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        bool cleaned = false;
        while (std::chrono::steady_clock::now() < cleanupDeadline)
        {
            stepper->Step(10);
            cleaned = true;
            for (const auto &model : world->Models())
                if (model->GetName().find("dynamic_terrain_classic_") == 0) cleaned = false;
            if (cleaned) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        CHECK(cleaned);
        stepper.reset();
        gazebo::util::LogRecord::Instance()->Stop();
        world.reset();
        CHECK(gazebo::shutdown());
        serverStarted = false;
        fs::remove_all(cache);
        std::cout << "Classic plugin loading, offline terrain, ODE contact, repeated recenter and live cleanup passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        stepper.reset();
        if (serverStarted) gazebo::util::LogRecord::Instance()->Stop();
        world.reset();
        if (serverStarted) gazebo::shutdown();
        fs::remove_all(cache);
        std::cerr << error.what() << '\n';
        return 1;
    }
}
