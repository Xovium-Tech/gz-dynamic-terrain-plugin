#include "dynamic_terrain/core/TileStore.hh"
#include "dynamic_terrain/adapters/gzsim/GzGeographicTransform.hh"
#include <gz/plugin/Loader.hh>
#include <gz/plugin/PluginPtr.hh>
#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/SphericalCoordinates.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Collision.hh>
#include <sdf/Root.hh>
#include <sdf/Model.hh>
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <thread>
#include <iostream>

#define CHECK(x) do { if (!(x)) throw std::runtime_error(#x); } while (false)

int main()
{
    using namespace dynamic_terrain;
    const auto cache = fs::temp_directory_path() / ("terrain_system_smoke_" +
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    try
    {
        gz::plugin::Loader loader;
        CHECK(!loader.LoadLib(TERRAIN_SYSTEM_LIBRARY).empty());
        gz::sim::EntityComponentManager ecm;
        gz::sim::EventManager events;
        const auto world = ecm.CreateEntity();
        ecm.CreateComponent(world, gz::sim::components::World());
        ecm.CreateComponent(world, gz::sim::components::Name("terrain_smoke"));
        gz::math::SphericalCoordinates spherical(gz::math::SphericalCoordinates::EARTH_WGS84,
            gz::math::Angle(52.2297*kPi/180), gz::math::Angle(21.0122*kPi/180),
            123.4, gz::math::Angle(0.3));
        ecm.CreateComponent(world, gz::sim::components::SphericalCoordinates(spherical));
        const auto aircraft = ecm.CreateEntity();
        ecm.CreateComponent(aircraft, gz::sim::components::Model());
        ecm.CreateComponent(aircraft, gz::sim::components::Name("aircraft"));
        ecm.CreateComponent(aircraft, gz::sim::components::Pose(gz::math::Pose3d(0,0,1,0,0,0)));
        ecm.CreateComponent(aircraft, gz::sim::components::ParentEntity(world));
        ecm.SetParentEntity(aircraft, world);

        Config cfg;
        cfg.cacheDir = cache.string(); cfg.imageryProvider = "synthetic";
        cfg.imageryExtension = "png";
        TileStore store(cfg, std::make_shared<GzGeographicTransform>(spherical));
        const auto center = latLonToTile(52.2297, 21.0122, 14);
        for (int x=center.x-3; x<=center.x+3; ++x)
            for (int y=center.y-3; y<=center.y+3; ++y)
            {
                const auto file = store.ImageryPath({x,y,14});
                fs::create_directories(file.parent_path());
                CHECK(cv::imwrite(file.string(), cv::Mat(8,8,CV_8UC3,cv::Scalar(40,90,120))));
            }
        sdf::Root config;
        CHECK(config.LoadSdfString("<sdf version='1.9'><model name='aircraft'><link name='body'/>"
            "<plugin name='custom::DynamicTerrainConfig' filename='unused'>"
            "<cache_dir>" + xmlEscape(cache.string()) + "</cache_dir>"
            "<imagery_provider>synthetic</imagery_provider><imagery_extension>png</imagery_extension>"
            "<imagery_url>file:///no-network-in-smoke/{z}/{x}/{y}.png</imagery_url>"
            "<elevation_provider>flat</elevation_provider><download_retries>0</download_retries>"
            "<visual_radius_m>1000</visual_radius_m><visual_mesh_cells_per_tile>8</visual_mesh_cells_per_tile>"
            "<visual_geometry_zoom>14</visual_geometry_zoom><visual_bootstrap_imagery_zoom>14</visual_bootstrap_imagery_zoom>"
            "<visual_page_texture_max_size>256</visual_page_texture_max_size><visual_refine_texture>false</visual_refine_texture>"
            "<radius_tiles>0</radius_tiles><heightmap_size>129</heightmap_size><dynamic_zoom>false</dynamic_zoom>"
            "<static_zoom>17</static_zoom><startup_preload>false</startup_preload><diagnostics>false</diagnostics>"
            "<startup_safety_remove_delay_sec>0</startup_safety_remove_delay_sec>"
            "</plugin></model></sdf>").empty());
        auto modelPlugin = loader.Instantiate("custom::DynamicTerrainConfig");
        auto systemPlugin = loader.Instantiate("custom::DynamicTerrainSystem");
        CHECK(modelPlugin && systemPlugin);
        auto *modelConfigure = modelPlugin->QueryInterface<gz::sim::ISystemConfigure>();
        auto *configure = systemPlugin->QueryInterface<gz::sim::ISystemConfigure>();
        auto *update = systemPlugin->QueryInterface<gz::sim::ISystemPreUpdate>();
        CHECK(modelConfigure && configure && update);
        modelConfigure->Configure(aircraft, config.Model()->Element()->GetElement("plugin"), ecm, events);
        configure->Configure(world, {}, ecm, events);
        bool collisionCreated = false, safetyRemoved = false;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
        gz::sim::UpdateInfo info;
        info.paused = false;
        for (int tick=1; std::chrono::steady_clock::now()<deadline; ++tick)
        {
            info.simTime = std::chrono::milliseconds(tick*100);
            info.dt = std::chrono::milliseconds(100);
            update->PreUpdate(info, ecm);
            ecm.ProcessRemoveEntityRequests();
            bool safety = false;
            ecm.Each<gz::sim::components::Name>([&](const gz::sim::Entity &,
                const gz::sim::components::Name *name) {
                if (name->Data().find("dynamic_terrain_collision_")==0) collisionCreated=true;
                if (name->Data()=="dynamic_terrain_startup_safety") safety=true;
                return true;
            });
            safetyRemoved = collisionCreated && !safety;
            if (safetyRemoved) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        CHECK(collisionCreated && safetyRemoved);
        // Plugin destructors join workers while ECM and event manager still exist.
        systemPlugin = {};
        modelPlugin = {};
        fs::remove_all(cache);
        std::cout << "Plugin aliases, SDF configuration, tracking, collision insertion and safety retirement passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        fs::remove_all(cache);
        std::cerr << error.what() << '\n';
        return 1;
    }
}
