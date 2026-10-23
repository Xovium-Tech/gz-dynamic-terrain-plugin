#include "dynamic_terrain/adapters/gzsim/GzGeographicTransform.hh"
#include "dynamic_terrain/core/TerrainTypes.hh"
#include <gz/sim/TestFixture.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/config.hh>
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>
#include <vector>

namespace fs = std::filesystem;
using namespace dynamic_terrain;
using Clock = std::chrono::steady_clock;
#define CHECK(x) do { if (!(x)) throw std::runtime_error(#x); } while (false)

int main(int argc, char **argv)
{
    if (argc != 3) { std::cerr << "usage: TerrainPhysicsSmoke PLUGIN_LIBRARY REPORT_JSON\n"; return 2; }
    const auto scratch = fs::temp_directory_path() / ("terrain_physics_" +
        std::to_string(Clock::now().time_since_epoch().count()));
    const auto started = Clock::now();
    try
    {
        const auto tiles = scratch / "source";
        const auto cache = scratch / "cache";
        constexpr int zoom = 17;
        const double lat = tileYToLat(65536.5, zoom), lon = tileXToLon(65536.5, zoom);
        const gz::math::SphericalCoordinates spherical(gz::math::SphericalCoordinates::EARTH_WGS84,
            gz::math::Angle(lat*kPi/180), gz::math::Angle(lon*kPi/180), 0.0, gz::math::Angle(0));
        GzGeographicTransform transform(spherical);
        auto seed = [&](double x, int height) {
            const auto geo = transform.GeodeticFromLocal({x, 0, 0});
            const auto tile = latLonToTile(geo.X(), geo.Y(), zoom);
            const int encoded = height + 32768;
            for (int tx=tile.x-2; tx<=tile.x+2; ++tx)
                for (int ty=tile.y-2; ty<=tile.y+2; ++ty)
                {
                    const auto file = tiles / std::to_string(zoom) / std::to_string(tx) /
                        (std::to_string(ty) + ".png");
                    fs::create_directories(file.parent_path());
                    CHECK(cv::imwrite(file.string(), cv::Mat(256,256,CV_8UC3,
                        cv::Scalar(0, encoded % 256, encoded / 256))));
                    // Upstream's downloader rejects file:// status 0. Seed its
                    // documented cache layout too, so the same offline physical
                    // fixture can compare the original libraries without HTTP.
                    const auto legacy = cache / "elevation" / "terrarium" / std::to_string(zoom) /
                        std::to_string(tx) / (std::to_string(ty) + ".png");
                    fs::create_directories(legacy.parent_path());
                    fs::copy_file(file, legacy, fs::copy_options::overwrite_existing);
                }
        };
        auto expectedAt = [&](double x, double height) {
            const auto geo = transform.GeodeticFromLocal({x, 0, 0});
            return transform.LocalFromGeodetic(geo.X(), geo.Y(), height).Z() + 0.5;
        };
        seed(0, 10);
        std::ostringstream sdf;
        sdf << std::setprecision(16) << "<sdf version='1.9'><world name='terrain_physics'>"
            "<gravity>0 0 -9.81</gravity><physics name='default' type='ignored'>"
            "<max_step_size>0.005</max_step_size><real_time_factor>0</real_time_factor></physics>"
            "<spherical_coordinates><surface_model>EARTH_WGS84</surface_model>"
            "<world_frame_orientation>ENU</world_frame_orientation><latitude_deg>" << lat <<
            "</latitude_deg><longitude_deg>" << lon << "</longitude_deg><elevation>0</elevation>"
            "<heading_deg>0</heading_deg></spherical_coordinates>"
            "<plugin filename='gz-sim-physics-system' name='gz::sim::systems::Physics'/>"
            "<plugin filename='" << xmlEscape(argv[1]) << "' name='custom::DynamicTerrainSystem'/>"
            "<model name='aircraft'><static>true</static><pose>0 0 50 0 0 0</pose><link name='body'/>"
            "<plugin filename='" << xmlEscape(argv[1]) << "' name='custom::DynamicTerrainConfig'>"
            "<cache_dir>" << xmlEscape(cache.string()) << "</cache_dir>"
            "<imagery_provider>synthetic</imagery_provider><imagery_extension>png</imagery_extension>"
            "<imagery_url>file:///missing-physics-smoke/{z}/{x}/{y}.png</imagery_url>"
            "<elevation_provider>terrarium</elevation_provider><elevation_max_zoom>17</elevation_max_zoom>"
            "<elevation_url>file://" << xmlEscape(tiles.string()) << "/{z}/{x}/{y}.png</elevation_url>"
            "<align_origin_to_ground>false</align_origin_to_ground><radius_tiles>0</radius_tiles>"
            "<heightmap_size>129</heightmap_size><dynamic_zoom>false</dynamic_zoom><static_zoom>17</static_zoom>"
            "<visual_radius_m>1000</visual_radius_m><visual_mesh_cells_per_tile>8</visual_mesh_cells_per_tile>"
            "<visual_bootstrap_imagery_zoom>14</visual_bootstrap_imagery_zoom><visual_refine_texture>false</visual_refine_texture>"
            "<download_retries>0</download_retries><http_timeout_ms>500</http_timeout_ms>"
            "<startup_preload>false</startup_preload><startup_safety_remove_delay_sec>0</startup_safety_remove_delay_sec>"
            "<update_period_sec>0.05</update_period_sec><retry_delay_sec>0.2</retry_delay_sec>"
            "<diagnostics>false</diagnostics></plugin></model>"
            "<model name='probe'><pose>0 0 30 0 0 0</pose><link name='body'>"
            "<inertial><mass>1</mass><inertia><ixx>0.1666667</ixx><iyy>0.1666667</iyy><izz>0.1666667</izz></inertia></inertial>"
            "<collision name='box'><geometry><box><size>1 1 1</size></box></geometry></collision>"
            "</link></model></world></sdf>";
        fs::create_directories(scratch);
        { std::ofstream file(scratch / "world.sdf"); file << sdf.str(); }
        gz::sim::ServerConfig config;
        CHECK(config.SetSdfString(sdf.str()));
        gz::sim::TestFixture fixture(config);
        gz::sim::Entity aircraft{}, probe{};
        bool moveAircraft = false, moveProbe = false, clearProbeVelocity = false;
        double requestedX = 0, requestedZ = 0, probeZ = 30, probeX = 0;
        unsigned newest = 0;
        std::size_t iterations = 0;
        fixture.OnConfigure([&](const auto &, const auto &, auto &ecm, auto &) {
            aircraft = ecm.EntityByComponents(gz::sim::components::Name("aircraft"), gz::sim::components::Model());
            probe = ecm.EntityByComponents(gz::sim::components::Name("probe"), gz::sim::components::Model());
            CHECK(aircraft && probe);
        }).OnPreUpdate([&](const auto &, auto &ecm) {
            if (clearProbeVelocity)
            {
                const auto link = gz::sim::Model(probe).LinkByName(ecm,"body");
                ecm.template RemoveComponent<gz::sim::components::LinearVelocityCmd>(link);
                ecm.template RemoveComponent<gz::sim::components::AngularVelocityCmd>(link);
                clearProbeVelocity=false;
            }
            if (moveAircraft) { gz::sim::Model(aircraft).SetWorldPoseCmd(ecm,
                gz::math::Pose3d(requestedX,0,50,0,0,0)); moveAircraft=false; }
            if (moveProbe)
            {
                gz::sim::Model model(probe);
                model.SetWorldPoseCmd(ecm, gz::math::Pose3d(requestedX,0,requestedZ,0,0,0));
                gz::sim::Link link(model.LinkByName(ecm,"body"));
                link.SetLinearVelocity(ecm, gz::math::Vector3d::Zero);
                link.SetAngularVelocity(ecm, gz::math::Vector3d::Zero);
                moveProbe=false;
                clearProbeVelocity=true;
            }
        }).OnPostUpdate([&](const auto &, const auto &ecm) {
            const auto pose = gz::sim::worldPose(probe, ecm);
            probeZ = pose.Pos().Z(); probeX = pose.Pos().X(); ++iterations;
            newest = 0;
            ecm.template Each<gz::sim::components::Name>([&](const auto &, const auto *name) {
                const std::string prefix = "dynamic_terrain_collision_";
                if (name->Data().rfind(prefix, 0) == 0)
                    newest = std::max(newest, static_cast<unsigned>(std::stoul(name->Data().substr(prefix.size()))));
                return true;
            });
        }).Finalize();
        auto step = [&] {
            CHECK(std::chrono::duration<double>(Clock::now()-started).count() < 60);
            CHECK(fixture.Server()->Run(true, 1, false));
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        };
        while (!newest) step();
        struct Contact { double x, expected, measured; unsigned generation; };
        std::vector<Contact> contacts;
        auto settle = [&](double x, double height) {
            requestedX=x; requestedZ=expectedAt(x,height)+5; moveProbe=true;
            int stable=0;
            double previousZ=probeZ;
            while (stable < 200)
            {
                step();
                // Existing collision encoding uses a minimum 0.10 m vertical
                // span. Allow the small heightfield/solver deviation also
                // measured with the original libraries.
                stable = std::abs(probeX-x)<0.02 && std::abs(probeZ-expectedAt(x,height))<0.15
                    && std::abs(probeZ-previousZ)<0.001 ? stable+1 : 0;
                previousZ=probeZ;
                if (iterations % 1000 == 0)
                    std::cout << "settling iteration=" << iterations << " x=" << probeX
                              << " z=" << probeZ << " expected=" << expectedAt(x,height) << std::endl;
            }
            contacts.push_back({x,expectedAt(x,height),probeZ,newest});
            std::cout << "contact x=" << x << " expected=" << expectedAt(x,height)
                      << " actual=" << probeZ << " generation=" << newest << std::endl;
        };
        settle(0,10);
        const unsigned beforeFailure=newest;
        requestedX=3000; moveAircraft=true;
        double retainedMin=probeZ, retainedMax=probeZ;
        const auto failedUntil=Clock::now()+std::chrono::seconds(2);
        while (Clock::now()<failedUntil)
        {
            step(); retainedMin=std::min(retainedMin,probeZ); retainedMax=std::max(retainedMax,probeZ);
            CHECK(newest==beforeFailure);
            CHECK(std::abs(probeZ-expectedAt(0,10))<0.15);
        }
        seed(3000,20);
        while (newest==beforeFailure) step();
        settle(3000,20);
        for (const auto [x,height] : {std::pair<double,int>{0,10}, {3000,20}})
        {
            const unsigned previous=newest;
            requestedX=x; moveAircraft=true;
            while (newest==previous) step();
            settle(x,height);
        }
        std::ostringstream report;
        report << std::setprecision(12) << "{\"gazebo_major\":" << GZ_SIM_MAJOR_VERSION
            << ",\"physics\":\"default DART engine via gz-sim Physics system\",\"gravity\":-9.81,\"probe\":\"1m dynamic box\""
            << ",\"heightmap_size\":129,\"elevation_zoom\":17,\"height_tolerance_m\":0.15,\"plugin_library\":\"" << argv[1]
            << "\",\"iterations\":" << iterations << ",\"seconds\":"
            << std::chrono::duration<double>(Clock::now()-started).count()
            << ",\"failed_replacement_retained_ground\":true,\"retained_min_z\":" << retainedMin
            << ",\"retained_max_z\":" << retainedMax << ",\"contacts\":[";
        for (std::size_t i=0;i<contacts.size();++i)
        {
            const auto &c=contacts[i];
            if(i)report<<',';
            report << "{\"x\":"<<c.x<<",\"expected_z\":"<<c.expected<<",\"measured_z\":"
                <<c.measured<<",\"generation\":"<<c.generation<<'}';
        }
        report << "],\"passed\":true}";
        std::ofstream(argv[2]) << report.str() << '\n';
        std::cout << report.str() << std::endl;
        fs::remove_all(scratch);
        return 0;
    }
    catch(const std::exception &error)
    {
        std::cerr << "Physics smoke failed: " << error.what() << "; fixture retained at " << scratch << '\n';
        return 1;
    }
}
