#include "dynamic_terrain/adapters/classic/ClassicCollisionAdapter.hh"

#include <gazebo/common/MeshManager.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
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

int main()
{
    using namespace dynamic_terrain;
    const auto directory = fs::temp_directory_path() / ("terrain_classic_pool_" +
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    gazebo::physics::WorldPtr world;
    std::unique_ptr<ClassicCollisionAdapter> adapter;
    bool serverStarted = false;
    try
    {
        fs::create_directories(directory);
        const auto filename = directory / "pool.world";
        std::ofstream file(filename);
        file << "<sdf version='1.6'><world name='terrain_pool_test'>"
            "<physics type='ode'><max_step_size>0.005</max_step_size><real_time_update_rate>0</real_time_update_rate></physics>"
            "<gravity>0 0 -9.8</gravity><model name='probe'><pose>0 0 10 0 0 0</pose><link name='body'>"
            "<inertial><mass>1</mass><inertia><ixx>0.025</ixx><iyy>0.025</iyy><izz>0.025</izz></inertia></inertial>"
            "<collision name='sphere'><geometry><sphere><radius>0.25</radius></sphere></geometry></collision>"
            "</link></model></world></sdf>";
        file.close();
        const std::string master = "http://127.0.0.1:" + std::to_string(20000 + getpid() % 30000);
        setenv("GAZEBO_MASTER_URI", master.c_str(), 1);
        setenv("GAZEBO_MODEL_DATABASE_URI", "", 1);
        unsetenv("DISPLAY");
        CHECK(gazebo::setupServer());
        serverStarted = true;
        world = gazebo::loadWorld(filename.string());
        CHECK(world);
        world->_SetSensorsInitialized(true);
        world->SetPaused(false);
        adapter = std::make_unique<ClassicCollisionAdapter>(world, "pool", directory / "meshes");
        const auto probe = world->ModelByName("probe");
        CHECK(probe);
        CollisionPatch patch;
        patch.heightmap = directory / "samples.png";
        patch.sizeX = patch.sizeY = 20;
        patch.sizeZ = 8;
        std::uint64_t generation = 0;
        std::set<std::string> previous;
        for (const auto sample : {6000, 24000, 12000, 36000, 18000, 42000, 30000})
        {
            CHECK(cv::imwrite(patch.heightmap.string(), cv::Mat(3, 3, CV_16UC1, cv::Scalar(sample))));
            CHECK(adapter->Submit(patch, ++generation));
            bool ready = false;
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
            while (std::chrono::steady_clock::now() < deadline)
            {
                adapter->Update(world->SimTime().Double());
                gazebo::runWorld(world, 10);
                std::set<std::string> names;
                for (const auto &model : world->Models())
                    if (model->GetName().find("pool_collision_") == 0) names.insert(model->GetName());
                if (adapter->HasActive() && !adapter->HasPending() && names.size() == 1 && names != previous)
                {
                    previous = std::move(names);
                    ready = true;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
            CHECK(ready);
            const double surface = sample / 65535.0 * patch.sizeZ;
            probe->SetWorldPose(ignition::math::Pose3d(0, 0, surface + 1.0, 0, 0, 0));
            probe->SetLinearVel(ignition::math::Vector3d::Zero);
            probe->SetAngularVel(ignition::math::Vector3d::Zero);
            gazebo::runWorld(world, 300);
            CHECK(std::abs(probe->WorldPose().Pos().Z() - (surface + 0.25)) < 0.03);
        }
        // Hold retirement time while factory insertions finish, filling all
        // three slots. A newer waiting request must replace the older one.
        const double heldTime = world->SimTime().Double();
        for (const auto sample : {5000, 10000})
        {
            CHECK(cv::imwrite(patch.heightmap.string(), cv::Mat(3, 3, CV_16UC1, cv::Scalar(sample))));
            CHECK(adapter->Submit(patch, ++generation));
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
            while (adapter->HasPending() && std::chrono::steady_clock::now() < deadline)
            {
                adapter->Update(heldTime);
                gazebo::runWorld(world, 1);
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
            CHECK(!adapter->HasPending());
        }
        for (const auto sample : {20000, 40000})
        {
            CHECK(cv::imwrite(patch.heightmap.string(), cv::Mat(3, 3, CV_16UC1, cv::Scalar(sample))));
            CHECK(adapter->Submit(patch, ++generation));
            CHECK(adapter->HasPending());
        }
        const auto resumeDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (std::chrono::steady_clock::now() < resumeDeadline)
        {
            adapter->Update(world->SimTime().Double());
            gazebo::runWorld(world, 10);
            std::size_t live = 0;
            for (const auto &model : world->Models())
                if (model->GetName().find("pool_collision_") == 0) ++live;
            if (!adapter->HasPending() && live == 1) break;
        }
        CHECK(!adapter->HasPending());
        const double latestSurface = 40000.0 / 65535.0 * patch.sizeZ;
        probe->SetWorldPose(ignition::math::Pose3d(0, 0, latestSurface + 1.0, 0, 0, 0));
        probe->SetLinearVel(ignition::math::Vector3d::Zero);
        probe->SetAngularVel(ignition::math::Vector3d::Zero);
        gazebo::runWorld(world, 300);
        CHECK(std::abs(probe->WorldPose().Pos().Z() - (latestSurface + 0.25)) < 0.03);
        std::size_t meshFiles = 0;
        for (const auto &entry : fs::directory_iterator(directory / "meshes"))
        {
            if (entry.path().extension() != ".dae") continue;
            ++meshFiles;
            CHECK(gazebo::common::MeshManager::Instance()->HasMesh(entry.path().string()));
        }
        CHECK(meshFiles == 3);
        adapter->RemoveAll();
        adapter.reset();
        world.reset();
        CHECK(gazebo::shutdown());
        serverStarted = false;
        fs::remove_all(directory);
        std::cout << "Changing terrain heights, latest-request backpressure and three-slot collision cache passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        adapter.reset();
        world.reset();
        if (serverStarted) gazebo::shutdown();
        fs::remove_all(directory);
        std::cerr << error.what() << '\n';
        return 1;
    }
}
