#include "dynamic_terrain/core/CollisionTerrain.hh"

#include <opencv2/imgcodecs.hpp>

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>

#define CHECK(value) do { if (!(value)) throw std::runtime_error(#value); } while (false)

int main()
{
    using namespace dynamic_terrain;
    const auto path = fs::temp_directory_path() / ("terrain-collision-mesh-" +
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".png");
    try
    {
        cv::Mat heightmap(3, 3, CV_16UC1);
        for (int row = 0; row < 3; ++row)
            for (int col = 0; col < 3; ++col)
                heightmap.at<std::uint16_t>(row, col) = static_cast<std::uint16_t>(row * 10000 + col * 1000 + 257);
        CHECK(cv::imwrite(path.string(), heightmap));
        CollisionPatch patch;
        patch.heightmap = path;
        patch.sizeX = 40.0;
        patch.sizeY = 60.0;
        patch.sizeZ = 1000.0;
        patch.baseZ = -20.0;
        patch.yaw = 0.75;
        std::string error;
        const auto mesh = collisionPatchMesh(patch, error);
        CHECK(mesh && error.empty());
        CHECK(mesh->positions.size() == 9 && mesh->indices.size() == 24);
        CHECK(mesh->positions.front().X() == -20.0 && mesh->positions.front().Y() == 30.0);
        CHECK(mesh->positions.back().X() == 20.0 && mesh->positions.back().Y() == -30.0);
        for (std::size_t i = 0; i < mesh->positions.size(); ++i)
        {
            const double expected = heightmap.at<std::uint16_t>(static_cast<int>(i / 3),
                static_cast<int>(i % 3)) / 65535.0 * patch.sizeZ;
            CHECK(std::abs(mesh->positions[i].Z() - expected) < 1e-12);
        }
        for (std::size_t i = 0; i < mesh->indices.size(); i += 3)
        {
            const auto &a = mesh->positions.at(mesh->indices[i]);
            const auto &b = mesh->positions.at(mesh->indices[i + 1]);
            const auto &c = mesh->positions.at(mesh->indices[i + 2]);
            CHECK((b - a).Cross(c - a).Z() > 0.0);
        }
        fs::remove(path);
        std::cout << "Collision grid dimensions, 16-bit precision and winding passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        fs::remove(path);
        std::cerr << error.what() << '\n';
        return 1;
    }
}
