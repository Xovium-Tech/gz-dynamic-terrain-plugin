#include "dynamic_terrain/adapters/CollisionSdf.hh"

#include <iostream>
#include <stdexcept>

int main()
{
    try
    {
        dynamic_terrain::CollisionPatch patch;
        patch.heightmap = "/tmp/terrain & elevation/height.png";
        patch.centerX = 10.0;
        patch.centerY = -20.0;
        patch.baseZ = 30.0;
        patch.yaw = 0.5;
        patch.sizeX = 200.0;
        patch.sizeY = 300.0;
        patch.sizeZ = 40.0;
        const auto text = dynamic_terrain::collisionSdf(patch, 42);
        for (const auto &required : {"<pose>10 -20 30 0 0 0.5</pose>",
                "file:///tmp/terrain &amp; elevation/height.png",
                "<size>200 300 40</size>", "<static>true</static>"})
        {
            if (text.find(required) == std::string::npos)
                throw std::runtime_error("missing collision geometry field");
        }
        if (text.find("version='1.9'") == std::string::npos ||
            text.find("name='dynamic_terrain_collision_42'") == std::string::npos)
            throw std::runtime_error("collision SDF version/name failed");
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
