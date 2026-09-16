#include "dynamic_terrain/adapters/CollisionSdf.hh"

#include <iomanip>
#include <sstream>

namespace dynamic_terrain
{
std::string collisionSdf(const CollisionPatch &patch,
                         std::uint64_t serial, const std::string &version,
                         const std::string &modelName)
{
    std::ostringstream out;
    out << std::setprecision(17)
        << "<sdf version='" << xmlEscape(version) << "'>"
        << "<model name='" << xmlEscape(modelName.empty()
            ? "dynamic_terrain_collision_" + std::to_string(serial) : modelName) << "'>"
        << "<static>true</static><link name='ground'>"
        << "<collision name='collision'><pose>"
        << patch.centerX << ' ' << patch.centerY << ' ' << patch.baseZ
        << " 0 0 " << patch.yaw << "</pose><geometry><heightmap><uri>"
        << xmlEscape("file://" + patch.heightmap.string())
        << "</uri><size>" << patch.sizeX << ' ' << patch.sizeY << ' '
        << patch.sizeZ
        << "</size><pos>0 0 0</pos><sampling>1</sampling>"
        << "</heightmap></geometry></collision>"
        << "</link></model></sdf>";
    return out.str();
}

}
