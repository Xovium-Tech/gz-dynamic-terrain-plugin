#pragma once

#include "dynamic_terrain/core/CollisionTerrain.hh"

namespace dynamic_terrain
{
std::string collisionSdf(const CollisionPatch &patch, std::uint64_t serial,
                         const std::string &version = "1.9",
                         const std::string &modelName = "");
}
