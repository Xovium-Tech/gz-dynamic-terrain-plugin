#pragma once

#include "dynamic_terrain/core/TerrainConfig.hh"
#include <sdf/Element.hh>
#include <memory>

namespace dynamic_terrain
{
Config parseTerrainConfig(const std::shared_ptr<const sdf::Element> &sdf);
}
