#pragma once

#include "dynamic_terrain/core/TerrainConfig.hh"
#include <gz/sim/Entity.hh>
#include <vector>

namespace dynamic_terrain
{
struct RegisteredModelConfig
{
    gz::sim::Entity entity{gz::sim::kNullEntity};
    Config config;
};

void registerModelConfig(gz::sim::Entity entity, const Config &cfg);
void unregisterModelConfig(gz::sim::Entity entity);
std::vector<RegisteredModelConfig> registeredModelConfigs();

}
