#include "dynamic_terrain/adapters/gzsim/ModelConfigRegistry.hh"

#include <algorithm>
#include <mutex>
#include <unordered_map>

namespace dynamic_terrain
{
namespace
{
std::unordered_map<gz::sim::Entity, Config> &modelConfigRegistry()
{
    static std::unordered_map<gz::sim::Entity, Config> registry;
    return registry;
}

std::mutex &modelConfigRegistryMutex()
{
    static std::mutex mutex;
    return mutex;
}
}

void registerModelConfig(gz::sim::Entity entity, const Config &cfg)
{
    std::lock_guard<std::mutex> lock(modelConfigRegistryMutex());
    modelConfigRegistry()[entity] = cfg;
}

void unregisterModelConfig(gz::sim::Entity entity)
{
    std::lock_guard<std::mutex> lock(modelConfigRegistryMutex());
    modelConfigRegistry().erase(entity);
}

std::vector<RegisteredModelConfig> registeredModelConfigs()
{
    std::vector<RegisteredModelConfig> result;
    std::lock_guard<std::mutex> lock(modelConfigRegistryMutex());
    result.reserve(modelConfigRegistry().size());
    for (const auto &[entity, cfg] : modelConfigRegistry())
        result.push_back({entity, cfg});
    std::sort(result.begin(), result.end(),
              [](const auto &a, const auto &b) { return a.entity < b.entity; });
    return result;
}

}
