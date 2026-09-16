#pragma once

#include "dynamic_terrain/core/TerrainData.hh"

#include <memory>
#include <optional>

namespace dynamic_terrain
{
// Rendering owns simulator resources; the runtime publishes immutable CPU data.
class TerrainRenderSink
{
public:
    virtual ~TerrainRenderSink() = default;
    virtual void QueueSnapshot(std::shared_ptr<const TerrainSnapshot> snapshot) = 0;
    virtual void QueueTexture(TextureUpdate update) = 0;
    virtual std::uint64_t ActiveGeneration() const = 0;
    virtual bool HasActiveTerrain() const = 0;
    virtual std::optional<TileKey> ActiveCenterTile() const = 0;
};
}
