#pragma once

#include "dynamic_terrain/core/TerrainRenderSink.hh"
#include <gazebo/rendering/RenderTypes.hh>
#include <memory>

namespace dynamic_terrain
{
// All Ogre1 mutations occur in Classic's pre/post-render callbacks. Transport
// callbacks only replace immutable CPU snapshots and merge texture queues.
class ClassicTerrainRenderer final : public TerrainRenderSink
{
public:
    explicit ClassicTerrainRenderer(gazebo::rendering::VisualPtr parent);
    ~ClassicTerrainRenderer() override;
    void QueueSnapshot(std::shared_ptr<const TerrainSnapshot> snapshot) override;
    void QueueSnapshot(std::shared_ptr<const TerrainSnapshot> snapshot, Config config);
    void QueueTexture(TextureUpdate update) override;
    std::uint64_t ActiveGeneration() const override;
    bool HasActiveTerrain() const override;
    std::optional<TileKey> ActiveCenterTile() const override;
private:
    class Impl;
    std::unique_ptr<Impl> data_;
};
}
