#pragma once

#include "dynamic_terrain/core/TerrainRenderSink.hh"

#include <memory>
#include <string>

namespace dynamic_terrain
{
// Publishes the shared runtime's output to Classic sensor/client render scenes.
// Generation state describes the published terrain, so headless collision and
// streaming never wait for a GUI or a render sensor to connect.
class ClassicTerrainSource final : public TerrainRenderSink
{
public:
    ClassicTerrainSource(Config config, const std::string &worldName,
                         const std::string &topic);
    ~ClassicTerrainSource() override;
    void QueueSnapshot(std::shared_ptr<const TerrainSnapshot> snapshot) override;
    void QueueTexture(TextureUpdate update) override;
    std::uint64_t ActiveGeneration() const override;
    bool HasActiveTerrain() const override;
    std::optional<TileKey> ActiveCenterTile() const override;
private:
    class Impl;
    std::unique_ptr<Impl> data_;
};
}
