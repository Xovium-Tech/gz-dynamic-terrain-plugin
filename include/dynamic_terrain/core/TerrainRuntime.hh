#pragma once
#include "CollisionTerrain.hh"
#include "TerrainRenderSink.hh"
#include <functional>
#include <memory>
namespace dynamic_terrain
{
struct CollisionRequest
{
    TileKey center;
    std::uint64_t generation{0};
};

struct CollisionResult
{
    CollisionRequest request;
    std::optional<CollisionPatch> patch;
    std::string error;
};


// The adapter owns the sink and keeps it alive until this runtime is destroyed.
// Calls to Update/PollCollision come from one simulation thread; sinks are thread-safe.
class TerrainRuntime
{
public:
    using SnapshotCallback = std::function<void(std::shared_ptr<const TerrainSnapshot>)>;
    using TextureCallback = std::function<void(const TextureUpdate &)>;
    TerrainRuntime(Config config, std::shared_ptr<const GeographicTransform> geographic,
                   TerrainRenderSink &renderer, SnapshotCallback snapshot = {},
                   TextureCallback texture = {});
    ~TerrainRuntime();
    TerrainRuntime(const TerrainRuntime &) = delete;
    TerrainRuntime &operator=(const TerrainRuntime &) = delete;
    void UpdateVisual(const Vec3 &position, double latitude, double longitude, bool force = false);
    void UpdateCollision(double altitude, double latitude, double longitude, bool force = false);
    std::optional<CollisionResult> PollCollision(bool hasActiveCollision);
    bool VisualBuilding() const;
    bool RefinementActive() const;
    std::uint64_t LatestVisualGeneration() const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}
