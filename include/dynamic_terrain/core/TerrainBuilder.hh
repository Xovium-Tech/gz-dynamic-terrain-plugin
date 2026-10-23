#pragma once

#include "dynamic_terrain/core/TerrainData.hh"
#include "dynamic_terrain/core/TileStore.hh"

#include <opencv2/core.hpp>

#include <memory>
#include <functional>
#include <mutex>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace dynamic_terrain
{
class PersistentTerrainBuilder
{
public:
    // Optional initial-load publication. Each call exposes a cumulative prefix
    // of immutable pages; returning false cancels without discarding that prefix.
    // Empty keeps the existing atomic build path used by other adapters.
    using ProgressCallback = std::function<bool(const TerrainSnapshot &)>;
    explicit PersistentTerrainBuilder(std::shared_ptr<TileStore> store);

    std::shared_ptr<TerrainSnapshot> BuildBootstrap(const TileKey &center,
                                                    std::uint64_t generation,
                                                    std::string &error,
                                                    const ProgressCallback &progress = {});
    // The initial protected near patch can activate first. This worker-only
    // operation adds a complete visual-only outer partition without modifying
    // any protected near geometry, imagery, normals or collision behavior.
    std::shared_ptr<TerrainSnapshot> BuildOuterCoverage(
        const TerrainSnapshot &near, std::string &error,
        const ProgressCallback &progress = {});
    std::optional<TextureUpdate> BuildTextureStage(
        const TerrainSnapshot &snapshot,
        const std::vector<std::size_t> &pageIndices,
        int sourceZoom,
        std::string &error);

    std::vector<std::size_t> ProgressivePageOrder(
        const TerrainSnapshot &snapshot) const;
    int TargetZoomForPage(const TerrainSnapshot &snapshot,
                          std::size_t pageIndex) const;

    void ApplyTextureUpdate(TerrainSnapshot &snapshot,
                            TextureUpdate &update) const;

    TileKey SnappedCenter(double latDeg, double lonDeg) const;

    struct PageCacheStats
    {
        std::size_t pages{0};
        std::size_t bytes{0};
        std::size_t limitBytes{0};
        std::uint64_t hits{0};
        std::uint64_t misses{0};
    };
    PageCacheStats CachedPageStats() const;
    PageCacheStats CachedOuterMeshStats() const;

    // Pin only the current generation's already-owned images. Overlapping pages
    // must not lose detail just because the disposable LRU is full or disabled.
    void PinActiveTextures(const TerrainSnapshot &snapshot);

private:
    struct CachedPageTexture
    {
        int imageryZoom{0};
        int textureSize{256};
        std::shared_ptr<const ImageData> texture;
        std::string textureName;
        std::uint64_t touch{0};
        std::size_t bytes{0};
    };

    std::optional<CachedPageTexture> CachedTextureForPage(
        const TileKey &key) const;
    void RememberPageTexture(const TileKey &key, int imageryZoom,
                             int textureSize,
                             const std::shared_ptr<const ImageData> &texture,
                             const std::string &textureName) const;

    int RadiusTiles(const TileKey &center) const;
    int EffectiveCellsPerTile(const TileRect &rect) const;
    int PageTargetZoom(const TerrainSnapshot &snapshot,
                       const TileKey &pageKey) const;
    bool SourceTileWanted(const TerrainSnapshot &snapshot,
                          const TileKey &key) const;
    std::vector<TileKey> PageSourceKeys(const TileKey &pageKey,
                                        int sourceZoom) const;
    cv::Mat ComposeBootstrapPage(const TileKey &pageKey,
                                 int geometryZoom,
                                 int bootstrapZoom,
                                 int outputSize) const;
    cv::Mat ComposeRefinedPage(const TileKey &pageKey,
                               int geometryZoom,
                               int sourceZoom,
                               int outputSize,
                               const std::unordered_set<TileKey, TileKeyHash> &failed) const;
    std::shared_ptr<const ImageData> ToImage(const cv::Mat &bgr) const;

    std::shared_ptr<TileStore> store_;

    mutable std::mutex pageTextureCacheMutex_;
    mutable std::unordered_map<TileKey, CachedPageTexture, TileKeyHash>
        pageTextureCache_;
    mutable std::uint64_t pageTextureCacheTouch_{0};
    mutable std::size_t pageTextureCacheBytes_{0};
    std::uint64_t activeTextureGeneration_{0};
    std::uint64_t activeTextureCoverageRevision_{0};
    std::unordered_map<TileKey, CachedPageTexture, TileKeyHash> activeTextures_;

    struct CachedOuterMesh
    {
        std::shared_ptr<const MeshData> mesh;
        std::size_t bytes{0};
        std::uint64_t touch{0};
    };
    mutable std::mutex outerMeshCacheMutex_;
    std::unordered_map<std::string, CachedOuterMesh> outerMeshCache_;
    std::size_t outerMeshCacheBytes_{0};
    std::uint64_t outerMeshCacheTouch_{0};
    std::uint64_t outerMeshSerial_{0};
    std::uint64_t outerMeshCacheHits_{0};
    std::uint64_t outerMeshCacheMisses_{0};
};

}
