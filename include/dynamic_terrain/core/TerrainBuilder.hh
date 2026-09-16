#pragma once

#include "dynamic_terrain/core/TerrainData.hh"
#include "dynamic_terrain/core/TileStore.hh"

#include <opencv2/core.hpp>

#include <memory>
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
    explicit PersistentTerrainBuilder(std::shared_ptr<TileStore> store);

    std::shared_ptr<TerrainSnapshot> BuildBootstrap(const TileKey &center,
                                                    std::uint64_t generation,
                                                    std::string &error);
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
    };
    PageCacheStats CachedPageStats() const;

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
    static constexpr std::size_t kPageTextureCacheLimit = 24u;
};

}
