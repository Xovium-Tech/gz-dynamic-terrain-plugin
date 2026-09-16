#include "dynamic_terrain/core/TerrainBuilder.hh"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace dynamic_terrain
{
namespace
{
std::string terrainPageSubmeshName(const TileKey &key)
{
    return "terrain_page_z" + std::to_string(key.z) +
           "_x" + std::to_string(key.x) +
           "_y" + std::to_string(key.y);
}

double horizontalDistance(const Vec3 &a,
                          const Vec3 &b)
{
    const double dx = a.X() - b.X();
    const double dy = a.Y() - b.Y();
    return std::sqrt(dx * dx + dy * dy);
}

int floorLog2Positive(int value)
{
    int result = 0;
    while (value > 1)
    {
        value >>= 1;
        ++result;
    }
    return result;
}

}

PersistentTerrainBuilder::PersistentTerrainBuilder(std::shared_ptr<TileStore> store)
    : store_(std::move(store))
{
}

std::optional<PersistentTerrainBuilder::CachedPageTexture>
PersistentTerrainBuilder::CachedTextureForPage(const TileKey &key) const
{
    std::lock_guard<std::mutex> lock(pageTextureCacheMutex_);
    const auto it = pageTextureCache_.find(key);
    if (it == pageTextureCache_.end() || !it->second.texture ||
        !it->second.texture->Valid())
        return std::nullopt;
    it->second.touch = ++pageTextureCacheTouch_;
    return it->second;
}

void PersistentTerrainBuilder::RememberPageTexture(
    const TileKey &key, int imageryZoom, int textureSize,
    const std::shared_ptr<const ImageData> &texture,
    const std::string &textureName) const
{
    if (!texture || !texture->Valid())
        return;

    if (imageryZoom <= store_->GetConfig().visualGeometryZoom)
        return;

    const std::size_t limit = store_->GetConfig().visualPageCacheMb * 1024u * 1024u;
    const std::size_t bytes = texture->rgb.size();
    std::lock_guard<std::mutex> lock(pageTextureCacheMutex_);
    if (bytes > limit || limit == 0u)
    {
        const auto old = pageTextureCache_.find(key);
        if (old != pageTextureCache_.end())
        {
            pageTextureCacheBytes_ -= old->second.bytes;
            pageTextureCache_.erase(old);
        }
        return;
    }
    auto &entry = pageTextureCache_[key];
    if (!entry.texture || imageryZoom >= entry.imageryZoom)
    {
        entry.imageryZoom = imageryZoom;
        entry.textureSize = textureSize;
        pageTextureCacheBytes_ -= entry.bytes;
        entry.bytes = bytes;
        pageTextureCacheBytes_ += entry.bytes;
        entry.texture = texture;
        entry.textureName = textureName;
    }
    entry.touch = ++pageTextureCacheTouch_;

    while (pageTextureCache_.size() > kPageTextureCacheLimit ||
           pageTextureCacheBytes_ > limit)
    {
        auto victim = pageTextureCache_.end();
        for (auto it = pageTextureCache_.begin(); it != pageTextureCache_.end(); ++it)
        {
            if (victim == pageTextureCache_.end() ||
                it->second.touch < victim->second.touch)
                victim = it;
        }
        if (victim == pageTextureCache_.end())
            break;
        pageTextureCacheBytes_ -= victim->second.bytes;
        pageTextureCache_.erase(victim);
    }
}

PersistentTerrainBuilder::PageCacheStats
PersistentTerrainBuilder::CachedPageStats() const
{
    std::lock_guard<std::mutex> lock(pageTextureCacheMutex_);
    return {pageTextureCache_.size(), pageTextureCacheBytes_,
            store_->GetConfig().visualPageCacheMb * 1024u * 1024u};
}

TileKey PersistentTerrainBuilder::SnappedCenter(double latDeg, double lonDeg) const
{
    return latLonToTile(latDeg, lonDeg, store_->GetConfig().visualGeometryZoom);
}

int PersistentTerrainBuilder::RadiusTiles(const TileKey &center) const
{
    const auto &cfg = store_->GetConfig();
    const double reference = store_->Spherical().ElevationReference();
    const double centerLat = tileYToLat(center.y + 0.5, center.z);
    const double centerLon = tileXToLon(center.x + 0.5, center.z);
    const auto centerLocal = localFromGeodetic(
        store_->Spherical(), centerLat, centerLon, reference);

    for (int radius = 1; radius <= 32; ++radius)
    {
        const int count = 1 << center.z;
        const TileRect rect{
            clampValue(center.x - radius, 0, count - 1),
            clampValue(center.y - radius, 0, count - 1),
            clampValue(center.x + radius, 0, count - 1),
            clampValue(center.y + radius, 0, count - 1), center.z};
        const TileBounds b = tileRectBounds(rect);
        const auto west = localFromGeodetic(
            store_->Spherical(), centerLat, b.west, reference);
        const auto east = localFromGeodetic(
            store_->Spherical(), centerLat, b.east, reference);
        const auto north = localFromGeodetic(
            store_->Spherical(), b.north, centerLon, reference);
        const auto south = localFromGeodetic(
            store_->Spherical(), b.south, centerLon, reference);
        const double halfX = std::min(horizontalDistance(centerLocal, west),
                                      horizontalDistance(centerLocal, east));
        const double halfY = std::min(horizontalDistance(centerLocal, north),
                                      horizontalDistance(centerLocal, south));
        if (halfX >= cfg.visualRadiusM && halfY >= cfg.visualRadiusM)
            return radius;
    }
    return 32;
}

int PersistentTerrainBuilder::EffectiveCellsPerTile(const TileRect &rect) const
{
    const auto &cfg = store_->GetConfig();
    const int maxSideTiles = std::max(rect.Width(), rect.Height());
    int cells = cfg.visualMeshCellsPerTile;
    if (maxSideTiles * cells > cfg.visualMaxMeshCells)
    {
        cells = std::max(8, cfg.visualMaxMeshCells / std::max(1, maxSideTiles));
        if (cfg.diagnostics)
            logInfo("[DynamicTerrain][VISUAL] mesh cells reduced per_tile=",
                    cfg.visualMeshCellsPerTile, " -> ", cells,
                    " to stay within visual_max_mesh_cells=",
                    cfg.visualMaxMeshCells);
    }
    return cells;
}

std::vector<TileKey> PersistentTerrainBuilder::PageSourceKeys(
    const TileKey &pageKey, int sourceZoom) const
{
    std::vector<TileKey> result;
    if (sourceZoom <= pageKey.z)
    {
        const int shift = pageKey.z - sourceZoom;
        result.push_back({pageKey.x >> shift, pageKey.y >> shift, sourceZoom});
        return result;
    }

    const int delta = sourceZoom - pageKey.z;
    const int factor = 1 << delta;
    result.reserve(static_cast<std::size_t>(factor) * factor);
    const int baseX = pageKey.x * factor;
    const int baseY = pageKey.y * factor;
    for (int y = 0; y < factor; ++y)
        for (int x = 0; x < factor; ++x)
            result.push_back({baseX + x, baseY + y, sourceZoom});
    return result;
}

cv::Mat PersistentTerrainBuilder::ComposeBootstrapPage(
    const TileKey &pageKey, int geometryZoom, int bootstrapZoom,
    int outputSize) const
{
    outputSize = std::max(256, outputSize);
    cv::Mat output(outputSize, outputSize, CV_8UC3, cv::Scalar(96, 96, 96));

    if (bootstrapZoom > geometryZoom)
        bootstrapZoom = geometryZoom;
    const int shift = geometryZoom - bootstrapZoom;
    const TileKey source{pageKey.x >> shift, pageKey.y >> shift, bootstrapZoom};
    cv::Mat tile = store_->LoadImagery(source);
    if (tile.empty())
        return output;

    if (shift == 0)
    {
        cv::resize(tile, output, output.size(), 0.0, 0.0,
                   tile.cols > outputSize || tile.rows > outputSize ?
                       cv::INTER_AREA : cv::INTER_LINEAR);
        return output;
    }

    const int factor = 1 << shift;
    const int localX = pageKey.x - (source.x << shift);
    const int localY = pageKey.y - (source.y << shift);
    const int sx0 = localX * tile.cols / factor;
    const int sx1 = (localX + 1) * tile.cols / factor;
    const int sy0 = localY * tile.rows / factor;
    const int sy1 = (localY + 1) * tile.rows / factor;
    const int x0 = clampValue(sx0, 0, tile.cols - 1);
    const int x1 = clampValue(sx1, x0 + 1, tile.cols);
    const int y0 = clampValue(sy0, 0, tile.rows - 1);
    const int y1 = clampValue(sy1, y0 + 1, tile.rows);
    const cv::Rect src{x0, y0, x1 - x0, y1 - y0};
    cv::resize(tile(src), output, output.size(), 0.0, 0.0, cv::INTER_LINEAR);
    return output;
}

int PersistentTerrainBuilder::PageTargetZoom(
    const TerrainSnapshot &snapshot, const TileKey &pageKey) const
{
    const auto &cfg = store_->GetConfig();
    const double reference = store_->Spherical().ElevationReference();
    const double lat = tileYToLat(pageKey.y + 0.5, pageKey.z);
    const double lon = tileXToLon(pageKey.x + 0.5, pageKey.z);
    const auto local = localFromGeodetic(
        store_->Spherical(), lat, lon, reference);
    const TileBounds tb = tileRectBounds(
        {pageKey.x, pageKey.y, pageKey.x, pageKey.y, pageKey.z});
    const auto corner = localFromGeodetic(
        store_->Spherical(), tb.north, tb.west, reference);
    const double pagePad = horizontalDistance(local, corner);
    const double distance = horizontalDistance(local, snapshot.patchCenterLocal);

    int target = snapshot.geometryRect.zoom;
    if (cfg.visualDetailMode == "bottom_camera_only")
    {
        if (distance <= cfg.visualDetailRadiusM + pagePad)
            target = cfg.visualDetailZoom;
    }
    else
    {
        for (const auto &[ringDistance, zoom] : cfg.visualImageryLodTable)
        {
            const double protectedDistance = ringDistance +
                cfg.visualRecenterDistanceM + cfg.visualTextureGuardM + pagePad;
            if (distance <= protectedDistance)
            {
                target = zoom;
                break;
            }
        }
    }

    target = std::max(target, snapshot.geometryRect.zoom);
    const int maxScale = std::max(1, cfg.visualPageTextureMaxSize / 256);
    const int maxDelta = floorLog2Positive(maxScale);
    target = std::min(target, snapshot.geometryRect.zoom + maxDelta);
    return clampValue(target, snapshot.geometryRect.zoom, 20);
}

bool PersistentTerrainBuilder::SourceTileWanted(
    const TerrainSnapshot &snapshot, const TileKey &key) const
{
    const auto &cfg = store_->GetConfig();
    if (cfg.visualDetailMode != "bottom_camera_only" ||
        key.z <= snapshot.geometryRect.zoom)
        return true;

    const int delta = key.z - snapshot.geometryRect.zoom;
    const TileKey parent{key.x >> delta, key.y >> delta,
                         snapshot.geometryRect.zoom};
    return PageTargetZoom(snapshot, parent) >= key.z;
}

cv::Mat PersistentTerrainBuilder::ComposeRefinedPage(
    const TileKey &pageKey,
    int geometryZoom,
    int sourceZoom,
    int outputSize,
    const std::unordered_set<TileKey, TileKeyHash> &failed) const
{
    sourceZoom = std::max(sourceZoom, geometryZoom);
    outputSize = std::max(256, outputSize);

    cv::Mat output = ComposeBootstrapPage(
        pageKey, geometryZoom,
        store_->GetConfig().visualBootstrapImageryZoom,
        outputSize);

    for (int zoom = geometryZoom; zoom <= sourceZoom; ++zoom)
    {
        const int levelDelta = zoom - geometryZoom;
        const int factor = 1 << levelDelta;
        if (outputSize % factor != 0)
            continue;
        const int regionPixels = outputSize / factor;
        const int baseX = pageKey.x * factor;
        const int baseY = pageKey.y * factor;
        for (int y = 0; y < factor; ++y)
        {
            for (int x = 0; x < factor; ++x)
            {
                const TileKey key{baseX + x, baseY + y, zoom};
                if (zoom == sourceZoom && failed.count(key))
                    continue;
                cv::Mat tile = store_->LoadImagery(key);
                if (tile.empty())
                    continue;

                cv::Mat normalized;
                if (tile.cols == regionPixels && tile.rows == regionPixels)
                    normalized = tile;
                else
                    cv::resize(tile, normalized,
                               cv::Size(regionPixels, regionPixels), 0.0, 0.0,
                               tile.cols > regionPixels || tile.rows > regionPixels ?
                                   cv::INTER_AREA : cv::INTER_LINEAR);
                normalized.copyTo(output(cv::Rect(
                    x * regionPixels, y * regionPixels,
                    regionPixels, regionPixels)));
            }
        }
    }
    return output;
}

std::shared_ptr<TerrainSnapshot> PersistentTerrainBuilder::BuildBootstrap(
    const TileKey &center, std::uint64_t generation, std::string &error)
{
    const auto &cfg = store_->GetConfig();
    const auto started = std::chrono::steady_clock::now();
    const int radius = RadiusTiles(center);
    const int count = 1 << center.z;
    TileRect rect{
        clampValue(center.x - radius, 0, count - 1),
        clampValue(center.y - radius, 0, count - 1),
        clampValue(center.x + radius, 0, count - 1),
        clampValue(center.y + radius, 0, count - 1), center.z};
    const TileBounds bounds = tileRectBounds(rect);

    if (cfg.diagnostics)
        logInfo("[DynamicTerrain][VISUAL] build generation=", generation,
                " center=", tileText(center),
                " rect=", rect.Width(), "x", rect.Height(),
                " geometry_zoom=", center.z,
                " elevation_zoom=", cfg.visualElevationZoom,
                " radius~=", cfg.visualRadiusM, "m mode=paged-textures");

    auto dem = store_->BuildElevationMosaic(
        bounds, cfg.visualElevationZoom, 1, error);
    if (!dem)
        return {};
    const double elevationOffset =
        store_->ElevationAlignmentOffset(cfg.visualElevationZoom) + cfg.zOffsetM;

    auto snapshot = std::make_shared<TerrainSnapshot>();
    snapshot->generation = generation;
    snapshot->centerTile = center;
    snapshot->geometryRect = rect;
    snapshot->bounds = bounds;
    snapshot->cellsPerTile = EffectiveCellsPerTile(rect);
    snapshot->cellsX = rect.Width() * snapshot->cellsPerTile;
    snapshot->cellsY = rect.Height() * snapshot->cellsPerTile;

    snapshot->resourcePrefix = store_->ResourcePrefix();

    const double centerLat = tileYToLat(center.y + 0.5, center.z);
    const double centerLon = tileXToLon(center.x + 0.5, center.z);
    snapshot->patchCenterLocal = localFromGeodetic(
        store_->Spherical(), centerLat, centerLon,
        store_->Spherical().ElevationReference());

    const int vertexCols = snapshot->cellsX + 1;
    const int vertexRows = snapshot->cellsY + 1;
    const std::size_t vertexCount = static_cast<std::size_t>(vertexCols) * vertexRows;
    if (vertexCount > 4'000'000u)
    {
        error = "visual mesh exceeds 4 million vertices; reduce radius or mesh cells";
        return {};
    }

    std::vector<Vec3> vertices(vertexCount);
    for (int y = 0; y < vertexRows; ++y)
    {
        const double tileY = static_cast<double>(rect.minY) +
            static_cast<double>(y) / snapshot->cellsPerTile;
        const double lat = tileYToLat(tileY, rect.zoom);
        for (int x = 0; x < vertexCols; ++x)
        {
            const double tileX = static_cast<double>(rect.minX) +
                static_cast<double>(x) / snapshot->cellsPerTile;
            const double lon = tileXToLon(tileX, rect.zoom);
            const double elevation = dem->Sample(lat, lon) + elevationOffset;
            vertices[static_cast<std::size_t>(y) * vertexCols + x] =
                localFromGeodetic(store_->Spherical(), lat, lon, elevation);
        }
    }

    auto vertexAt = [&](int x, int y) -> const Vec3 &
    {
        return vertices[static_cast<std::size_t>(y) * vertexCols + x];
    };
    std::vector<Vec3> normals(vertexCount);
    for (int y = 0; y < vertexRows; ++y)
    {
        for (int x = 0; x < vertexCols; ++x)
        {
            const auto &left = vertexAt(std::max(0, x - 1), y);
            const auto &right = vertexAt(std::min(vertexCols - 1, x + 1), y);
            const auto &north = vertexAt(x, std::max(0, y - 1));
            const auto &south = vertexAt(x, std::min(vertexRows - 1, y + 1));
            Vec3 eastTangent = right - left;
            Vec3 northTangent = north - south;
            Vec3 normal = eastTangent.Cross(northTangent);
            if (normal.Length() < 1e-9)
                normal = Vec3{0.0, 0.0, 1.0};
            else
                normal.Normalize();
            if (normal.Z() < 0.0)
                normal *= -1.0;
            normals[static_cast<std::size_t>(y) * vertexCols + x] = normal;
        }
    }

    const int bootstrapZoom = std::min(cfg.visualBootstrapImageryZoom, rect.zoom);
    std::unordered_set<TileKey, TileKeyHash> bootstrapSet;
    for (int ty = rect.minY; ty <= rect.maxY; ++ty)
        for (int tx = rect.minX; tx <= rect.maxX; ++tx)
            for (const auto &key : PageSourceKeys({tx, ty, rect.zoom}, bootstrapZoom))
                bootstrapSet.insert(key);
    std::vector<TileKey> bootstrapKeys(bootstrapSet.begin(), bootstrapSet.end());
    std::vector<TileKey> bootstrapFailed;
    store_->EnsureImagery(bootstrapKeys, &bootstrapFailed);

    snapshot->pages.reserve(static_cast<std::size_t>(rect.Width()) * rect.Height());
    std::size_t carriedPageTextures = 0;
    std::size_t pageIndex = 0;
    for (int ty = rect.minY; ty <= rect.maxY; ++ty)
    {
        for (int tx = rect.minX; tx <= rect.maxX; ++tx, ++pageIndex)
        {
            const TileKey pageKey{tx, ty, rect.zoom};
            const std::string submeshName = terrainPageSubmeshName(pageKey);

            TerrainPage page;
            page.key = pageKey;
            page.index = pageIndex;
            page.submeshName = submeshName;
            page.imageryZoom = bootstrapZoom;
            page.targetImageryZoom = PageTargetZoom(*snapshot, page.key);

            const auto cached = CachedTextureForPage(page.key);
            if (cached)
                page.targetImageryZoom = std::max(page.targetImageryZoom,
                                                  cached->imageryZoom);

            page.textureSize = 256 <<
                (page.targetImageryZoom - snapshot->geometryRect.zoom);
            page.textureSize = std::min(page.textureSize,
                                        cfg.visualPageTextureMaxSize);

            if (cached && cached->textureSize == page.textureSize &&
                cached->imageryZoom >= bootstrapZoom)
            {
                page.imageryZoom = cached->imageryZoom;
                page.texture = cached->texture;
                page.textureName = cached->textureName;
                ++carriedPageTextures;
            }
            else
            {
                const auto raster = ComposeBootstrapPage(
                    page.key, rect.zoom, bootstrapZoom, page.textureSize);
                page.texture = ToImage(raster);
                page.textureName = snapshot->resourcePrefix + "_page_z" +
                    std::to_string(page.key.z) + "_x" +
                    std::to_string(page.key.x) + "_y" +
                    std::to_string(page.key.y) + "_q" +
                    std::to_string(bootstrapZoom) + "_s" +
                    std::to_string(page.textureSize);
            }
            if (!page.texture || !page.texture->Valid())
            {
                error = "failed to create bootstrap page texture";
                return {};
            }

            auto mesh = std::make_shared<MeshData>();
            mesh->name = snapshot->resourcePrefix + "_mesh_g" +
                std::to_string(generation) + "_z" +
                std::to_string(page.key.z) + "_x" +
                std::to_string(page.key.x) + "_y" +
                std::to_string(page.key.y);
            mesh->submeshName = submeshName;
            const int baseGridX = (tx - rect.minX) * snapshot->cellsPerTile;
            const int baseGridY = (ty - rect.minY) * snapshot->cellsPerTile;
            const int pageVertexCols = snapshot->cellsPerTile + 1;
            for (int py = 0; py <= snapshot->cellsPerTile; ++py)
            {
                for (int px = 0; px <= snapshot->cellsPerTile; ++px)
                {
                    const int gx = baseGridX + px;
                    const int gy = baseGridY + py;
                    const std::size_t globalIndex =
                        static_cast<std::size_t>(gy) * vertexCols + gx;
                    mesh->positions.push_back(vertices[globalIndex]);
                    mesh->normals.push_back(normals[globalIndex]);
                    mesh->texCoords.emplace_back(
                        static_cast<double>(px) / snapshot->cellsPerTile,
                        static_cast<double>(py) / snapshot->cellsPerTile);
                }
            }
            for (int py = 0; py < snapshot->cellsPerTile; ++py)
            {
                for (int px = 0; px < snapshot->cellsPerTile; ++px)
                {
                    const unsigned int a = static_cast<unsigned int>(
                        py * pageVertexCols + px);
                    const unsigned int b = a + 1u;
                    const unsigned int c = static_cast<unsigned int>(
                        (py + 1) * pageVertexCols + px);
                    const unsigned int d = c + 1u;
                    mesh->indices.push_back(a);
                    mesh->indices.push_back(c);
                    mesh->indices.push_back(b);
                    mesh->indices.push_back(b);
                    mesh->indices.push_back(c);
                    mesh->indices.push_back(d);
                }
            }
            page.mesh = std::move(mesh);
            snapshot->estimatedTextureBytes += mipmappedRgbaBytes(
                page.textureSize, page.textureSize);
            snapshot->pages.push_back(std::move(page));
        }
    }

    if (cfg.diagnostics)
    {
        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - started).count();
        logInfo("[DynamicTerrain][VISUAL] bootstrap ready generation=", generation,
                " vertices=", vertexCount,
                " triangles=", static_cast<std::size_t>(snapshot->cellsX) *
                    snapshot->cellsY * 2u,
                " pages=", snapshot->pages.size(),
                " bootstrap_z=", bootstrapZoom,
                " texture_grid=stable-per-page",
                " max_page_texture=", cfg.visualPageTextureMaxSize,
                " gpu_layout=independent-geographic-pages",
                " estimated_resident_texture_mib=",
                static_cast<double>(snapshot->estimatedTextureBytes) /
                    (1024.0 * 1024.0),
                " carried_exact_pages=", carriedPageTextures,
                " time=", elapsed, "s");
    }
    return snapshot;
}

std::vector<std::size_t> PersistentTerrainBuilder::ProgressivePageOrder(
    const TerrainSnapshot &snapshot) const
{
    std::vector<std::size_t> order(snapshot.pages.size());
    for (std::size_t i = 0; i < order.size(); ++i)
        order[i] = i;

    const double reference = store_->Spherical().ElevationReference();
    std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b)
    {
        auto distanceFor = [&](std::size_t index)
        {
            const auto &page = snapshot.pages[index];
            const double lat = tileYToLat(page.key.y + 0.5, page.key.z);
            const double lon = tileXToLon(page.key.x + 0.5, page.key.z);
            const auto local = localFromGeodetic(
                store_->Spherical(), lat, lon, reference);
            return horizontalDistance(local, snapshot.patchCenterLocal);
        };
        const double da = distanceFor(a);
        const double db = distanceFor(b);
        if (std::abs(da - db) > 1e-6)
            return da < db;
        return a < b;
    });
    return order;
}

int PersistentTerrainBuilder::TargetZoomForPage(
    const TerrainSnapshot &snapshot, std::size_t pageIndex) const
{
    if (pageIndex >= snapshot.pages.size())
        return snapshot.geometryRect.zoom;
    return PageTargetZoom(snapshot, snapshot.pages[pageIndex].key);
}

std::optional<TextureUpdate> PersistentTerrainBuilder::BuildTextureStage(
    const TerrainSnapshot &snapshot,
    const std::vector<std::size_t> &pageIndices,
    int sourceZoom,
    std::string &error)
{
    if (!store_->GetConfig().visualRefineTexture || pageIndices.empty())
        return std::nullopt;

    sourceZoom = clampValue(sourceZoom, snapshot.geometryRect.zoom, 20);
    std::unordered_set<TileKey, TileKeyHash> requestSet;
    struct Plan
    {
        std::size_t pageIndex{0};
        std::vector<TileKey> keys;
    };
    std::vector<Plan> plans;
    plans.reserve(pageIndices.size());

    for (const std::size_t pageIndex : pageIndices)
    {
        if (pageIndex >= snapshot.pages.size())
            continue;
        const auto &page = snapshot.pages[pageIndex];
        if (sourceZoom > PageTargetZoom(snapshot, page.key))
            continue;
        Plan plan;
        plan.pageIndex = pageIndex;
        for (const auto &key : PageSourceKeys(page.key, sourceZoom))
        {
            if (SourceTileWanted(snapshot, key))
            {
                plan.keys.push_back(key);
                requestSet.insert(key);
            }
        }
        if (!plan.keys.empty())
            plans.push_back(std::move(plan));
    }
    if (plans.empty())
        return std::nullopt;

    std::vector<TileKey> requests(requestSet.begin(), requestSet.end());
    std::vector<TileKey> failed;
    store_->EnsureImagery(requests, &failed);
    const std::unordered_set<TileKey, TileKeyHash> failedSet(
        failed.begin(), failed.end());

    TextureUpdate update;
    update.generation = snapshot.generation;
    update.pages.reserve(plans.size());
    std::size_t partialPages = 0;
    std::size_t fullyFailedPages = 0;

    for (const auto &plan : plans)
    {
        std::size_t failedKeys = 0;
        for (const auto &key : plan.keys)
            if (failedSet.count(key))
                ++failedKeys;

        if (failedKeys == plan.keys.size() && !plan.keys.empty())
        {
            ++fullyFailedPages;
            continue;
        }
        if (failedKeys > 0)
        {
            ++partialPages;
            continue;
        }

        const auto &page = snapshot.pages[plan.pageIndex];
        cv::Mat texture = ComposeRefinedPage(
            page.key, snapshot.geometryRect.zoom, sourceZoom,
            page.textureSize, failedSet);
        auto image = ToImage(texture);
        if (!image || !image->Valid())
            continue;

        TexturePageUpdate pageUpdate;
        pageUpdate.pageIndex = plan.pageIndex;
        pageUpdate.pageKey = page.key;
        pageUpdate.submeshName = page.submeshName;
        pageUpdate.imageryZoom = sourceZoom;
        pageUpdate.textureSize = page.textureSize;
        pageUpdate.texture = std::move(image);
        pageUpdate.textureName = snapshot.resourcePrefix + "_page_z" +
            std::to_string(page.key.z) + "_x" + std::to_string(page.key.x) +
            "_y" + std::to_string(page.key.y) + "_q" +
            std::to_string(sourceZoom) + "_s" + std::to_string(page.textureSize);
        RememberPageTexture(page.key, sourceZoom, page.textureSize,
                            pageUpdate.texture,
                            pageUpdate.textureName);
        update.pages.push_back(std::move(pageUpdate));
    }

    if (partialPages > 0 || fullyFailedPages > 0)
    {
        error = std::to_string(partialPages) + " partial pages, " +
            std::to_string(fullyFailedPages) +
            " pages kept at previous quality";
    }

    if (store_->GetConfig().diagnostics)
    {
        const auto cache = CachedPageStats();
        logInfo("[DynamicTerrain][TEXTURE] progressive stage generation=",
                snapshot.generation,
                " z=", sourceZoom,
                " pages_requested=", plans.size(),
                " pages_ready=", update.pages.size(),
                " source_tiles=", requests.size(),
                " failed_tiles=", failed.size(),
                " partial_pages=", partialPages,
                " kept_previous=", fullyFailedPages,
                " cpu_cache_pages=", cache.pages,
                " cpu_cache_bytes=", cache.bytes,
                " cpu_cache_limit_bytes=", cache.limitBytes);
    }

    if (update.pages.empty())
        return std::nullopt;
    update.changedPageCount = update.pages.size();
    return update;
}

void PersistentTerrainBuilder::ApplyTextureUpdate(
    TerrainSnapshot &snapshot, TextureUpdate &update) const
{
    if (snapshot.generation != update.generation)
        return;
    for (const auto &pageUpdate : update.pages)
    {
        if (pageUpdate.pageIndex >= snapshot.pages.size())
            continue;
        auto &page = snapshot.pages[pageUpdate.pageIndex];
        if (page.key != pageUpdate.pageKey ||
            page.submeshName != pageUpdate.submeshName)
        {
            logError("[DynamicTerrain][MAPPING] rejected snapshot texture update index=",
                     pageUpdate.pageIndex, " expected=", tileText(page.key),
                     " incoming=", tileText(pageUpdate.pageKey),
                     " expected_submesh=", page.submeshName,
                     " incoming_submesh=", pageUpdate.submeshName);
            continue;
        }
        if (pageUpdate.imageryZoom < page.imageryZoom)
            continue;
        if (pageUpdate.textureSize != page.textureSize ||
            !pageUpdate.texture || !pageUpdate.texture->Valid())
            continue;
        page.imageryZoom = pageUpdate.imageryZoom;
        page.texture = pageUpdate.texture;
        page.textureName = pageUpdate.textureName;
    }
}

std::shared_ptr<const ImageData> PersistentTerrainBuilder::ToImage(
    const cv::Mat &bgr) const
{
    if (bgr.empty() || bgr.type() != CV_8UC3)
        return {};

    cv::Mat rgb;
    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
    if (!rgb.isContinuous())
        rgb = rgb.clone();

    auto image = std::make_shared<ImageData>();
    image->width = static_cast<std::uint32_t>(rgb.cols);
    image->height = static_cast<std::uint32_t>(rgb.rows);
    image->rgb.assign(rgb.data, rgb.data + rgb.total() * rgb.elemSize());
    return image;
}

}
