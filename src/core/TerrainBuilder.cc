#include "dynamic_terrain/core/TerrainBuilder.hh"
#include "dynamic_terrain/core/TerrainCoverage.hh"

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
    const auto active = activeTextures_.find(key);
    if (active != activeTextures_.end() &&
        (it == pageTextureCache_.end() ||
         active->second.imageryZoom >= it->second.imageryZoom))
        return active->second;
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

    while (pageTextureCacheBytes_ > limit)
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

void PersistentTerrainBuilder::PinActiveTextures(const TerrainSnapshot &snapshot)
{
    std::lock_guard<std::mutex> lock(pageTextureCacheMutex_);
    if (snapshot.generation < activeTextureGeneration_ ||
        (snapshot.generation == activeTextureGeneration_ &&
         snapshot.coverageRevision < activeTextureCoverageRevision_))
        return;
    activeTextureGeneration_ = snapshot.generation;
    activeTextureCoverageRevision_ = snapshot.coverageRevision;
    std::unordered_map<TileKey, CachedPageTexture, TileKeyHash> next;
    for (const auto &page : snapshot.pages)
    {
        if (!page.texture || !page.texture->Valid())
            continue;
        const auto old = activeTextures_.find(page.key);
        // The builder belongs to one immutable provider configuration. Keep
        // resident detail across recenter generations as well as outer appends.
        if (old != activeTextures_.end() &&
            old->second.imageryZoom >= page.imageryZoom &&
            old->second.textureSize >= page.textureSize)
        {
            next.emplace(page.key, old->second);
            continue;
        }
        next.emplace(page.key, CachedPageTexture{
            page.imageryZoom, page.textureSize, page.texture, page.textureName,
            0, page.texture->rgb.size()});
    }
    activeTextures_ = std::move(next);
}

PersistentTerrainBuilder::PageCacheStats
PersistentTerrainBuilder::CachedPageStats() const
{
    std::lock_guard<std::mutex> lock(pageTextureCacheMutex_);
    return {pageTextureCache_.size(), pageTextureCacheBytes_,
            store_->GetConfig().visualPageCacheMb * 1024u * 1024u};
}

PersistentTerrainBuilder::PageCacheStats
PersistentTerrainBuilder::CachedOuterMeshStats() const
{
    std::lock_guard<std::mutex> lock(outerMeshCacheMutex_);
    return {outerMeshCache_.size(), outerMeshCacheBytes_,
        store_->GetConfig().visualOuterMeshCacheMb * 1024u * 1024u,
        outerMeshCacheHits_, outerMeshCacheMisses_};
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
    const TileKey &center, std::uint64_t generation, std::string &error,
    const ProgressCallback &progress)
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

    std::optional<ElevationMosaic> dem;
    if (!progress)
    {
        dem = store_->BuildElevationMosaic(bounds, cfg.visualElevationZoom, 1, error);
        if (!dem) return {};
    }
    const double elevationOffset =
        store_->ElevationAlignmentOffset(cfg.visualElevationZoom) + cfg.zOffsetM;

    auto snapshot = std::make_shared<TerrainSnapshot>();
    snapshot->generation = generation;
    snapshot->progressiveInitial = static_cast<bool>(progress);
    snapshot->progressiveComplete = !progress;
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
    std::vector<double> longitudes(vertexCols);
    for (int x = 0; x < vertexCols; ++x)
        longitudes[x] = tileXToLon(static_cast<double>(rect.minX) +
            static_cast<double>(x) / snapshot->cellsPerTile, rect.zoom);
    // Global coordinates and the original finite-difference normal stencil are
    // shared across pages. Initial streaming only evaluates the needed page and
    // one vertex of halo; it does not change any sample or boundary calculation.
    std::vector<bool> evaluated(progress ? vertexCount : 0, false);
    auto evaluate = [&](const ElevationMosaic &mosaic,
                        int minX, int minY, int maxX, int maxY)
    {
        for (int y = minY; y <= maxY; ++y)
        {
            if (ScopedDownloadContext::Cancelled()) return false;
            const double tileY = static_cast<double>(rect.minY) +
                static_cast<double>(y) / snapshot->cellsPerTile;
            const double lat = tileYToLat(tileY, rect.zoom);
            for (int x = minX; x <= maxX; ++x)
            {
                const auto index = static_cast<std::size_t>(y) * vertexCols + x;
                if (progress && evaluated[index]) continue;
                const double lon = longitudes[x];
                const double elevation = mosaic.Sample(lat, lon) + elevationOffset;
                vertices[index] = localFromGeodetic(store_->Spherical(), lat, lon, elevation);
                if (progress) evaluated[index] = true;
            }
        }
        return true;
    };
    if (!progress && !evaluate(*dem, 0, 0, vertexCols - 1, vertexRows - 1))
    { error = "visual build cancelled"; return {}; }

    auto vertexAt = [&](int x, int y) -> const Vec3 &
    {
        return vertices[static_cast<std::size_t>(y) * vertexCols + x];
    };
    std::vector<Vec3> normals(vertexCount);
    auto calculateNormals = [&](int minX, int minY, int maxX, int maxY)
    {
        for (int y = minY; y <= maxY; ++y)
        {
            for (int x = minX; x <= maxX; ++x)
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

    };
    if (!progress) calculateNormals(0, 0, vertexCols - 1, vertexRows - 1);

    const int bootstrapZoom = std::min(cfg.visualBootstrapImageryZoom, rect.zoom);
    std::unordered_set<TileKey, TileKeyHash> bootstrapSet;
    for (int ty = rect.minY; ty <= rect.maxY; ++ty)
        for (int tx = rect.minX; tx <= rect.maxX; ++tx)
            for (const auto &key : PageSourceKeys({tx, ty, rect.zoom}, bootstrapZoom))
                bootstrapSet.insert(key);
    std::vector<TileKey> bootstrapKeys(bootstrapSet.begin(), bootstrapSet.end());
    std::vector<TileKey> bootstrapFailed;
    if (!progress) store_->EnsureImagery(bootstrapKeys, &bootstrapFailed);

    snapshot->pages.reserve(static_cast<std::size_t>(rect.Width()) * rect.Height());
    std::size_t carriedPageTextures = 0;
    std::vector<TileKey> pageKeys = keysForRect(rect);
    if (progress)
        std::stable_sort(pageKeys.begin(), pageKeys.end(), [&](const TileKey &a, const TileKey &b) {
            const auto distance = [&](const TileKey &key) {
                const auto dx = static_cast<long long>(key.x) - center.x;
                const auto dy = static_cast<long long>(key.y) - center.y;
                return dx * dx + dy * dy;
            };
            return distance(a) < distance(b);
        });
    for (const auto &pageKey : pageKeys)
    {
        if (ScopedDownloadContext::Cancelled()) { error = "visual build cancelled"; return {}; }
        const int tx = pageKey.x, ty = pageKey.y;
        const std::size_t pageIndex = snapshot->pages.size();
        const int baseGridX = (tx - rect.minX) * snapshot->cellsPerTile;
        const int baseGridY = (ty - rect.minY) * snapshot->cellsPerTile;
        if (progress)
        {
            const int minX = std::max(0, baseGridX - 1);
            const int minY = std::max(0, baseGridY - 1);
            const int maxX = std::min(vertexCols - 1, baseGridX + snapshot->cellsPerTile + 1);
            const int maxY = std::min(vertexRows - 1, baseGridY + snapshot->cellsPerTile + 1);
            const TileBounds pageBounds{
                tileYToLat(rect.minY + static_cast<double>(minY) / snapshot->cellsPerTile, rect.zoom),
                tileYToLat(rect.minY + static_cast<double>(maxY) / snapshot->cellsPerTile, rect.zoom),
                longitudes[minX], longitudes[maxX]};
            auto pageDem = store_->BuildElevationMosaic(pageBounds, cfg.visualElevationZoom, 1, error);
            if (!pageDem || !evaluate(*pageDem, minX, minY, maxX, maxY))
            { if (error.empty()) error = "visual build cancelled"; return {}; }
            calculateNormals(baseGridX, baseGridY,
                baseGridX + snapshot->cellsPerTile, baseGridY + snapshot->cellsPerTile);
            if (!store_->EnsureImagery(PageSourceKeys(pageKey, bootstrapZoom)))
            { error = "bootstrap imagery unavailable for " + tileText(pageKey); return {}; }
        }
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
        const int pageVertexCols = snapshot->cellsPerTile + 1;
        const std::size_t pageVertices = static_cast<std::size_t>(pageVertexCols) * pageVertexCols;
        mesh->positions.reserve(pageVertices);
        mesh->normals.reserve(pageVertices);
        mesh->texCoords.reserve(pageVertices);
        mesh->indices.reserve(static_cast<std::size_t>(snapshot->cellsPerTile) * snapshot->cellsPerTile * 6u);
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
        if (progress)
        {
            ++snapshot->coverageRevision;
            snapshot->progressiveComplete = cfg.visualOuterRadiusM <= 0 &&
                snapshot->pages.size() == pageKeys.size();
            if (!progress(*snapshot)) { error = "visual publication cancelled"; return {}; }
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

std::shared_ptr<TerrainSnapshot> PersistentTerrainBuilder::BuildOuterCoverage(
    const TerrainSnapshot &near, std::string &error,
    const ProgressCallback &progress)
{
    const auto &cfg = store_->GetConfig();
    auto snapshot = std::make_shared<TerrainSnapshot>(near);
    if (cfg.visualOuterRadiusM <= 0 ||
        std::any_of(near.pages.begin(), near.pages.end(), [](const TerrainPage &page) { return page.visualOnly; }))
        return snapshot;
    const auto began = std::chrono::steady_clock::now();
    try
    {
        const auto &inner = near.geometryRect;
        const double latitude = tileYToLat(near.centerTile.y + 0.5, inner.zoom);
        const double longitude = tileXToLon(near.centerTile.x + 0.5, inner.zoom);
        const double angularRadius = cfg.visualOuterRadiusM / 6371000.0 * 180.0 / kPi;
        const double north = std::min(kMercatorLatLimit, latitude + angularRadius);
        const double south = std::max(-kMercatorLatLimit, latitude - angularRadius);
        const double longitudeRadius = std::min(180.0, angularRadius /
            std::max(0.01, std::cos(std::max(std::abs(north), std::abs(south)) * kPi / 180.0)));
        const TileBounds requested{north, south,
            std::max(-180.0, longitude - longitudeRadius),
            std::min(180.0, longitude + longitudeRadius)};
        const double localCircumference = 40075016.68557849 *
            std::max(0.01, std::cos(latitude * kPi / 180.0));
        const int rootZoom = clampValue(static_cast<int>(std::floor(std::log2(
            localCircumference / std::max(1.0, cfg.visualOuterRadiusM)))), 0, inner.zoom);
        auto outer = boundsToTileRect(requested, rootZoom);
        const int shift = inner.zoom - rootZoom;
        const int fineCount = 1 << inner.zoom;
        outer.minX = std::min(outer.minX, std::max(0, inner.minX - 1) >> shift);
        outer.minY = std::min(outer.minY, std::max(0, inner.minY - 1) >> shift);
        outer.maxX = std::max(outer.maxX, std::min(fineCount - 1, inner.maxX + 1) >> shift);
        outer.maxY = std::max(outer.maxY, std::min(fineCount - 1, inner.maxY + 1) >> shift);
        const auto plan = planTerrainCoverage(
            {inner.minX, inner.minY, inner.maxX, inner.maxY, inner.zoom},
            {outer.minX, outer.minY, outer.maxX, outer.maxY, outer.zoom});

        std::unordered_set<TileKey, TileKeyHash> elevationRequests, imageryRequests;
        for (const auto &leaf : plan.leaves)
        {
            if (leaf.inner) continue;
            const TileKey key{leaf.x, leaf.y, leaf.z};
            const auto bounds = tileRectBounds({key.x, key.y, key.x, key.y, key.z});
            const int demZoom = std::max(1, std::min(cfg.visualElevationZoom, key.z));
            for (const auto &demKey : keysForRect(boundsToTileRect(bounds, demZoom, 1)))
                elevationRequests.insert(demKey);
            const int bootstrapZoom = std::min(cfg.visualBootstrapImageryZoom, key.z);
            const int targetZoom = std::min(inner.zoom, key.z +
                std::min(1, floorLog2Positive(std::max(1, cfg.visualPageTextureMaxSize / 256))));
            const int textureSize = 256 << (targetZoom - key.z);
            const auto cached = CachedTextureForPage(key);
            if (!cached || cached->imageryZoom < bootstrapZoom || cached->textureSize < textureSize)
                for (const auto &imageKey : PageSourceKeys(key, bootstrapZoom))
                    imageryRequests.insert(imageKey);
        }
        // Deduplicate neighbors before handing work to the persistent bounded
        // scheduler, so cold coverage uses the configured network concurrency.
        if (!progress && !store_->EnsureElevation(std::vector<TileKey>(elevationRequests.begin(), elevationRequests.end())))
        {
            error = "outer terrain elevation batch incomplete";
            return {};
        }
        if (!progress && !store_->EnsureImagery(std::vector<TileKey>(imageryRequests.begin(), imageryRequests.end())))
        {
            error = "outer terrain imagery batch incomplete";
            return {};
        }

        const int coarseCells = clampValue(near.cellsPerTile, 8, 64);
        std::size_t verticesPlanned = 0;
        for (const auto &leaf : plan.leaves)
        {
            const auto side = static_cast<std::size_t>(
                (leaf.z == inner.zoom ? near.cellsPerTile : coarseCells) + 1);
            verticesPlanned += side * side;
        }
        if (verticesPlanned > 4'000'000u)
            throw std::runtime_error("outer terrain exceeds the vertex budget");
        std::unordered_map<TileKey, std::shared_ptr<const MeshData>, TileKeyHash> innerMeshes;
        for (const auto &page : near.pages) innerMeshes.emplace(page.key, page.mesh);
        std::vector<std::shared_ptr<const MeshData>> meshes(plan.leaves.size());
        std::size_t reusedMeshes = 0;
        std::size_t verticesBuilt = 0;

        std::vector<std::size_t> buildOrder;
        buildOrder.reserve(plan.leaves.size());
        if (progress)
        {
            // The planner's coarse-first order includes distant, often invisible
            // roots. Prefer nearby pages, visiting just their coarser boundary
            // dependencies first so the exact stitched mesh remains unchanged.
            std::vector<std::size_t> wanted(plan.leaves.size());
            for (std::size_t i = 0; i < wanted.size(); ++i) wanted[i] = i;
            const auto distance = [&](std::size_t index) {
                const auto &leaf = plan.leaves[index];
                const double scale = std::ldexp(1.0, inner.zoom - leaf.z);
                const double cx = near.centerTile.x + 0.5, cy = near.centerTile.y + 0.5;
                const double dx = cx - clampValue(cx, leaf.x * scale, (leaf.x + 1) * scale);
                const double dy = cy - clampValue(cy, leaf.y * scale, (leaf.y + 1) * scale);
                return dx * dx + dy * dy;
            };
            std::stable_sort(wanted.begin(), wanted.end(), [&](std::size_t a, std::size_t b) {
                return distance(a) < distance(b);
            });
            std::vector<bool> visited(plan.leaves.size(), false);
            std::function<void(std::size_t)> visit = [&](std::size_t index) {
                if (visited[index]) return;
                visited[index] = true;
                for (const int coarse : plan.coarserNeighbors[index])
                    if (coarse >= 0) visit(static_cast<std::size_t>(coarse));
                buildOrder.push_back(index);
            };
            for (const auto index : wanted) visit(index);
        }
        else
            for (std::size_t i = 0; i < plan.leaves.size(); ++i) buildOrder.push_back(i);

        // Every page requests at most its own small DEM neighborhood. Never
        // allocate one fine-resolution DEM mosaic across the distant horizon.
        for (const auto leafIndex : buildOrder)
        {
            if (ScopedDownloadContext::Cancelled())
                throw std::runtime_error("outer terrain build cancelled");
            const auto &leaf = plan.leaves[leafIndex];
            const TileKey key{leaf.x, leaf.y, leaf.z};
            if (leaf.inner)
            {
                const auto found = innerMeshes.find(key);
                if (found == innerMeshes.end() || !found->second)
                    throw std::runtime_error("protected terrain page missing from outer plan");
                meshes[leafIndex] = found->second;
                verticesBuilt += found->second->positions.size();
                continue;
            }
            const int cells = key.z == inner.zoom ? near.cellsPerTile : coarseCells;
            const std::size_t side = static_cast<std::size_t>(cells + 1);
            const auto bounds = tileRectBounds({key.x, key.y, key.x, key.y, key.z});
            const int demZoom = std::max(1, std::min(cfg.visualElevationZoom, key.z));
            const bool cacheable = key.z < inner.zoom && cfg.visualOuterMeshCacheMb != 0;
            std::array<std::vector<double>, 4> boundaryCoordinates;
            std::array<std::map<double, std::uint32_t>, 4> coarseBoundaryVertices;
            std::string finerTopology;
            for (int edge = 0; edge < 4; ++edge)
            {
                const int coarseIndex = plan.coarserNeighbors[leafIndex][edge];
                if (coarseIndex < 0) continue;
                const auto &coarse = plan.leaves[coarseIndex];
                const int neighborCells = coarse.z == inner.zoom ? near.cellsPerTile : coarseCells;
                const auto scale = std::int64_t{1} << (key.z - coarse.z);
                const auto offset = (edge < 2 ? key.y : key.x) -
                    std::int64_t(edge < 2 ? coarse.y : coarse.x) * scale;
                // Include the coarse grid points too when supported arbitrary
                // fine subdivisions do not already contain them.
                for (int point = 0; point <= neighborCells; ++point)
                {
                    const auto numerator = (std::int64_t(point) * scale - offset * neighborCells) * cells;
                    const double coordinate = static_cast<double>(numerator) / neighborCells;
                    if (coordinate >= 0 && coordinate <= cells)
                    {
                        boundaryCoordinates[edge].push_back(coordinate);
                        const auto side = static_cast<std::uint32_t>(neighborCells + 1);
                        const auto source = edge == 0 ? point * side + neighborCells :
                            edge == 1 ? point * side :
                            edge == 2 ? neighborCells * side + point : point;
                        coarseBoundaryVertices[edge][coordinate] = source;
                    }
                }
            }
            for (std::size_t fineIndex = leafIndex + 1; fineIndex < plan.leaves.size(); ++fineIndex)
            {
                const auto &fine = plan.leaves[fineIndex];
                const int fineCells = fine.z == inner.zoom ? near.cellsPerTile : coarseCells;
                for (int fineEdge = 0; fineEdge < 4; ++fineEdge)
                {
                    if (plan.coarserNeighbors[fineIndex][fineEdge] != static_cast<int>(leafIndex)) continue;
                    finerTopology += "_fine" + std::to_string(fineEdge) + ":" +
                        tileText({fine.x, fine.y, fine.z}) + ":" + std::to_string(fineCells);
                    auto &coordinates = boundaryCoordinates[fineEdge ^ 1];
                    for (int point = 0; point <= fineCells; ++point)
                        coordinates.push_back(coverageBoundaryCoordinate(point, fineCells,
                            fineEdge < 2 ? fine.y : fine.x, fine.z,
                            fineEdge < 2 ? key.y : key.x, key.z, cells));
                }
            }
            std::string meshCacheKey;
            std::shared_ptr<const MeshData> readyMesh;
            if (cacheable)
            {
                meshCacheKey = tileText(key) + "_cells" + std::to_string(cells) +
                    "_dem" + std::to_string(demZoom) + finerTopology;
                for (int edge = 0; edge < 4; ++edge)
                {
                    const int coarse = plan.coarserNeighbors[leafIndex][edge];
                    if (coarse >= 0)
                        meshCacheKey += "_edge" + std::to_string(edge) + ":" + meshes[coarse]->name;
                }
                std::lock_guard<std::mutex> lock(outerMeshCacheMutex_);
                const auto found = outerMeshCache_.find(meshCacheKey);
                if (found != outerMeshCache_.end())
                {
                    found->second.touch = ++outerMeshCacheTouch_;
                    readyMesh = found->second.mesh;
                    ++reusedMeshes;
                    ++outerMeshCacheHits_;
                }
                else ++outerMeshCacheMisses_;
            }
            if (!readyMesh)
            {
                auto dem = store_->BuildElevationMosaic(bounds, demZoom, 1, error);
                if (!dem) return {};
                const double offset = store_->ElevationAlignmentOffset(demZoom) + cfg.zOffsetM;
                auto mesh = std::make_shared<MeshData>();
                mesh->submeshName = terrainPageSubmeshName(key);
                mesh->name = snapshot->resourcePrefix + "_mesh_g" +
                    std::to_string(snapshot->generation) + "_outer_z" + std::to_string(key.z) +
                    "_x" + std::to_string(key.x) + "_y" + std::to_string(key.y);
                if (cacheable)
                    mesh->name = snapshot->resourcePrefix + "_outer_resident_" +
                        std::to_string(++outerMeshSerial_);
                mesh->positions.reserve(side * side);
                mesh->normals.resize(side * side);
                mesh->texCoords.reserve(side * side);
                mesh->indices.reserve(static_cast<std::size_t>(cells) * cells * 6u);
                for (int y = 0; y <= cells; ++y)
                {
                    const double lat = tileYToLat(key.y + static_cast<double>(y) / cells, key.z);
                    for (int x = 0; x <= cells; ++x)
                    {
                        const double lon = tileXToLon(key.x + static_cast<double>(x) / cells, key.z);
                        mesh->positions.push_back(localFromGeodetic(
                            store_->Spherical(), lat, lon, dem->Sample(lat, lon) + offset));
                        mesh->texCoords.emplace_back(static_cast<double>(x) / cells,
                                                      static_cast<double>(y) / cells);
                    }
                }
                // Stitch only the newly added outer geometry. Fine boundary
                // vertices lie on the existing coarser edge segments; the near
                // rectangle has a same-resolution halo and is never changed.
                auto stitch = [&](bool normals)
                {
                    for (int edge = 0; edge < 4; ++edge)
                    {
                        const MeshData *neighbor = nullptr;
                        TileKey neighborKey;
                        const int coarse = plan.coarserNeighbors[leafIndex][edge];
                        if (coarse >= 0)
                        {
                            neighbor = meshes[coarse].get();
                            const auto &n = plan.leaves[coarse];
                            neighborKey = {n.x, n.y, n.z};
                        }
                        else if (key.z == inner.zoom)
                        {
                            neighborKey = key;
                            if (edge == 0) --neighborKey.x;
                            if (edge == 1) ++neighborKey.x;
                            if (edge == 2) --neighborKey.y;
                            if (edge == 3) ++neighborKey.y;
                            const auto found = innerMeshes.find(neighborKey);
                            if (found != innerMeshes.end()) neighbor = found->second.get();
                        }
                        if (!neighbor) continue;
                        // The regular grid comes first; coarse meshes may also
                        // contain appended seam vertices and triangle centers.
                        const int neighborCells = neighborKey.z == inner.zoom
                            ? near.cellsPerTile : coarseCells;
                        const int neighborSide = neighborCells + 1;
                        for (int point = 0; point <= cells; ++point)
                        {
                            const double coordinate = coverageBoundaryCoordinate(point, cells,
                                edge < 2 ? key.y : key.x, key.z,
                                edge < 2 ? neighborKey.y : neighborKey.x, neighborKey.z, neighborCells);
                            const double bounded = clampValue(coordinate, 0.0, static_cast<double>(neighborCells));
                            const int first = std::min(neighborCells - 1, static_cast<int>(std::floor(bounded)));
                            const double fraction = bounded - first;
                            auto sourceIndex = [&](int along) -> std::size_t
                            {
                                if (edge == 0) return static_cast<std::size_t>(along) * neighborSide + neighborCells;
                                if (edge == 1) return static_cast<std::size_t>(along) * neighborSide;
                                if (edge == 2) return static_cast<std::size_t>(neighborCells) * neighborSide + along;
                                return static_cast<std::size_t>(along);
                            };
                            const std::size_t destination = edge == 0 ? point * side :
                                edge == 1 ? point * side + cells :
                                edge == 2 ? point : cells * side + point;
                            const auto &source = normals ? neighbor->normals : neighbor->positions;
                            auto value = source[sourceIndex(first)] * (1.0 - fraction) +
                                         source[sourceIndex(first + 1)] * fraction;
                            if (normals) mesh->normals[destination] = value.Normalized();
                            else mesh->positions[destination] = value;
                        }
                    }
                };
                stitch(false);
                for (int y = 0; y <= cells; ++y)
                    for (int x = 0; x <= cells; ++x)
                    {
                        const auto at = [&](int px, int py) -> const Vec3 &
                        { return mesh->positions[static_cast<std::size_t>(py) * side + px]; };
                        auto normal = (at(std::min(cells, x + 1), y) - at(std::max(0, x - 1), y)).Cross(
                            at(x, std::max(0, y - 1)) - at(x, std::min(cells, y + 1)));
                        if (normal.Length() < 1e-9) normal = Vec3{0, 0, 1};
                        else normal.Normalize();
                        if (normal.Z() < 0.0) normal *= -1.0;
                        mesh->normals[static_cast<std::size_t>(y) * side + x] = normal;
                    }
                stitch(true);
                const auto topology = stitchCoverageBoundary(cells, boundaryCoordinates);
                for (const auto &vertex : topology.appended)
                {
                    auto position = mesh->positions[vertex.indices[0]] * vertex.weights[0] +
                        mesh->positions[vertex.indices[1]] * vertex.weights[1];
                    auto normal = mesh->normals[vertex.indices[0]] * vertex.weights[0] +
                        mesh->normals[vertex.indices[1]] * vertex.weights[1];
                    double u = 0, v = 0;
                    for (int corner = 0; corner < 3; ++corner)
                    {
                        u += mesh->texCoords[vertex.indices[corner]].X() * vertex.weights[corner];
                        v += mesh->texCoords[vertex.indices[corner]].Y() * vertex.weights[corner];
                    }
                    if (vertex.weights[2] != 0)
                    {
                        position += mesh->positions[vertex.indices[2]] * vertex.weights[2];
                        normal += mesh->normals[vertex.indices[2]] * vertex.weights[2];
                    }
                    else
                    {
                        const auto a = vertex.indices[0], b = vertex.indices[1];
                        const int edge = a % side == 0 && b % side == 0 ? 0 :
                            a % side == cells && b % side == cells ? 1 :
                            a < side && b < side ? 2 : 3;
                        const double coordinate = (edge < 2 ? a / side : a % side) + vertex.weights[1];
                        const auto found = coarseBoundaryVertices[edge].find(coordinate);
                        if (found != coarseBoundaryVertices[edge].end())
                        {
                            const auto &coarse = *meshes[plan.coarserNeighbors[leafIndex][edge]];
                            position = coarse.positions[found->second];
                            normal = coarse.normals[found->second];
                        }
                    }
                    mesh->positions.push_back(position);
                    mesh->normals.push_back(normal.Normalized());
                    mesh->texCoords.emplace_back(u, v);
                }
                mesh->indices = topology.indices;
                readyMesh = std::move(mesh);
                if (cacheable)
                {
                    const auto bytes = readyMesh->positions.size() * sizeof(Vec3) +
                        readyMesh->normals.size() * sizeof(Vec3) + readyMesh->texCoords.size() * sizeof(Vec2) +
                        readyMesh->indices.size() * sizeof(std::uint32_t);
                    const std::size_t limit = cfg.visualOuterMeshCacheMb * 1024u * 1024u;
                    std::lock_guard<std::mutex> lock(outerMeshCacheMutex_);
                    if (bytes <= limit)
                    {
                        auto &entry = outerMeshCache_[meshCacheKey];
                        outerMeshCacheBytes_ -= entry.bytes;
                        entry = {readyMesh, bytes, ++outerMeshCacheTouch_};
                        outerMeshCacheBytes_ += bytes;
                        while (outerMeshCacheBytes_ > limit)
                        {
                            const auto victim = std::min_element(outerMeshCache_.begin(), outerMeshCache_.end(),
                                [](const auto &a, const auto &b) { return a.second.touch < b.second.touch; });
                            outerMeshCacheBytes_ -= victim->second.bytes;
                            outerMeshCache_.erase(victim);
                        }
                    }
                }
            }
            verticesBuilt += readyMesh->positions.size();
            if (verticesBuilt > 4'000'000u)
                throw std::runtime_error("stitched outer terrain exceeds the vertex budget");
            meshes[leafIndex] = readyMesh;

            TerrainPage page;
            page.visualOnly = true;
            page.key = key;
            page.index = snapshot->pages.size();
            page.submeshName = readyMesh->submeshName;
            page.mesh = std::move(readyMesh);
            page.targetImageryZoom = std::min(inner.zoom, key.z +
                std::min(1, floorLog2Positive(std::max(1, cfg.visualPageTextureMaxSize / 256))));
            page.textureSize = std::min(cfg.visualPageTextureMaxSize,
                256 << (page.targetImageryZoom - key.z));
            page.imageryZoom = std::min(cfg.visualBootstrapImageryZoom, key.z);
            const auto cached = CachedTextureForPage(key);
            if (cached && cached->imageryZoom >= page.imageryZoom &&
                cached->textureSize >= page.textureSize)
            {
                page.imageryZoom = cached->imageryZoom;
                page.targetImageryZoom = std::max(page.targetImageryZoom, cached->imageryZoom);
                page.textureSize = cached->textureSize;
                page.texture = cached->texture;
                page.textureName = cached->textureName;
            }
            else
            {
                std::vector<TileKey> failed;
                store_->EnsureImagery(PageSourceKeys(key, page.imageryZoom), &failed);
                if (!failed.empty())
                {
                    error = "outer terrain bootstrap imagery unavailable for " + tileText(key);
                    return {};
                }
                page.texture = ToImage(ComposeBootstrapPage(
                    key, key.z, page.imageryZoom, page.textureSize));
                page.textureName = snapshot->resourcePrefix + "_page_z" + std::to_string(key.z) +
                    "_x" + std::to_string(key.x) + "_y" + std::to_string(key.y) +
                    "_q" + std::to_string(page.imageryZoom) + "_s" + std::to_string(page.textureSize);
            }
            if (!page.texture || !page.texture->Valid())
                throw std::runtime_error("invalid outer terrain texture");
            snapshot->estimatedTextureBytes += mipmappedRgbaBytes(page.textureSize, page.textureSize);
            snapshot->pages.push_back(std::move(page));
            if (progress)
            {
                snapshot->progressiveInitial = true;
                snapshot->progressiveComplete = false;
                ++snapshot->coverageRevision;
                if (!progress(*snapshot)) { error = "outer publication cancelled"; return {}; }
            }
        }
        // Initial near imagery may have refined while this worker prepared the
        // outer ring. Publishing an expanded snapshot must retain that detail.
        for (std::size_t i = 0; i < near.pages.size(); ++i)
        {
            auto &page = snapshot->pages[i];
            const auto cached = CachedTextureForPage(page.key);
            if (!cached || cached->imageryZoom < page.imageryZoom ||
                cached->textureSize < page.textureSize) continue;
            snapshot->estimatedTextureBytes -= mipmappedRgbaBytes(page.textureSize, page.textureSize);
            page.imageryZoom = cached->imageryZoom;
            page.targetImageryZoom = std::max(page.targetImageryZoom, cached->imageryZoom);
            page.textureSize = cached->textureSize;
            page.texture = cached->texture;
            page.textureName = cached->textureName;
            snapshot->estimatedTextureBytes += mipmappedRgbaBytes(page.textureSize, page.textureSize);
        }
        if (progress)
        {
            ++snapshot->coverageRevision;
            snapshot->progressiveComplete = true;
            if (!progress(*snapshot)) { error = "outer publication cancelled"; return {}; }
        }
        else snapshot->coverageRevision = 1;
        if (cfg.diagnostics)
            logInfo("[DynamicTerrain][VISUAL] outer coverage ready generation=", snapshot->generation,
                " near_pages=", near.pages.size(), " outer_pages=", snapshot->pages.size() - near.pages.size(),
                " root_zoom=", rootZoom, " radius_m=", cfg.visualOuterRadiusM,
                " coarse_cells_per_page=", coarseCells, " vertices_planned=", verticesPlanned, " elapsed_s=",
                std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count(),
                " reused_outer_meshes=", reusedMeshes, " mesh_cache_bytes=", CachedOuterMeshStats().bytes);
        return snapshot;
    }
    catch (const std::exception &failure)
    {
        error = failure.what();
        return {};
    }
}

std::vector<std::size_t> PersistentTerrainBuilder::ProgressivePageOrder(
    const TerrainSnapshot &snapshot) const
{
    std::vector<std::size_t> order(snapshot.pages.size());
    for (std::size_t i = 0; i < order.size(); ++i)
        order[i] = i;

    const double reference = store_->Spherical().ElevationReference();
    std::vector<double> distances(snapshot.pages.size());
    for (std::size_t index = 0; index < distances.size(); ++index)
    {
        const auto &page = snapshot.pages[index];
        const double lat = tileYToLat(page.key.y + 0.5, page.key.z);
        const double lon = tileXToLon(page.key.x + 0.5, page.key.z);
        const auto local = localFromGeodetic(store_->Spherical(), lat, lon, reference);
        distances[index] = horizontalDistance(local, snapshot.patchCenterLocal);
    }
    std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b)
    {
        const double da = distances[a];
        const double db = distances[b];
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
    const auto &page = snapshot.pages[pageIndex];
    return page.visualOnly ? page.targetImageryZoom : PageTargetZoom(snapshot, page.key);
}

std::optional<TextureUpdate> PersistentTerrainBuilder::BuildTextureStage(
    const TerrainSnapshot &snapshot,
    const std::vector<std::size_t> &pageIndices,
    int sourceZoom,
    std::string &error)
{
    if (!store_->GetConfig().visualRefineTexture || pageIndices.empty())
        return std::nullopt;

    sourceZoom = clampValue(sourceZoom, 0, 20);
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
        if (sourceZoom < page.key.z || sourceZoom > TargetZoomForPage(snapshot, pageIndex))
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
        if (ScopedDownloadContext::Cancelled()) { error = "texture build cancelled"; return std::nullopt; }
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
            page.key, page.key.z, sourceZoom,
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

    auto image = std::make_shared<ImageData>();
    image->width = static_cast<std::uint32_t>(bgr.cols);
    image->height = static_cast<std::uint32_t>(bgr.rows);
    image->rgb.resize(bgr.total() * bgr.elemSize());
    cv::Mat rgb(bgr.rows, bgr.cols, CV_8UC3, image->rgb.data());
    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
    return image;
}

}
