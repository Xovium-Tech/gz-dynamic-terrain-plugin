#include "dynamic_terrain/core/TileStore.hh"

#include <curl/curl.h>
#include <opencv2/imgcodecs.hpp>
#if defined(XOVIUM_PRIVATE_JPEG)
#include "ue_px4/terrain_image.hpp"
#else
#include "JpegValidation.hh"
#endif

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <condition_variable>
#include <cmath>
#include <fstream>
#include <set>
#include <stdexcept>
#include <thread>
#include <unordered_set>

namespace dynamic_terrain
{
TileStoreDiagnostics TileStore::Diagnostics() const
{
    TileStoreDiagnostics result;
    result.downloads = downloader_.Diagnostics();
    std::scoped_lock lock(demMutex_, imageryMutex_);
    result.decodedDemBytes = demMemoryBytes_;
    result.decodedDemEntries = demMemory_.size();
    result.decodedImageryBytes = imageryMemoryBytes_;
    result.decodedImageryEntries = imageryMemory_.size();
    return result;
}

namespace
{
bool fileReady(const fs::path &path)
{
    std::error_code ec;
    return fs::is_regular_file(path, ec) && fs::file_size(path, ec) > 0 && !ec;
}

bool validateRasterBytes(const std::vector<unsigned char> &bytes)
{
#if defined(XOVIUM_PRIVATE_JPEG)
    return !loadTerrainImageBytes(bytes.data(), bytes.size()).empty();
#else
    if (bytes.size() >= 2 && bytes[0] == 0xff && bytes[1] == 0xd8)
        return detail::ValidateJpeg(bytes);
    return !cv::imdecode(bytes, cv::IMREAD_COLOR).empty();
#endif
}

cv::Mat loadRasterFile(const std::string &path)
{
#if defined(XOVIUM_PRIVATE_JPEG)
    return loadTerrainImage(path);
#else
    return cv::imread(path, cv::IMREAD_COLOR);
#endif
}

DownloadCacheOptions downloadOptions(const Config &cfg)
{
    DownloadCacheOptions options;
    options.concurrency = cfg.downloadConcurrency;
    options.perHost = cfg.downloadPerHost;
    options.retries = cfg.downloadRetries;
    options.timeoutMs = cfg.httpTimeoutMs;
    options.userAgent = cfg.userAgent;
    return options;
}

std::string safeName(std::string value)
{
    for (char &c : value)
        if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-'))
            c = '_';
    return value;
}

std::int64_t positiveModulo(std::int64_t value, std::int64_t modulus)
{
    const std::int64_t result = value % modulus;
    return result < 0 ? result + modulus : result;
}

std::atomic<std::uint64_t> &terrainInstanceCounter()
{
    static std::atomic<std::uint64_t> counter{0};
    return counter;
}
}


double ElevationMosaic::Sample(double latDeg, double lonDeg) const
{
    if (!Valid() || tileSize <= 0)
        return 0.0;

    const TileCoordF tile = latLonToTileFraction(latDeg, lonDeg, zoom);
    const double globalPx = tile.x * tileSize - 0.5;
    const double globalPy = tile.y * tileSize - 0.5;
    const double localPx = globalPx - static_cast<double>(rect.minX * tileSize);
    const double localPy = globalPy - static_cast<double>(rect.minY * tileSize);

    const double px = clampValue(localPx, 0.0,
                                 static_cast<double>(heights.cols - 1));
    const double py = clampValue(localPy, 0.0,
                                 static_cast<double>(heights.rows - 1));
    const int x0 = clampValue(static_cast<int>(std::floor(px)), 0,
                              heights.cols - 1);
    const int y0 = clampValue(static_cast<int>(std::floor(py)), 0,
                              heights.rows - 1);
    const int x1 = std::min(x0 + 1, heights.cols - 1);
    const int y1 = std::min(y0 + 1, heights.rows - 1);
    const double tx = px - x0;
    const double ty = py - y0;
    const double h00 = heights.at<float>(y0, x0);
    const double h10 = heights.at<float>(y0, x1);
    const double h01 = heights.at<float>(y1, x0);
    const double h11 = heights.at<float>(y1, x1);
    const double top = h00 * (1.0 - tx) + h10 * tx;
    const double bottom = h01 * (1.0 - tx) + h11 * tx;
    return top * (1.0 - ty) + bottom * ty;
}

BatchDownloader::BatchDownloader(Config config)
    : cfg_(std::move(config)), cache_(downloadOptions(cfg_)) {}

DownloadStats BatchDownloader::Ensure(const std::vector<DownloadSpec> &requests,
                                      std::vector<fs::path> *failedPaths) const
{
    auto stats = cache_.Ensure(requests, failedPaths, ScopedDownloadContext::Check(),
                              ScopedDownloadContext::Priority());
    if (cfg_.diagnostics)
        logInfo("[DynamicTerrain][HTTP] requested=", stats.requested,
                " cached=", stats.cached, " downloaded=", stats.downloaded,
                " failed=", stats.failed, " cancelled=", stats.cancelled,
                " bytes=", stats.bytes, " connections=", stats.connections,
                " retries=", stats.retries, " time=", stats.elapsedSec, "s");
    return stats;
}

TileStore::TileStore(Config config, std::shared_ptr<const GeographicTransform> spherical)
    : cfg_(std::move(config)),
      spherical_(std::move(spherical)),
      cacheRoot_(expandHome(cfg_.cacheDir)),
      imagery_(resolveImageryProvider(cfg_)),
      downloader_(cfg_), terrarium_(lower(cfg_.elevationProvider) == "terrarium")
{
    if (!spherical_)
        throw std::invalid_argument("TileStore requires a geographic transform");
    originLatDeg_ = spherical_->LatitudeDeg();
    originLonDeg_ = spherical_->LongitudeDeg();
    const auto instance = terrainInstanceCounter().fetch_add(
        1, std::memory_order_relaxed) + 1;
    resourcePrefix_ = "dynamic_terrain_" +
        safeName(cfg_.modelName.empty() ? "model" : cfg_.modelName) + "_i" +
        std::to_string(instance);
}

fs::path TileStore::ImageryPath(const TileKey &key) const
{
    return cacheRoot_ / "imagery" / TerrainCacheNamespace(imagery_.name, cfg_.imageryUrl, imagery_.extension) /
           std::to_string(key.z) / std::to_string(key.x) /
           (std::to_string(key.y) + "." + imagery_.extension);
}

fs::path TileStore::ElevationPath(const TileKey &key) const
{
    const std::string provider = lower(cfg_.elevationProvider);
    const std::string ext = provider == "terrarium" ? "png" : "webp";
    return cacheRoot_ / "elevation" / TerrainCacheNamespace(provider, cfg_.elevationUrl, ext) /
           std::to_string(key.z) / std::to_string(key.x) /
           (std::to_string(key.y) + "." + ext);
}

std::string TileStore::ImageryUrl(const TileKey &key) const
{
    return buildUrl(imagery_, key, cfg_.imageryToken);
}

std::string TileStore::ElevationUrl(const TileKey &key) const
{
    const std::string provider = lower(cfg_.elevationProvider);
    std::string templ = cfg_.elevationUrl;
    if (templ.empty())
    {
        if (provider == "terrarium")
            templ = "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png";
        else
            templ = "https://api.mapbox.com/raster/v1/mapbox.mapbox-terrain-dem-v1/{z}/{x}/{y}.webp?sku=101CUGorpzzyK&access_token={token}";
    }
    Provider elevation{provider, templ, "", false};
    return buildUrl(elevation, key, cfg_.elevationToken);
}

bool TileStore::EnsureImagery(const std::vector<TileKey> &keys,
                              std::vector<TileKey> *failed)
{
    std::vector<DownloadSpec> requests;
    requests.reserve(keys.size());
    for (const auto &key : keys)
        requests.push_back({"IMG " + tileText(key), ImageryUrl(key), ImageryPath(key), validateRasterBytes});
    std::vector<fs::path> failedPaths;
    const auto stats = downloader_.Ensure(requests, &failedPaths);
    if (failed)
    {
        for (const auto &key : keys)
            if (std::find(failedPaths.begin(), failedPaths.end(), fs::absolute(ImageryPath(key)).lexically_normal()) != failedPaths.end())
                failed->push_back(key);
    }
    return stats.failed == 0 && stats.cancelled == 0;
}

bool TileStore::EnsureElevation(const std::vector<TileKey> &keys,
                                std::vector<TileKey> *failed)
{
    const std::string provider = lower(cfg_.elevationProvider);
    if (provider == "none" || provider == "flat")
        return true;
    std::vector<DownloadSpec> requests;
    requests.reserve(keys.size());
    for (const auto &key : keys)
        requests.push_back({"DEM " + tileText(key), ElevationUrl(key), ElevationPath(key), validateRasterBytes});
    std::vector<fs::path> failedPaths;
    const auto stats = downloader_.Ensure(requests, &failedPaths);
    if (failed)
    {
        for (const auto &key : keys)
            if (std::find(failedPaths.begin(), failedPaths.end(), fs::absolute(ElevationPath(key)).lexically_normal()) != failedPaths.end())
                failed->push_back(key);
    }
    return stats.failed == 0 && stats.cancelled == 0;
}

bool TileStore::EnsureElevationForBounds(const TileBounds &bounds, int zoom,
                                         int halo)
{
    return EnsureElevation(keysForRect(boundsToTileRect(bounds, zoom, halo)));
}

std::optional<ElevationMosaic> TileStore::BuildElevationMosaic(
    const TileBounds &bounds, int zoom, int halo, std::string &error)
{
    zoom = clampValue(zoom, 1, cfg_.elevationMaxZoom);
    const TileRect rect = boundsToTileRect(bounds, zoom, halo);
    const std::string provider = lower(cfg_.elevationProvider);

    if (provider == "none" || provider == "flat")
    {
        ElevationMosaic mosaic;
        mosaic.zoom = zoom;
        mosaic.tileSize = 256;
        mosaic.rect = rect;
        mosaic.heights = cv::Mat(rect.Height() * mosaic.tileSize,
                                 rect.Width() * mosaic.tileSize,
                                 CV_32FC1,
                                 cv::Scalar(static_cast<float>(spherical_->ElevationReference())));
        return mosaic;
    }

    const auto keys = keysForRect(rect);
    std::vector<TileKey> failed;
    EnsureElevation(keys, &failed);
    if (!failed.empty())
    {
        error = std::to_string(failed.size()) +
                " elevation tiles unavailable for mosaic at z" +
                std::to_string(zoom);
        return std::nullopt;
    }

    int tileWidth = 0;
    int tileHeight = 0;
    std::vector<std::pair<TileKey, std::shared_ptr<cv::Mat>>> rasters;
    rasters.reserve(keys.size());
    for (const auto &key : keys)
    {
        auto raster = LoadElevationRaster(key);
        if (!raster || raster->empty())
        {
            error = "failed to decode elevation tile " + tileText(key);
            return std::nullopt;
        }
        if (tileWidth == 0)
        {
            tileWidth = raster->cols;
            tileHeight = raster->rows;
        }
        if (raster->cols != tileWidth || raster->rows != tileHeight)
        {
            error = "elevation tile dimensions are inconsistent";
            return std::nullopt;
        }
        rasters.emplace_back(key, std::move(raster));
    }
    if (tileWidth <= 0 || tileHeight <= 0 || tileWidth != tileHeight)
    {
        error = "invalid elevation tile dimensions";
        return std::nullopt;
    }

    ElevationMosaic mosaic;
    mosaic.zoom = zoom;
    mosaic.tileSize = tileWidth;
    mosaic.rect = rect;
    mosaic.heights = cv::Mat(rect.Height() * tileHeight,
                             rect.Width() * tileWidth,
                             CV_32FC1);

    for (const auto &[key, raster] : rasters)
    {
        if (ScopedDownloadContext::Cancelled()) { error = "elevation build cancelled"; return std::nullopt; }
        const int ox = (key.x - rect.minX) * tileWidth;
        const int oy = (key.y - rect.minY) * tileHeight;
        cv::Mat dst = mosaic.heights(cv::Rect(ox, oy, tileWidth, tileHeight));
        for (int row = 0; row < tileHeight; ++row)
        {
            const auto *src = raster->ptr<cv::Vec3b>(row);
            float *out = dst.ptr<float>(row);
            for (int col = 0; col < tileWidth; ++col)
                out[col] = static_cast<float>(DecodeHeight(src[col]));
        }
    }

    if (cfg_.diagnostics)
        logInfo("[DynamicTerrain][DEM] mosaic ready z=", zoom,
                " tiles=", rect.Width(), "x", rect.Height(),
                " raster=", mosaic.heights.cols, "x", mosaic.heights.rows);
    return mosaic;
}

double TileStore::ElevationAlignmentOffset(int zoom)
{
    return OriginAlignmentOffset(zoom);
}

cv::Mat TileStore::LoadImagery(const TileKey &key) const
{
    // Serialise only equal hash stripes during decode, never the entire cache.
    auto &stripe = imageryFileMutexes_[TileKeyHash{}(key) % imageryFileMutexes_.size()];
    std::lock_guard<std::mutex> stripeLock(stripe);
    {
        std::lock_guard<std::mutex> lock(imageryMutex_);
        const auto found = imageryMemory_.find(key);
        if (found != imageryMemory_.end())
        {
            imageryLru_.splice(imageryLru_.begin(), imageryLru_, imageryLruPos_.at(key));
            return found->second;
        }
    }
    const auto path = ImageryPath(key);
    if (!fileReady(path) || ScopedDownloadContext::Cancelled()) return {};
    cv::Mat decoded = loadRasterFile(path.string());
    if (decoded.empty())
    {
        downloader_.InvalidateValidation(path);
        const auto repaired = downloader_.Ensure({{"IMG " + tileText(key), ImageryUrl(key), path, validateRasterBytes}});
        if (repaired.failed || repaired.cancelled) return {};
        decoded = loadRasterFile(path.string());
        if (decoded.empty()) return {};
    }
    const std::size_t bytes = decoded.total() * decoded.elemSize();
    const std::size_t limit = cfg_.decodedImageryCacheMb * 1024ull * 1024ull;
    if (limit == 0 || bytes > limit) return decoded;
    std::lock_guard<std::mutex> lock(imageryMutex_);
    while (!imageryLru_.empty() && imageryMemoryBytes_ + bytes > limit)
    {
        const auto victim = imageryLru_.back();
        const auto &raster = imageryMemory_.at(victim);
        imageryMemoryBytes_ -= raster.total() * raster.elemSize();
        imageryMemory_.erase(victim); imageryLruPos_.erase(victim); imageryLru_.pop_back();
    }
    imageryLru_.push_front(key); imageryLruPos_[key] = imageryLru_.begin();
    imageryMemory_.emplace(key, decoded); imageryMemoryBytes_ += bytes;
    return decoded;
}

void TileStore::TouchDemLru(const TileKey &key, std::size_t bytes)
{
    auto pos = demLruPos_.find(key);
    if (pos != demLruPos_.end())
        demLru_.erase(pos->second);
    else
    {
        demMemoryBytes_ += bytes;
        demBytes_[key] = bytes;
    }
    demLru_.push_front(key);
    demLruPos_[key] = demLru_.begin();
}

void TileStore::EvictDemLruIfNeeded()
{
    const std::size_t limit = cfg_.decodedDemCacheMb * 1024ull * 1024ull;
    while (demMemoryBytes_ > limit && demLru_.size() > 1)
    {
        const TileKey victim = demLru_.back();
        demLru_.pop_back();
        demLruPos_.erase(victim);
        auto bytesIt = demBytes_.find(victim);
        if (bytesIt != demBytes_.end())
        {
            demMemoryBytes_ -= std::min(demMemoryBytes_, bytesIt->second);
            demBytes_.erase(bytesIt);
        }
        demMemory_.erase(victim);
    }
}

std::shared_ptr<cv::Mat> TileStore::LoadElevationRaster(const TileKey &key)
{
    {
        std::lock_guard<std::mutex> lock(demMutex_);
        auto it = demMemory_.find(key);
        if (it != demMemory_.end())
        {
            const std::size_t bytes = it->second->total() * it->second->elemSize();
            TouchDemLru(key, bytes);
            return it->second;
        }
    }

    auto &stripe = demFileMutexes_[TileKeyHash{}(key) % demFileMutexes_.size()];
    std::lock_guard<std::mutex> stripeLock(stripe);
    {
        std::lock_guard<std::mutex> lock(demMutex_);
        auto it = demMemory_.find(key);
        if (it != demMemory_.end())
            return it->second;
    }

    if (!fileReady(ElevationPath(key)))
    {
        if (!EnsureElevation({key}))
            return {};
    }

    cv::Mat decoded = loadRasterFile(ElevationPath(key).string());
    if (decoded.empty() && !ScopedDownloadContext::Cancelled())
    {
        downloader_.InvalidateValidation(ElevationPath(key));
        if (EnsureElevation({key})) decoded = loadRasterFile(ElevationPath(key).string());
    }
    if (decoded.empty())
    {
        logError("[DynamicTerrain][DEM] decode failed ", tileText(key),
                 " path=", ElevationPath(key).string());
        return {};
    }
    auto raster = std::make_shared<cv::Mat>(std::move(decoded));
    const std::size_t bytes = raster->total() * raster->elemSize();
    const std::size_t limit = cfg_.decodedDemCacheMb * 1024ull * 1024ull;
    if (limit == 0 || bytes > limit) return raster;
    {
        std::lock_guard<std::mutex> lock(demMutex_);
        demMemory_[key] = raster;
        TouchDemLru(key, bytes);
        EvictDemLruIfNeeded();
    }
    return raster;
}

double TileStore::DecodeHeight(const cv::Vec3b &bgr) const
{
    const double b = bgr[0];
    const double g = bgr[1];
    const double r = bgr[2];
    if (terrarium_)
        return r * 256.0 + g + b / 256.0 - 32768.0;
    return ((r * 256.0 * 256.0 + g * 256.0 + b) * 0.1) - 10000.0;
}

double TileStore::SampleInsideRaster(const cv::Mat &raster, double u, double v) const
{
    u = clampValue(u, 0.0, 1.0);
    v = clampValue(v, 0.0, 1.0);
    const double px = u * static_cast<double>(raster.cols - 1);
    const double py = v * static_cast<double>(raster.rows - 1);
    const int x0 = clampValue(static_cast<int>(std::floor(px)), 0, raster.cols - 1);
    const int y0 = clampValue(static_cast<int>(std::floor(py)), 0, raster.rows - 1);
    const int x1 = std::min(x0 + 1, raster.cols - 1);
    const int y1 = std::min(y0 + 1, raster.rows - 1);
    const double tx = px - x0;
    const double ty = py - y0;
    auto h = [&](int x, int y) { return DecodeHeight(raster.at<cv::Vec3b>(y, x)); };
    const double top = h(x0, y0) * (1.0 - tx) + h(x1, y0) * tx;
    const double bottom = h(x0, y1) * (1.0 - tx) + h(x1, y1) * tx;
    return top * (1.0 - ty) + bottom * ty;
}

std::optional<double> TileStore::GlobalPixelHeight(std::int64_t globalX,
                                                    std::int64_t globalY,
                                                    int zoom,
                                                    int tileWidth,
                                                    int tileHeight)
{
    const std::int64_t tileCount = std::int64_t{1} << zoom;
    const std::int64_t globalWidth = tileCount * tileWidth;
    const std::int64_t globalHeight = tileCount * tileHeight;
    globalX = positiveModulo(globalX, globalWidth);
    globalY = clampValue<std::int64_t>(globalY, 0, globalHeight - 1);
    const int tileX = static_cast<int>(globalX / tileWidth);
    const int tileY = static_cast<int>(globalY / tileHeight);
    const int pixelX = static_cast<int>(globalX % tileWidth);
    const int pixelY = static_cast<int>(globalY % tileHeight);
    auto raster = LoadElevationRaster({tileX, tileY, zoom});
    if (!raster || raster->empty())
        return std::nullopt;
    const int sx = raster->cols == tileWidth ? pixelX : clampValue(
        static_cast<int>(std::llround(static_cast<double>(pixelX) *
                                     (raster->cols - 1) /
                                     std::max(1, tileWidth - 1))),
        0, raster->cols - 1);
    const int sy = raster->rows == tileHeight ? pixelY : clampValue(
        static_cast<int>(std::llround(static_cast<double>(pixelY) *
                                     (raster->rows - 1) /
                                     std::max(1, tileHeight - 1))),
        0, raster->rows - 1);
    return DecodeHeight(raster->at<cv::Vec3b>(sy, sx));
}

double TileStore::RawElevation(double latDeg, double lonDeg, int zoom)
{
    const std::string provider = lower(cfg_.elevationProvider);
    if (provider == "flat" || provider == "none")
        return spherical_->ElevationReference();

    zoom = clampValue(zoom, 1, cfg_.elevationMaxZoom);
    const TileKey key = latLonToTile(latDeg, lonDeg, zoom);
    auto referenceRaster = LoadElevationRaster(key);
    if (!referenceRaster || referenceRaster->empty())
        return spherical_->ElevationReference();

    const double n = static_cast<double>(1u << zoom);
    const double latRad = clampValue(latDeg, -kMercatorLatLimit, kMercatorLatLimit) * kPi / 180.0;
    const double x = (lonDeg + 180.0) / 360.0 * n;
    const double y = (1.0 - std::asinh(std::tan(latRad)) / kPi) * 0.5 * n;
    const int tileWidth = referenceRaster->cols;
    const int tileHeight = referenceRaster->rows;
    const double globalPx = x * tileWidth - 0.5;
    const double globalPy = y * tileHeight - 0.5;
    const std::int64_t gx0 = static_cast<std::int64_t>(std::floor(globalPx));
    const std::int64_t gy0 = static_cast<std::int64_t>(std::floor(globalPy));
    const std::int64_t gx1 = gx0 + 1;
    const std::int64_t gy1 = gy0 + 1;
    const double tx = globalPx - gx0;
    const double ty = globalPy - gy0;
    const auto h00 = GlobalPixelHeight(gx0, gy0, zoom, tileWidth, tileHeight);
    const auto h10 = GlobalPixelHeight(gx1, gy0, zoom, tileWidth, tileHeight);
    const auto h01 = GlobalPixelHeight(gx0, gy1, zoom, tileWidth, tileHeight);
    const auto h11 = GlobalPixelHeight(gx1, gy1, zoom, tileWidth, tileHeight);
    if (!h00 || !h10 || !h01 || !h11)
        return SampleInsideRaster(*referenceRaster, x - std::floor(x), y - std::floor(y));
    const double top = *h00 * (1.0 - tx) + *h10 * tx;
    const double bottom = *h01 * (1.0 - tx) + *h11 * tx;
    return top * (1.0 - ty) + bottom * ty;
}

double TileStore::OriginAlignmentOffset(int zoom)
{
    if (!cfg_.alignOriginToGround)
        return 0.0;
    {
        std::lock_guard<std::mutex> lock(originMutex_);
        auto it = originOffsets_.find(zoom);
        if (it != originOffsets_.end())
            return it->second;
    }
    const double rawOrigin = RawElevation(originLatDeg_, originLonDeg_, zoom);
    const double offset = spherical_->ElevationReference() - rawOrigin;
    {
        std::lock_guard<std::mutex> lock(originMutex_);
        originOffsets_[zoom] = offset;
    }
    if (cfg_.diagnostics)
        logInfo("[DynamicTerrain][DEM] origin alignment zoom=", zoom,
                " raw=", rawOrigin,
                " reference=", spherical_->ElevationReference(),
                " offset=", offset);
    return offset;
}

double TileStore::SampleElevation(double latDeg, double lonDeg, int zoom)
{
    return RawElevation(latDeg, lonDeg, zoom) + OriginAlignmentOffset(zoom) + cfg_.zOffsetM;
}

Vec3 TileStore::LocalPoint(double latDeg, double lonDeg, int zoom)
{
    const double elevation = SampleElevation(latDeg, lonDeg, zoom);
    return localFromGeodetic(*spherical_, latDeg, lonDeg, elevation);
}

std::vector<TileKey> keysForRect(const TileRect &rect)
{
    std::vector<TileKey> keys;
    keys.reserve(static_cast<std::size_t>(std::max(0, rect.Width())) *
                 static_cast<std::size_t>(std::max(0, rect.Height())));
    for (int y = rect.minY; y <= rect.maxY; ++y)
        for (int x = rect.minX; x <= rect.maxX; ++x)
            keys.push_back({x, y, rect.zoom});
    return keys;
}

}
