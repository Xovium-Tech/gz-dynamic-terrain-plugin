#pragma once

#include "dynamic_terrain/core/TerrainTypes.hh"
#include "dynamic_terrain/core/DownloadCache.hh"

#include <opencv2/core.hpp>

#include <array>
#include <filesystem>
#include <list>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace dynamic_terrain
{

using DownloadSpec = DownloadRequest;

struct TileStoreDiagnostics
{
    DownloadDiagnostics downloads;
    std::size_t decodedDemBytes{0}, decodedDemEntries{0};
    std::size_t decodedImageryBytes{0}, decodedImageryEntries{0};
};

struct ElevationMosaic
{
    int zoom{0};
    int tileSize{256};
    TileRect rect;
    cv::Mat heights;

    bool Valid() const { return !heights.empty() && heights.type() == CV_32FC1; }
    double Sample(double latDeg, double lonDeg) const;
};

// Worker-local cancellation/priority, restored on scope exit. CPU builders also
// inspect this context at row/page boundaries without changing sampled output.
class ScopedDownloadContext
{
public:
    ScopedDownloadContext(DownloadCancelCheck cancel, int priority)
        : previous_(std::move(check_)), previousPriority_(priority_)
    { check_ = std::move(cancel); priority_ = priority; }
    ~ScopedDownloadContext() { check_ = std::move(previous_); priority_ = previousPriority_; }
    ScopedDownloadContext(const ScopedDownloadContext &) = delete;
    ScopedDownloadContext &operator=(const ScopedDownloadContext &) = delete;
    static bool Cancelled() { return check_ && check_(); }
    static DownloadCancelCheck Check() { return check_; }
    static int Priority() { return priority_; }
private:
    DownloadCancelCheck previous_;
    int previousPriority_;
    inline static thread_local DownloadCancelCheck check_;
    inline static thread_local int priority_{0};
};

class BatchDownloader
{
public:
    explicit BatchDownloader(Config config);
    DownloadStats Ensure(const std::vector<DownloadSpec> &requests,
                         std::vector<fs::path> *failedPaths = nullptr) const;
    void Stop() const { cache_.Stop(); }
    DownloadDiagnostics Diagnostics() const { return cache_.Diagnostics(); }
    void InvalidateValidation(const fs::path &path) const { cache_.InvalidateValidation(path); }

private:
    Config cfg_;
    DownloadCache cache_;
};

class TileStore
{
public:
    TileStore(Config config, std::shared_ptr<const GeographicTransform> spherical);

    const Config &GetConfig() const { return cfg_; }
    const Provider &ImageryProvider() const { return imagery_; }
    const GeographicTransform &Spherical() const { return *spherical_; }
    const fs::path &CacheRoot() const { return cacheRoot_; }
    const std::string &ResourcePrefix() const { return resourcePrefix_; }

    double OriginLatDeg() const { return originLatDeg_; }
    double OriginLonDeg() const { return originLonDeg_; }
    void StopDownloads() const { downloader_.Stop(); }
    TileStoreDiagnostics Diagnostics() const;

    fs::path ImageryPath(const TileKey &key) const;
    fs::path ElevationPath(const TileKey &key) const;
    std::string ImageryUrl(const TileKey &key) const;
    std::string ElevationUrl(const TileKey &key) const;

    bool EnsureImagery(const std::vector<TileKey> &keys,
                       std::vector<TileKey> *failed = nullptr);
    bool EnsureElevation(const std::vector<TileKey> &keys,
                         std::vector<TileKey> *failed = nullptr);
    bool EnsureElevationForBounds(const TileBounds &bounds, int zoom,
                                  int halo = 1);
    std::optional<ElevationMosaic> BuildElevationMosaic(
        const TileBounds &bounds, int zoom, int halo, std::string &error);
    double ElevationAlignmentOffset(int zoom);

    cv::Mat LoadImagery(const TileKey &key) const;
    std::shared_ptr<cv::Mat> LoadElevationRaster(const TileKey &key);

    double RawElevation(double latDeg, double lonDeg, int zoom);
    double SampleElevation(double latDeg, double lonDeg, int zoom);
    Vec3 LocalPoint(double latDeg, double lonDeg, int zoom);

private:
    double DecodeHeight(const cv::Vec3b &bgr) const;
    std::optional<double> GlobalPixelHeight(std::int64_t globalX,
                                             std::int64_t globalY,
                                             int zoom,
                                             int tileWidth,
                                             int tileHeight);
    double SampleInsideRaster(const cv::Mat &raster, double u, double v) const;
    double OriginAlignmentOffset(int zoom);
    void TouchDemLru(const TileKey &key, std::size_t bytes);
    void EvictDemLruIfNeeded();

    Config cfg_;
    std::shared_ptr<const GeographicTransform> spherical_;
    fs::path cacheRoot_;
    Provider imagery_;
    BatchDownloader downloader_;
    bool terrarium_{true};
    std::string resourcePrefix_;
    double originLatDeg_{0.0};
    double originLonDeg_{0.0};

    mutable std::mutex demMutex_;
    std::unordered_map<TileKey, std::shared_ptr<cv::Mat>, TileKeyHash> demMemory_;
    std::unordered_map<TileKey, std::list<TileKey>::iterator, TileKeyHash> demLruPos_;
    std::unordered_map<TileKey, std::size_t, TileKeyHash> demBytes_;
    std::list<TileKey> demLru_;
    std::size_t demMemoryBytes_{0};
    std::array<std::mutex, 64> demFileMutexes_;

    mutable std::mutex imageryMutex_;
    mutable std::unordered_map<TileKey, cv::Mat, TileKeyHash> imageryMemory_;
    mutable std::unordered_map<TileKey, std::list<TileKey>::iterator, TileKeyHash> imageryLruPos_;
    mutable std::list<TileKey> imageryLru_;
    mutable std::size_t imageryMemoryBytes_{0};
    mutable std::array<std::mutex, 64> imageryFileMutexes_;

    std::mutex originMutex_;
    std::unordered_map<int, double> originOffsets_;
};

std::vector<TileKey> keysForRect(const TileRect &rect);

}
