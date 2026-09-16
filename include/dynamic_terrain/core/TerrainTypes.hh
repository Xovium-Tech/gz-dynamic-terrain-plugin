#pragma once

#include "dynamic_terrain/core/TerrainConfig.hh"
#include "dynamic_terrain/core/GeographicTransform.hh"

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace dynamic_terrain
{
namespace fs = std::filesystem;

constexpr double kPi = 3.14159265358979323846;
constexpr double kMercatorLatLimit = 85.05112878;

struct TileKey
{
    int x{0};
    int y{0};
    int z{0};

    bool operator==(const TileKey &other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }
    bool operator!=(const TileKey &other) const { return !(*this == other); }
};

struct TileKeyHash
{
    std::size_t operator()(const TileKey &key) const noexcept;
};

struct TileCoordF
{
    double x{0.0};
    double y{0.0};
};

struct TileBounds
{
    double north{0.0};
    double south{0.0};
    double west{0.0};
    double east{0.0};
};

struct TileRect
{
    int minX{0};
    int minY{0};
    int maxX{0};
    int maxY{0};
    int zoom{0};

    int Width() const { return maxX - minX + 1; }
    int Height() const { return maxY - minY + 1; }
    bool Contains(const TileKey &key) const
    {
        return key.z == zoom && key.x >= minX && key.x <= maxX &&
               key.y >= minY && key.y <= maxY;
    }
};

struct Provider
{
    std::string name;
    std::string url;
    std::string extension;
    bool quadKey{false};
};


class ResourceReferenceCounter
{
public:
    std::size_t Acquire(const std::string &name);
    bool Release(const std::string &name);
    std::size_t References(const std::string &name) const;
    std::size_t ResourceCount() const { return references_.size(); }

private:
    std::unordered_map<std::string, std::size_t> references_;
};

template <typename T>
T clampValue(T value, T lo, T hi)
{
    return std::max(lo, std::min(value, hi));
}

std::mutex &diagnosticLogMutex();

template <typename... Args>
void logInfo(Args &&...args)
{
    std::lock_guard<std::mutex> lock(diagnosticLogMutex());
    (std::cout << ... << std::forward<Args>(args)) << std::endl;
}

template <typename... Args>
void logError(Args &&...args)
{
    std::lock_guard<std::mutex> lock(diagnosticLogMutex());
    (std::cerr << ... << std::forward<Args>(args)) << std::endl;
}

std::string lower(std::string value);
std::string providerKey(std::string value);
std::string expandHome(std::string path);
std::string xmlEscape(std::string value);
std::string tileText(const TileKey &key);
std::string redactUrl(std::string url);

TileKey latLonToTile(double latDeg, double lonDeg, int zoom);
TileCoordF latLonToTileFraction(double latDeg, double lonDeg, int zoom);
double tileXToLon(double x, int zoom);
double tileYToLat(double y, int zoom);
TileBounds tileRectBounds(const TileRect &rect);
TileRect boundsToTileRect(const TileBounds &bounds, int zoom, int halo = 0);
Provider resolveImageryProvider(const Config &cfg);
std::string buildUrl(const Provider &provider, const TileKey &key, const std::string &token);
int zoomForAltitude(const Config &cfg, double altitude);
int validHeightmapSize(int requested);

Vec3 localFromGeodetic(const GeographicTransform &transform, double latDeg, double lonDeg, double elevationM);
Vec3 geodeticFromLocal(const GeographicTransform &transform, const Vec3 &local);

std::size_t mipmappedRgbaBytes(int width, int height);

void normalizeConfig(Config &cfg);


}
