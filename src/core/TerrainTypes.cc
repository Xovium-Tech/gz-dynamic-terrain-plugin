#include "dynamic_terrain/core/TerrainTypes.hh"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace dynamic_terrain
{
namespace
{
std::string quadKey(int tileX, int tileY, int zoom)
{
    std::string out;
    out.reserve(static_cast<std::size_t>(zoom));
    for (int i = zoom; i > 0; --i)
    {
        char digit = '0';
        const int mask = 1 << (i - 1);
        if (tileX & mask)
            ++digit;
        if (tileY & mask)
            digit += 2;
        out.push_back(digit);
    }
    return out;
}

void replaceAll(std::string &value, const std::string &needle,
                const std::string &replacement)
{
    if (needle.empty())
        return;
    std::size_t pos = 0;
    while ((pos = value.find(needle, pos)) != std::string::npos)
    {
        value.replace(pos, needle.size(), replacement);
        pos += replacement.size();
    }
}

Provider builtinImageryProvider(const std::string &requested)
{
    const std::string name = providerKey(requested);
    if (name == "google_street")
        return {"google_street",
                "https://mt{s}.google.com/vt/lyrs=m&hl=en&x={x}&y={y}&z={z}",
                "png", false};
    if (name == "google_terrain")
        return {"google_terrain",
                "https://mt{s}.google.com/vt/v=t,r&hl=en&x={x}&y={y}&z={z}",
                "png", false};
    if (name == "google_hybrid")
        return {"google_hybrid",
                "https://mt{s}.google.com/vt/lyrs=y&hl=en&x={x}&y={y}&z={z}",
                "png", false};
    if (name == "google_labels")
        return {"google_labels",
                "https://mt{s}.google.com/vt/lyrs=h&hl=en&x={x}&y={y}&z={z}",
                "png", false};
    if (name == "bing_road")
        return {"bing_road",
                "https://ecn.t{s}.tiles.virtualearth.net/tiles/r{q}.png?g=2981&mkt=en",
                "png", true};
    if (name == "bing_satellite")
        return {"bing_satellite",
                "https://ecn.t{s}.tiles.virtualearth.net/tiles/a{q}.jpg?g=2981&mkt=en",
                "jpg", true};
    if (name == "bing_hybrid")
        return {"bing_hybrid",
                "https://ecn.t{s}.tiles.virtualearth.net/tiles/h{q}.jpg?g=2981&mkt=en",
                "jpg", true};
    return {"google_satellite",
            "https://mt{s}.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}",
            "jpg", false};
}

} // namespace

std::size_t TileKeyHash::operator()(const TileKey &key) const noexcept
{
    std::size_t h = std::hash<int>{}(key.z);
    h ^= std::hash<int>{}(key.x) + 0x9e3779b9u + (h << 6u) + (h >> 2u);
    h ^= std::hash<int>{}(key.y) + 0x9e3779b9u + (h << 6u) + (h >> 2u);
    return h;
}

std::size_t ResourceReferenceCounter::Acquire(const std::string &name)
{
    if (name.empty())
        return 0;
    return ++references_[name];
}

bool ResourceReferenceCounter::Release(const std::string &name)
{
    const auto it = references_.find(name);
    if (it == references_.end())
        return false;
    if (it->second > 1)
    {
        --it->second;
        return false;
    }
    references_.erase(it);
    return true;
}

std::size_t ResourceReferenceCounter::References(const std::string &name) const
{
    const auto it = references_.find(name);
    return it == references_.end() ? 0u : it->second;
}

std::mutex &diagnosticLogMutex()
{
    static std::mutex mutex;
    return mutex;
}

std::string lower(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

std::string providerKey(std::string value)
{
    value = lower(std::move(value));
    for (char &c : value)
        if (c == ' ' || c == '-')
            c = '_';
    return value;
}

std::string expandHome(std::string path)
{
    if (!path.empty() && path.front() == '~')
    {
        if (const char *home = std::getenv("HOME"))
            path.replace(0, 1, home);
    }
    return path;
}

std::string xmlEscape(std::string value)
{
    replaceAll(value, "&", "&amp;");
    replaceAll(value, "<", "&lt;");
    replaceAll(value, ">", "&gt;");
    replaceAll(value, "\"", "&quot;");
    replaceAll(value, "'", "&apos;");
    return value;
}

std::string tileText(const TileKey &key)
{
    return std::to_string(key.z) + "/" + std::to_string(key.x) + "/" +
           std::to_string(key.y);
}

std::string redactUrl(std::string url)
{
    const std::vector<std::string> keys{"access_token=", "token=", "api_key=", "key="};
    for (const auto &key : keys)
    {
        std::size_t pos = 0;
        while ((pos = url.find(key, pos)) != std::string::npos)
        {
            const std::size_t start = pos + key.size();
            std::size_t end = url.find('&', start);
            if (end == std::string::npos)
                end = url.size();
            url.replace(start, end - start, "<redacted>");
            pos = start + 10;
        }
    }
    return url;
}

TileKey latLonToTile(double latDeg, double lonDeg, int zoom)
{
    latDeg = clampValue(latDeg, -kMercatorLatLimit, kMercatorLatLimit);
    lonDeg = clampValue(lonDeg, -180.0, 180.0);
    const int count = 1 << zoom;
    const double latRad = latDeg * kPi / 180.0;
    const int x = clampValue(static_cast<int>(std::floor((lonDeg + 180.0) / 360.0 * count)), 0, count - 1);
    const int y = clampValue(static_cast<int>(std::floor((1.0 - std::asinh(std::tan(latRad)) / kPi) * 0.5 * count)), 0, count - 1);
    return {x, y, zoom};
}

TileCoordF latLonToTileFraction(double latDeg, double lonDeg, int zoom)
{
    latDeg = clampValue(latDeg, -kMercatorLatLimit, kMercatorLatLimit);
    lonDeg = clampValue(lonDeg, -180.0, 180.0);
    const double n = static_cast<double>(1u << zoom);
    const double latRad = latDeg * kPi / 180.0;
    return {(lonDeg + 180.0) / 360.0 * n,
            (1.0 - std::asinh(std::tan(latRad)) / kPi) * 0.5 * n};
}

double tileXToLon(double x, int zoom)
{
    return x / static_cast<double>(1u << zoom) * 360.0 - 180.0;
}

double tileYToLat(double y, int zoom)
{
    const double n = kPi - 2.0 * kPi * y / static_cast<double>(1u << zoom);
    return std::atan(std::sinh(n)) * 180.0 / kPi;
}

TileBounds tileRectBounds(const TileRect &rect)
{
    return {tileYToLat(rect.minY, rect.zoom),
            tileYToLat(rect.maxY + 1.0, rect.zoom),
            tileXToLon(rect.minX, rect.zoom),
            tileXToLon(rect.maxX + 1.0, rect.zoom)};
}

TileRect boundsToTileRect(const TileBounds &bounds, int zoom, int halo)
{
    const int count = 1 << zoom;
    const TileCoordF nw = latLonToTileFraction(bounds.north, bounds.west, zoom);
    const TileCoordF se = latLonToTileFraction(bounds.south, bounds.east, zoom);
    int minX = static_cast<int>(std::floor(std::min(nw.x, se.x))) - halo;
    int maxX = static_cast<int>(std::floor(std::max(nw.x, se.x) - 1e-10)) + halo;
    int minY = static_cast<int>(std::floor(std::min(nw.y, se.y))) - halo;
    int maxY = static_cast<int>(std::floor(std::max(nw.y, se.y) - 1e-10)) + halo;
    minX = clampValue(minX, 0, count - 1);
    maxX = clampValue(maxX, 0, count - 1);
    minY = clampValue(minY, 0, count - 1);
    maxY = clampValue(maxY, 0, count - 1);
    return {minX, minY, maxX, maxY, zoom};
}

Provider resolveImageryProvider(const Config &cfg)
{
    Provider provider = builtinImageryProvider(cfg.imageryProvider);
    if (!cfg.imageryUrl.empty())
    {
        provider.name = providerKey(cfg.imageryProvider);
        if (provider.name.empty())
            provider.name = "custom";
        provider.url = cfg.imageryUrl;
        provider.quadKey = provider.url.find("{q}") != std::string::npos;
    }
    if (!cfg.imageryExtension.empty())
        provider.extension = cfg.imageryExtension;
    return provider;
}

std::string buildUrl(const Provider &provider, const TileKey &key,
                     const std::string &token)
{
    std::string url = provider.url;
    const int server = (key.x + 2 * key.y) % 4;
    replaceAll(url, "{s}", std::to_string(server));
    replaceAll(url, "{s4}", std::to_string(server + 1));
    replaceAll(url, "{x}", std::to_string(key.x));
    replaceAll(url, "{y}", std::to_string(key.y));
    replaceAll(url, "{z}", std::to_string(key.z));
    replaceAll(url, "{q}", quadKey(key.x, key.y, key.z));
    replaceAll(url, "{token}", token);
    return url;
}

int zoomForAltitude(const Config &cfg, double altitude)
{
    if (!cfg.dynamicZoom)
        return clampValue(cfg.staticZoom, cfg.minZoom, cfg.maxZoom);
    altitude = std::max(0.0, altitude);
    for (const auto &[maxAltitude, zoom] : cfg.zoomTable)
        if (altitude <= maxAltitude)
            return clampValue(zoom, cfg.minZoom, cfg.maxZoom);
    return cfg.minZoom;
}

int validHeightmapSize(int requested)
{
    constexpr int sizes[] = {129, 257, 513, 1025, 2049, 4097};
    int best = sizes[0];
    int distance = std::abs(requested - best);
    for (const int candidate : sizes)
    {
        const int d = std::abs(requested - candidate);
        if (d < distance)
        {
            best = candidate;
            distance = d;
        }
    }
    return best;
}

Vec3 localFromGeodetic(const GeographicTransform &transform,
                       double latDeg, double lonDeg, double elevationM)
{
    return transform.LocalFromGeodetic(latDeg, lonDeg, elevationM);
}

Vec3 geodeticFromLocal(const GeographicTransform &transform, const Vec3 &local)
{
    return transform.GeodeticFromLocal(local);
}

std::size_t mipmappedRgbaBytes(int width, int height)
{
    if (width <= 0 || height <= 0)
        return 0;
    std::size_t pixels = 0;
    while (true)
    {
        pixels += static_cast<std::size_t>(width) *
                  static_cast<std::size_t>(height);
        if (width == 1 && height == 1)
            break;
        width = std::max(1, width / 2);
        height = std::max(1, height / 2);
    }
    return pixels * 4u;
}

void normalizeConfig(Config &cfg)
{
    cfg.imageryProvider = providerKey(cfg.imageryProvider);
    cfg.elevationProvider = providerKey(cfg.elevationProvider);

    cfg.minZoom = clampValue(cfg.minZoom, 1, 20);
    cfg.maxZoom = clampValue(cfg.maxZoom, cfg.minZoom, 20);
    cfg.staticZoom = clampValue(cfg.staticZoom, cfg.minZoom, cfg.maxZoom);
    cfg.elevationMaxZoom = clampValue(cfg.elevationMaxZoom, 1, 20);

    cfg.visualRadiusM = clampValue(cfg.visualRadiusM, 1000.0, 30000.0);
    cfg.visualGeometryZoom = clampValue(cfg.visualGeometryZoom, 8, 18);
    cfg.visualElevationZoom = clampValue(cfg.visualElevationZoom, 8, cfg.elevationMaxZoom);
    cfg.visualMeshCellsPerTile = clampValue(cfg.visualMeshCellsPerTile, 8, 128);
    cfg.visualMaxMeshCells = clampValue(cfg.visualMaxMeshCells, 128, 1536);
    cfg.visualTextureSize = clampValue(cfg.visualTextureSize, 1024, 16384);
    cfg.visualPageTextureMaxSize = clampValue(cfg.visualPageTextureMaxSize, 256, 4096);
    cfg.visualPageCacheMb = std::min<std::size_t>(cfg.visualPageCacheMb, 2048u);
    cfg.visualAtlasPagePixels = clampValue(cfg.visualAtlasPagePixels, 256, 2048);
    cfg.visualAtlasMaxSize = clampValue(cfg.visualAtlasMaxSize, 4096, 16384);
    if (cfg.visualPageTextureMaxSize < 512) cfg.visualPageTextureMaxSize = 256;
    else if (cfg.visualPageTextureMaxSize < 1024) cfg.visualPageTextureMaxSize = 512;
    else if (cfg.visualPageTextureMaxSize < 2048) cfg.visualPageTextureMaxSize = 1024;
    else if (cfg.visualPageTextureMaxSize < 4096) cfg.visualPageTextureMaxSize = 2048;
    else cfg.visualPageTextureMaxSize = 4096;
    cfg.visualTextureGuardM = clampValue(cfg.visualTextureGuardM, 0.0, 5000.0);
    cfg.visualDetailMode = lower(cfg.visualDetailMode);
    if (cfg.visualDetailMode != "bottom_camera_only" &&
        cfg.visualDetailMode != "all")
        cfg.visualDetailMode = "all";
    cfg.visualDetailRadiusM = clampValue(cfg.visualDetailRadiusM, 100.0,
                                         cfg.visualRadiusM);
    cfg.visualDetailZoom = clampValue(cfg.visualDetailZoom,
                                      cfg.visualGeometryZoom, 20);
    cfg.visualBootstrapImageryZoom = clampValue(cfg.visualBootstrapImageryZoom, 8, cfg.visualGeometryZoom);
    cfg.visualRecenterDistanceM = clampValue(cfg.visualRecenterDistanceM, 100.0,
                                             std::max(100.0, cfg.visualRadiusM * 0.5));
    cfg.visualWarmupFrames = clampValue(cfg.visualWarmupFrames, 1, 10);
    cfg.visualOffscreenFrames = clampValue(cfg.visualOffscreenFrames, 1, 600);
    cfg.visualRecenterReadyZoom = clampValue(
        cfg.visualRecenterReadyZoom, cfg.visualGeometryZoom, 20);
    cfg.visualRefineMaxSourceTilesPerBatch = clampValue(
        cfg.visualRefineMaxSourceTilesPerBatch, 1, 256);
    for (auto &entry : cfg.visualImageryLodTable)
        entry.second = clampValue(entry.second, 1, 20);
    std::sort(cfg.visualImageryLodTable.begin(), cfg.visualImageryLodTable.end(),
              [](const auto &a, const auto &b) { return a.first < b.first; });

    cfg.downloadConcurrency = clampValue(cfg.downloadConcurrency, 1, 16);
    cfg.downloadPerHost = clampValue(cfg.downloadPerHost, 1,
                                     cfg.downloadConcurrency);
    cfg.downloadRetries = clampValue(cfg.downloadRetries, 0, 5);
    cfg.httpTimeoutMs = clampValue(cfg.httpTimeoutMs, 1000u, 30000u);
    cfg.decodedDemCacheMb = clampValue<std::size_t>(cfg.decodedDemCacheMb, 32u, 2048u);

    cfg.radiusTiles = clampValue(cfg.radiusTiles, 0, 8);
    cfg.meshCells = clampValue(cfg.meshCells, 4, 128);
    cfg.heightmapSize = validHeightmapSize(cfg.heightmapSize);
    cfg.collisionOverlapM = clampValue(cfg.collisionOverlapM, 0.0, 10.0);
    cfg.collisionRecenterFraction = clampValue(cfg.collisionRecenterFraction, 0.10, 0.49);
    cfg.startupSafetySizeM = clampValue(cfg.startupSafetySizeM, 10.0, 10000.0);
    cfg.startupSafetyThicknessM = clampValue(cfg.startupSafetyThicknessM, 0.02, 10.0);
    cfg.startupSafetyRemoveDelaySec = clampValue(cfg.startupSafetyRemoveDelaySec, 0.0, 10.0);
    cfg.updatePeriodSec = std::max(0.02, cfg.updatePeriodSec);
    cfg.retryDelaySec = std::max(0.2, cfg.retryDelaySec);
    cfg.statusPeriodSec = std::max(0.5, cfg.statusPeriodSec);
}

}
