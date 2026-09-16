#include "dynamic_terrain/core/TileStore.hh"
#include "TestGeographicTransform.hh"

#include <opencv2/imgcodecs.hpp>

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace
{
using namespace dynamic_terrain;

void require(bool value, const std::string &message)
{
    if (!value)
        throw std::runtime_error(message);
}

void near(double actual, double expected)
{
    require(std::abs(actual - expected) < 1e-7,
            "height mismatch: " + std::to_string(actual) + " vs " + std::to_string(expected));
}

struct CacheFixture
{
    fs::path path = fs::temp_directory_path() / ("dynamic-terrain-dem-test-" +
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    CacheFixture() { fs::create_directories(path); }
    ~CacheFixture() { std::error_code ec; fs::remove_all(path, ec); }
};

void testElevationDecodeAndCache(const std::string &provider, const cv::Vec3b &pixel,
                                 double expected)
{
    CacheFixture cache;
    Config cfg;
    cfg.cacheDir = cache.path.string();
    cfg.imageryProvider = "Custom provider";
    cfg.imageryUrl = "https://example.invalid/{z}/{x}/{y}.png";
    cfg.imageryExtension = "png";
    cfg.elevationProvider = provider;
    cfg.elevationUrl = "https://example.invalid/{z}/{x}/{y}";
    cfg.alignOriginToGround = false;
    cfg.diagnostics = false;
    TileStore store(cfg, std::make_shared<TestGeographicTransform>());
    const TileKey key{3, 5, 4};
    require(store.ImageryPath(key) == cache.path / "imagery/custom_provider/4/3/5.png",
            "imagery cache path lost provider or XYZ key");
    require(store.ElevationPath(key) == cache.path / "elevation" / provider / "4/3" /
                (provider == "terrarium" ? "5.png" : "5.webp"),
            "elevation cache path mismatch");
    fs::create_directories(store.ElevationPath(key).parent_path());
    const cv::Mat pixels(4, 4, CV_8UC3, cv::Scalar(pixel[0], pixel[1], pixel[2]));
    // WebP quality > 100 explicitly requests lossless encoding of the RGB DEM.
    require(cv::imwrite(store.ElevationPath(key).string(), pixels,
                provider == "terrarium" ? std::vector<int>{} :
                    std::vector<int>{cv::IMWRITE_WEBP_QUALITY, 101}), "write DEM fixture");
    const double latitude = tileYToLat(key.y + 0.5, key.z);
    const double longitude = tileXToLon(key.x + 0.5, key.z);
    near(store.RawElevation(latitude, longitude, key.z), expected);
    auto first = store.LoadElevationRaster(key);
    require(first && !first->empty(), "decoded DEM missing");
    fs::remove(store.ElevationPath(key));
    require(store.LoadElevationRaster(key) == first, "decoded DEM memory cache missed");
    near(store.RawElevation(latitude, longitude, key.z), expected);
}

void testDisabledElevation()
{
    CacheFixture cache;
    for (const auto &provider : {"none", "flat"})
    {
        Config cfg;
        cfg.cacheDir = cache.path.string();
        cfg.elevationProvider = provider;
        cfg.zOffsetM = 7.5;
        cfg.diagnostics = false;
        const auto geographic = std::make_shared<TestGeographicTransform>();
        TileStore store(cfg, geographic);
        near(store.RawElevation(0.0, 0.0, 14), geographic->ElevationReference());
        near(store.LocalPoint(geographic->LatitudeDeg(), geographic->LongitudeDeg(), 14).Z(), 7.5);
        std::string error;
        const TileRect rect{3, 5, 3, 5, 4};
        auto mosaic = store.BuildElevationMosaic(tileRectBounds(rect), 4, 0, error);
        require(mosaic && mosaic->Valid() && error.empty(), "flat elevation mosaic failed");
        near(mosaic->Sample(tileYToLat(5.5, 4), tileXToLon(3.5, 4)),
             static_cast<float>(geographic->ElevationReference()));
    }
    require(fs::is_empty(cache.path), "disabled elevation unexpectedly wrote cache files");
}
}

int main()
{
    try
    {
        testElevationDecodeAndCache("terrarium", cv::Vec3b{128, 1, 128}, 1.5);
        testElevationDecodeAndCache("terrain_rgb", cv::Vec3b{164, 134, 1}, 0.4);
        testDisabledElevation();
        std::cout << "TileStore tests passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
