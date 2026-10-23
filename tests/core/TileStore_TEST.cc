#include "dynamic_terrain/core/TileStore.hh"
#include "TestGeographicTransform.hh"

#include <opencv2/imgcodecs.hpp>

#include <chrono>
#include <cmath>
#include <fstream>
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
    require(store.ImageryPath(key) == cache.path / "imagery" /
                TerrainCacheNamespace("custom_provider", cfg.imageryUrl, "png") / "4/3/5.png",
            "imagery cache path lost provider or XYZ key");
    require(store.ElevationPath(key) == cache.path / "elevation" /
                TerrainCacheNamespace(provider, cfg.elevationUrl, provider == "terrarium" ? "png" : "webp") / "4/3" /
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

void testDatasetIdentityAndImageryCache()
{
    CacheFixture cache;
    Config cfg; cfg.cacheDir = cache.path.string(); cfg.diagnostics = false;
    cfg.decodedImageryCacheMb = 1;
    const auto transform = std::make_shared<TestGeographicTransform>();
    TileStore builtin(cfg, transform);
    const TileKey first{3, 5, 4}, second{4, 5, 4};
    require(builtin.ImageryPath(first) == cache.path / "imagery/google_satellite/4/3/5.jpg",
            "built-in imagery cache compatibility changed");
    require(builtin.ElevationPath(first) == cache.path / "elevation/terrarium/4/3/5.png",
            "built-in DEM cache compatibility changed");
    cfg.imageryProvider = "custom"; cfg.imageryExtension = "png";
    cfg.imageryUrl = "https://dataset-a.invalid/{z}/{x}/{y}.png";
    cfg.elevationUrl = "https://dataset-a.invalid/dem/{z}/{x}/{y}.png";
    TileStore a(cfg, transform);
    cfg.imageryUrl = "https://dataset-b.invalid/{z}/{x}/{y}.png";
    cfg.elevationUrl = "https://dataset-b.invalid/dem/{z}/{x}/{y}.png";
    TileStore b(cfg, transform);
    require(a.ImageryPath(first) != b.ImageryPath(first) && a.ElevationPath(first) != b.ElevationPath(first),
            "custom datasets collide in shared cache");
    for (const auto key : {first, second})
    {
        fs::create_directories(a.ImageryPath(key).parent_path());
        require(cv::imwrite(a.ImageryPath(key).string(), cv::Mat(512, 512, CV_8UC3, cv::Scalar(1, 2, 3))),
                "imagery fixture write failed");
    }
    const auto decoded = a.LoadImagery(first);
    require(!decoded.empty() && a.LoadImagery(first).data == decoded.data, "decoded imagery was not reused");
    require(!a.LoadImagery(second).empty(), "second imagery decode failed");
    fs::remove(a.ImageryPath(first)); fs::remove(a.ImageryPath(second));
    require(a.LoadImagery(first).empty() && !a.LoadImagery(second).empty(), "imagery cache exceeded byte budget");
}

void testOversizedDemBypassesCache()
{
    CacheFixture cache;
    Config cfg; cfg.cacheDir = cache.path.string(); cfg.diagnostics = false;
    cfg.alignOriginToGround = false;
    const TileKey key{3, 5, 4};
    for (std::size_t limit : {0u, 1u})
    {
        cfg.decodedDemCacheMb = limit;
        TileStore store(cfg, std::make_shared<TestGeographicTransform>());
        fs::create_directories(store.ElevationPath(key).parent_path());
        require(cv::imwrite(store.ElevationPath(key).string(),
            cv::Mat(1024, 1024, CV_8UC3, cv::Scalar(128, 1, 128))), "oversized DEM fixture write failed");
        const auto raster = store.LoadElevationRaster(key);
        require(raster && raster->rows == 1024 && raster->cols == 1024,
                "oversized DEM contents were reduced");
        near(store.RawElevation(tileYToLat(key.y + 0.5, key.z), tileXToLon(key.x + 0.5, key.z), key.z), 1.5);
        const auto stats = store.Diagnostics();
        require(stats.decodedDemBytes == 0 && stats.decodedDemEntries == 0,
                "oversized single DEM exceeded decoded cache budget");
    }
}

void testCompleteJpegValidationAndRepair()
{
    CacheFixture cache;
    Config cfg; cfg.cacheDir = (cache.path / "cache").string(); cfg.diagnostics = false;
    cfg.imageryProvider = "jpeg-test"; cfg.imageryExtension = "jpg";
    cfg.imageryUrl = (cache.path / "source/{z}/{x}/{y}.jpg").string();
    TileStore store(cfg, std::make_shared<TestGeographicTransform>());
    cv::Mat pixels(64, 64, CV_8UC3);
    for (int y = 0; y < pixels.rows; ++y)
        for (int x = 0; x < pixels.cols; ++x)
            pixels.at<cv::Vec3b>(y, x) = {static_cast<unsigned char>(x * 7 + y * 3),
                static_cast<unsigned char>(x * 3 + y * 5), static_cast<unsigned char>(x * 5 + y * 7)};
    std::vector<unsigned char> complete;
    require(cv::imencode(".jpg", pixels, complete), "JPEG fixture encoding failed");
    auto missingEnd = complete; missingEnd.resize(missingEnd.size() - 2);
    auto truncated = complete; truncated.resize(truncated.size() * 3 / 4);
    truncated.push_back(0xff); truncated.push_back(0xd9);
    const TileKey valid{3, 5, 4}, noEnd{4, 5, 4}, noEntropy{5, 5, 4};
    auto write = [](const fs::path &path, const std::vector<unsigned char> &bytes) {
        fs::create_directories(path.parent_path());
        std::ofstream out(path, std::ios::binary);
        out.write(reinterpret_cast<const char *>(bytes.data()), bytes.size());
    };
    write(store.ImageryUrl(valid), complete);
    write(store.ImageryUrl(noEnd), missingEnd);
    write(store.ImageryUrl(noEntropy), truncated);
    require(store.EnsureImagery({valid}), "complete JPEG was rejected");
    require(!store.EnsureImagery({noEnd}) && !fs::exists(store.ImageryPath(noEnd)), "JPEG missing EOI was published");
    require(!store.EnsureImagery({noEntropy}) && !fs::exists(store.ImageryPath(noEntropy)), "JPEG missing entropy was published");

    require(store.EnsureImagery({valid}), "JPEG cache validation failed");
    const auto stamp = fs::last_write_time(store.ImageryPath(valid));
    write(store.ImageryPath(valid), std::vector<unsigned char>(complete.size(), 'x'));
    fs::last_write_time(store.ImageryPath(valid), stamp);
    const auto recovered = store.LoadImagery(valid);
    require(!recovered.empty() && recovered.rows == pixels.rows && recovered.cols == pixels.cols,
            "actual decoder failure with preserved file metadata did not trigger repair");
}
}

int main()
{
    try
    {
        testElevationDecodeAndCache("terrarium", cv::Vec3b{128, 1, 128}, 1.5);
        testElevationDecodeAndCache("terrain_rgb", cv::Vec3b{164, 134, 1}, 0.4);
        testDisabledElevation();
        testDatasetIdentityAndImageryCache();
        testOversizedDemBypassesCache();
        testCompleteJpegValidationAndRepair();
        std::cout << "TileStore tests passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
