#include "dynamic_terrain/core/TerrainTypes.hh"


#include <array>
#include <cmath>
#include <cstddef>
#include <exception>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace
{
class TestFailure : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};

void check(bool condition, const char *expression, const char *file, int line)
{
    if (condition)
        return;
    std::ostringstream out;
    out << file << ':' << line << ": CHECK failed: " << expression;
    throw TestFailure(out.str());
}

void checkNear(double actual, double expected, double tolerance,
               const char *expression, const char *file, int line)
{
    if (std::isfinite(actual) && std::isfinite(expected) &&
        std::abs(actual - expected) <= tolerance)
        return;
    std::ostringstream out;
    out << std::setprecision(17) << file << ':' << line
        << ": CHECK_NEAR failed: " << expression << " (actual=" << actual
        << ", expected=" << expected << ", tolerance=" << tolerance << ')';
    throw TestFailure(out.str());
}

#define CHECK(expression) \
    check(static_cast<bool>(expression), #expression, __FILE__, __LINE__)
#define CHECK_NEAR(actual, expected, tolerance) \
    checkNear((actual), (expected), (tolerance), \
              #actual " ~= " #expected, __FILE__, __LINE__)

void testTileCoordinatesAndBounds()
{
    using namespace dynamic_terrain;
    const TileKey origin{1, 1, 1};
    CHECK(latLonToTile(0.0, 0.0, 1) == origin);
    const TileKey southwest{0, 7, 3};
    CHECK(latLonToTile(-90.0, -200.0, 3) == southwest);
    const TileKey northeast{7, 0, 3};
    CHECK(latLonToTile(90.0, 200.0, 3) == northeast);
    for (int zoom : {1, 7, 14, 20})
    {
        for (const auto &point : std::array<Vec2, 3>{{{21.0122, 52.2297}, {-123.0, -45.0}, {179.0, 78.0}}})
        {
            const auto fractional = latLonToTileFraction(point.Y(), point.X(), zoom);
            CHECK_NEAR(tileXToLon(fractional.x, zoom), point.X(), 1e-10);
            CHECK_NEAR(tileYToLat(fractional.y, zoom), point.Y(), 1e-10);
            const auto tile = latLonToTile(point.Y(), point.X(), zoom);
            const TileRect rect{tile.x, tile.y, tile.x, tile.y, zoom};
            const auto bounds = tileRectBounds(rect);
            CHECK(bounds.north >= point.Y() && bounds.south <= point.Y());
            CHECK(bounds.west <= point.X() && bounds.east >= point.X());
            // Inset by a tiny tile-relative amount to avoid inverse Mercator
            // floating-point ambiguity exactly on a tile boundary.
            const double inset = (bounds.north - bounds.south) * 1e-6;
            const TileBounds interior{bounds.north - inset, bounds.south + inset,
                                      bounds.west + inset, bounds.east - inset};
            const auto recovered = boundsToTileRect(interior, zoom);
            CHECK(recovered.minX == rect.minX && recovered.maxX == rect.maxX);
            CHECK(recovered.minY == rect.minY && recovered.maxY == rect.maxY);
        }
    }
    CHECK(tileText({3, 5, 7}) == "7/3/5");
}

void testProviderUrlsAndLod()
{
    using namespace dynamic_terrain;
    const Provider provider{"test", "https://tiles/{z}/{x}/{y}/{q}/{s}/{s4}?token={token}", "png", true};
    CHECK(buildUrl(provider, {3, 5, 3}, "secret") == "https://tiles/3/3/5/213/1/2?token=secret");
    CHECK(redactUrl("https://tiles?token=secret&x=2") == "https://tiles?token=<redacted>&x=2");
    Config cfg;
    CHECK(zoomForAltitude(cfg, -10.0) == 18);
    CHECK(zoomForAltitude(cfg, 80.0) == 18);
    CHECK(zoomForAltitude(cfg, 80.01) == 17);
    CHECK(zoomForAltitude(cfg, 5000.0) == 12);
    cfg.dynamicZoom = false;
    cfg.staticZoom = 30;
    CHECK(zoomForAltitude(cfg, 1.0) == cfg.maxZoom);
    CHECK(validHeightmapSize(250) == 257);
    CHECK(validHeightmapSize(1) == 129);
}

void testVectors()
{
    using namespace dynamic_terrain;
    CHECK_NEAR(Vec3(3, 4, 12).Length(), 13.0, 1e-14);
    const auto normal = Vec3::UnitX.Cross(Vec3::UnitY);
    CHECK_NEAR(normal.Distance(Vec3::UnitZ), 0.0, 1e-14);
    CHECK_NEAR(Vec3(3, 4, 12).Normalized().Length(), 1.0, 1e-14);
    CHECK_NEAR(Vec3{}.Normalized().Length(), 0.0, 0.0);
}

std::string textureName(int x, int y)
{
    return "dynamic_terrain_test_page_z14_x" + std::to_string(x) +
           "_y" + std::to_string(y) + "_q17_s2048";
}

std::vector<std::string> textureWindow(int centerX)
{
    std::vector<std::string> result;
    result.reserve(9);
    for (int y = 40; y <= 42; ++y)
        for (int x = centerX - 1; x <= centerX + 1; ++x)
            result.push_back(textureName(x, y));
    return result;
}

void checkReferenceModel(
    const dynamic_terrain::ResourceReferenceCounter &counter,
    const std::vector<std::string> &active,
    const std::vector<std::string> &staging)
{
    std::unordered_map<std::string, std::size_t> expected;
    for (const auto &name : active)
        ++expected[name];
    for (const auto &name : staging)
        ++expected[name];

    CHECK(counter.ResourceCount() == expected.size());
    for (const auto &[name, references] : expected)
        CHECK(counter.References(name) == references);
}

void testReferenceCounterAcrossRecenters()
{
    dynamic_terrain::ResourceReferenceCounter counter;
    CHECK(counter.Acquire("") == 0u);
    CHECK(!counter.Release(""));
    CHECK(!counter.Release("never-acquired"));
    CHECK(counter.ResourceCount() == 0u);

    std::vector<std::string> active;
    std::unordered_set<std::string> allNames;
    std::unordered_map<std::string, std::size_t> deletionCount;
    std::size_t acquiredReferences = 0;
    std::size_t releasedReferences = 0;

    for (int cycle = 0; cycle < 100; ++cycle)
    {
        auto staging = textureWindow(cycle);
        for (const auto &name : staging)
        {
            counter.Acquire(name);
            allNames.insert(name);
            ++acquiredReferences;
        }

        checkReferenceModel(counter, active, staging);
        CHECK(counter.ResourceCount() == (active.empty() ? 9u : 12u));

        for (const auto &name : active)
        {
            const std::size_t before = counter.References(name);
            CHECK(before > 0u);
            const bool finalReference = counter.Release(name);
            ++releasedReferences;
            CHECK(finalReference == (before == 1u));
            if (finalReference)
                ++deletionCount[name];
        }

        active = std::move(staging);
        checkReferenceModel(counter, active, {});
        CHECK(counter.ResourceCount() == 9u);
        for (const auto &name : active)
            CHECK(counter.References(name) == 1u);
    }

    for (const auto &name : active)
    {
        CHECK(counter.Release(name));
        ++releasedReferences;
        ++deletionCount[name];
    }
    active.clear();

    CHECK(acquiredReferences == 900u);
    CHECK(releasedReferences == acquiredReferences);
    CHECK(allNames.size() == 306u);
    CHECK(deletionCount.size() == allNames.size());
    for (const auto &name : allNames)
        CHECK(deletionCount[name] == 1u);
    CHECK(counter.ResourceCount() == 0u);
    CHECK(!counter.Release(textureName(99, 41)));
}

void testCpuPageCacheConfig()
{
    dynamic_terrain::Config config;
    CHECK(config.visualPageCacheMb == 128u);
    config.visualPageCacheMb = 0u;
    dynamic_terrain::normalizeConfig(config);
    CHECK(config.visualPageCacheMb == 0u);
    config.visualPageCacheMb = 4096u;
    dynamic_terrain::normalizeConfig(config);
    CHECK(config.visualPageCacheMb == 2048u);
}

void testMipmappedByteAccounting()
{
    CHECK(dynamic_terrain::mipmappedRgbaBytes(0, 64) == 0u);
    CHECK(dynamic_terrain::mipmappedRgbaBytes(64, 0) == 0u);
    CHECK(dynamic_terrain::mipmappedRgbaBytes(-1, 64) == 0u);
    CHECK(dynamic_terrain::mipmappedRgbaBytes(1, 1) == 4u);
    CHECK(dynamic_terrain::mipmappedRgbaBytes(2, 1) == 12u);
    CHECK(dynamic_terrain::mipmappedRgbaBytes(3, 5) == 72u);
    CHECK(dynamic_terrain::mipmappedRgbaBytes(64, 64) == 21844u);
    CHECK(dynamic_terrain::mipmappedRgbaBytes(256, 256) == 349524u);
}
}

int main()
{
    try
    {
        testTileCoordinatesAndBounds();
        testProviderUrlsAndLod();
        testVectors();
        testReferenceCounterAcrossRecenters();
        testMipmappedByteAccounting();
        testCpuPageCacheConfig();
        std::cout << "TerrainTypes tests passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
