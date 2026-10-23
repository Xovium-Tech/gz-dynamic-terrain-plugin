#include "dynamic_terrain/core/TerrainCoverage.hh"
#include "dynamic_terrain/core/TerrainBuilder.hh"
#include "TestGeographicTransform.hh"

#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <fstream>
#include <set>
#include <stdexcept>
#include <unistd.h>

using namespace dynamic_terrain;
namespace
{
void check(bool value, const char *message)
{
    if (!value) throw std::runtime_error(message);
}
using Box = std::array<std::int64_t, 4>;
Box bounds(const CoverageTile &tile, int zoom)
{
    const auto scale = std::int64_t{1} << (zoom - tile.z);
    return {tile.x * scale, tile.y * scale, (tile.x + 1) * scale, (tile.y + 1) * scale};
}
void planProperties(const CoverageRect &inner, const CoverageRect &outer)
{
    const auto plan = planTerrainCoverage(inner, outer);
    check(plan.leaves.size() <= 1024, "coverage page cap exceeded");
    std::int64_t area = 0;
    std::size_t protectedPages = 0;
    for (std::size_t i = 0; i < plan.leaves.size(); ++i)
    {
        const auto &tile = plan.leaves[i];
        const auto a = bounds(tile, inner.zoom);
        area += (a[2] - a[0]) * (a[3] - a[1]);
        if (tile.inner)
        {
            check(tile.z == inner.zoom, "protected tile coarsened");
            ++protectedPages;
        }
        for (std::size_t j = 0; j < i; ++j)
        {
            const auto b = bounds(plan.leaves[j], inner.zoom);
            const bool overlapX = std::max(a[0], b[0]) < std::min(a[2], b[2]);
            const bool overlapY = std::max(a[1], b[1]) < std::min(a[3], b[3]);
            check(!(overlapX && overlapY), "coverage leaves overlap");
            const bool adjacent = (overlapX && (a[1] == b[3] || a[3] == b[1])) ||
                (overlapY && (a[0] == b[2] || a[2] == b[0]));
            if (adjacent) check(std::abs(tile.z - plan.leaves[j].z) <= 1, "unbalanced edge");
        }
    }
    const auto rootScale = std::int64_t{1} << (inner.zoom - outer.zoom);
    check(area == std::int64_t(outer.maxX - outer.minX + 1) *
        (outer.maxY - outer.minY + 1) * rootScale * rootScale, "coverage has an area hole");
    check(protectedPages == std::size_t(inner.maxX - inner.minX + 1) *
        (inner.maxY - inner.minY + 1), "protected rectangle changed");
    for (int y = std::max(0, inner.minY - 1); y <= std::min((1 << inner.zoom) - 1, inner.maxY + 1); ++y)
        for (int x = std::max(0, inner.minX - 1); x <= std::min((1 << inner.zoom) - 1, inner.maxX + 1); ++x)
            check(std::any_of(plan.leaves.begin(), plan.leaves.end(), [&](const auto &tile)
                { return tile.x == x && tile.y == y && tile.z == inner.zoom; }), "fine halo missing");
}

class NorthEastTransform : public TestGeographicTransform
{
public:
    using TestGeographicTransform::TestGeographicTransform;
    Vec3 LocalFromGeodetic(double lat, double lon, double elevation) const override
    {
        const auto point = TestGeographicTransform::LocalFromGeodetic(lat, lon, elevation);
        return {point.Y(), point.X(), point.Z()};
    }
    Vec3 GeodeticFromLocal(const Vec3 &point) const override
    { return TestGeographicTransform::GeodeticFromLocal({point.Y(), point.X(), point.Z()}); }
};

void testBoundaryTopology()
{
    for (const int cells : {8, 33, 64})
        for (const int fineCells : {8, 33, 67, 128})
        {
            std::array<std::vector<double>, 4> coordinates;
            for (auto &edge : coordinates)
                for (int half = 0; half < 2; ++half)
                    for (int point = 0; point <= fineCells; ++point)
                        edge.push_back(coverageBoundaryCoordinate(point, fineCells,
                            40000 + half, 16, 20000, 15, cells));
            const auto topology = stitchCoverageBoundary(cells, coordinates);
            std::vector<Vec3> vertices;
            for (int y = 0; y <= cells; ++y)
                for (int x = 0; x <= cells; ++x)
                    vertices.emplace_back(x, y, 2 * x + 3 * y + 7);
            for (const auto &recipe : topology.appended)
            {
                Vec3 vertex;
                for (int i = 0; i < 3; ++i) vertex += vertices[recipe.indices[i]] * recipe.weights[i];
                check(std::abs(vertex.Z() - (2 * vertex.X() + 3 * vertex.Y() + 7)) < 1e-10,
                    "edge split changed the original triangle plane");
                vertices.push_back(vertex);
            }
            double area = 0;
            for (std::size_t i = 0; i < topology.indices.size(); i += 3)
            {
                const auto &a = vertices[topology.indices[i]], &b = vertices[topology.indices[i + 1]],
                    &c = vertices[topology.indices[i + 2]];
                const double signedArea = ((b - a).Cross(c - a)).Z() / 2;
                check(signedArea < 0, "edge split changed winding or introduced a degenerate triangle");
                area -= signedArea;
            }
            check(std::abs(area - cells * cells) < 1e-8, "edge split introduced a hole or overlap");
        }
}

void testOuterBuilder(int meshCells, int benchmarkRuns = 0,
                      std::size_t meshCacheMb = 128, bool northEast = false)
{
    const fs::path temp = fs::temp_directory_path() / ("terrain-outer-test-" +
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directories(temp);
    struct Cleanup { fs::path path; ~Cleanup() { std::error_code e; fs::remove_all(path, e); } } cleanup{temp};
    const auto imageFile = temp / "image.png", demFile = temp / "dem.png";
    cv::Mat imagery(32, 32, CV_8UC3, cv::Scalar(45, 80, 120));
    cv::Mat dem(32, 32, CV_8UC3);
    for (int y = 0; y < dem.rows; ++y)
        for (int x = 0; x < dem.cols; ++x)
        {
            const int value = 32768 + 50 + x * 3 + y * 2;
            dem.at<cv::Vec3b>(y, x) = cv::Vec3b(0, value % 256, value / 256);
        }
    check(cv::imwrite(imageFile.string(), imagery) && cv::imwrite(demFile.string(), dem), "fixture write failed");
    Config cfg;
    cfg.cacheDir = (temp / "cache").string();
    cfg.imageryProvider = "custom";
    cfg.imageryUrl = "file://" + imageFile.string();
    cfg.imageryExtension = "png";
    cfg.elevationProvider = "terrarium";
    cfg.elevationUrl = "file://" + demFile.string();
    cfg.visualGeometryZoom = cfg.visualElevationZoom = 15;
    cfg.visualBootstrapImageryZoom = 12;
    cfg.visualRadiusM = 1500;
    cfg.visualOuterRadiusM = 2'000'000;
    cfg.visualOuterMeshCacheMb = meshCacheMb;
    cfg.visualMeshCellsPerTile = meshCells;
    cfg.visualPageTextureMaxSize = benchmarkRuns ? 2048 : 256;
    cfg.visualPageCacheMb = benchmarkRuns ? 128 : 0;
    cfg.downloadRetries = 0;
    cfg.alignOriginToGround = false;
    cfg.diagnostics = false;
    normalizeConfig(cfg);
    const std::shared_ptr<TestGeographicTransform> geo = northEast
        ? std::make_shared<NorthEastTransform>(35.7162542, -120.7615102, 280, 0)
        : std::make_shared<TestGeographicTransform>(35.7162542, -120.7615102, 280, 0);
    auto store = std::make_shared<TileStore>(cfg, geo);
    PersistentTerrainBuilder builder(store);
    std::string error;
    auto near = builder.BuildBootstrap(builder.SnappedCenter(geo->LatitudeDeg(), geo->LongitudeDeg()), 1, error);
    check(bool(near), error.c_str());
    builder.PinActiveTextures(*near);
    auto outer = builder.BuildOuterCoverage(*near, error);
    check(bool(outer), error.c_str());
    check(outer->coverageRevision == 1 && outer->generation == near->generation, "outer revision wrong");
    check(outer->pages.size() > near->pages.size() && outer->pages.size() <= 1024, "outer page count wrong");
    if (benchmarkRuns)
    {
        auto active = outer;
        const auto center = near->centerTile;
        near.reset(); outer.reset();
        builder.PinActiveTextures(*active);
        std::vector<double> durations;
        for (int iteration = 0; iteration < benchmarkRuns; ++iteration)
        {
            const std::array<std::array<int, 2>, 4> route{{{0, 0}, {1, 0}, {1, 1}, {0, 1}}};
            const auto step = route[iteration % route.size()];
            const auto started = std::chrono::steady_clock::now();
            auto local = builder.BuildBootstrap({center.x + step[0], center.y + step[1], center.z},
                                                iteration + 2, error);
            check(bool(local), error.c_str());
            const auto nearDone = std::chrono::steady_clock::now();
            auto replacement = builder.BuildOuterCoverage(*local, error);
            check(bool(replacement), error.c_str());
            const auto finished = std::chrono::steady_clock::now();
            std::size_t reused = 0, overlapping = 0, vertices = 0;
            for (std::size_t i = 0; i < local->pages.size(); ++i)
            {
                check(replacement->pages[i].mesh == local->pages[i].mesh,
                    "benchmark changed protected near mesh");
                check(replacement->pages[i].imageryZoom >= local->pages[i].imageryZoom &&
                    replacement->pages[i].textureSize >= local->pages[i].textureSize,
                    "benchmark downgraded protected near imagery");
            }
            for (const auto &page : replacement->pages)
            {
                vertices += page.mesh->positions.size();
                const auto previous = std::find_if(active->pages.begin(), active->pages.end(),
                    [&](const auto &candidate) { return candidate.key == page.key; });
                if (previous != active->pages.end())
                { ++overlapping; reused += previous->mesh == page.mesh; }
            }
            active = std::move(replacement);
            builder.PinActiveTextures(*active);
            std::uint64_t fingerprint = 14695981039346656037ull;
            auto hash = [&](const void *data, std::size_t bytes)
            {
                const auto *first = static_cast<const unsigned char *>(data);
                for (std::size_t byte = 0; byte < bytes; ++byte)
                { fingerprint ^= first[byte]; fingerprint *= 1099511628211ull; }
            };
            for (const auto &page : active->pages)
            {
                hash(page.mesh->positions.data(), page.mesh->positions.size() * sizeof(Vec3));
                hash(page.mesh->normals.data(), page.mesh->normals.size() * sizeof(Vec3));
                hash(page.mesh->texCoords.data(), page.mesh->texCoords.size() * sizeof(Vec2));
                hash(page.mesh->indices.data(), page.mesh->indices.size() * sizeof(std::uint32_t));
                hash(page.texture->rgb.data(), page.texture->rgb.size());
            }
            const auto cached = builder.CachedOuterMeshStats();
            check(cached.bytes <= cached.limitBytes, "outer mesh cache exceeded byte limit");
            std::ifstream memory("/proc/self/statm");
            std::uint64_t virtualPages = 0, residentPages = 0; memory >> virtualPages >> residentPages;
            const double outerMs = std::chrono::duration<double, std::milli>(finished - nearDone).count();
            durations.push_back(outerMs);
            std::cout << "{\"iteration\":" << iteration << ",\"near_ms\":" <<
                std::chrono::duration<double, std::milli>(nearDone - started).count() <<
                ",\"outer_ms\":" << outerMs << ",\"pages\":" << active->pages.size() <<
                ",\"vertices\":" << vertices << ",\"overlapping_pages\":" << overlapping <<
                ",\"reused_meshes\":" << reused << ",\"mesh_cache_bytes\":" << cached.bytes <<
                ",\"mesh_cache_limit\":" << cached.limitBytes << ",\"buffer_fingerprint\":" << fingerprint <<
                ",\"rss_bytes\":" <<
                residentPages * static_cast<std::uint64_t>(sysconf(_SC_PAGESIZE)) << "}\n";
        }
        std::sort(durations.begin(), durations.end());
        std::cout << "{\"runs\":" << benchmarkRuns << ",\"outer_p50_ms\":" << durations[durations.size() / 2]
                  << ",\"outer_p95_ms\":" << durations[std::min(durations.size() - 1,
                      std::size_t(std::ceil(durations.size() * 0.95) - 1))]
                  << ",\"exact_near_meshes\":true,\"retained_near_imagery\":true}\n";
        return;
    }
    for (std::size_t i = 0; i < near->pages.size(); ++i)
    {
        check(outer->pages[i].mesh == near->pages[i].mesh, "protected mesh copied or changed");
        check(outer->pages[i].texture == near->pages[i].texture, "protected imagery changed");
        check(!outer->pages[i].visualOnly, "near collision membership changed");
    }
    std::size_t seams = 0;
    for (std::size_t i = near->pages.size(); i < outer->pages.size(); ++i)
    {
        const auto &page = outer->pages[i];
        check(page.visualOnly, "outer page has collision");
        for (const auto &normal : page.mesh->normals)
            check(normal.Z() >= 0, "outer normal points down in adapter coordinate system");
        check(builder.TargetZoomForPage(*outer, i) == page.key.z, "outer target promoted to global zoom");
        const auto a = bounds({page.key.x, page.key.y, page.key.z, false}, near->geometryRect.zoom);
        const int cells = page.key.z == near->geometryRect.zoom ? near->cellsPerTile :
            clampValue(near->cellsPerTile, 8, 64);
        for (const auto &neighbor : outer->pages)
        {
            if (neighbor.key == page.key || neighbor.key.z > page.key.z) continue;
            // The fixed inner boundary and every fine/coarse seam must lie on
            // precisely the same piecewise-linear world-space edge.
            if (neighbor.key.z == page.key.z && neighbor.visualOnly) continue;
            const auto b = bounds({neighbor.key.x, neighbor.key.y, neighbor.key.z, false}, near->geometryRect.zoom);
            const bool overlapX = std::max(a[0], b[0]) < std::min(a[2], b[2]);
            const bool overlapY = std::max(a[1], b[1]) < std::min(a[3], b[3]);
            int edge = -1;
            if (a[0] == b[2] && overlapY) edge = 0;
            if (a[2] == b[0] && overlapY) edge = 1;
            if (a[1] == b[3] && overlapX) edge = 2;
            if (a[3] == b[1] && overlapX) edge = 3;
            if (edge < 0) continue;
            const int n = neighbor.key.z == near->geometryRect.zoom ? near->cellsPerTile :
                clampValue(near->cellsPerTile, 8, 64);
            using FloatPoint = std::array<float, 3>;
            using FloatEdge = std::pair<FloatPoint, FloatPoint>;
            auto floatPoint = [](const Vec3 &point) -> FloatPoint
            { return {float(point.X() * 100), float(point.Y() * 100), float(point.Z() * 100)}; };
            auto floatEdge = [&](const Vec3 &first, const Vec3 &second) -> FloatEdge
            {
                const auto a = floatPoint(first), b = floatPoint(second);
                return a < b ? FloatEdge{a, b} : FloatEdge{b, a};
            };
            std::set<FloatEdge> neighborEdges;
            std::set<FloatEdge> pageEdges;
            const double boundaryBegin = coverageBoundaryCoordinate(0, cells,
                edge < 2 ? page.key.y : page.key.x, page.key.z,
                edge < 2 ? neighbor.key.y : neighbor.key.x, neighbor.key.z, 1);
            const double boundaryEnd = coverageBoundaryCoordinate(cells, cells,
                edge < 2 ? page.key.y : page.key.x, page.key.z,
                edge < 2 ? neighbor.key.y : neighbor.key.x, neighbor.key.z, 1);
            auto onNeighborBoundary = [&](std::uint32_t index)
            {
                const auto &uv = neighbor.mesh->texCoords[index];
                const double coordinate = edge < 2 ? uv.X() : uv.Y();
                const double along = edge < 2 ? uv.Y() : uv.X();
                return std::abs(coordinate - ((edge == 0 || edge == 2) ? 1.0 : 0.0)) < 1e-12 &&
                    along >= boundaryBegin - 1e-12 && along <= boundaryEnd + 1e-12;
            };
            for (std::size_t index = 0; index < neighbor.mesh->indices.size(); index += 3)
                for (int side = 0; side < 3; ++side)
                {
                    const auto a = neighbor.mesh->indices[index + side];
                    const auto b = neighbor.mesh->indices[index + (side + 1) % 3];
                    if (onNeighborBoundary(a) && onNeighborBoundary(b))
                        neighborEdges.insert(floatEdge(neighbor.mesh->positions[a], neighbor.mesh->positions[b]));
                }
            auto onPageBoundary = [&](std::uint32_t index)
            {
                const auto &uv = page.mesh->texCoords[index];
                return std::abs((edge < 2 ? uv.X() : uv.Y()) -
                    ((edge == 0 || edge == 2) ? 0.0 : 1.0)) < 1e-12;
            };
            for (std::size_t index = 0; index < page.mesh->indices.size(); index += 3)
                for (int side = 0; side < 3; ++side)
                {
                    const auto a = page.mesh->indices[index + side];
                    const auto b = page.mesh->indices[index + (side + 1) % 3];
                    if (onPageBoundary(a) && onPageBoundary(b))
                        pageEdges.insert(floatEdge(page.mesh->positions[a], page.mesh->positions[b]));
                }
            check(pageEdges == neighborEdges,
                "raster seam: coarse/fine referenced edges differ after centimeter float conversion");
            for (int p = 0; p <= cells; ++p)
            {
                const double coordinate = coverageBoundaryCoordinate(p, cells,
                    edge < 2 ? page.key.y : page.key.x, page.key.z,
                    edge < 2 ? neighbor.key.y : neighbor.key.x, neighbor.key.z, n);
                const int first = std::min(n - 1, int(std::floor(std::max(0.0, coordinate))));
                const double fraction = std::min(double(n), coordinate) - first;
                auto index = [&](int along) -> std::size_t
                {
                    if (edge == 0) return along * (n + 1) + n;
                    if (edge == 1) return along * (n + 1);
                    if (edge == 2) return n * (n + 1) + along;
                    return along;
                };
                const auto expected = neighbor.mesh->positions[index(first)] * (1.0 - fraction) +
                    neighbor.mesh->positions[index(first + 1)] * fraction;
                const auto destination = edge == 0 ? p * (cells + 1) : edge == 1 ? p * (cells + 1) + cells :
                    edge == 2 ? p : cells * (cells + 1) + p;
                check((page.mesh->positions[destination] - expected).Length() < 1e-6, "terrain edge crack");
            }
            ++seams;
        }
    }
    check(seams > 50, "insufficient mixed-resolution seam coverage");
    auto promoted = *outer;
    promoted.pages.front().imageryZoom += 1;
    promoted.pages.front().textureName += "_promoted";
    builder.PinActiveTextures(promoted);
    builder.PinActiveTextures(*near); // Older revision must not erase outer pins.
    auto again = builder.BuildOuterCoverage(*near, error);
    check(bool(again), error.c_str());
    check(again->pages.front().textureName == promoted.pages.front().textureName,
          "older revision erased a newer active texture");
    ScopedDownloadContext cancelled([] { return true; }, 10);
    check(!builder.BuildOuterCoverage(*near, error), "cancelled outer build published");
    std::cout << "outer-coverage mesh_cells=" << meshCells << " pages=" << outer->pages.size() << " near_pages=" << near->pages.size()
              << " checked_seams=" << seams << " exact_near_buffers=true radius_m=2000000\n";
}
}

int main(int argc, char **argv)
{
    try
    {
        if ((argc == 3 || argc == 5) && std::string(argv[1]) == "--benchmark")
        {
            const int runs = std::stoi(argv[2]);
            check(runs > 0 && runs <= 1000, "invalid benchmark run count");
            const auto cacheMb = argc == 5 ? std::stoul(argv[4]) : 128;
            check(argc != 5 || std::string(argv[3]) == "--mesh-cache-mb", "unknown benchmark option");
            testOuterBuilder(64, runs, cacheMb);
            return 0;
        }
        planProperties({100, 100, 104, 104, 8}, {2, 2, 4, 4, 3});
        planProperties({0, 0, 2, 2, 8}, {0, 0, 0, 0, 0});
        planProperties({32764, 16382, 32767, 16386, 15}, {0, 0, 0, 0, 0});
        bool capped = false;
        try { planTerrainCoverage({100, 100, 104, 104, 8}, {2, 2, 4, 4, 3}, 10); }
        catch (const std::runtime_error &) { capped = true; }
        check(capped, "coverage limit ignored");
        testBoundaryTopology();
        testOuterBuilder(8);
        testOuterBuilder(33);
        testOuterBuilder(67);
        testOuterBuilder(128);
        testOuterBuilder(8, 0, 128, true);
        return 0;
    }
    catch (const std::exception &failure)
    { std::cerr << failure.what() << '\n'; return 1; }
}
