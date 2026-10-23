// Compile this SAME file against baseline and candidate core libraries. The hash
// covers exact mesh/image/collision data, excluding resource names and timings.
#include "dynamic_terrain/core/TerrainBuilder.hh"
#include "dynamic_terrain/core/CollisionTerrain.hh"
#include "../tests/core/TestGeographicTransform.hh"
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <unistd.h>

using namespace dynamic_terrain;
namespace
{
std::uint64_t hashValue = 14695981039346656037ull;
void bytes(const void *data, std::size_t size)
{
    const auto *p = static_cast<const unsigned char *>(data);
    for (std::size_t i = 0; i < size; ++i) { hashValue ^= p[i]; hashValue *= 1099511628211ull; }
}
template<class T> void value(const T &v) { bytes(&v, sizeof v); }
void hashSnapshot(const TerrainSnapshot &snapshot)
{
    value(snapshot.cellsPerTile); value(snapshot.cellsX); value(snapshot.cellsY);
    for (const auto &page : snapshot.pages)
    {
        value(page.key.x); value(page.key.y); value(page.key.z);
        value(page.imageryZoom); value(page.targetImageryZoom); value(page.textureSize);
        for (const auto &p : page.mesh->positions) { value(p.X()); value(p.Y()); value(p.Z()); }
        for (const auto &n : page.mesh->normals) { value(n.X()); value(n.Y()); value(n.Z()); }
        for (const auto &uv : page.mesh->texCoords) { value(uv.X()); value(uv.Y()); }
        bytes(page.mesh->indices.data(), page.mesh->indices.size() * sizeof(std::uint32_t));
        bytes(page.texture->rgb.data(), page.texture->rgb.size());
    }
}
}

int main(int argc, char **argv)
{
    const fs::path root = fs::temp_directory_path() / ("terrain-parity-" + std::to_string(getpid()));
    try
    {
        const int loops = argc > 1 ? std::max(1, std::stoi(argv[1])) : 4;
        Config cfg; cfg.modelName = "parity"; cfg.cacheDir = root.string(); cfg.diagnostics = false;
        cfg.imageryProvider = "synthetic"; cfg.imageryUrl = "http://127.0.0.1:1/{z}/{x}/{y}.png";
        cfg.imageryExtension = "png"; cfg.elevationProvider = "terrarium";
        cfg.elevationUrl = "http://127.0.0.1:1/dem/{z}/{x}/{y}.png";
        cfg.visualGeometryZoom = 14; cfg.visualElevationZoom = 13; cfg.visualRadiusM = 1000;
        cfg.visualMeshCellsPerTile = 32; cfg.visualMaxMeshCells = 512;
        cfg.visualPageTextureMaxSize = 1024; cfg.visualBootstrapImageryZoom = 12;
        cfg.visualImageryLodTable = {{1000000, 16}}; cfg.alignOriginToGround = false;
        cfg.visualPageCacheMb = 0; cfg.downloadRetries = 0;
        normalizeConfig(cfg);
        auto geography = std::make_shared<TestGeographicTransform>();
        auto store = std::make_shared<TileStore>(cfg, geography);
        const auto center = latLonToTile(geography->LatitudeDeg(), geography->LongitudeDeg(), 14);
        const TileRect unionRect{center.x - 1, center.y - 1, center.x + 2, center.y + 1, 14};
        const auto bounds = tileRectBounds(unionRect);
        for (int zoom = 12; zoom <= 16; ++zoom)
            for (const auto &key : keysForRect(boundsToTileRect(bounds, zoom, 1)))
            {
                cv::Mat raster(256, 256, CV_8UC3);
                for (int y = 0; y < 256; ++y)
                    for (int x = 0; x < 256; ++x)
                        raster.at<cv::Vec3b>(y, x) = cv::Vec3b((x + key.x) & 255, (y + key.y) & 255, (x * 3 + y * 7) & 255);
                const auto path = store->ImageryPath(key); fs::create_directories(path.parent_path());
                if (!cv::imwrite(path.string(), raster)) throw std::runtime_error("write imagery");
            }
        for (int zoom : {13, 14, 15})
            for (const auto &key : keysForRect(boundsToTileRect(bounds, zoom, 2)))
            {
                cv::Mat raster(256, 256, CV_8UC3);
                for (int y = 0; y < 256; ++y)
                    for (int x = 0; x < 256; ++x)
                        raster.at<cv::Vec3b>(y, x) = cv::Vec3b((x + y) & 255, (key.x + key.y + x / 8 + y / 8) & 255, 128);
                const auto path = store->ElevationPath(key); fs::create_directories(path.parent_path());
                if (!cv::imwrite(path.string(), raster)) throw std::runtime_error("write elevation");
            }
        PersistentTerrainBuilder builder(store);
        CollisionTerrainBuilder collision(store);
        double coldMs = 0, warmMs = 0;
        for (int loop = 0; loop < loops; ++loop)
        {
            const auto start = std::chrono::steady_clock::now();
            std::string error; const TileKey here{center.x + loop % 2, center.y, center.z};
            auto snapshot = builder.BuildBootstrap(here, loop + 1, error);
            if (!snapshot) throw std::runtime_error(error);
            hashSnapshot(*snapshot);
            const auto order = builder.ProgressivePageOrder(*snapshot);
            for (const auto index : order) value(index);
            std::vector<std::size_t> pages(snapshot->pages.size()); std::iota(pages.begin(), pages.end(), 0);
            for (int zoom = 14; zoom <= 16; ++zoom)
            {
                auto update = builder.BuildTextureStage(*snapshot, pages, zoom, error);
                if (!update) throw std::runtime_error(error);
                builder.ApplyTextureUpdate(*snapshot, *update); hashSnapshot(*snapshot);
            }
            auto patch = collision.Build(here, error);
            if (!patch) throw std::runtime_error(error);
            const auto encoded = cv::imread(patch->heightmap.string(), cv::IMREAD_UNCHANGED);
            bytes(encoded.data, encoded.total() * encoded.elemSize());
            const auto elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
            if (loop == 0) coldMs = elapsed; else warmMs += elapsed;
        }
        std::cout << "{\"hash\":\"" << std::hex << hashValue << std::dec << "\",\"loops\":" << loops
                  << ",\"cold_ms\":" << coldMs << ",\"warm_mean_ms\":" << (loops > 1 ? warmMs / (loops - 1) : 0) << "}\n";
        fs::remove_all(root); return 0;
    }
    catch (const std::exception &error) { std::cerr << error.what() << '\n'; fs::remove_all(root); return 1; }
}
