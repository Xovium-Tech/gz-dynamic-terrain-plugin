#include "dynamic_terrain/core/CollisionTerrain.hh"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace dynamic_terrain
{
namespace
{
bool encodeHeightmap(const cv::Mat &heights, const fs::path &path,
                     double &baseZ, double &sizeZ, std::string &error)
{
    double minZ = 0.0;
    double maxZ = 0.0;
    cv::minMaxLoc(heights, &minZ, &maxZ);
    const double realRange = maxZ - minZ;
    const double encodedRange = std::max(realRange, 0.10);
    baseZ = minZ;
    sizeZ = encodedRange;
    cv::Mat encoded(heights.rows, heights.cols, CV_16UC1);
    if (realRange <= 1e-9)
        encoded.setTo(cv::Scalar(0));
    else
    {
        for (int row = 0; row < heights.rows; ++row)
        {
            const float *src = heights.ptr<float>(row);
            auto *dst = encoded.ptr<std::uint16_t>(row);
            for (int col = 0; col < heights.cols; ++col)
            {
                const double normalized = clampValue(
                    (static_cast<double>(src[col]) - minZ) / encodedRange,
                    0.0, 1.0);
                dst[col] = static_cast<std::uint16_t>(
                    std::llround(normalized * 65535.0));
            }
        }
    }
    std::error_code ec;
    fs::create_directories(path.parent_path(), ec);
    if (!cv::imwrite(path.string(), encoded))
    {
        error = "cannot write collision heightmap " + path.string();
        return false;
    }
    return true;
}
}

CollisionTerrainBuilder::CollisionTerrainBuilder(std::shared_ptr<TileStore> store)
    : store_(std::move(store))
{
}

std::shared_ptr<const MeshData> collisionPatchMesh(const CollisionPatch &patch,
                                                std::string &error)
{
    const cv::Mat encoded = cv::imread(patch.heightmap.string(), cv::IMREAD_UNCHANGED);
    if (encoded.empty() || encoded.type() != CV_16UC1 ||
        encoded.rows < 2 || encoded.cols < 2)
    {
        error = "collision mesh requires a 16-bit heightmap: " + patch.heightmap.string();
        return {};
    }
    auto mesh = std::make_shared<MeshData>();
    mesh->name = "collision_" + tileText(patch.center);
    mesh->submeshName = "ground";
    const std::size_t width = static_cast<std::size_t>(encoded.cols);
    const std::size_t height = static_cast<std::size_t>(encoded.rows);
    mesh->positions.reserve(width * height);
    mesh->indices.reserve((width - 1) * (height - 1) * 6);
    for (int row = 0; row < encoded.rows; ++row)
    {
        const auto *samples = encoded.ptr<std::uint16_t>(row);
        const double y = (0.5 - static_cast<double>(row) / (encoded.rows - 1)) * patch.sizeY;
        for (int col = 0; col < encoded.cols; ++col)
        {
            const double x = (static_cast<double>(col) / (encoded.cols - 1) - 0.5) * patch.sizeX;
            const double z = static_cast<double>(samples[col]) / 65535.0 * patch.sizeZ;
            mesh->positions.emplace_back(x, y, z);
        }
    }
    for (std::size_t row = 0; row + 1 < height; ++row)
        for (std::size_t col = 0; col + 1 < width; ++col)
        {
            const auto northwest = static_cast<std::uint32_t>(row * width + col);
            const auto northeast = northwest + 1;
            const auto southwest = static_cast<std::uint32_t>((row + 1) * width + col);
            const auto southeast = southwest + 1;
            mesh->indices.insert(mesh->indices.end(),
                {northwest, southwest, northeast, northeast, southwest, southeast});
        }
    return mesh;
}

std::optional<CollisionPatch> CollisionTerrainBuilder::Build(
    const TileKey &center, std::string &error)
{
    const auto &cfg = store_->GetConfig();
    const int count = 1 << center.z;
    const int radius = cfg.radiusTiles;
    const TileRect rect{
        clampValue(center.x - radius, 0, count - 1),
        clampValue(center.y - radius, 0, count - 1),
        clampValue(center.x + radius, 0, count - 1),
        clampValue(center.y + radius, 0, count - 1), center.z};
    const TileBounds bounds = tileRectBounds(rect);

    auto dem = store_->BuildElevationMosaic(
        bounds, cfg.elevationMaxZoom, 1, error);
    if (!dem)
    {
        if (error.empty())
            error = "collision DEM is incomplete";
        return std::nullopt;
    }
    const double elevationOffset =
        store_->ElevationAlignmentOffset(cfg.elevationMaxZoom) + cfg.zOffsetM;

    const int size = validHeightmapSize(cfg.heightmapSize);
    cv::Mat heights(size, size, CV_32FC1);
    const int cells = size - 1;
    for (int row = 0; row < size; ++row)
    {
        const double v = static_cast<double>(row) / cells;
        const double tileY = rect.minY + (rect.maxY + 1.0 - rect.minY) * v;
        const double lat = tileYToLat(tileY, rect.zoom);
        for (int col = 0; col < size; ++col)
        {
            const double u = static_cast<double>(col) / cells;
            const double tileX = rect.minX + (rect.maxX + 1.0 - rect.minX) * u;
            const double lon = tileXToLon(tileX, rect.zoom);
            const double elevation = dem->Sample(lat, lon) + elevationOffset;
            heights.at<float>(row, col) = static_cast<float>(
                localFromGeodetic(
                    store_->Spherical(), lat, lon, elevation).Z());
        }
    }

    const double centerLat = tileYToLat((rect.minY + rect.maxY + 1.0) * 0.5, rect.zoom);
    const double centerLon = tileXToLon((rect.minX + rect.maxX + 1.0) * 0.5, rect.zoom);
    const double reference = store_->Spherical().ElevationReference();
    const auto west = localFromGeodetic(
        store_->Spherical(), centerLat, bounds.west, reference);
    const auto east = localFromGeodetic(
        store_->Spherical(), centerLat, bounds.east, reference);
    const auto north = localFromGeodetic(
        store_->Spherical(), bounds.north, centerLon, reference);
    const auto south = localFromGeodetic(
        store_->Spherical(), bounds.south, centerLon, reference);
    const auto patchCenter = localFromGeodetic(
        store_->Spherical(), centerLat, centerLon, reference);
    const Vec3 eastAxis = east - west;
    const Vec3 northAxis = north - south;

    CollisionPatch patch;
    patch.center = center;
    patch.radius = radius;
    patch.centerX = patchCenter.X();
    patch.centerY = patchCenter.Y();
    patch.sizeX = std::max(0.1, std::hypot(eastAxis.X(), eastAxis.Y())) +
                  2.0 * cfg.collisionOverlapM;
    patch.sizeY = std::max(0.1, std::hypot(northAxis.X(), northAxis.Y())) +
                  2.0 * cfg.collisionOverlapM;
    patch.yaw = std::atan2(eastAxis.Y(), eastAxis.X());

    const fs::path root = store_->CacheRoot() / "collision_v18" /
        std::to_string(center.z) /
        (std::to_string(center.x) + "_" + std::to_string(center.y) +
         "_r" + std::to_string(radius) + "_s" + std::to_string(size));
    patch.heightmap = root / "height.png";
    if (!encodeHeightmap(heights, patch.heightmap,
                         patch.baseZ, patch.sizeZ, error))
        return std::nullopt;

    if (cfg.diagnostics)
        logInfo("[DynamicTerrain][COLLISION] prepared center=", tileText(center),
                " size=", patch.sizeX, "x", patch.sizeY,
                " yaw=", patch.yaw,
                " raster=", size, "x", size,
                " z=[", patch.baseZ, ",", patch.baseZ + patch.sizeZ, "]");
    return patch;
}

}
