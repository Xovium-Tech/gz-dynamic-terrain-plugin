#pragma once

#include "dynamic_terrain/core/TerrainTypes.hh"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace dynamic_terrain
{
// CPU data shared by builders and rendering adapters. Assets are immutable once
// published, so copies of a snapshot share their large mesh and image buffers.
struct MeshData
{
    std::string name;
    std::string submeshName;
    std::vector<Vec3> positions;
    std::vector<Vec3> normals;
    std::vector<Vec2> texCoords;
    std::vector<std::uint32_t> indices;
};

struct ImageData
{
    std::uint32_t width{0};
    std::uint32_t height{0};
    std::vector<std::uint8_t> rgb;

    bool Valid() const
    {
        return width != 0u && height != 0u && rgb.size() % 3u == 0u &&
            rgb.size() / 3u == static_cast<std::size_t>(width) * height;
    }
};

struct TerrainPage
{
    TileKey key;
    std::size_t index{0};
    std::string submeshName;
    int imageryZoom{0};
    int textureSize{256};
    int targetImageryZoom{0};
    std::shared_ptr<const ImageData> texture;
    std::string textureName;
    std::shared_ptr<const MeshData> mesh;
};

struct TerrainSnapshot
{
    std::uint64_t generation{0};
    TileKey centerTile;
    TileRect geometryRect;
    TileBounds bounds;
    Vec3 patchCenterLocal;
    std::string resourcePrefix;
    int cellsPerTile{0};
    int cellsX{0};
    int cellsY{0};
    std::vector<TerrainPage> pages;
    std::size_t estimatedTextureBytes{0};
};

struct TexturePageUpdate
{
    std::size_t pageIndex{0};
    TileKey pageKey;
    std::string submeshName;
    int imageryZoom{0};
    int textureSize{256};
    std::shared_ptr<const ImageData> texture;
    std::string textureName;
};

struct TextureUpdate
{
    std::uint64_t generation{0};
    std::size_t changedPageCount{0};
    std::vector<TexturePageUpdate> pages;
};

}
