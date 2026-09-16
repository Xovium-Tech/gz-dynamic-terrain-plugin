#include "dynamic_terrain/adapters/classic/ClassicTerrainCodec.hh"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <type_traits>

namespace
{
void check(bool condition, const char *message)
{ if (!condition) throw std::runtime_error(message); }
}
int main()
{
    try
    {
        dynamic_terrain::TerrainSnapshot source;
        source.generation = 42;
        source.centerTile = {100, 200, 14};
        source.geometryRect = {99, 199, 101, 201, 14};
        source.bounds = {52.3, 52.1, 21.0, 21.2};
        source.patchCenterLocal = {1.25, -2.5, 0.75};
        source.cellsPerTile = 1; source.cellsX = 1; source.cellsY = 1;
        source.resourcePrefix = "codec_precision";
        auto mesh = std::make_shared<dynamic_terrain::MeshData>();
        mesh->name = "mesh"; mesh->submeshName = "page";
        mesh->positions = {{123456.123456789, -2, 7}, {3, 4, 5}, {5, 6, 7}};
        mesh->normals.assign(3u, {0, 0, 1}); mesh->texCoords = {{0, 0}, {1, 0}, {0, 1}};
        mesh->indices = {0, 1, 2};
        auto image = std::make_shared<dynamic_terrain::ImageData>();
        image->width = 2; image->height = 2;
        image->rgb = {0, 1, 255, 2, 3, 254, 4, 5, 253, 6, 7, 252};
        dynamic_terrain::TerrainPage page;
        page.key = source.centerTile; page.submeshName = "page"; page.imageryZoom = 15;
        page.targetImageryZoom = 18; page.textureSize = 2; page.textureName = "rgb";
        page.mesh = mesh; page.texture = image; source.pages.push_back(page);
        dynamic_terrain::Config config;
        config.visualLightingEnabled = false; config.visualFrustumEviction = true;
        config.cameraNames = {"aircraft::bottom_camera", "front_camera"};
        auto encoded = dynamic_terrain::EncodeClassicSnapshot(source, config);
        std::string wire;
        check(encoded.SerializeToString(&wire), "snapshot serialization");
        dynamic_terrain::classic_msgs::Snapshot parsed;
        check(parsed.ParseFromString(wire), "snapshot parse");
        dynamic_terrain::Config decodedConfig;
        std::string error;
        const auto decoded = dynamic_terrain::DecodeClassicSnapshot(parsed, decodedConfig, error);
        check(static_cast<bool>(decoded), error.c_str());
        check(decoded->generation == 42 && decoded->centerTile == source.centerTile, "generation/tile identity");
        check(decoded->geometryRect.minY == 199 && decoded->cellsPerTile == 1, "mesh extent metadata");
        check(decoded->pages[0].mesh->positions[0].X() == mesh->positions[0].X(), "double precision was lost");
        check(decoded->pages[0].mesh->indices == mesh->indices, "triangle indexing changed");
        check(decoded->pages[0].texture->rgb == image->rgb, "RGB bytes changed");
        check(decodedConfig.cameraNames == config.cameraNames && !decodedConfig.visualLightingEnabled, "render options changed");

        const auto manifestMetadata = dynamic_terrain::EncodeClassicSnapshot(source, config, false);
        check(manifestMetadata.pages_size() == 0, "manifest copied terrain assets");
        const auto metadata = dynamic_terrain::DecodeClassicSnapshot(manifestMetadata, decodedConfig, error, true);
        check(metadata && metadata->pages.empty() && metadata->generation == source.generation,
              "manifest metadata round trip");
        const auto pageOnly = dynamic_terrain::EncodeClassicPage(page, true);
        const auto decodedPage = dynamic_terrain::DecodeClassicPage(pageOnly, error);
        check(decodedPage && decodedPage->mesh && decodedPage->texture->rgb == image->rgb,
              "bounded page response round trip");
        const auto refinement = dynamic_terrain::EncodeClassicPage(page, false);
        check(!refinement.has_mesh(), "refinement resent mesh geometry");
        const auto decodedRefinement = dynamic_terrain::DecodeClassicPage(refinement, error);
        check(decodedRefinement && !decodedRefinement->mesh && decodedRefinement->texture->rgb == image->rgb,
              "page refinement round trip");

        dynamic_terrain::TextureUpdate update;
        update.generation = source.generation;
        update.pages.push_back({0, page.key, page.submeshName, 16, 2, image, "refined"});
        auto textureMessage = dynamic_terrain::EncodeClassicTexture(update);
        const auto texture = dynamic_terrain::DecodeClassicTexture(textureMessage, error);
        check(texture && texture->pages.size() == 1 && texture->pages[0].imageryZoom == 16, "texture stage round trip");
        check(texture->pages[0].texture->rgb == image->rgb, "texture stage pixels");

        auto invalid = parsed;
        invalid.mutable_pages(0)->mutable_mesh()->set_indices(0, 3);
        check(!dynamic_terrain::DecodeClassicSnapshot(invalid, decodedConfig, error), "out of range mesh index accepted");
        invalid = parsed;
        invalid.mutable_pages(0)->mutable_image()->set_rgb("short");
        check(!dynamic_terrain::DecodeClassicSnapshot(invalid, decodedConfig, error), "truncated image accepted");
        invalid = parsed;
        invalid.mutable_pages(0)->mutable_mesh()->set_positions(0, std::numeric_limits<double>::quiet_NaN());
        check(!dynamic_terrain::DecodeClassicSnapshot(invalid, decodedConfig, error), "non-finite geometry accepted");
        invalid = parsed; invalid.set_protocol_version(999);
        check(!dynamic_terrain::DecodeClassicSnapshot(invalid, decodedConfig, error), "unknown protocol accepted");
        invalid = parsed; *invalid.add_pages() = parsed.pages(0);
        check(!dynamic_terrain::DecodeClassicSnapshot(invalid, decodedConfig, error), "duplicate tile accepted");
        std::cout << "Classic terrain codec tests passed\n";
        return 0;
    }
    catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
