#include "dynamic_terrain/adapters/classic/ClassicTerrainCodec.hh"

#include <cmath>
#include <stdexcept>
#include <unordered_set>

namespace dynamic_terrain
{
namespace
{
constexpr std::uint32_t protocolVersion = 1;
constexpr std::size_t maxPages = 65u * 65u;
constexpr std::size_t maxVertices = 4'000'000u;
void encodeKey(const TileKey &key, classic_msgs::TileKey *out)
{
    out->set_x(key.x); out->set_y(key.y); out->set_z(key.z);
}
TileKey decodeKey(const classic_msgs::TileKey &key)
{
    if (key.z() < 0 || key.z() > 20 || key.x() < 0 || key.y() < 0 ||
        key.x() >= (1 << key.z()) || key.y() >= (1 << key.z()))
        throw std::runtime_error("invalid terrain tile key");
    return {key.x(), key.y(), key.z()};
}
void encodeImage(const ImageData &image, classic_msgs::Image *out)
{
    out->set_width(image.width); out->set_height(image.height);
    out->set_rgb(image.rgb.data(), image.rgb.size());
}
std::shared_ptr<const ImageData> decodeImage(const classic_msgs::Image &image)
{
    if (image.width() == 0 || image.height() == 0 ||
        image.width() > 4096 || image.height() > 4096 ||
        image.rgb().size() != static_cast<std::size_t>(image.width()) * image.height() * 3u)
        throw std::runtime_error("invalid terrain RGB image");
    auto out = std::make_shared<ImageData>();
    out->width = image.width(); out->height = image.height();
    out->rgb.assign(image.rgb().begin(), image.rgb().end());
    return out;
}
void encodePage(const TerrainPage &page, classic_msgs::Page *out)
{
    encodeKey(page.key, out->mutable_key());
    out->set_index(page.index); out->set_submesh_name(page.submeshName);
    out->set_imagery_zoom(page.imageryZoom); out->set_texture_size(page.textureSize);
    out->set_target_imagery_zoom(page.targetImageryZoom);
    encodeImage(*page.texture, out->mutable_image());
    out->set_texture_name(page.textureName);
}
TerrainPage decodePage(const classic_msgs::Page &page)
{
    if (page.index() >= maxPages || page.submesh_name().empty() ||
        page.texture_name().empty() || page.texture_size() < 1 ||
        page.texture_size() > 4096)
        throw std::runtime_error("invalid terrain page metadata");
    TerrainPage out;
    out.key = decodeKey(page.key()); out.index = page.index();
    out.submeshName = page.submesh_name(); out.imageryZoom = page.imagery_zoom();
    out.textureSize = page.texture_size(); out.targetImageryZoom = page.target_imagery_zoom();
    out.texture = decodeImage(page.image()); out.textureName = page.texture_name();
    if (out.texture->width != static_cast<unsigned int>(out.textureSize) ||
        out.texture->height != static_cast<unsigned int>(out.textureSize))
        throw std::runtime_error("terrain page/image dimensions disagree");
    return out;
}
void checkFinite(double value)
{
    if (!std::isfinite(value))
        throw std::runtime_error("non-finite terrain geometry");
}
}
classic_msgs::Snapshot EncodeClassicSnapshot(const TerrainSnapshot &snapshot, const Config &config, bool includePages)
{
    classic_msgs::Snapshot out;
    out.set_protocol_version(protocolVersion); out.set_generation(snapshot.generation);
    encodeKey(snapshot.centerTile, out.mutable_center());
    out.set_resource_prefix(snapshot.resourcePrefix);
    out.set_estimated_texture_bytes(snapshot.estimatedTextureBytes);
    for (int n : {snapshot.geometryRect.minX, snapshot.geometryRect.minY,
                  snapshot.geometryRect.maxX, snapshot.geometryRect.maxY, snapshot.geometryRect.zoom})
        out.add_geometry_rect(n);
    for (double n : {snapshot.bounds.north, snapshot.bounds.south,
                     snapshot.bounds.west, snapshot.bounds.east}) out.add_bounds(n);
    for (double n : {snapshot.patchCenterLocal.X(), snapshot.patchCenterLocal.Y(),
                     snapshot.patchCenterLocal.Z()}) out.add_patch_center(n);
    out.set_cells_per_tile(snapshot.cellsPerTile); out.set_cells_x(snapshot.cellsX); out.set_cells_y(snapshot.cellsY);
    auto *cfg = out.mutable_config();
    cfg->set_warmup_frames(config.visualWarmupFrames); cfg->set_lighting(config.visualLightingEnabled);
    cfg->set_cast_shadows(config.visualCastShadows); cfg->set_receive_shadows(config.visualReceiveShadows);
    cfg->set_frustum_eviction(config.visualFrustumEviction); cfg->set_offscreen_frames(config.visualOffscreenFrames);
    cfg->set_texture_guard_m(config.visualTextureGuardM); cfg->set_diagnostics(config.diagnostics);
    for (const auto &name : config.cameraNames) cfg->add_camera_names(name);
    if (includePages)
        for (const auto &page : snapshot.pages)
            *out.add_pages() = EncodeClassicPage(page, true);
    return out;
}
classic_msgs::Page EncodeClassicPage(const TerrainPage &page, bool includeMesh)
{
    classic_msgs::Page out;
    encodePage(page, &out);
    if (includeMesh)
    {
        auto *mesh = out.mutable_mesh();
        mesh->set_name(page.mesh->name); mesh->set_submesh_name(page.mesh->submeshName);
        for (const auto &v : page.mesh->positions)
            for (double n : {v.X(), v.Y(), v.Z()}) mesh->add_positions(n);
        for (const auto &v : page.mesh->normals)
            for (double n : {v.X(), v.Y(), v.Z()}) mesh->add_normals(n);
        for (const auto &v : page.mesh->texCoords)
            for (double n : {v.X(), v.Y()}) mesh->add_tex_coords(n);
        for (auto n : page.mesh->indices) mesh->add_indices(n);
    }
    return out;
}
classic_msgs::TextureUpdate EncodeClassicTexture(const TextureUpdate &update)
{
    classic_msgs::TextureUpdate out;
    out.set_protocol_version(protocolVersion); out.set_generation(update.generation);
    for (const auto &page : update.pages)
    {
        TerrainPage input;
        input.key = page.pageKey; input.index = page.pageIndex; input.submeshName = page.submeshName;
        input.imageryZoom = page.imageryZoom; input.textureSize = page.textureSize;
        input.targetImageryZoom = page.imageryZoom; input.texture = page.texture; input.textureName = page.textureName;
        encodePage(input, out.add_pages());
    }
    return out;
}
std::shared_ptr<const TerrainSnapshot> DecodeClassicSnapshot(
    const classic_msgs::Snapshot &input, Config &config, std::string &error, bool allowEmpty)
{
    try
    {
        if (!input.IsInitialized() || input.protocol_version() != protocolVersion ||
            (!allowEmpty && input.pages_size() <= 0) || static_cast<std::size_t>(input.pages_size()) > maxPages ||
            input.geometry_rect_size() != 5 || input.bounds_size() != 4 || input.patch_center_size() != 3)
            throw std::runtime_error("unsupported or incomplete terrain snapshot");
        auto out = std::make_shared<TerrainSnapshot>();
        out->generation = input.generation(); out->centerTile = decodeKey(input.center());
        out->resourcePrefix = input.resource_prefix(); out->estimatedTextureBytes = input.estimated_texture_bytes();
        out->geometryRect = {input.geometry_rect(0), input.geometry_rect(1), input.geometry_rect(2),
                             input.geometry_rect(3), input.geometry_rect(4)};
        out->bounds = {input.bounds(0), input.bounds(1), input.bounds(2), input.bounds(3)};
        out->patchCenterLocal = {input.patch_center(0), input.patch_center(1), input.patch_center(2)};
        out->cellsPerTile = input.cells_per_tile(); out->cellsX = input.cells_x(); out->cellsY = input.cells_y();
        std::unordered_set<TileKey, TileKeyHash> keys;
        std::size_t vertexTotal = 0;
        for (const auto &page : input.pages())
        {
            if (!keys.insert(decodeKey(page.key())).second || !page.has_mesh())
                throw std::runtime_error("duplicate or missing terrain mesh");
            auto decoded = DecodeClassicPage(page, error);
            if (!decoded) throw std::runtime_error(error);
            vertexTotal += decoded->mesh->positions.size();
            if (vertexTotal > maxVertices + maxPages * 258u)
                throw std::runtime_error("terrain snapshot exceeds geometry limit");
            out->pages.push_back(std::move(*decoded));
        }
        const auto &cfg = input.config();
        config.visualWarmupFrames = clampValue(cfg.warmup_frames(), 0, 10);
        config.visualLightingEnabled = cfg.lighting(); config.visualCastShadows = cfg.cast_shadows();
        config.visualReceiveShadows = cfg.receive_shadows(); config.visualFrustumEviction = cfg.frustum_eviction();
        config.visualOffscreenFrames = std::max(1, cfg.offscreen_frames());
        checkFinite(cfg.texture_guard_m()); config.visualTextureGuardM = std::max(0.0, cfg.texture_guard_m());
        config.cameraNames.assign(cfg.camera_names().begin(), cfg.camera_names().end());
        config.diagnostics = cfg.diagnostics();
        return out;
    }
    catch (const std::exception &e) { error = e.what(); return {}; }
}
std::optional<TerrainPage> DecodeClassicPage(const classic_msgs::Page &page, std::string &error)
{
    try
    {
        if (!page.IsInitialized()) throw std::runtime_error("incomplete terrain page");
        auto target = decodePage(page);
        if (page.has_mesh())
        {
            const auto &source = page.mesh();
            const auto count = static_cast<std::size_t>(source.positions_size()) / 3u;
            if (source.name().empty() || source.submesh_name() != page.submesh_name() ||
                count == 0 || count > 129u * 129u ||
                source.positions_size() % 3 != 0 || source.normals_size() != source.positions_size() ||
                static_cast<std::size_t>(source.tex_coords_size()) != count * 2u ||
                source.indices_size() % 3 != 0 || static_cast<std::size_t>(source.indices_size()) > count * 6u)
                throw std::runtime_error("invalid terrain mesh buffers");
            auto mesh = std::make_shared<MeshData>();
            mesh->name = source.name(); mesh->submeshName = source.submesh_name();
            mesh->positions.reserve(count); mesh->normals.reserve(count); mesh->texCoords.reserve(count);
            for (std::size_t i = 0; i < count; ++i)
            {
                for (int j = 0; j < 3; ++j)
                { checkFinite(source.positions(i * 3u + j)); checkFinite(source.normals(i * 3u + j)); }
                checkFinite(source.tex_coords(i * 2u)); checkFinite(source.tex_coords(i * 2u + 1u));
                mesh->positions.emplace_back(source.positions(i * 3u), source.positions(i * 3u + 1u), source.positions(i * 3u + 2u));
                mesh->normals.emplace_back(source.normals(i * 3u), source.normals(i * 3u + 1u), source.normals(i * 3u + 2u));
                mesh->texCoords.emplace_back(source.tex_coords(i * 2u), source.tex_coords(i * 2u + 1u));
            }
            for (const auto index : source.indices())
            {
                if (index >= count) throw std::runtime_error("terrain mesh index out of bounds");
                mesh->indices.push_back(index);
            }
            target.mesh = std::move(mesh);
        }
        return target;
    }
    catch (const std::exception &e) { error = e.what(); return std::nullopt; }
}
std::optional<TextureUpdate> DecodeClassicTexture(const classic_msgs::TextureUpdate &input, std::string &error)
{
    try
    {
        if (!input.IsInitialized() || input.protocol_version() != protocolVersion ||
            static_cast<std::size_t>(input.pages_size()) > maxPages)
            throw std::runtime_error("unsupported terrain texture update");
        TextureUpdate out; out.generation = input.generation();
        for (const auto &page : input.pages())
        {
            auto decoded = decodePage(page);
            out.pages.push_back({decoded.index, decoded.key, decoded.submeshName, decoded.imageryZoom,
                                 decoded.textureSize, decoded.texture, decoded.textureName});
        }
        out.changedPageCount = out.pages.size(); return out;
    }
    catch (const std::exception &e) { error = e.what(); return std::nullopt; }
}
}
