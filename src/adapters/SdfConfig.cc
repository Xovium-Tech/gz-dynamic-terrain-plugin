#include "dynamic_terrain/adapters/SdfConfig.hh"

#include <algorithm>
#include <sstream>

namespace dynamic_terrain
{
namespace
{
template <typename T>
void readSdf(const std::shared_ptr<const sdf::Element> &sdf,
             const std::string &name, T &value)
{
    if (sdf && sdf->HasElement(name))
        value = sdf->Get<T>(name);
}

std::vector<std::string> splitCommaList(const std::string &text)
{
    std::vector<std::string> result;
    std::stringstream ss(text);
    std::string token;
    while (std::getline(ss, token, ','))
    {
        const auto first = token.find_first_not_of(" \t\r\n");
        if (first == std::string::npos)
            continue;
        const auto last = token.find_last_not_of(" \t\r\n");
        result.emplace_back(token.substr(first, last - first + 1));
    }
    return result;
}

std::vector<std::pair<double, int>> parseZoomTable(const std::string &text)
{
    std::vector<std::pair<double, int>> result;
    std::stringstream ss(text);
    std::string token;
    while (std::getline(ss, token, ','))
    {
        const auto colon = token.find(':');
        if (colon == std::string::npos)
            continue;
        try
        {
            result.emplace_back(std::stod(token.substr(0, colon)),
                                std::stoi(token.substr(colon + 1)));
        }
        catch (...)
        {
        }
    }
    std::sort(result.begin(), result.end(),
              [](const auto &a, const auto &b) { return a.first < b.first; });
    return result;
}

}

Config parseTerrainConfig(const std::shared_ptr<const sdf::Element> &sdf)
{
    Config cfg;
    readSdf(sdf, "model_name", cfg.modelName);
    readSdf(sdf, "tracked_model", cfg.modelName);
    readSdf(sdf, "imagery_provider", cfg.imageryProvider);
    readSdf(sdf, "imagery_url", cfg.imageryUrl);
    readSdf(sdf, "imagery_extension", cfg.imageryExtension);
    readSdf(sdf, "imagery_token", cfg.imageryToken);
    readSdf(sdf, "elevation_provider", cfg.elevationProvider);
    readSdf(sdf, "elevation_url", cfg.elevationUrl);
    readSdf(sdf, "elevation_token", cfg.elevationToken);
    readSdf(sdf, "elevation_max_zoom", cfg.elevationMaxZoom);

    readSdf(sdf, "visual_radius_m", cfg.visualRadiusM);
    readSdf(sdf, "visual_gui", cfg.visualGui);
    readSdf(sdf, "visual_geometry_zoom", cfg.visualGeometryZoom);
    readSdf(sdf, "visual_elevation_zoom", cfg.visualElevationZoom);
    readSdf(sdf, "visual_mesh_cells_per_tile", cfg.visualMeshCellsPerTile);
    readSdf(sdf, "visual_max_mesh_cells", cfg.visualMaxMeshCells);
    readSdf(sdf, "visual_texture_size", cfg.visualTextureSize);
    readSdf(sdf, "visual_page_texture_max_size", cfg.visualPageTextureMaxSize);
    int visualPageCacheMb = static_cast<int>(cfg.visualPageCacheMb);
    readSdf(sdf, "visual_page_cache_mb", visualPageCacheMb);
    cfg.visualPageCacheMb = static_cast<std::size_t>(std::max(0, visualPageCacheMb));
    readSdf(sdf, "visual_atlas_page_pixels", cfg.visualAtlasPagePixels);
    readSdf(sdf, "visual_atlas_max_size", cfg.visualAtlasMaxSize);
    readSdf(sdf, "visual_texture_guard_m", cfg.visualTextureGuardM);
    readSdf(sdf, "visual_detail_mode", cfg.visualDetailMode);
    readSdf(sdf, "visual_detail_camera_name", cfg.visualDetailCameraName);
    readSdf(sdf, "visual_detail_radius_m", cfg.visualDetailRadiusM);
    readSdf(sdf, "visual_detail_zoom", cfg.visualDetailZoom);
    readSdf(sdf, "visual_bootstrap_imagery_zoom", cfg.visualBootstrapImageryZoom);
    readSdf(sdf, "visual_recenter_distance_m", cfg.visualRecenterDistanceM);
    readSdf(sdf, "visual_warmup_frames", cfg.visualWarmupFrames);
    readSdf(sdf, "visual_frustum_eviction", cfg.visualFrustumEviction);
    readSdf(sdf, "visual_offscreen_frames", cfg.visualOffscreenFrames);
    readSdf(sdf, "visual_refine_texture", cfg.visualRefineTexture);
    readSdf(sdf, "visual_recenter_ready_zoom", cfg.visualRecenterReadyZoom);
    readSdf(sdf, "visual_refine_max_source_tiles_per_batch",
            cfg.visualRefineMaxSourceTilesPerBatch);
    readSdf(sdf, "visual_lighting_enabled", cfg.visualLightingEnabled);
    readSdf(sdf, "visual_cast_shadows", cfg.visualCastShadows);
    readSdf(sdf, "visual_receive_shadows", cfg.visualReceiveShadows);
    std::string visualLod;
    readSdf(sdf, "visual_imagery_lod_table", visualLod);
    if (!visualLod.empty())
    {
        auto parsed = parseZoomTable(visualLod);
        if (!parsed.empty())
            cfg.visualImageryLodTable = std::move(parsed);
    }

    readSdf(sdf, "download_concurrency", cfg.downloadConcurrency);
    readSdf(sdf, "download_per_host", cfg.downloadPerHost);
    readSdf(sdf, "download_retries", cfg.downloadRetries);
    readSdf(sdf, "http_timeout_ms", cfg.httpTimeoutMs);
    readSdf(sdf, "user_agent", cfg.userAgent);
    readSdf(sdf, "cache_dir", cfg.cacheDir);
    int decodedDemCacheMb = static_cast<int>(cfg.decodedDemCacheMb);
    readSdf(sdf, "decoded_dem_cache_mb", decodedDemCacheMb);
    cfg.decodedDemCacheMb = static_cast<std::size_t>(std::max(1, decodedDemCacheMb));

    readSdf(sdf, "dynamic_zoom", cfg.dynamicZoom);
    readSdf(sdf, "static_zoom", cfg.staticZoom);
    readSdf(sdf, "min_zoom", cfg.minZoom);
    readSdf(sdf, "max_zoom", cfg.maxZoom);
    std::string zoomTable;
    readSdf(sdf, "zoom_table", zoomTable);
    if (!zoomTable.empty())
    {
        auto parsed = parseZoomTable(zoomTable);
        if (!parsed.empty())
            cfg.zoomTable = std::move(parsed);
    }
    readSdf(sdf, "radius_tiles", cfg.radiusTiles);
    readSdf(sdf, "mesh_cells", cfg.meshCells);
    readSdf(sdf, "heightmap_size", cfg.heightmapSize);
    readSdf(sdf, "enable_collision", cfg.enableCollision);
    readSdf(sdf, "collision_overlap_m", cfg.collisionOverlapM);
    readSdf(sdf, "collision_recenter_fraction", cfg.collisionRecenterFraction);

    readSdf(sdf, "align_origin_to_ground", cfg.alignOriginToGround);
    readSdf(sdf, "z_offset_m", cfg.zOffsetM);
    readSdf(sdf, "startup_preload", cfg.startupPreload);
    readSdf(sdf, "startup_safety_ground", cfg.startupSafetyGround);
    readSdf(sdf, "startup_safety_size_m", cfg.startupSafetySizeM);
    readSdf(sdf, "startup_safety_thickness_m", cfg.startupSafetyThicknessM);
    readSdf(sdf, "startup_safety_top_z", cfg.startupSafetyTopZ);
    readSdf(sdf, "startup_safety_remove_delay_sec", cfg.startupSafetyRemoveDelaySec);
    readSdf(sdf, "update_period_sec", cfg.updatePeriodSec);
    readSdf(sdf, "retry_delay_sec", cfg.retryDelaySec);
    readSdf(sdf, "diagnostics", cfg.diagnostics);
    readSdf(sdf, "status_period_sec", cfg.statusPeriodSec);
    readSdf(sdf, "coverage_mode", cfg.coverageMode);
    std::string cameraNames;
    readSdf(sdf, "camera_names", cameraNames);
    cfg.cameraNames = splitCommaList(cameraNames);

    normalizeConfig(cfg);
    return cfg;
}

}
