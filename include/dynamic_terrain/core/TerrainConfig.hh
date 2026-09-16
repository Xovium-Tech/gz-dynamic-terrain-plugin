#pragma once

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace dynamic_terrain
{
struct TerrainConfig
{
    std::string modelName;

    std::string imageryProvider{"google_satellite"};
    std::string imageryUrl;
    std::string imageryExtension;
    std::string imageryToken;

    std::string elevationProvider{"terrarium"};
    std::string elevationUrl;
    std::string elevationToken;
    int elevationMaxZoom{15};
    double visualRadiusM{7500.0};
    bool visualGui{false};
    int visualGeometryZoom{14};
    int visualElevationZoom{13};
    int visualMeshCellsPerTile{64};
    int visualMaxMeshCells{768};
    int visualTextureSize{4096};
    int visualPageTextureMaxSize{2048};
    std::size_t visualPageCacheMb{128};
    int visualAtlasPagePixels{1280};
    int visualAtlasMaxSize{16384};
    double visualTextureGuardM{300.0};
    std::string visualDetailMode{"all"};
    std::string visualDetailCameraName{"camera_down"};
    double visualDetailRadiusM{2500.0};
    int visualDetailZoom{17};
    int visualBootstrapImageryZoom{12};
    std::vector<std::pair<double, int>> visualImageryLodTable{
        {1000.0, 17},
        {2500.0, 16},
        {5500.0, 15},
        {1000000.0, 14},
    };
    double visualRecenterDistanceM{1800.0};
    int visualWarmupFrames{1};
    bool visualFrustumEviction{true};
    int visualOffscreenFrames{30};
    bool visualRefineTexture{true};
    int visualRecenterReadyZoom{14};
    int visualRefineMaxSourceTilesPerBatch{32};
    bool visualLightingEnabled{false};
    bool visualCastShadows{false};
    bool visualReceiveShadows{false};
    int downloadConcurrency{4};
    int downloadPerHost{1};
    int downloadRetries{3};
    unsigned int httpTimeoutMs{5000};
    std::string userAgent{"gz-dynamic-terrain/0.2.1"};
    std::string cacheDir{"~/.cache/gz_dynamic_terrain"};
    std::size_t decodedDemCacheMb{256};
    bool dynamicZoom{true};
    int staticZoom{17};
    int minZoom{12};
    int maxZoom{18};
    std::vector<std::pair<double, int>> zoomTable{
        {80.0, 18},
        {180.0, 17},
        {400.0, 16},
        {900.0, 15},
        {1800.0, 14},
        {4000.0, 13},
        {1000000.0, 12},
    };
    int radiusTiles{1};
    int meshCells{32};
    int heightmapSize{257};
    bool enableCollision{true};
    double collisionOverlapM{0.75};
    double collisionRecenterFraction{0.35};

    bool alignOriginToGround{true};
    double zOffsetM{0.0};

    bool startupPreload{true};
    bool startupSafetyGround{true};
    double startupSafetySizeM{800.0};
    double startupSafetyThicknessM{0.20};
    double startupSafetyTopZ{0.0};
    double startupSafetyRemoveDelaySec{2.0};

    double updatePeriodSec{0.20};
    double retryDelaySec{1.5};
    bool diagnostics{true};
    double statusPeriodSec{2.0};
    std::string coverageMode{"camera_projection"};
    std::vector<std::string> cameraNames;
};

// Preserve the original source-level name for existing terrain algorithms.
using Config = TerrainConfig;

void normalizeConfig(Config &cfg);
}
