#include "dynamic_terrain/core/TerrainRuntime.hh"
#include "dynamic_terrain/adapters/CollisionSdf.hh"
#include "dynamic_terrain/adapters/SdfConfig.hh"
#include "dynamic_terrain/adapters/gzsim/GzTerrainRenderer.hh"
#include "dynamic_terrain/adapters/gzsim/GzGeographicTransform.hh"
#include "dynamic_terrain/adapters/gzsim/ModelConfigRegistry.hh"
#include "dynamic_terrain/adapters/gzsim/gui/GuiTerrain.hh"
#include <gz/plugin/Register.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/SphericalCoordinates.hh>

#include <sdf/Root.hh>

#include <curl/curl.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <iomanip>
#include <mutex>
#include <optional>
#include <sstream>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace dynamic_terrain
{
    namespace
    {
        class DynamicTerrainSystem final
            : public gz::sim::System,
              public gz::sim::ISystemConfigure,
              public gz::sim::ISystemPreUpdate
        {
        public:
            ~DynamicTerrainSystem() override
            {
                runtime_.reset();
                renderer_.reset();
            }

            void Configure(const gz::sim::Entity &entity,
                           const std::shared_ptr<const sdf::Element> &,
                           gz::sim::EntityComponentManager &ecm,
                           gz::sim::EventManager &eventManager) override
            {
                worldEntity_ = entity;
                eventManager_ = &eventManager;
                const auto *sphericalComponent =
                    ecm.Component<gz::sim::components::SphericalCoordinates>(worldEntity_);
                if (!sphericalComponent)
                {
                    logError("[DynamicTerrain][STARTUP] world has no <spherical_coordinates>; plugin disabled");
                    disabled_ = true;
                    return;
                }
                spherical_ = sphericalComponent->Data();
                logInfo("[DynamicTerrain][STARTUP] v0.2.1 dynamic terrain renderer enabled",
                        " worldEntity=", worldEntity_,
                        " elevation_ref=", spherical_->ElevationReference());
                logInfo("[DynamicTerrain][STARTUP] visual terrain will NOT create SDF heightmap tiles");
                logInfo("[DynamicTerrain][STARTUP] waiting for model-side custom::DynamicTerrainConfig");
            }

            void PreUpdate(const gz::sim::UpdateInfo &info,
                           gz::sim::EntityComponentManager &ecm) override
            {
                if (disabled_ || !eventManager_ || !spherical_)
                    return;

                const double simTime = std::chrono::duration<double>(info.simTime).count();
                ensureStartupSafetyGround(ecm);

                if (!configured_)
                {
                    if (!tryBindModelConfiguration(ecm))
                    {
                        periodicStatus("waiting-for-model-config");
                        return;
                    }
                }

                applyCollisionResult(simTime, ecm);
                retireOldCollision(simTime, ecm);
                updateStartupSafetyGround(simTime, ecm);

                if (info.paused)
                    return;
                if (simTime - lastUpdateTime_ < cfg_.updatePeriodSec)
                    return;
                lastUpdateTime_ = simTime;

                if (trackedEntity_ == gz::sim::kNullEntity ||
                    !ecm.Component<gz::sim::components::Model>(trackedEntity_))
                {
                    periodicStatus("tracked-model-missing");
                    return;
                }

                const auto pose = gz::sim::worldPose(trackedEntity_, ecm);
                const auto geo = gz::sim::sphericalCoordinates(trackedEntity_, ecm);
                if (!geo)
                {
                    periodicStatus("spherical-position-unavailable");
                    return;
                }

                runtime_->UpdateVisual(Vec3{pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z()}, geo->X(), geo->Y());
                runtime_->UpdateCollision(pose.Pos().Z(), geo->X(), geo->Y());
                periodicStatus("running");
            }

        private:
            bool tryBindModelConfiguration(gz::sim::EntityComponentManager &ecm)
            {
                for (auto registration : registeredModelConfigs())
                {
                    if (!ecm.Component<gz::sim::components::Model>(registration.entity))
                        continue;

                    trackedEntity_ = registration.entity;
                    cfg_ = std::move(registration.config);
                    if (const auto *name = ecm.Component<gz::sim::components::Name>(trackedEntity_))
                        cfg_.modelName = name->Data();
                    if (cfg_.modelName.empty())
                        cfg_.modelName = "entity_" + std::to_string(trackedEntity_);

                    renderer_ = std::make_unique<PersistentTerrainRenderer>(cfg_, *eventManager_);
                    if (cfg_.visualGui)
                    {
                        const auto *worldName = ecm.Component<gz::sim::components::Name>(worldEntity_);
                        const std::string service = "/world/" +
                                                    (worldName ? worldName->Data() : std::to_string(worldEntity_)) +
                                                    "/model/" + cfg_.modelName + "/terrain_gui";
                        try
                        {
                            guiSource_ = std::make_unique<GuiTerrainSource>(service);
                            logInfo("[DynamicTerrain][GUI] on-demand preview enabled service=", service);
                        }
                        catch (const std::exception &e)
                        {
                            logError("[DynamicTerrain][GUI] preview unavailable: ", e.what());
                        }
                    }

                    runtime_ = std::make_unique<TerrainRuntime>(cfg_,
                        std::make_shared<GzGeographicTransform>(*spherical_), *renderer_,
                        [this](std::shared_ptr<const TerrainSnapshot> snapshot) {
                            if (guiSource_) guiSource_->SetSnapshot(snapshot);
                        },
                        [this](const TextureUpdate &update) {
                            if (guiSource_) guiSource_->ApplyTexture(update);
                        });
                    configured_ = true;

                    if (!cfg_.startupSafetyGround && startupSafetyEntity_ != gz::sim::kNullEntity)
                    {
                        gz::sim::SdfEntityCreator creator(ecm, *eventManager_);
                        creator.RequestRemoveEntity(startupSafetyEntity_, true);
                        startupSafetyEntity_ = gz::sim::kNullEntity;
                        startupSafetyRemoved_ = true;
                    }

                    const auto provider = resolveImageryProvider(cfg_);
                    logInfo("[DynamicTerrain][CONFIG] bound model='", cfg_.modelName,
                            "' entity=", trackedEntity_,
                            " imagery=", provider.name,
                            " elevation=", cfg_.elevationProvider);
                    logInfo("[DynamicTerrain][CONFIG] persistent visual radius=", cfg_.visualRadiusM,
                            "m geometry_z=", cfg_.visualGeometryZoom,
                            " dem_z=", cfg_.visualElevationZoom,
                            " cells_per_tile=", cfg_.visualMeshCellsPerTile,
                            " page_texture_max=", cfg_.visualPageTextureMaxSize,
                            " page_cache_mb=", cfg_.visualPageCacheMb,
                            " bootstrap_z=", cfg_.visualBootstrapImageryZoom,
                            " recenter=", cfg_.visualRecenterDistanceM, "m",
                            " frustum_eviction=",
                            cfg_.visualFrustumEviction ? "on" : "off",
                            " offscreen_frames=", cfg_.visualOffscreenFrames,
                            " texture_guard=", cfg_.visualTextureGuardM, "m",
                            " detail_mode=", cfg_.visualDetailMode,
                            " detail_camera=", cfg_.visualDetailCameraName,
                            " detail_radius=", cfg_.visualDetailRadiusM, "m",
                            " detail_zoom=", cfg_.visualDetailZoom,
                            " recenter_ready_z=", cfg_.visualRecenterReadyZoom,
                            " refine_batch_source_tiles=",
                            cfg_.visualRefineMaxSourceTilesPerBatch,
                            " lighting=", cfg_.visualLightingEnabled ? "on" : "off");
                    logInfo("[DynamicTerrain][CONFIG] downloader concurrency=",
                            cfg_.downloadConcurrency,
                            " per_host=", cfg_.downloadPerHost,
                            " retries=", cfg_.downloadRetries,
                            " no_started_request_cancellation=true");

                    if (cfg_.startupPreload)
                    {
                        if (const auto geo = gz::sim::sphericalCoordinates(trackedEntity_, ecm))
                        {
                            const auto pose = gz::sim::worldPose(trackedEntity_, ecm);
                            runtime_->UpdateVisual(Vec3{pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z()}, geo->X(), geo->Y(), true);
                            runtime_->UpdateCollision(pose.Pos().Z(), geo->X(), geo->Y(), true);
                        }
                        else
                        {
                            const double lat = spherical_->LatitudeReference().Degree();
                            const double lon = spherical_->LongitudeReference().Degree();
                            runtime_->UpdateVisual(Vec3{}, lat, lon, true);
                            runtime_->UpdateCollision(0.0, lat, lon, true);
                        }
                    }
                    return true;
                }
                return false;
            }

            void applyCollisionResult(double simTime, gz::sim::EntityComponentManager &ecm)
            {
                auto result = runtime_->PollCollision(currentCollisionEntity_ != gz::sim::kNullEntity);
                if (!result) return;

                sdf::Root root;
                const std::string sdfText = collisionSdf(*result->patch, ++collisionSerial_);
                const auto errors = root.LoadSdfString(sdfText);
                if (!errors.empty() || !root.Model())
                {
                    logError("[DynamicTerrain][COLLISION] generated SDF failed to parse");
                    return;
                }
                gz::sim::SdfEntityCreator creator(ecm, *eventManager_);
                const auto newEntity = creator.CreateEntities(root.Model());
                if (newEntity == gz::sim::kNullEntity)
                {
                    logError("[DynamicTerrain][COLLISION] CreateEntities failed; old collision retained");
                    return;
                }
                creator.SetParent(newEntity, worldEntity_);

                if (currentCollisionEntity_ != gz::sim::kNullEntity)
                {
                    retiringCollisionEntity_ = currentCollisionEntity_;
                    retireCollisionAtSimTime_ = simTime + 0.10;
                }
                currentCollisionEntity_ = newEntity;
                currentCollisionGeneration_ = result->request.generation;
                currentCollisionCenter_ = result->request.center;
                logInfo("[DynamicTerrain][COLLISION] new patch entity=", newEntity,
                        " generation=", currentCollisionGeneration_,
                        " center=", tileText(*currentCollisionCenter_),
                        " old retained for make-before-break=0.10s");
            }

            void retireOldCollision(double simTime, gz::sim::EntityComponentManager &ecm)
            {
                if (retiringCollisionEntity_ == gz::sim::kNullEntity ||
                    simTime < retireCollisionAtSimTime_)
                    return;
                gz::sim::SdfEntityCreator creator(ecm, *eventManager_);
                creator.RequestRemoveEntity(retiringCollisionEntity_, true);
                if (cfg_.diagnostics)
                    logInfo("[DynamicTerrain][COLLISION] retired old entity=",
                            retiringCollisionEntity_);
                retiringCollisionEntity_ = gz::sim::kNullEntity;
                retireCollisionAtSimTime_ = -1.0;
            }

            void ensureStartupSafetyGround(gz::sim::EntityComponentManager &ecm)
            {
                if (!cfg_.startupSafetyGround || startupSafetyCreated_ ||
                    startupSafetyRemoved_ || !eventManager_)
                    return;

                const double centerZ = cfg_.startupSafetyTopZ -
                                       0.5 * cfg_.startupSafetyThicknessM;
                std::ostringstream sdfText;
                sdfText << std::setprecision(17)
                        << "<sdf version='1.9'><model name='dynamic_terrain_startup_safety'>"
                        << "<static>true</static><link name='safety_ground'>"
                        << "<collision name='collision'><pose>0 0 " << centerZ << " 0 0 0</pose>"
                        << "<geometry><box><size>" << cfg_.startupSafetySizeM << ' '
                        << cfg_.startupSafetySizeM << ' ' << cfg_.startupSafetyThicknessM
                        << "</size></box></geometry></collision></link></model></sdf>";
                sdf::Root root;
                const auto errors = root.LoadSdfString(sdfText.str());
                if (!errors.empty() || !root.Model())
                    return;
                gz::sim::SdfEntityCreator creator(ecm, *eventManager_);
                startupSafetyEntity_ = creator.CreateEntities(root.Model());
                if (startupSafetyEntity_ == gz::sim::kNullEntity)
                    return;
                creator.SetParent(startupSafetyEntity_, worldEntity_);
                startupSafetyCreated_ = true;
                logInfo("[DynamicTerrain][SAFETY] startup collision live entity=",
                        startupSafetyEntity_);
            }

            void updateStartupSafetyGround(double simTime,
                                           gz::sim::EntityComponentManager &ecm)
            {
                if (!cfg_.startupSafetyGround || startupSafetyRemoved_ ||
                    startupSafetyEntity_ == gz::sim::kNullEntity)
                    return;
                if (trackedEntity_ == gz::sim::kNullEntity ||
                    currentCollisionEntity_ == gz::sim::kNullEntity)
                {
                    realTerrainReadySince_ = -1.0;
                    return;
                }
                const auto pose = gz::sim::worldPose(trackedEntity_, ecm);
                if (pose.Pos().Z() < cfg_.startupSafetyTopZ - 0.5)
                {
                    realTerrainReadySince_ = -1.0;
                    return;
                }
                if (realTerrainReadySince_ < 0.0)
                {
                    realTerrainReadySince_ = simTime;
                    return;
                }
                if (simTime - realTerrainReadySince_ < cfg_.startupSafetyRemoveDelaySec)
                    return;
                gz::sim::SdfEntityCreator creator(ecm, *eventManager_);
                creator.RequestRemoveEntity(startupSafetyEntity_, true);
                logInfo("[DynamicTerrain][SAFETY] removed startup collision after real collision became live");
                startupSafetyEntity_ = gz::sim::kNullEntity;
                startupSafetyRemoved_ = true;
            }

            void periodicStatus(const std::string &state)
            {
                if (!cfg_.diagnostics)
                    return;
                const auto now = std::chrono::steady_clock::now();
                if (lastStatusWall_.time_since_epoch().count() != 0)
                {
                    const double elapsed = std::chrono::duration<double>(now - lastStatusWall_).count();
                    if (elapsed < cfg_.statusPeriodSec)
                        return;
                }
                lastStatusWall_ = now;
                logInfo("[DynamicTerrain][STATUS] ", state,
                        " active_visual_generation=", renderer_ ? renderer_->ActiveGeneration() : 0,
                        " visual_building=", (runtime_ && runtime_->VisualBuilding()) ? "true" : "false",
                        " refinement_active=", (runtime_ && runtime_->RefinementActive()) ? "true" : "false",
                        " latest_visual_request=", (runtime_ ? runtime_->LatestVisualGeneration() : 0),
                        " collision_entity=", currentCollisionEntity_,
                        " collision_generation=", currentCollisionGeneration_);
            }

        private:
            gz::sim::Entity worldEntity_{gz::sim::kNullEntity};
            gz::sim::Entity trackedEntity_{gz::sim::kNullEntity};
            gz::sim::EventManager *eventManager_{nullptr};
            std::optional<gz::math::SphericalCoordinates> spherical_;
            Config cfg_;
            bool disabled_{false};
            bool configured_{false};

            std::unique_ptr<GuiTerrainSource> guiSource_;
            std::unique_ptr<PersistentTerrainRenderer> renderer_;
            std::unique_ptr<TerrainRuntime> runtime_;

            gz::sim::Entity currentCollisionEntity_{gz::sim::kNullEntity};
            gz::sim::Entity retiringCollisionEntity_{gz::sim::kNullEntity};
            double retireCollisionAtSimTime_{-1.0};
            std::uint64_t currentCollisionGeneration_{0};
            std::uint64_t collisionSerial_{0};
            std::optional<TileKey> currentCollisionCenter_;

            gz::sim::Entity startupSafetyEntity_{gz::sim::kNullEntity};
            bool startupSafetyCreated_{false};
            bool startupSafetyRemoved_{false};
            double realTerrainReadySince_{-1.0};

            double lastUpdateTime_{-1e9};
            std::chrono::steady_clock::time_point lastStatusWall_{};
        };

        class DynamicTerrainConfig final
            : public gz::sim::System,
              public gz::sim::ISystemConfigure
        {
        public:
            ~DynamicTerrainConfig() override
            {
                if (entity_ != gz::sim::kNullEntity)
                    unregisterModelConfig(entity_);
            }

            void Configure(const gz::sim::Entity &entity,
                           const std::shared_ptr<const sdf::Element> &sdf,
                           gz::sim::EntityComponentManager &ecm,
                           gz::sim::EventManager &) override
            {
                if (!ecm.Component<gz::sim::components::Model>(entity))
                {
                    logError("[DynamicTerrain][CONFIG] custom::DynamicTerrainConfig must be attached to a <model>");
                    return;
                }
                entity_ = entity;
                Config cfg = parseTerrainConfig(sdf);
                if (const auto *name = ecm.Component<gz::sim::components::Name>(entity))
                    cfg.modelName = name->Data();
                registerModelConfig(entity, cfg);
                logInfo("[DynamicTerrain][CONFIG] registered v0.2.1 model configuration model=",
                        cfg.modelName.empty() ? std::to_string(entity) : cfg.modelName,
                        " entity=", entity);
            }

        private:
            gz::sim::Entity entity_{gz::sim::kNullEntity};
        };

    }
}

GZ_ADD_PLUGIN(dynamic_terrain::DynamicTerrainSystem, gz::sim::System, dynamic_terrain::DynamicTerrainSystem::ISystemConfigure, dynamic_terrain::DynamicTerrainSystem::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(dynamic_terrain::DynamicTerrainSystem, "dynamic_terrain::DynamicTerrainSystem")
GZ_ADD_PLUGIN_ALIAS(dynamic_terrain::DynamicTerrainSystem, "custom::DynamicTerrainSystem")

GZ_ADD_PLUGIN(dynamic_terrain::DynamicTerrainConfig, gz::sim::System, dynamic_terrain::DynamicTerrainConfig::ISystemConfigure)
GZ_ADD_PLUGIN_ALIAS(dynamic_terrain::DynamicTerrainConfig, "dynamic_terrain::DynamicTerrainConfig")
GZ_ADD_PLUGIN_ALIAS(dynamic_terrain::DynamicTerrainConfig, "custom::DynamicTerrainConfig")
