#include "dynamic_terrain/adapters/SdfConfig.hh"
#include "dynamic_terrain/adapters/classic/ClassicCollisionAdapter.hh"
#include "dynamic_terrain/adapters/classic/ClassicGeographicTransform.hh"
#include "dynamic_terrain/adapters/classic/ClassicTerrainTransport.hh"
#include "dynamic_terrain/core/TerrainRuntime.hh"

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/TransportIface.hh>

#include <atomic>
#include <chrono>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>

namespace dynamic_terrain
{
class ClassicDynamicTerrainPlugin final : public gazebo::WorldPlugin
{
public:
    ~ClassicDynamicTerrainPlugin() override
    {
        stopping_.store(true);
        updateConnection_.reset();
        std::lock_guard<std::mutex> lock(updateMutex_);
        // All worker publication finishes before its transport sink disappears.
        runtime_.reset();
        source_.reset();
        // World::Fini marks the world stopped before destroying plugins and
        // owns entity teardown at that point. A live world handles cleanup
        // through its deletion queue, synchronized with native state readers.
        const bool worldStillLive = world_ && world_->Running();
        if (collision_ && worldStillLive) collision_->RemoveAll();
        collision_.reset();
        if (worldStillLive)
        {
            if (!safetyName_.empty())
                gazebo::transport::requestNoReply(world_->Name(), "entity_delete", safetyName_);
            if (!anchorName_.empty())
                gazebo::transport::requestNoReply(world_->Name(), "entity_delete", anchorName_);
        }
    }

    void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) override
    {
        std::lock_guard<std::mutex> lock(updateMutex_);
        try
        {
            world_ = std::move(world);
            cfg_ = parseTerrainConfig(sdf);
            if (!world_ || cfg_.modelName.empty())
            {
                logError("[DynamicTerrain][CLASSIC] world plugin requires <tracked_model>model_name</tracked_model>");
                return;
            }
            const auto reference = world_->SphericalCoords();
            if (!reference)
            {
                logError("[DynamicTerrain][CLASSIC] world spherical coordinates unavailable");
                return;
            }
            geographic_ = std::make_shared<ClassicGeographicTransform>(*reference);
            static std::atomic<std::uint64_t> instances{0};
            const std::string prefix = "dynamic_terrain_classic_" + std::to_string(++instances);
            const std::string topic = "~/" + prefix + "/terrain";
            source_ = std::make_unique<ClassicTerrainSource>(cfg_, world_->Name(), topic);
            collision_ = std::make_unique<ClassicCollisionAdapter>(world_, prefix,
                fs::path(expandHome(cfg_.cacheDir)) / "classic_collision");
            CreateVisualAnchor(prefix + "_visual", topic);
            if (cfg_.startupSafetyGround)
                CreateSafetyGround(prefix + "_startup_safety");
            runtime_ = std::make_unique<TerrainRuntime>(cfg_, geographic_, *source_);

            if (cfg_.startupPreload)
            {
                const auto model = world_->ModelByName(cfg_.modelName);
                Vec3 position;
                if (model)
                {
                    const auto p = model->WorldPose().Pos();
                    position = {p.X(), p.Y(), p.Z()};
                }
                RequestTerrain(position, true);
            }
            updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
                [this](const gazebo::common::UpdateInfo &info) { OnUpdate(info); });
            logInfo("[DynamicTerrain][CLASSIC] tracking model=", cfg_.modelName,
                    " world=", world_->Name(), " visual_topic=", topic);
        }
        catch (const std::exception &error)
        {
            logError("[DynamicTerrain][CLASSIC] configuration failed: ", error.what());
            runtime_.reset();
        }
    }

private:
    void RequestTerrain(const Vec3 &position, bool force)
    {
        const auto geographic = geographic_->GeodeticFromLocal(position);
        runtime_->UpdateVisual(position, geographic.X(), geographic.Y(), force);
        runtime_->UpdateCollision(position.Z(), geographic.X(), geographic.Y(),
                                  force || retryCollision_);
        retryCollision_ = false;
    }

    void OnUpdate(const gazebo::common::UpdateInfo &info)
    {
        if (stopping_.load()) return;
        std::lock_guard<std::mutex> lock(updateMutex_);
        if (stopping_.load() || !runtime_) return;
        const double simTime = info.simTime.Double();
        bool force = false;
        if (simTime < lastSimTime_)
        {
            lastRequestTime_ = -std::numeric_limits<double>::infinity();
            readySince_ = -1.0;
            force = true;
        }
        lastSimTime_ = simTime;
        collision_->Update(simTime);
        retryCollision_ = collision_->ConsumeInsertionFailure() || retryCollision_;
        if (!collision_->HasPending())
        {
            const auto result = runtime_->PollCollision(collision_->HasActive());
            if (result)
                collision_->Submit(*result->patch, result->request.generation);
        }

        const auto model = world_->ModelByName(cfg_.modelName);
        if (!model)
        {
            trackedWasAvailable_ = false;
            readySince_ = -1.0;
            Status("tracked-model-missing");
            return;
        }
        const auto position = model->WorldPose().Pos();
        UpdateSafetyGround(simTime, position.Z());
        force = force || !trackedWasAvailable_;
        trackedWasAvailable_ = true;
        if (world_->IsPaused() || (!force && simTime - lastRequestTime_ < cfg_.updatePeriodSec))
            return;
        lastRequestTime_ = simTime;
        RequestTerrain({position.X(), position.Y(), position.Z()}, force);
        Status("running");
    }

    void CreateVisualAnchor(const std::string &name, const std::string &topic)
    {
        anchorName_ = name;
        // An origin-fixed VisualPlugin runs independently in each rendering
        // process: the client and the server's sensor scene receive the same data.
        std::ostringstream sdf;
        sdf << "<sdf version='1.6'><model name='" << xmlEscape(name)
            << "'><static>true</static><link name='terrain'><visual name='terrain'>"
            << "<geometry><box><size>0.001 0.001 0.001</size></box></geometry>"
            << "<cast_shadows>false</cast_shadows>"
            << "<plugin name='dynamic_terrain_visual' filename='libgazebo-classic-dynamic-terrain-visual.so'>"
            << "<topic>" << xmlEscape(topic) << "</topic></plugin>"
            << "</visual></link></model></sdf>";
        world_->InsertModelString(sdf.str());
    }

    void CreateSafetyGround(const std::string &name)
    {
        safetyName_ = name;
        const double centerZ = cfg_.startupSafetyTopZ - 0.5 * cfg_.startupSafetyThicknessM;
        std::ostringstream sdf;
        sdf << std::setprecision(17)
            << "<sdf version='1.6'><model name='" << xmlEscape(name)
            << "'><static>true</static><link name='safety_ground'>"
            << "<collision name='collision'><pose>0 0 " << centerZ << " 0 0 0</pose>"
            << "<geometry><box><size>" << cfg_.startupSafetySizeM << ' '
            << cfg_.startupSafetySizeM << ' ' << cfg_.startupSafetyThicknessM
            << "</size></box></geometry></collision></link></model></sdf>";
        world_->InsertModelString(sdf.str());
    }

    void UpdateSafetyGround(double simTime, double vehicleZ)
    {
        if (safetyName_.empty() || safetyRemoved_) return;
        if (safetyRemovalRequested_)
        {
            if (!world_->ModelByName(safetyName_))
            {
                safetyRemoved_ = true;
                logInfo("[DynamicTerrain][CLASSIC] startup safety removed; terrain collision live");
            }
            return;
        }
        if (!collision_->HasActive() || vehicleZ < cfg_.startupSafetyTopZ - 0.5)
        {
            readySince_ = -1.0;
            return;
        }
        if (readySince_ < 0.0) readySince_ = simTime;
        if (simTime - readySince_ < cfg_.startupSafetyRemoveDelaySec) return;
        // Do not drop the pending safety model's name before factory insertion.
        if (world_->ModelByName(safetyName_))
        {
            gazebo::transport::requestNoReply(world_->Name(), "entity_delete", safetyName_);
            safetyRemovalRequested_ = true;
        }
    }

    void Status(const char *state)
    {
        if (!cfg_.diagnostics) return;
        const auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(now - lastStatus_).count() < cfg_.statusPeriodSec) return;
        lastStatus_ = now;
        logInfo("[DynamicTerrain][CLASSIC] state=", state,
                " visual_generation=", source_->ActiveGeneration(),
                " collision=", collision_->HasActive() ? "live" : "pending");
    }

    gazebo::physics::WorldPtr world_;
    gazebo::event::ConnectionPtr updateConnection_;
    std::atomic<bool> stopping_{false};
    std::mutex updateMutex_;
    Config cfg_;
    std::shared_ptr<const GeographicTransform> geographic_;
    std::unique_ptr<ClassicTerrainSource> source_;
    std::unique_ptr<ClassicCollisionAdapter> collision_;
    std::unique_ptr<TerrainRuntime> runtime_;
    std::string anchorName_, safetyName_;
    bool safetyRemoved_{false};
    bool safetyRemovalRequested_{false};
    bool trackedWasAvailable_{false};
    bool retryCollision_{false};
    double readySince_{-1.0};
    double lastSimTime_{-1.0};
    double lastRequestTime_{-std::numeric_limits<double>::infinity()};
    std::chrono::steady_clock::time_point lastStatus_{};
};

GZ_REGISTER_WORLD_PLUGIN(ClassicDynamicTerrainPlugin)
}
