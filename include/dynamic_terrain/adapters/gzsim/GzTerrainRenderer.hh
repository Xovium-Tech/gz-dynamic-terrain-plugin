#pragma once

#include "dynamic_terrain/core/TerrainRenderSink.hh"

#include <gz/common/Event.hh>
#include <gz/common/Image.hh>
#include <gz/common/Mesh.hh>
#include <gz/rendering/Material.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/Mesh.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
#include <gz/sim/EventManager.hh>

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace dynamic_terrain
{

class PersistentTerrainRenderer : public TerrainRenderSink
{
public:
    PersistentTerrainRenderer(Config config, gz::sim::EventManager &events);
    ~PersistentTerrainRenderer() override;

    void QueueSnapshot(std::shared_ptr<const TerrainSnapshot> snapshot) override;
    void QueueTexture(TextureUpdate update) override;
    std::uint64_t ActiveGeneration() const override;
    bool HasActiveTerrain() const override;
    std::optional<TileKey> ActiveCenterTile() const override;

private:
    enum class CameraObservation
    {
        Visible,
        Offscreen,
        Unknown
    };

    struct ResidencyCameraSet
    {
        std::vector<gz::rendering::CameraPtr> cameras;
        bool complete{false};
    };

    struct PageSlot
    {
        TileKey key;
        std::size_t pageIndex{0};
        std::string geographicName;
        std::shared_ptr<gz::common::Mesh> sourceMesh;
        std::shared_ptr<gz::common::Image> sourceTexture;
        std::string sourceTextureName;
        int sourceTextureSize{256};
        gz::math::Vector3d boundsMin;
        gz::math::Vector3d boundsMax;
        int offscreenFrames{0};
        gz::rendering::VisualPtr visual;
        gz::rendering::MeshPtr geometry;
        gz::rendering::SubMeshPtr submesh;
        gz::rendering::MaterialPtr material;
        std::string meshName;
        bool meshReferenceHeld{false};
        std::string textureName;
        bool textureReferenceHeld{false};
        std::size_t textureBytes{0};
        bool gpuResident{false};
    };

    struct Slot
    {
        std::uint64_t generation{0};
        TileKey centerTile;
        std::string resourcePrefix;
        std::vector<PageSlot> pages;
        std::size_t estimatedTextureBytes{0};
        int warmupFrames{0};
    };

    std::shared_ptr<gz::common::Image> RenderImage(
        const std::string &name, const std::shared_ptr<const ImageData> &data);
    void PruneRenderImageCache();
    void FindScene();
    void OnPreRender();
    void OnPostRender();
    void OnRenderTeardown();
    std::optional<Slot> CreateSlot(
        const std::shared_ptr<const TerrainSnapshot> &snapshot,
        bool *resourcePressureCandidate = nullptr);
    std::optional<Slot> CreateSlotWithRecovery(
        const std::shared_ptr<const TerrainSnapshot> &snapshot);
    std::optional<PageSlot> CreatePage(
        const TerrainSnapshot &snapshot, const TerrainPage &page);
    bool MakePageResident(const std::string &resourcePrefix,
                          std::uint64_t generation,
                          PageSlot &page, bool visible);
    bool UnloadPageGpu(PageSlot &page);
    ResidencyCameraSet ResidencyCameras() const;
    CameraObservation ObservePageFromCamera(
        const PageSlot &page,
        const gz::rendering::CameraPtr &camera) const;
    void UpdateActivePageResidency();
    void ConfigureMaterial(const gz::rendering::MaterialPtr &material) const;
    void DestroyPage(PageSlot &page);
    void DrainDeferredPageReleases();
    bool DetachAndDestroyMaterial(gz::rendering::MaterialPtr &material);
    void DestroyPageMaterial(PageSlot &page);
    void DrainDeferredTextureReleases();
    void DrainDeferredTextureDeletes();
    void ReleaseMeshResource(PageSlot &page);
    void DrainDeferredMeshDeletes();
    void ReleaseTextureReference(PageSlot &page);
    void ClearActiveState();
    void DestroySlot(Slot &slot);
    void ApplyPendingTexture();

    // Weak entries preserve CPU image sharing between active/staging pages
    // without extending the lifetime of retired resources.
    std::unordered_map<std::string, std::weak_ptr<gz::common::Image>> imageCache_;
    Config cfg_;
    mutable std::mutex queueMutex_;
    std::shared_ptr<const TerrainSnapshot> pendingSnapshot_;
    std::optional<TextureUpdate> pendingTexture_;

    gz::rendering::ScenePtr scene_;
    gz::rendering::VisualPtr root_;
    std::optional<Slot> active_;
    std::optional<Slot> staging_;
    std::optional<Slot> retired_;

    std::atomic<std::uint64_t> activeGeneration_{0};
    std::atomic<bool> hasActive_{false};
    std::atomic<int> activeCenterX_{0};
    std::atomic<int> activeCenterY_{0};
    std::atomic<int> activeCenterZ_{0};
    std::atomic<std::uint64_t> renderSerial_{0};
    std::atomic<std::uint64_t> materialSerial_{0};
    std::atomic<bool> renderShuttingDown_{false};
    std::uint64_t rendererId_{0};
    ResourceReferenceCounter textureReferences_;
    ResourceReferenceCounter meshReferences_;
    std::vector<PageSlot> deferredPageReleases_;
    std::vector<PageSlot> deferredTextureReleases_;
    std::unordered_set<std::string> deferredTextureDeletes_;
    std::unordered_set<std::string> deferredMeshDeletes_;
    std::uint64_t destroyedMaterials_{0};
    std::uint64_t destroyedTextures_{0};
    std::uint64_t destroyedMeshes_{0};
    bool warnedNoResidencyCameras_{false};

    gz::common::ConnectionPtr preRenderConnection_;
    gz::common::ConnectionPtr postRenderConnection_;
    gz::common::ConnectionPtr teardownConnection_;
};

}
