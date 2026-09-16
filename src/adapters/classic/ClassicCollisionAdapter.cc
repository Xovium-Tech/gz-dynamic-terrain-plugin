#include "dynamic_terrain/adapters/classic/ClassicCollisionAdapter.hh"

#include <gazebo/common/Mesh.hh>
#include <gazebo/common/MeshManager.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/MeshShape.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/TransportIface.hh>

#include <fstream>
#include <iomanip>
#include <sstream>

namespace dynamic_terrain
{
namespace
{
bool writeCollada(const MeshData &mesh, const fs::path &path)
{
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    out << std::setprecision(17)
        << "<?xml version='1.0' encoding='utf-8'?>"
        << "<COLLADA xmlns='http://www.collada.org/2005/11/COLLADASchema' version='1.4.1'>"
        << "<asset><unit name='meter' meter='1'/><up_axis>Z_UP</up_axis></asset>"
        << "<library_geometries><geometry id='terrain'><mesh><source id='positions'>"
        << "<float_array id='positions-array' count='" << mesh.positions.size() * 3 << "'>";
    for (const auto &p : mesh.positions) out << p.X() << ' ' << p.Y() << ' ' << p.Z() << ' ';
    out << "</float_array><technique_common><accessor source='#positions-array' count='"
        << mesh.positions.size() << "' stride='3'>"
        << "<param name='X' type='float'/><param name='Y' type='float'/><param name='Z' type='float'/>"
        << "</accessor></technique_common></source><vertices id='vertices'>"
        << "<input semantic='POSITION' source='#positions'/></vertices><triangles count='"
        << mesh.indices.size() / 3 << "'><input semantic='VERTEX' source='#vertices' offset='0'/><p>";
    for (const auto index : mesh.indices) out << index << ' ';
    out << "</p></triangles></mesh></geometry></library_geometries>"
        << "<library_visual_scenes><visual_scene id='scene'><node id='terrain-node'>"
        << "<instance_geometry url='#terrain'/></node></visual_scene></library_visual_scenes>"
        << "<scene><instance_visual_scene url='#scene'/></scene></COLLADA>";
    out.close();
    return !out.fail();
}

std::string meshSdf(const CollisionPatch &patch, const fs::path &mesh, const std::string &name)
{
    std::ostringstream out;
    out << std::setprecision(17)
        << "<sdf version='1.6'><model name='" << xmlEscape(name) << "'>"
        << "<static>true</static><link name='ground'><collision name='collision'><pose>"
        << patch.centerX << ' ' << patch.centerY << ' ' << patch.baseZ << " 0 0 " << patch.yaw
        << "</pose><geometry><mesh><uri>" << xmlEscape("file://" + mesh.string())
        << "</uri></mesh></geometry></collision></link></model></sdf>";
    return out.str();
}
}

ClassicCollisionAdapter::ClassicCollisionAdapter(gazebo::physics::WorldPtr world,
                                                 std::string prefix, fs::path meshCacheRoot)
    : world_(std::move(world)), prefix_(std::move(prefix))
{
    meshCacheRoot = fs::absolute(meshCacheRoot);
    fs::create_directories(meshCacheRoot);
    for (std::size_t i = 0; i < slots_.size(); ++i)
        slots_[i].path = meshCacheRoot / (prefix_ + "_slot_" + std::to_string(i) + ".dae");
}

bool ClassicCollisionAdapter::Submit(const CollisionPatch &patch, std::uint64_t generation)
{
    // Keep the newest consumed runtime result if all three slots are still in
    // physics use. The simulation loop retries it after retirement completes.
    if (!waiting_ || generation >= waiting_->generation)
        waiting_ = Waiting{patch, generation};
    if (!pending_ && TrySubmit(waiting_->patch, waiting_->generation)) waiting_.reset();
    return true;
}

bool ClassicCollisionAdapter::TrySubmit(const CollisionPatch &patch, std::uint64_t generation)
{
    std::size_t slotIndex = 0;
    while (slotIndex < slots_.size() && !slots_[slotIndex].owner.empty()) ++slotIndex;
    if (slotIndex == slots_.size()) return false;
    auto &slot = slots_[slotIndex];
    const std::string name = prefix_ + "_collision_" + std::to_string(++serial_);
    std::string error;
    const auto mesh = collisionPatchMesh(patch, error);
    if (!mesh || !writeCollada(*mesh, slot.path))
    {
        logError("[DynamicTerrain][CLASSIC] collision mesh export failed: ", error);
        insertionFailed_ = true;
        return true;
    }
    // Classic MeshManager retains its mesh objects for process lifetime. Reuse
    // three names, updating public SubMesh data only after the former owner is
    // absent from the world and a physics step has completed without it.
    const auto *cached = gazebo::common::MeshManager::Instance()->GetMesh(slot.path.string());
    if (cached)
    {
        if (cached->GetSubMeshCount() != 1)
        {
            logError("[DynamicTerrain][CLASSIC] unexpected collision cache mesh layout");
            insertionFailed_ = true;
            return true;
        }
        auto *submesh = const_cast<gazebo::common::SubMesh *>(cached->GetSubMesh(0u));
        submesh->SetVertexCount(0);
        submesh->SetIndexCount(0);
        submesh->SetNormalCount(0);
        submesh->SetTexCoordCount(0);
        for (const auto &p : mesh->positions) submesh->AddVertex(p.X(), p.Y(), p.Z());
        for (const auto index : mesh->indices) submesh->AddIndex(index);
    }
    slot.owner = name;
    world_->InsertModelString(meshSdf(patch, slot.path, name));
    pending_ = Pending{name, generation, std::chrono::steady_clock::now(), slotIndex};
    return true;
}

bool ClassicCollisionAdapter::CollisionReady(const std::string &name) const
{
    if (name.empty())
        return false;
    const auto model = world_->ModelByName(name);
    if (!model)
        return false;
    const auto link = model->GetLink("ground");
    const auto collision = link ? link->GetCollision("collision") : gazebo::physics::CollisionPtr{};
    const auto shape = collision
        ? boost::dynamic_pointer_cast<gazebo::physics::MeshShape>(collision->GetShape())
        : gazebo::physics::MeshShapePtr{};
    if (!shape) return false;
    std::string path = shape->GetMeshURI();
    if (path.compare(0, 7, "file://") == 0) path.erase(0, 7);
    const auto *mesh = gazebo::common::MeshManager::Instance()->GetMesh(path);
    return mesh && mesh->GetVertexCount() > 0 && mesh->GetIndexCount() > 0;
}

bool ClassicCollisionAdapter::HasActive() const { return CollisionReady(active_); }

void ClassicCollisionAdapter::Update(double simTime)
{
    if (simTime < lastSimTime_)
        for (auto &retiring : retiring_)
            retiring.removeAt = simTime + 0.10;
    lastSimTime_ = simTime;
    if (pending_ && CollisionReady(pending_->name))
    {
        if (!active_.empty())
            retiring_.push_back({active_, simTime + 0.10, true, activeSlot_});
        active_ = pending_->name;
        activeSlot_ = pending_->slot;
        logInfo("[DynamicTerrain][CLASSIC] collision live model=", active_,
                " generation=", pending_->generation,
                "; old patch retained for 0.10s");
        pending_.reset();
    }
    else if (pending_ && std::chrono::steady_clock::now() - pending_->submitted >
                              std::chrono::seconds(10))
    {
        logError("[DynamicTerrain][CLASSIC] collision insertion timed out model=",
                 pending_->name, "; old patch retained");
        // Keep watching for late insertion so a timed-out factory request does
        // not leave an unowned patch behind after a retry succeeds.
        retiring_.push_back({pending_->name, simTime, false, pending_->slot});
        pending_.reset();
        insertionFailed_ = true;
    }
    for (auto it = retiring_.begin(); it != retiring_.end();)
    {
        if (simTime < it->removeAt)
        {
            ++it;
            continue;
        }
        if (world_->ModelByName(it->name))
        {
            it->seen = true;
            if (!it->removalRequested)
            {
                // World::RemoveModel bypasses the mutex protecting LogWorker's
                // link snapshots. The native request queue takes that mutex
                // before finalizing links, and does not block this update.
                gazebo::transport::requestNoReply(world_->Name(), "entity_delete", it->name);
                it->removalRequested = true;
            }
            ++it;
        }
        else if (it->seen)
        {
            slots_[it->slot].owner.clear();
            it = retiring_.erase(it);
        }
        else
            ++it;
    }
    if (!pending_ && waiting_ && TrySubmit(waiting_->patch, waiting_->generation))
        waiting_.reset();
}

bool ClassicCollisionAdapter::ConsumeInsertionFailure()
{
    const bool failed = insertionFailed_;
    insertionFailed_ = false;
    return failed;
}

void ClassicCollisionAdapter::RemoveAll()
{
    if (!active_.empty())
        gazebo::transport::requestNoReply(world_->Name(), "entity_delete", active_);
    if (pending_)
        gazebo::transport::requestNoReply(world_->Name(), "entity_delete", pending_->name);
    for (const auto &retiring : retiring_)
        if (!retiring.removalRequested)
            gazebo::transport::requestNoReply(world_->Name(), "entity_delete", retiring.name);
    active_.clear();
    pending_.reset();
    waiting_.reset();
    retiring_.clear();
    // This is terminal cleanup. Keep mesh slots reserved until destruction;
    // their native models remain live until the world handles these requests.
}
}
