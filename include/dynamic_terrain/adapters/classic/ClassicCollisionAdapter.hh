#pragma once

#include "dynamic_terrain/core/CollisionTerrain.hh"

#include <gazebo/physics/PhysicsTypes.hh>

#include <chrono>
#include <array>
#include <deque>
#include <optional>

namespace dynamic_terrain
{
// Used only on the simulation update thread. Model insertion is asynchronous;
// an old patch stays live until its replacement's physics collision exists.
class ClassicCollisionAdapter
{
public:
    ClassicCollisionAdapter(gazebo::physics::WorldPtr world, std::string prefix,
                            fs::path meshCacheRoot);
    bool Submit(const CollisionPatch &patch, std::uint64_t generation);
    void Update(double simTime);
    bool HasActive() const;
    bool HasPending() const { return pending_.has_value() || waiting_.has_value(); }
    bool ConsumeInsertionFailure();
    // Queue terminal cleanup before destruction; do not submit further patches.
    void RemoveAll();

private:
    struct Pending
    {
        std::string name;
        std::uint64_t generation;
        std::chrono::steady_clock::time_point submitted;
        std::size_t slot;
    };
    struct Retiring
    {
        std::string name;
        double removeAt;
        bool seen{false};
        std::size_t slot{0};
        bool removalRequested{false};
    };
    struct Waiting { CollisionPatch patch; std::uint64_t generation; };
    struct Slot { fs::path path; std::string owner; };
    bool TrySubmit(const CollisionPatch &patch, std::uint64_t generation);
    bool CollisionReady(const std::string &name) const;
    gazebo::physics::WorldPtr world_;
    std::string prefix_;
    std::string active_;
    std::optional<Pending> pending_;
    std::optional<Waiting> waiting_;
    std::deque<Retiring> retiring_;
    std::array<Slot, 3> slots_;
    std::size_t activeSlot_{0};
    std::uint64_t serial_{0};
    bool insertionFailed_{false};
    double lastSimTime_{-1.0};
};
}
