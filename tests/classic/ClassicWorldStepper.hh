#pragma once

#include <gazebo/common/Events.hh>
#include <gazebo/physics/World.hh>

#include <condition_variable>
#include <exception>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

// Keep one native RunLoop / LogWorker lifetime, as gzserver does. Repeated
// RunBlocking calls restart LogWorker's initial, unlocked WorldState snapshot
// while pending entity deletions may execute. Parking at WorldUpdateEnd also
// prevents ProcessMessages from mutating the model list during test inspection.
class ClassicWorldStepper
{
public:
    explicit ClassicWorldStepper(gazebo::physics::WorldPtr world)
        : world_(std::move(world))
    {
        connection_ = gazebo::event::Events::ConnectWorldUpdateEnd([this] { OnUpdateEnd(); });
        thread_ = std::thread([this]
        {
            try
            {
                world_->RunBlocking(0);
            }
            catch (...)
            {
                world_->Stop();
                std::lock_guard<std::mutex> lock(mutex_);
                failure_ = std::current_exception();
            }
            std::lock_guard<std::mutex> lock(mutex_);
            finished_ = true;
            condition_.notify_all();
        });
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] { return parked_ || finished_; });
        if (finished_)
        {
            lock.unlock();
            Stop();
            if (failure_) std::rethrow_exception(failure_);
            throw std::runtime_error("Classic world stopped before its first update");
        }
    }

    ~ClassicWorldStepper() { Stop(); }
    ClassicWorldStepper(const ClassicWorldStepper &) = delete;
    ClassicWorldStepper &operator=(const ClassicWorldStepper &) = delete;

    void Step(unsigned int count)
    {
        if (count == 0) return;
        std::unique_lock<std::mutex> lock(mutex_);
        if (finished_) throw std::runtime_error("Classic world is no longer running");
        remaining_ = count;
        parked_ = false;
        condition_.notify_all();
        condition_.wait(lock, [this] { return parked_ || finished_; });
        if (failure_) std::rethrow_exception(failure_);
        if (finished_) throw std::runtime_error("Classic world stopped during a step");
    }

private:
    void Stop()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
            condition_.notify_all();
        }
        if (thread_.joinable()) thread_.join();
        connection_.reset();
    }

    void OnUpdateEnd()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (remaining_ > 0) --remaining_;
        if (remaining_ == 0 && !stopping_)
        {
            parked_ = true;
            condition_.notify_all();
            condition_.wait(lock, [this] { return !parked_ || stopping_; });
        }
        if (stopping_)
        {
            lock.unlock();
            world_->Stop();
        }
    }

    gazebo::physics::WorldPtr world_;
    gazebo::event::ConnectionPtr connection_;
    std::thread thread_;
    std::mutex mutex_;
    std::condition_variable condition_;
    std::exception_ptr failure_;
    unsigned int remaining_{0};
    bool parked_{false};
    bool stopping_{false};
    bool finished_{false};
};
