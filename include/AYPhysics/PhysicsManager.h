#pragma once
// AYPhysics/PhysicsManager.h - public manager entry (§9, §4.5)
//
// Construction-time backend selection (no runtime swap). step() is enqueue + return.
// fetchResults() returns the latest sparse PhysFrameSnapshot for game-thread read.
// shutdown() drains the queue, joins the physics thread.

#include "AYPhysics/PhysicsTypes.h"
#include "AYPhysics/PhysicsCommandQueue.h"

#include <memory>
#include <thread>

namespace ayt::physics {

class PhysicsWorld3D;
class PhysicsWorld2D;
class IPhysicsBackend3D;
class IPhysicsBackend2D;
class PhysicsManager;

// Opaque Impl pointer used by PhysicsWorld{2D,3D} to reach back to the manager.
// friend declarations below make the access legal.
struct PhysicsWorldImplTag {};

class PhysicsManager {
public:
    static std::unique_ptr<PhysicsManager> create(const PhysicsBackendDescriptor& desc);

    ~PhysicsManager();

    PhysicsWorld3D* world3D() { return _world3D.get(); }
    PhysicsWorld2D* world2D() { return _world2D.get(); }

    // Enqueue step; returns immediately. deltaTime clamped by backend.
    PhysResult step(float deltaTime);

    // Game-thread read of the latest sparse snapshot. Reference valid until next call.
    const PhysFrameSnapshot& fetchResults();

    void shutdown();

    // Internal — accessed by World impls (friend below).
    PhysicsCommandQueue* commandQueue() { return _queue.get(); }
    PhysicsCommandQueue* commandQueue2D() { return _queue2D.get(); }
    PhysicsCreatePool*   createPool()   { return _createPool.get(); }
    SyncQueryMailbox*    syncMailbox()  { return _syncMailbox.get(); }
    SyncQueryMailbox*    syncMailbox2D() { return _syncMailbox2D.get(); }
    IPhysicsBackend3D*   backend3D()    { return _backend3D.get(); }
    IPhysicsBackend2D*   backend2D()    { return _backend2D.get(); }
    const PhysicsBackendDescriptor& descriptor() const { return _descriptor; }

    // Front/back snapshot access (physics thread writes back, swaps to front).
    PhysFrameSnapshot& backSnapshot() { return _snapshots[_backIndex]; }
    void publishSnapshot();           // physics thread swaps back->front

private:
    PhysicsManager() = default;
    void _physicsThreadMain();

    PhysicsBackendDescriptor _descriptor{};
    std::unique_ptr<PhysicsWorld3D> _world3D;
    std::unique_ptr<PhysicsWorld2D> _world2D;
    std::unique_ptr<IPhysicsBackend3D> _backend3D;
    std::unique_ptr<IPhysicsBackend2D> _backend2D;
    std::unique_ptr<PhysicsCommandQueue> _queue;
    std::unique_ptr<PhysicsCommandQueue> _queue2D;
    std::unique_ptr<PhysicsCreatePool> _createPool;
    std::unique_ptr<SyncQueryMailbox> _syncMailbox;
    std::unique_ptr<SyncQueryMailbox> _syncMailbox2D;
    std::thread _physicsThread;

    // Double-buffered snapshots (front index atomically swapped).
    PhysFrameSnapshot _snapshots[2];
    std::atomic<uint32_t> _frontIndex{0};
    std::atomic<uint32_t> _backIndex{1};

    std::atomic<bool> _running{false};
    std::atomic<uint64_t> _frameIndex{0};

    // Overflow telemetry (§5.4 / §17.1).
    std::atomic<uint64_t> _queueHighWater{0};
    std::atomic<uint64_t> _queueRejectCount{0};
};

} // namespace ayt::physics