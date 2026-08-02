#pragma once
// JoltBackend3D.h - real Jolt 3D physics backend (R1.5b).
//
// Public header MUST NOT include <Jolt/...>. All Jolt types live in the .cpp
// behind class Impl (Pimpl) per design.md §17.8 item 10.
//
// Forward declarations of JPH::BodyID are used only so the helper signatures
// (handleFromBodyId / pushCollisionEvent / _bodyIdsSnapshot) — called from
// JoltContactListener which is itself TU-private — can be declared. No
// <Jolt/...> headers are included; the JPH::BodyID type stays opaque to the
// header. Any code that calls these helpers must #include <Jolt/...> itself
// (only the .cpp does).

#include "IPhysicsBackend3D.h"

#include <cstdint>
#include <memory>
#include <vector>

namespace JPH { class BodyID; }

namespace ayt::physics {

class JoltBackend3D final : public IPhysicsBackend3D {
public:
    JoltBackend3D();
    ~JoltBackend3D() override;

    // IPhysicsBackend3D lifecycle.
    bool init3D(const PhysicsBackendDescriptor& desc) override;
    void setGravity(const ayt::math::FVector3& g) override;

    // IPhysicsBackend.
    bool  start(const PhysicsBackendInfo& info) override;
    void  stop() override;
    void  step(float deltaTime) override;
    void  publishSnapshot(PhysFrameSnapshot& outSnapshot) override;
    void  execute(const PhysicsCommand& cmd,
                  const PhysicsCreatePayload* createPayload) override;
    void  executeSync(const SyncQueryRequest& request,
                      SyncQueryResponse& outResponse) override;
    PhysicsBackendInfo describe() const override;
    bool  isRealDevice() const override { return true; }

    // Diagnostics (mirrors NullBackend3D; expanded in R1.5b).
    uint64_t stepCount()             const noexcept { return _stepCount; }
    uint64_t commandCount()          const noexcept { return _commandCount; }
    uint64_t lockstepRefusedCount()  const noexcept { return _lockstepRefusedCount; }
    uint64_t notFoundCount()         const noexcept { return _notFoundCount; }
    uint64_t collisionEventCount()   const noexcept { return _collisionEventCount; }

    // Test seam: force the lockstep gate to short-circuit step() until reset.
    // R1.5 lockstep is driven by the global isLockstepActive() stub which
    // returns false in production; tests need a way to assert the gate path.
    void forceLockstepActive(bool active) noexcept { _lockstepForcedActive = active; }

    // Internal helpers (TU-private consumers only — JoltContactListener).
    // BodyHandle resolution: BodyID's index maps to our handle slot; sequence
    // number must match the generation we recorded on create.
    BodyHandle              handleFromBodyId(const JPH::BodyID& id);
    void                    pushCollisionEvent(const CollisionEvent& ev);

    // Test seam: synchronous create/destroy that bypasses the manager queue.
    // Used by Test_JoltBackend3D cases that drive the backend directly. They
    // mirror PhysicsWorld3D::createRigidbody but skip the queue + thread drain
    // step; tests step() the backend synchronously and check the snapshot.
    PhysResult execute_createRigidbodyForTest(const RigidbodyDesc& desc,
                                              BodyHandle& outHandle);
    PhysResult execute_destroyRigidbodyForTest(BodyHandle h);
    // R2.0a: synchronous create-collider test seam. Bypasses manager queue;
    // returns the backend's actual accept/reject decision synchronously (the
    // Manager-level createCollider is async and cannot surface reject reasons
    // — see test_helpers::createColliderSync in Test_JoltBackend3D_AdvancedShapes).
    PhysResult execute_createColliderForTest(const ColliderDesc& desc,
                                              ColliderHandle& outHandle);
    PhysResult execute_destroyColliderForTest(ColliderHandle h);

private:
    struct Impl;
    std::unique_ptr<Impl> _impl;

    ayt::math::FVector3 _gravity{0.0f, -9.81f, 0.0f};
    uint64_t _stepCount            = 0;
    uint64_t _commandCount         = 0;
    uint64_t _lockstepRefusedCount = 0;
    uint64_t _notFoundCount        = 0;
    uint64_t _collisionEventCount  = 0;
    bool     _initialized          = false;
    bool     _running              = false;
    bool     _lockstepForcedActive = false;  // test seam (see forceLockstepActive)
};

} // namespace ayt::physics