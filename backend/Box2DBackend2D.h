#pragma once
// Box2DBackend2D.h - real Box2D 2D physics backend (R2.5).
//
// Public header MUST NOT include <box2d/...>. All Box2D types live in the .cpp
// behind class Impl (Pimpl) per design.md §17.8 item 10.

#include "AYPhysics/IPhysicsBackend2D.h"

#include <cstdint>
#include <memory>
#include <vector>

namespace ayt::physics {

class Box2DBackend2D final : public IPhysicsBackend2D {
public:
    Box2DBackend2D();
    ~Box2DBackend2D() override;

    bool init2D(const PhysicsBackendDescriptor& desc) override;
    void setGravity(const ayt::math::FVector2& g) override;

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

    uint64_t stepCount()             const noexcept { return _stepCount; }
    uint64_t commandCount()          const noexcept { return _commandCount; }
    uint64_t notFoundCount()         const noexcept { return _notFoundCount; }
    uint64_t lockstepRefusedCount()  const noexcept { return _lockstepRefusedCount; }
    uint64_t collisionEventCount()   const noexcept { return _collisionEventCount; }

    // Test seam: force the lockstep gate to short-circuit step() until reset.
    // Mirrors JoltBackend3D::forceLockstepActive for parity with §10.2.
    void forceLockstepActive(bool active) noexcept { _lockstepForcedActive = active; }

    // Test seam: synchronous create/destroy that bypasses the manager queue.
    PhysResult execute_createRigidbodyForTest(const RigidbodyDesc& desc,
                                              BodyHandle& outHandle);
    PhysResult execute_destroyRigidbodyForTest(BodyHandle h);

private:
    struct Impl;
    std::unique_ptr<Impl> _impl;

    ayt::math::FVector2 _gravity{0.0f, -9.81f};
    uint64_t _stepCount            = 0;
    uint64_t _commandCount         = 0;
    uint64_t _notFoundCount         = 0;
    uint64_t _lockstepRefusedCount  = 0;
    uint64_t _collisionEventCount  = 0;
    bool     _initialized          = false;
    bool     _running              = false;
    bool     _lockstepForcedActive = false;  // test seam
    int      _maxSubSteps          = 4;
};

} // namespace ayt::physics
