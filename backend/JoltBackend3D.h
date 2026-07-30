#pragma once
// JoltBackend3D.h - real Jolt 3D physics backend (R1.5 stub).
//
// Public header MUST NOT include <Jolt/...>. All Jolt types live in the .cpp
// behind class Impl (Pimpl) per design.md §17.8 item 10.
//
// R1.5a scope (B-6 stub): declares the class and the public surface that matches
// NullBackend3D's diagnostics. init3D() performs the minimum Jolt setup
// (Factory + RegisterTypes) to prove the link line works; no PhysicsSystem::Init
// or body creation yet. R1.5b replaces the .cpp with the real implementation
// (Box/Sphere/Capsule + ContactListener + lockstep gate) and the .h grows only
// slightly (the Pimpl stays opaque).

#include "IPhysicsBackend3D.h"

#include <cstdint>
#include <memory>

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
    uint64_t stepCount()    const noexcept { return _stepCount; }
    uint64_t commandCount() const noexcept { return _commandCount; }

private:
    struct Impl;
    std::unique_ptr<Impl> _impl;

    ayt::math::FVector3 _gravity{0.0f, -9.81f, 0.0f};
    uint64_t _stepCount    = 0;
    uint64_t _commandCount = 0;
    bool     _initialized  = false;
    bool     _running      = false;
};

} // namespace ayt::physics