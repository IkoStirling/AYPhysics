#pragma once
// NullBackend3D.h - null backend for AYPhysics (R1)
//
// Implements IPhysicsBackend3D as no-ops; start/stop succeed, step() bumps an
// internal step counter, execute() decrements/sets nothing. Used for headless CI
// and the determinism gate (Null + Null == bit-exact).
//
// Always built; no third-party dependency.

#include "IPhysicsBackend3D.h"

namespace ayt::physics {

class NullBackend3D final : public IPhysicsBackend3D {
public:
    NullBackend3D();
    ~NullBackend3D() override;

    bool init3D(const PhysicsBackendDescriptor& desc) override;
    void setGravity(const ayt::math::FVector3& g) override;

    bool start(const PhysicsBackendInfo& info) override;
    void stop() override;
    void step(float deltaTime) override;
    void publishSnapshot(PhysFrameSnapshot& outSnapshot) override;
    void execute(const PhysicsCommand& cmd,
                 const PhysicsCreatePayload* createPayload) override;
    void executeSync(const SyncQueryRequest& request,
                     SyncQueryResponse& outResponse) override;
    PhysicsBackendInfo describe() const override;
    bool isRealDevice() const override { return false; }

    // Diagnostics.
    uint64_t stepCount() const noexcept { return _stepCount; }
    uint64_t commandCount() const noexcept { return _commandCount; }

private:
    ayt::math::FVector3 _gravity{0.0f, -9.81f, 0.0f};
    uint64_t _stepCount    = 0;
    uint64_t _commandCount = 0;
    bool     _running      = false;
};

} // namespace ayt::physics