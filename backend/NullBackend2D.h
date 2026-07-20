#pragma once
// NullBackend2D.h - 2D null backend (R1 placeholder)
//
// Mirrors NullBackend3D; R1.5+ Box2D lives behind IPhysicsBackend2D.

#include "IPhysicsBackend2D.h"

namespace ayt::physics {

class NullBackend2D final : public IPhysicsBackend2D {
public:
    NullBackend2D();
    ~NullBackend2D() override;

    bool init2D(const PhysicsBackendDescriptor& desc) override;
    void setGravity(const ayt::math::FVector2& g) override;

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

private:
    ayt::math::FVector2 _gravity{0.0f, -9.81f};
    uint64_t _stepCount    = 0;
    uint64_t _commandCount = 0;
    bool     _running      = false;
};

} // namespace ayt::physics