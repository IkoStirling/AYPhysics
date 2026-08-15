#pragma once
// MockBackend3D.h - test backend that captures commands
//
// Used by unit tests to verify compact command sequence, create-pool
// resolution, and async query id flow. Test inspection lives in
// AYPhysics/PhysicsBackendTestAccess.h.

#include "AYPhysics/IPhysicsBackend3D.h"

#include <mutex>
#include <vector>

namespace ayt::physics {

class MockBackend3D;   // forward decl for testaccess::setCurrentMock parameter

namespace testaccess {
    const std::vector<class PhysicsCommand>&       mockBackendCommands();
    const std::vector<class PhysicsCreatePayload>& mockBackendCreatePayloads();
    uint32_t                                       lastIssuedQueryId();
    void                                           setCurrentMock(::ayt::physics::MockBackend3D*);
}  // namespace testaccess

class MockBackend3D final : public IPhysicsBackend3D {
public:
    MockBackend3D();
    ~MockBackend3D() override;

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

    // Test access via AYPhysics/PhysicsBackendTestAccess.h.
    void resetCapturedState();

private:
    friend class MockBackend3D_AccessHelper;   // see cpp
    mutable std::mutex               _mu;
    std::vector<PhysicsCommand>      _commands;
    std::vector<PhysicsCreatePayload> _payloads;
    uint32_t                         _lastQueryId = 0;
    ayt::math::FVector3              _gravity{0.0f, -9.81f, 0.0f};
    uint64_t                         _stepCount = 0;
    bool                             _running = false;
};

} // namespace ayt::physics