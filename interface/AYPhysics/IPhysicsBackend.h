#pragma once
// AYPhysics/IPhysicsBackend.h - common base interface for physics backends (§6.2)
//
// All methods are called from the physics thread only (except describe/isRealDevice).
// Public headers never include backend-specific headers (Jolt, Box2D, etc.); backend
// types live only in backend/<Xxx>Backend3D.cpp.

#include "AYPhysics/PhysicsTypes.h"
#include "AYPhysics/PhysicsCommandQueue.h"

namespace ayt::physics {

class IPhysicsBackend {
public:
    virtual ~IPhysicsBackend() = default;

    // Lifecycle (physics thread).
    virtual bool start(const PhysicsBackendInfo& info) = 0;
    virtual void stop() = 0;

    // Step (physics thread; dt clamped to backend's preferred range).
    virtual void step(float deltaTime) = 0;

    // Snapshot publish (physics thread writes back; manager swaps to front).
    virtual void publishSnapshot(PhysFrameSnapshot& outSnapshot) = 0;

    // Execute one compact command (physics thread; backpressure already handled).
    virtual void execute(const PhysicsCommand& cmd,
                         const PhysicsCreatePayload* createPayload) = 0;

    // Execute one sync query (physics thread; called from SyncQueryMailbox::serviceAll).
    virtual void executeSync(const SyncQueryRequest& request,
                             SyncQueryResponse& outResponse) = 0;

    // Diagnostic.
    virtual PhysicsBackendInfo describe() const = 0;
    virtual bool isRealDevice() const = 0;
};

} // namespace ayt::physics