#include "NullBackend3D.h"

namespace ayt::physics {

NullBackend3D::NullBackend3D()  = default;
NullBackend3D::~NullBackend3D() { stop(); }

bool NullBackend3D::init3D(const PhysicsBackendDescriptor& /*desc*/) {
    return true;
}

void NullBackend3D::setGravity(const ayt::math::FVector3& g) {
    _gravity = g;
}

bool NullBackend3D::start(const PhysicsBackendInfo& /*info*/) {
    _running = true;
    return true;
}

void NullBackend3D::stop() {
    _running = false;
}

void NullBackend3D::step(float /*deltaTime*/) {
    ++_stepCount;
    // Snapshot stays empty: no bodies, no events, no queries answered (unless
    // explicit populate elsewhere).
}

void NullBackend3D::publishSnapshot(PhysFrameSnapshot& outSnapshot) {
    // Null backend emits no transforms. Snapshot stays at whatever the caller
    // initialised.
    (void)outSnapshot;
}

void NullBackend3D::execute(const PhysicsCommand& /*cmd*/,
                             const PhysicsCreatePayload* /*createPayload*/) {
    ++_commandCount;
    // All commands are accepted; semantics are no-ops in Null mode.
}

void NullBackend3D::executeSync(const SyncQueryRequest& request,
                                SyncQueryResponse& outResponse) {
    outResponse.requestId = request.requestId;
    outResponse.status    = PhysResult::Ok;
    // hits remain default (RaycastHit{hit=false}); overlaps empty.
}

PhysicsBackendInfo NullBackend3D::describe() const {
    PhysicsBackendInfo info{};
    info.name = "Null";
    info.realDevice = false;
    return info;
}

} // namespace ayt::physics