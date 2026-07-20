#include "NullBackend2D.h"

namespace ayt::physics {

NullBackend2D::NullBackend2D()  = default;
NullBackend2D::~NullBackend2D() { stop(); }

bool NullBackend2D::init2D(const PhysicsBackendDescriptor& /*desc*/) {
    return true;
}

void NullBackend2D::setGravity(const ayt::math::FVector2& g) {
    _gravity = g;
}

bool NullBackend2D::start(const PhysicsBackendInfo& /*info*/) {
    _running = true;
    return true;
}

void NullBackend2D::stop() {
    _running = false;
}

void NullBackend2D::step(float /*deltaTime*/) {
    ++_stepCount;
}

void NullBackend2D::publishSnapshot(PhysFrameSnapshot& /*outSnapshot*/) {}

void NullBackend2D::execute(const PhysicsCommand& /*cmd*/,
                            const PhysicsCreatePayload* /*createPayload*/) {
    ++_commandCount;
}

void NullBackend2D::executeSync(const SyncQueryRequest& request,
                                SyncQueryResponse& outResponse) {
    outResponse.requestId = request.requestId;
    outResponse.status    = PhysResult::Ok;
}

PhysicsBackendInfo NullBackend2D::describe() const {
    PhysicsBackendInfo info{};
    info.name = "Null2D";
    info.realDevice = false;
    return info;
}

} // namespace ayt::physics