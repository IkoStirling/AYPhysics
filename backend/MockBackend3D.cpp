#include "MockBackend3D.h"

#include <cassert>

namespace ayt::physics {

MockBackend3D::MockBackend3D()  = default;
MockBackend3D::~MockBackend3D() { stop(); }

bool MockBackend3D::init3D(const PhysicsBackendDescriptor& /*desc*/) { return true; }

void MockBackend3D::setGravity(const ayt::math::FVector3& g) { _gravity = g; }

bool MockBackend3D::start(const PhysicsBackendInfo& /*info*/) {
    _running = true;
    return true;
}

void MockBackend3D::stop() { _running = false; }

void MockBackend3D::step(float /*deltaTime*/) { ++_stepCount; }

void MockBackend3D::publishSnapshot(PhysFrameSnapshot& /*outSnapshot*/) {}

void MockBackend3D::execute(const PhysicsCommand& cmd,
                            const PhysicsCreatePayload* createPayload) {
    std::lock_guard<std::mutex> lk(_mu);
    _commands.push_back(cmd);
    if (createPayload) _payloads.push_back(*createPayload);
    if (cmd.queryId != 0 && cmd.queryId > _lastQueryId) {
        _lastQueryId = cmd.queryId;
    }
}

void MockBackend3D::executeSync(const SyncQueryRequest& request,
                                SyncQueryResponse& outResponse) {
    outResponse.requestId = request.requestId;
    outResponse.status    = PhysResult::Ok;
    outResponse.firstHit.hit = false;
}

PhysicsBackendInfo MockBackend3D::describe() const {
    PhysicsBackendInfo info{};
    info.name = "Mock";
    info.realDevice = false;
    return info;
}

void MockBackend3D::resetCapturedState() {
    std::lock_guard<std::mutex> lk(_mu);
    _commands.clear();
    _payloads.clear();
    _lastQueryId = 0;
}

// Internal accessor used only by the testaccess namespace.
class MockBackend3D_AccessHelper {
public:
    static const std::vector<PhysicsCommand>&      commands(const MockBackend3D& b) { return b._commands; }
    static const std::vector<PhysicsCreatePayload>& payloads(const MockBackend3D& b) { return b._payloads; }
    static uint32_t                                lastQueryId(const MockBackend3D& b) { return b._lastQueryId; }
};

} // namespace ayt::physics

// =============================================================================
// AYPhysics/PhysicsBackendTestAccess.h implementation (test-only)
// =============================================================================
//
// Defined here (the .cpp) rather than the public header so production code does
// not pull the dependency.

#include "AYPhysics/PhysicsBackendTestAccess.h"

namespace ayt::physics::testaccess {

namespace {
::ayt::physics::MockBackend3D* g_current = nullptr;   // non-owning; set by manager create()
}

void setCurrentMock(::ayt::physics::MockBackend3D* m) {
    g_current = m;
}

const std::vector<PhysicsCommand>& mockBackendCommands() {
    static std::vector<PhysicsCommand> empty;
    if (!g_current) return empty;
    return MockBackend3D_AccessHelper::commands(*g_current);
}

const std::vector<PhysicsCreatePayload>& mockBackendCreatePayloads() {
    static std::vector<PhysicsCreatePayload> empty;
    if (!g_current) return empty;
    return MockBackend3D_AccessHelper::payloads(*g_current);
}

uint32_t lastIssuedQueryId() {
    return g_current ? MockBackend3D_AccessHelper::lastQueryId(*g_current) : 0;
}

void reset() {
    if (g_current) g_current->resetCapturedState();
}

} // namespace ayt::physics::testaccess