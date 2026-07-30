#include "JoltBackend3D.h"

// R1.5a (B-6 stub): only TU that may include <Jolt/...> headers (§17.8 item 10).
// All real bodies/shapes/filters live behind class Impl (Pimpl) so the public
// header is Jolt-free. R1.5a calls the minimum Jolt bootstrap (Factory + Register
// Types) to prove the link line works; PhysicsSystem::Init + body creation
// arrive in R1.5b.

#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/Memory.h>  // RegisterDefaultAllocator

#include <atomic>

namespace ayt::physics {

// -----------------------------------------------------------------------------
// Impl (Pimpl). Stub holds only Jolt global state. R1.5b expands to TempAllocator,
// JobSystemThreadPool, PhysicsSystem, BodyManager*, layer filters, contact
// listener, handle generation tables, shape cache.
// -----------------------------------------------------------------------------
struct JoltBackend3D::Impl {
    bool joltRegistered = false;

    Impl() {
        // Jolt 5.x ships its allocator as an exported symbol (JPH::Allocate /
        // Free / AlignedAllocate / AlignedFree) that starts out NULL when using
        // the static Jolt.lib. Calling `new JPH::Factory()` invokes Jolt's
        // JPH_OVERRIDE_NEW_DELETE, which routes to JPH::Allocate(inSize). If
        // Allocate is still NULL (i.e. nobody wired up the default allocator
        // yet) the operator new dereferences a NULL function pointer and
        // crashes with 0xC0000005 at address 0x0. RegisterDefaultAllocator()
        // wires JPH::Allocate / Free to malloc / free and must be called
        // exactly once before the first `new JPH::*`. R1.5a calls it here;
        // R1.5b moves the call to the first body creation step.
        JPH::RegisterDefaultAllocator();

        // Jolt recommends a single Factory instance for the process. The stub
        // claims it once; R1.5b will configure the cap via PhysicsSystem::Init.
        if (JPH::Factory::sInstance == nullptr) {
            JPH::Factory::sInstance = new JPH::Factory();
        }
        JPH::RegisterTypes();
        joltRegistered = true;
    }
    ~Impl() {
        if (joltRegistered) {
            JPH::UnregisterTypes();
            joltRegistered = false;
        }
        // Note: do NOT delete Factory::sInstance here. Multiple JoltBackend3D
        // instances could exist in a process (test fixtures); the Factory is a
        // process-wide singleton. A future R2 cleanup pass should refcount it.
    }
};

// -----------------------------------------------------------------------------
// Lifecycle
// -----------------------------------------------------------------------------
JoltBackend3D::JoltBackend3D()  = default;
JoltBackend3D::~JoltBackend3D() { stop(); }

bool JoltBackend3D::init3D(const PhysicsBackendDescriptor& /*desc*/) {
    if (!_impl) _impl = std::make_unique<Impl>();
    _initialized = true;
    return true;
}

void JoltBackend3D::setGravity(const ayt::math::FVector3& g) {
    _gravity = g;
    // R1.5b: physicsSystem.SetGravity(JPH::Vec3(g.x, g.y, g.z)).
}

bool JoltBackend3D::start(const PhysicsBackendInfo& /*info*/) {
    if (!_initialized) return false;
    _running = true;
    return true;
}

void JoltBackend3D::stop() {
    _running = false;
}

// -----------------------------------------------------------------------------
// Step / publish / dispatch (stub behaviour; R1.5b replaces with real Jolt calls)
// -----------------------------------------------------------------------------
void JoltBackend3D::step(float /*deltaTime*/) {
    if (!_running) return;
    ++_stepCount;
    // R1.5b: if (isLockstepActive()) { ++_lockstepRefusedCount; ++_stepCount; return; }
    //         _impl->physicsSystem.Update(ClampDt(deltaTime), ...);
}

void JoltBackend3D::publishSnapshot(PhysFrameSnapshot& outSnapshot) {
    // Stub emits nothing. R1.5b iterates active bodies + alwaysSync sleeping
    // bodies, appends BodyTransform, drains collisionEventQueue + pendingAsyncQueries.
    (void)outSnapshot;
}

void JoltBackend3D::execute(const PhysicsCommand& /*cmd*/,
                            const PhysicsCreatePayload* /*createPayload*/) {
    ++_commandCount;
    // R1.5b: full dispatch on PhysicsCommandType with generation validation,
    // out-of-band create payload consumption, body/collider/joint factory calls.
}

void JoltBackend3D::executeSync(const SyncQueryRequest& request,
                                SyncQueryResponse& outResponse) {
    outResponse.requestId = request.requestId;
    outResponse.status    = PhysResult::Ok;
    // R1.5b: NarrowPhaseQuery::CastRay / CollideSphere / CollideOrientedBox.
}

PhysicsBackendInfo JoltBackend3D::describe() const {
    PhysicsBackendInfo info{};
    info.name       = "Jolt (stub)";
    info.realDevice = true;  // marker per design.md §17.3 — Jolt is a real backend family
    return info;
}

} // namespace ayt::physics