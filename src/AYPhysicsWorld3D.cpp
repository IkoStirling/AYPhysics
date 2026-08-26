#include "AYPhysics/PhysicsWorld3D.h"

#include "AYPhysics/PhysicsManager.h"

#include <cmath>

namespace ayt::physics {

// =========================================================================
// Handle minting (game thread only)
// =========================================================================
//
// All counters live on the owning PhysicsWorld3D instance, not as TU-static
// atomics. Reason (correctness fix): previously each create*Rigidbody /
// create*Collider / create*Joint declared its own pair of `static std::atomic`
// at function scope. Function-scope `static` in C++ is one variable per
// function per TU — across the 6 functions there were 6 independent counter
// pairs, so a body and a collider could share an identical handle while
// being distinct objects. The fix collapses all 6 into World3D members.

template <typename Handle>
Handle PhysicsWorld3D::mintHandle(uint32_t& indexSlot, uint32_t& genSlot) noexcept {
    // F-A — cap by backend capacity. The backend slot arrays are sized to
    // maxBodies; minting an index >= maxBodies would hand the caller a handle
    // the backend's handle-validation gate (NotFound) immediately rejects —
    // silently producing a phantom body. The descriptor's maxBodies is the
    // authoritative bound. Index 0 is reserved (invalid handle) so legal
    // indices live in [1, maxBodies).
    const uint32_t maxBodies = _manager ? _manager->descriptor().maxBodies : 0u;
    const uint32_t nextIndex = indexSlot + 1u;
    if (nextIndex >= maxBodies) {
        return InvalidBodyHandle;  // aliased to 0u for all Handle types
    }

    // generation: bump on every mint so a reused slot is detectable by
    // backend handle validation (design §7.1 / §17.4). 0 is reserved.
    const uint32_t freshGen = bumpGeneration(genSlot);

    // Only commit the bump after both checks succeed. Rolling back means
    // the next successful mint gets the SAME index with the NEXT generation,
    // so no two mints ever share (idx, gen) — but we also don't skip a gen
    // number on the failure path.
    indexSlot = nextIndex;
    genSlot   = freshGen;

    return makeHandle(nextIndex, freshGen);
}

uint32_t PhysicsWorld3D::nextQueryId() noexcept {
    // queryId must never be 0 (game-side contract: 0 means "no query issued").
    const uint32_t v = _nextQueryId + 1u;
    _nextQueryId = (v == 0u) ? 1u : v;
    return _nextQueryId;
}

// =========================================================================
// Rigidbody
// =========================================================================

PhysResult PhysicsWorld3D::createRigidbody(const RigidbodyDesc& desc, BodyHandle& outHandle) {
    outHandle = InvalidBodyHandle;
    PhysicsManager* m = _manager;
    if (!m || !m->createPool() || !m->commandQueue()) return PhysResult::InvalidState;

    outHandle = mintHandle<BodyHandle>(_nextBodyIndex, _nextBodyGen);
    if (outHandle == InvalidBodyHandle) return PhysResult::OutOfRange;

    PhysicsCreatePayload payload{};
    payload.kind = PhysicsCreatePayload::Kind::Rigidbody;
    payload.rigidDesc = desc;
    const CreateSlotId slotId = m->createPool()->allocate(payload);
    if (slotId == InvalidCreateSlotId) {
        outHandle = InvalidBodyHandle;
        return PhysResult::NoMemory;
    }

    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::CreateRigidbody;
    cmd.body = outHandle;
    cmd.createSlot = slotId;
    if (!m->commandQueue()->tryPush(cmd)) {
        // Roll back the slot. The handle is "in flight" but never observed;
        // physics thread will see an unknown handle on its create-pool take
        // path and just drop it (R1 Null/Mock: no-op). Caller sees QueueFull.
        (void)m->createPool()->take(slotId, payload);
        outHandle = InvalidBodyHandle;
        return PhysResult::QueueFull;
    }
    return PhysResult::Ok;
}

PhysResult PhysicsWorld3D::destroyRigidbody(BodyHandle h) {
    PhysicsManager* m = _manager;
    if (!m || !m->commandQueue()) return PhysResult::InvalidState;
    if (!isValidHandle(h)) return PhysResult::InvalidParam;
    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::DestroyRigidbody;
    cmd.body = h;
    if (!m->commandQueue()->tryPush(cmd)) return PhysResult::QueueFull;
    return PhysResult::Ok;
}

PhysResult PhysicsWorld3D::setRigidbodyTransform(BodyHandle h, const ayt::math::FVector3& p,
                                                 const ayt::math::FQuaternion& r) {
    PhysicsManager* m = _manager;
    if (!m || !m->commandQueue()) return PhysResult::InvalidState;
    if (!isValidHandle(h)) return PhysResult::InvalidParam;
    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::SetRigidbodyTransform;
    cmd.body = h;
    cmd.u.xform.px = p.x; cmd.u.xform.py = p.y; cmd.u.xform.pz = p.z;
    cmd.u.xform.qx = r.x; cmd.u.xform.qy = r.y; cmd.u.xform.qz = r.z; cmd.u.xform.qw = r.w;
    if (!m->commandQueue()->tryPush(cmd)) return PhysResult::QueueFull;
    return PhysResult::Ok;
}

PhysResult PhysicsWorld3D::applyForce(BodyHandle h, const ayt::math::FVector3& f) {
    PhysicsManager* m = _manager;
    if (!m || !m->commandQueue()) return PhysResult::InvalidState;
    if (!isValidHandle(h)) return PhysResult::InvalidParam;
    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::ApplyForce;
    cmd.body = h;
    cmd.u.vec4.x = f.x; cmd.u.vec4.y = f.y; cmd.u.vec4.z = f.z; cmd.u.vec4.w = 0.0f;
    if (!m->commandQueue()->tryPush(cmd)) return PhysResult::QueueFull;
    return PhysResult::Ok;
}

PhysResult PhysicsWorld3D::applyImpulse(BodyHandle h, const ayt::math::FVector3& impulse,
                                        const ayt::math::FVector3& point) {
    PhysicsManager* m = _manager;
    if (!m || !m->commandQueue()) return PhysResult::InvalidState;
    if (!isValidHandle(h)) return PhysResult::InvalidParam;
    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::ApplyImpulse;
    cmd.body = h;
    cmd.u.vec4.x = impulse.x; cmd.u.vec4.y = impulse.y; cmd.u.vec4.z = impulse.z;
    cmd.u.vec4.w = 0.0f;
    // point stored in unused second-slot? For R1 we encode impulse only; point
    // is a separate parameter to applyImpulse at API level; R1.5 will widen
    // the command to carry 2 vec4s. Behaviour in Null: no-op.
    (void)point;
    if (!m->commandQueue()->tryPush(cmd)) return PhysResult::QueueFull;
    return PhysResult::Ok;
}

PhysResult PhysicsWorld3D::setRigidbodyCollideMask(BodyHandle h, PhysLayerMask newMask) {
    PhysicsManager* m = _manager;
    if (!m || !m->commandQueue()) return PhysResult::InvalidState;
    if (!isValidHandle(h)) return PhysResult::InvalidParam;
    PhysicsCommand cmd{};
    cmd.type      = PhysicsCommandType::SetRigidbodyCollideMask;
    cmd.body      = h;
    cmd.layerMask = newMask;
    if (!m->commandQueue()->tryPush(cmd)) return PhysResult::QueueFull;
    return PhysResult::Ok;
}

// =========================================================================
// R10: 2D-symmetric runtime setters
// =========================================================================

PhysResult PhysicsWorld3D::setRigidbodyVelocity(BodyHandle h, const ayt::math::FVector3& v) {
    PhysicsManager* m = _manager;
    if (!m || !m->commandQueue()) return PhysResult::InvalidState;
    if (!isValidHandle(h)) return PhysResult::InvalidParam;
    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::SetRigidbodyVelocity;
    cmd.body = h;
    cmd.u.vec4.x = v.x; cmd.u.vec4.y = v.y; cmd.u.vec4.z = v.z;
    if (!m->commandQueue()->tryPush(cmd)) return PhysResult::QueueFull;
    return PhysResult::Ok;
}

PhysResult PhysicsWorld3D::setGravityScale(BodyHandle h, float scale) {
    PhysicsManager* m = _manager;
    if (!m || !m->commandQueue()) return PhysResult::InvalidState;
    if (!isValidHandle(h)) return PhysResult::InvalidParam;
    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::SetGravityScale;
    cmd.body = h;
    cmd.u.vec4.x = scale;
    if (!m->commandQueue()->tryPush(cmd)) return PhysResult::QueueFull;
    return PhysResult::Ok;
}

PhysResult PhysicsWorld3D::applyTorque(BodyHandle h, const ayt::math::FVector3& torque) {
    PhysicsManager* m = _manager;
    if (!m || !m->commandQueue()) return PhysResult::InvalidState;
    if (!isValidHandle(h)) return PhysResult::InvalidParam;
    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::ApplyTorque;
    cmd.body = h;
    cmd.u.vec4.x = torque.x; cmd.u.vec4.y = torque.y; cmd.u.vec4.z = torque.z;
    if (!m->commandQueue()->tryPush(cmd)) return PhysResult::QueueFull;
    return PhysResult::Ok;
}

PhysResult PhysicsWorld3D::applyAngularImpulse(BodyHandle h, const ayt::math::FVector3& impulse) {
    PhysicsManager* m = _manager;
    if (!m || !m->commandQueue()) return PhysResult::InvalidState;
    if (!isValidHandle(h)) return PhysResult::InvalidParam;
    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::ApplyAngularImpulse;
    cmd.body = h;
    cmd.u.vec4.x = impulse.x; cmd.u.vec4.y = impulse.y; cmd.u.vec4.z = impulse.z;
    if (!m->commandQueue()->tryPush(cmd)) return PhysResult::QueueFull;
    return PhysResult::Ok;
}

PhysResult PhysicsWorld3D::setMass(BodyHandle h, float mass) {
    PhysicsManager* m = _manager;
    if (!m || !m->commandQueue()) return PhysResult::InvalidState;
    // F-P3: reject non-finite or non-positive mass synchronously — Inf/NaN
    // slip past the bare `!(mass > 0.0f)` check (Inf > 0 is true).
    if (!isValidHandle(h) || !std::isfinite(mass) || !(mass > 0.0f)) {
        return PhysResult::InvalidParam;
    }
    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::SetMass;
    cmd.body = h;
    cmd.u.vec4.x = mass;
    if (!m->commandQueue()->tryPush(cmd)) return PhysResult::QueueFull;
    return PhysResult::Ok;
}

PhysResult PhysicsWorld3D::setMaterial(ColliderHandle h, float friction, float restitution) {
    PhysicsManager* m = _manager;
    if (!m || !m->commandQueue()) return PhysResult::InvalidState;
    // F-P3: reject NaN/Inf/negative synchronously.
    if (!isValidHandle(h) ||
        !std::isfinite(friction) || friction < 0.0f ||
        !std::isfinite(restitution) || restitution < 0.0f) {
        return PhysResult::InvalidParam;
    }
    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::SetMaterial;
    cmd.collider = h;
    cmd.u.vec4.x = friction;
    cmd.u.vec4.y = restitution;
    if (!m->commandQueue()->tryPush(cmd)) return PhysResult::QueueFull;
    return PhysResult::Ok;
}

// =========================================================================
// Collider
// =========================================================================

PhysResult PhysicsWorld3D::createCollider(const ColliderDesc& desc, ColliderHandle& outHandle) {
    outHandle = InvalidColliderHandle;
    PhysicsManager* m = _manager;
    if (!m) return PhysResult::InvalidState;

    outHandle = mintHandle<ColliderHandle>(_nextColliderIndex, _nextColliderGen);
    if (outHandle == InvalidColliderHandle) return PhysResult::OutOfRange;

    PhysicsCreatePayload payload{};
    payload.kind = PhysicsCreatePayload::Kind::Collider;
    payload.colliderDesc = desc;
    const CreateSlotId slotId = m->createPool()->allocate(payload);
    if (slotId == InvalidCreateSlotId) {
        outHandle = InvalidColliderHandle;
        return PhysResult::NoMemory;
    }

    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::CreateCollider;
    cmd.collider = outHandle;
    cmd.createSlot = slotId;
    if (!m->commandQueue()->tryPush(cmd)) {
        (void)m->createPool()->take(slotId, payload);
        outHandle = InvalidColliderHandle;
        return PhysResult::QueueFull;
    }
    return PhysResult::Ok;
}

PhysResult PhysicsWorld3D::destroyCollider(ColliderHandle h) {
    PhysicsManager* m = _manager;
    if (!m) return PhysResult::InvalidState;
    if (!isValidHandle(h)) return PhysResult::InvalidParam;
    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::DestroyCollider;
    cmd.collider = h;
    if (!m->commandQueue()->tryPush(cmd)) return PhysResult::QueueFull;
    return PhysResult::Ok;
}

// =========================================================================
// Joint
// =========================================================================

PhysResult PhysicsWorld3D::createJoint(const JointDesc& desc, JointHandle& outHandle) {
    outHandle = InvalidJointHandle;
    PhysicsManager* m = _manager;
    if (!m) return PhysResult::InvalidState;

    outHandle = mintHandle<JointHandle>(_nextJointIndex, _nextJointGen);
    if (outHandle == InvalidJointHandle) return PhysResult::OutOfRange;

    PhysicsCreatePayload payload{};
    payload.kind = PhysicsCreatePayload::Kind::Joint;
    payload.jointDesc = desc;
    const CreateSlotId slotId = m->createPool()->allocate(payload);
    if (slotId == InvalidCreateSlotId) {
        outHandle = InvalidJointHandle;
        return PhysResult::NoMemory;
    }

    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::CreateJoint;
    cmd.joint = outHandle;
    cmd.createSlot = slotId;
    if (!m->commandQueue()->tryPush(cmd)) {
        (void)m->createPool()->take(slotId, payload);
        outHandle = InvalidJointHandle;
        return PhysResult::QueueFull;
    }
    return PhysResult::Ok;
}

PhysResult PhysicsWorld3D::destroyJoint(JointHandle h) {
    PhysicsManager* m = _manager;
    if (!m) return PhysResult::InvalidState;
    if (!isValidHandle(h)) return PhysResult::InvalidParam;
    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::DestroyJoint;
    cmd.joint = h;
    if (!m->commandQueue()->tryPush(cmd)) return PhysResult::QueueFull;
    return PhysResult::Ok;
}

// =========================================================================
// Async / Sync queries (game thread only — opaque queryIds)
// =========================================================================

uint32_t PhysicsWorld3D::raycastAsync(const ayt::math::Ray& ray, PhysLayerMask layerMask) {
    PhysicsManager* m = _manager;
    if (!m) return 0;
    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::RaycastAsync;
    cmd.queryId = nextQueryId();
    cmd.layerMask = layerMask;
    cmd.u.ray.ox = ray.origin.x; cmd.u.ray.oy = ray.origin.y; cmd.u.ray.oz = ray.origin.z;
    cmd.u.ray.dx = ray.dir.x;    cmd.u.ray.dy = ray.dir.y;    cmd.u.ray.dz = ray.dir.z;
    if (!m->commandQueue()->tryPush(cmd)) return 0;
    return cmd.queryId;
}

uint32_t PhysicsWorld3D::overlapSphereAsync(const ayt::math::FVector3& center, float radius,
                                            PhysLayerMask layerMask) {
    PhysicsManager* m = _manager;
    if (!m) return 0;
    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::OverlapSphereAsync;
    cmd.queryId = nextQueryId();
    cmd.layerMask = layerMask;
    cmd.u.sphere.cx = center.x; cmd.u.sphere.cy = center.y; cmd.u.sphere.cz = center.z;
    cmd.u.sphere.radius = radius;
    if (!m->commandQueue()->tryPush(cmd)) return 0;
    return cmd.queryId;
}

uint32_t PhysicsWorld3D::overlapBoxAsync(const ayt::math::FVector3& center,
                                         const ayt::math::FVector3& halfExtents,
                                         const ayt::math::FQuaternion& rot, PhysLayerMask layerMask) {
    PhysicsManager* m = _manager;
    if (!m) return 0;
    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::OverlapBoxAsync;
    cmd.queryId = nextQueryId();
    cmd.layerMask = layerMask;
    cmd.u.vec4.x = center.x; cmd.u.vec4.y = center.y; cmd.u.vec4.z = center.z; cmd.u.vec4.w = 0.0f;
    // R1 packs only center+radius/halfExtents; rotation/halfExtents encoded
    // separately once compact command grows. For now, store halfExtents in
    // second slot would overflow 64 B; we pack center only and rely on Mock/Null
    // for behaviour. R1.5 will widen.
    (void)halfExtents; (void)rot;
    if (!m->commandQueue()->tryPush(cmd)) return 0;
    return cmd.queryId;
}

PhysResult PhysicsWorld3D::raycastSync(const ayt::math::Ray& ray, RaycastHit& outHit,
                                       PhysLayerMask layerMask) {
    PhysicsManager* m = _manager;
    if (!m || !m->syncMailbox()) return PhysResult::InvalidState;
    SyncQueryRequest req{};
    req.type = SyncQueryType::Raycast;
    req.requestId = nextQueryId();
    req.layerMask = layerMask;
    req.ray = ray;
    SyncQueryResponse resp{};
    if (!m->syncMailbox()->submitAndWait(req, resp, /*timeout*/ 1000 * 1000)) {
        return PhysResult::BackendError;
    }
    outHit = resp.firstHit;
    return resp.status;
}

PhysResult PhysicsWorld3D::overlapSphereSync(const ayt::math::FVector3& center, float radius,
                                             std::vector<BodyHandle>& out, PhysLayerMask layerMask) {
    PhysicsManager* m = _manager;
    if (!m || !m->syncMailbox()) return PhysResult::InvalidState;
    SyncQueryRequest req{};
    req.type = SyncQueryType::OverlapSphere;
    req.requestId = nextQueryId();
    req.layerMask = layerMask;
    req.sphereCenter = center;
    req.sphereRadius = radius;
    SyncQueryResponse resp{};
    if (!m->syncMailbox()->submitAndWait(req, resp, /*timeout*/ 1000 * 1000)) {
        return PhysResult::BackendError;
    }
    out = resp.overlaps;
    return resp.status;
}

void PhysicsWorld3D::wakeAll() {
    PhysicsManager* m = _manager;
    if (!m) return;
    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::WakeAll;
    (void)m->commandQueue()->tryPush(cmd);
}

void PhysicsWorld3D::sleepAll() {
    PhysicsManager* m = _manager;
    if (!m) return;
    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::SleepAll;
    (void)m->commandQueue()->tryPush(cmd);
}

} // namespace ayt::physics
