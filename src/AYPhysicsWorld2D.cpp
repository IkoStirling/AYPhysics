#include "AYPhysicsWorld2D.h"

#include "AYPhysicsManager.h"

namespace ayt::physics {

template <typename Handle>
Handle PhysicsWorld2D::mintHandle(uint32_t& indexSlot, uint32_t& genSlot) noexcept {
    const uint32_t nextIndex = indexSlot + 1u;
    indexSlot = nextIndex;
    const uint32_t freshGen = bumpGeneration(genSlot);
    genSlot = freshGen;
    return makeHandle(nextIndex, freshGen);
}

uint32_t PhysicsWorld2D::nextQueryId() noexcept {
    const uint32_t v = _nextQueryId + 1u;
    _nextQueryId = (v == 0u) ? 1u : v;
    return _nextQueryId;
}

PhysResult PhysicsWorld2D::createRigidbody(const RigidbodyDesc& desc, BodyHandle& outHandle) {
    outHandle = InvalidBodyHandle;
    PhysicsManager* m = _manager;
    if (!m || !m->createPool() || !m->commandQueue2D()) return PhysResult::InvalidState;

    outHandle = mintHandle<BodyHandle>(_nextBodyIndex, _nextBodyGen);

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
    if (!m->commandQueue2D()->tryPush(cmd)) {
        (void)m->createPool()->take(slotId, payload);
        outHandle = InvalidBodyHandle;
        return PhysResult::QueueFull;
    }
    return PhysResult::Ok;
}

PhysResult PhysicsWorld2D::destroyRigidbody(BodyHandle h) {
    PhysicsManager* m = _manager;
    if (!m || !m->commandQueue2D()) return PhysResult::InvalidState;
    if (!isValidHandle(h)) return PhysResult::InvalidParam;
    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::DestroyRigidbody;
    cmd.body = h;
    if (!m->commandQueue2D()->tryPush(cmd)) return PhysResult::QueueFull;
    return PhysResult::Ok;
}

PhysResult PhysicsWorld2D::setRigidbodyTransform(BodyHandle h, const ayt::math::FVector3& p,
                                                   const ayt::math::FQuaternion& r) {
    PhysicsManager* m = _manager;
    if (!m || !m->commandQueue2D()) return PhysResult::InvalidState;
    if (!isValidHandle(h)) return PhysResult::InvalidParam;
    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::SetRigidbodyTransform;
    cmd.body = h;
    cmd.u.xform.px = p.x; cmd.u.xform.py = p.y; cmd.u.xform.pz = p.z;
    cmd.u.xform.qx = r.x; cmd.u.xform.qy = r.y; cmd.u.xform.qz = r.z; cmd.u.xform.qw = r.w;
    if (!m->commandQueue2D()->tryPush(cmd)) return PhysResult::QueueFull;
    return PhysResult::Ok;
}

PhysResult PhysicsWorld2D::applyForce(BodyHandle h, const ayt::math::FVector3& f) {
    PhysicsManager* m = _manager;
    if (!m || !m->commandQueue2D()) return PhysResult::InvalidState;
    if (!isValidHandle(h)) return PhysResult::InvalidParam;
    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::ApplyForce;
    cmd.body = h;
    cmd.u.vec4.x = f.x; cmd.u.vec4.y = f.y; cmd.u.vec4.z = f.z;
    if (!m->commandQueue2D()->tryPush(cmd)) return PhysResult::QueueFull;
    return PhysResult::Ok;
}

PhysResult PhysicsWorld2D::applyImpulse(BodyHandle h, const ayt::math::FVector3& impulse,
                                        const ayt::math::FVector3& point) {
    PhysicsManager* m = _manager;
    if (!m || !m->commandQueue2D()) return PhysResult::InvalidState;
    if (!isValidHandle(h)) return PhysResult::InvalidParam;
    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::ApplyImpulse;
    cmd.body = h;
    cmd.u.vec4.x = impulse.x; cmd.u.vec4.y = impulse.y; cmd.u.vec4.z = impulse.z;
    (void)point;
    if (!m->commandQueue2D()->tryPush(cmd)) return PhysResult::QueueFull;
    return PhysResult::Ok;
}

PhysResult PhysicsWorld2D::applyTorque(BodyHandle h, float torque) {
    PhysicsManager* m = _manager;
    if (!m || !m->commandQueue2D()) return PhysResult::InvalidState;
    if (!isValidHandle(h)) return PhysResult::InvalidParam;
    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::ApplyTorque;
    cmd.body = h;
    cmd.u.vec4.z = torque;
    if (!m->commandQueue2D()->tryPush(cmd)) return PhysResult::QueueFull;
    return PhysResult::Ok;
}

PhysResult PhysicsWorld2D::applyAngularImpulse(BodyHandle h, float angularImpulse) {
    PhysicsManager* m = _manager;
    if (!m || !m->commandQueue2D()) return PhysResult::InvalidState;
    if (!isValidHandle(h)) return PhysResult::InvalidParam;
    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::ApplyAngularImpulse;
    cmd.body = h;
    cmd.u.vec4.z = angularImpulse;
    if (!m->commandQueue2D()->tryPush(cmd)) return PhysResult::QueueFull;
    return PhysResult::Ok;
}

PhysResult PhysicsWorld2D::setRigidbodyCollideMask(BodyHandle h, PhysLayerMask newMask) {
    PhysicsManager* m = _manager;
    if (!m || !m->commandQueue2D()) return PhysResult::InvalidState;
    if (!isValidHandle(h)) return PhysResult::InvalidParam;
    PhysicsCommand cmd{};
    cmd.type      = PhysicsCommandType::SetRigidbodyCollideMask;
    cmd.body      = h;
    cmd.layerMask = newMask;
    if (!m->commandQueue2D()->tryPush(cmd)) return PhysResult::QueueFull;
    return PhysResult::Ok;
}

PhysResult PhysicsWorld2D::createCollider(const ColliderDesc& desc, ColliderHandle& outHandle) {
    outHandle = InvalidColliderHandle;
    PhysicsManager* m = _manager;
    if (!m) return PhysResult::InvalidState;

    outHandle = mintHandle<ColliderHandle>(_nextColliderIndex, _nextColliderGen);

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
    if (!m->commandQueue2D()->tryPush(cmd)) {
        (void)m->createPool()->take(slotId, payload);
        outHandle = InvalidColliderHandle;
        return PhysResult::QueueFull;
    }
    return PhysResult::Ok;
}

PhysResult PhysicsWorld2D::destroyCollider(ColliderHandle h) {
    PhysicsManager* m = _manager;
    if (!m) return PhysResult::InvalidState;
    if (!isValidHandle(h)) return PhysResult::InvalidParam;
    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::DestroyCollider;
    cmd.collider = h;
    if (!m->commandQueue2D()->tryPush(cmd)) return PhysResult::QueueFull;
    return PhysResult::Ok;
}

PhysResult PhysicsWorld2D::createJoint(const JointDesc& desc, JointHandle& outHandle) {
    outHandle = InvalidJointHandle;
    PhysicsManager* m = _manager;
    if (!m) return PhysResult::InvalidState;

    outHandle = mintHandle<JointHandle>(_nextJointIndex, _nextJointGen);

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
    if (!m->commandQueue2D()->tryPush(cmd)) {
        (void)m->createPool()->take(slotId, payload);
        outHandle = InvalidJointHandle;
        return PhysResult::QueueFull;
    }
    return PhysResult::Ok;
}

PhysResult PhysicsWorld2D::destroyJoint(JointHandle h) {
    PhysicsManager* m = _manager;
    if (!m) return PhysResult::InvalidState;
    if (!isValidHandle(h)) return PhysResult::InvalidParam;
    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::DestroyJoint;
    cmd.joint = h;
    if (!m->commandQueue2D()->tryPush(cmd)) return PhysResult::QueueFull;
    return PhysResult::Ok;
}

uint32_t PhysicsWorld2D::raycastAsync(const ayt::math::Ray& ray, PhysLayerMask layerMask) {
    PhysicsManager* m = _manager;
    if (!m) return 0;
    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::RaycastAsync;
    cmd.queryId = nextQueryId();
    cmd.layerMask = layerMask;
    cmd.u.ray.ox = ray.origin.x; cmd.u.ray.oy = ray.origin.y; cmd.u.ray.oz = ray.origin.z;
    cmd.u.ray.dx = ray.dir.x;    cmd.u.ray.dy = ray.dir.y;    cmd.u.ray.dz = ray.dir.z;
    if (!m->commandQueue2D()->tryPush(cmd)) return 0;
    return cmd.queryId;
}

uint32_t PhysicsWorld2D::overlapSphereAsync(const ayt::math::FVector3& center, float radius,
                                            PhysLayerMask layerMask) {
    PhysicsManager* m = _manager;
    if (!m) return 0;
    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::OverlapSphereAsync;
    cmd.queryId = nextQueryId();
    cmd.layerMask = layerMask;
    cmd.u.sphere.cx = center.x; cmd.u.sphere.cy = center.y; cmd.u.sphere.cz = center.z;
    cmd.u.sphere.radius = radius;
    if (!m->commandQueue2D()->tryPush(cmd)) return 0;
    return cmd.queryId;
}

uint32_t PhysicsWorld2D::overlapBoxAsync(const ayt::math::FVector3& center,
                                        const ayt::math::FVector3& halfExtents,
                                        PhysLayerMask layerMask) {
    PhysicsManager* m = _manager;
    if (!m) return 0;
    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::OverlapBoxAsync;
    cmd.queryId = nextQueryId();
    cmd.layerMask = layerMask;
    cmd.u.box.cx = center.x; cmd.u.box.cy = center.y; cmd.u.box.cz = center.z;
    cmd.u.box.hx = halfExtents.x; cmd.u.box.hy = halfExtents.y; cmd.u.box.hz = halfExtents.z;
    if (!m->commandQueue2D()->tryPush(cmd)) return 0;
    return cmd.queryId;
}

PhysResult PhysicsWorld2D::raycastSync(const ayt::math::Ray& ray, RaycastHit& outHit,
                                       PhysLayerMask layerMask) {
    PhysicsManager* m = _manager;
    if (!m || !m->syncMailbox2D()) return PhysResult::InvalidState;
    SyncQueryRequest req{};
    req.type = SyncQueryType::Raycast;
    req.requestId = nextQueryId();
    req.layerMask = layerMask;
    req.ray = ray;
    SyncQueryResponse resp{};
    if (!m->syncMailbox2D()->submitAndWait(req, resp, 1000 * 1000)) {
        return PhysResult::BackendError;
    }
    outHit = resp.firstHit;
    return resp.status;
}

PhysResult PhysicsWorld2D::overlapSphereSync(const ayt::math::FVector3& center, float radius,
                                             std::vector<BodyHandle>& out, PhysLayerMask layerMask) {
    PhysicsManager* m = _manager;
    if (!m || !m->syncMailbox2D()) return PhysResult::InvalidState;
    SyncQueryRequest req{};
    req.type = SyncQueryType::OverlapSphere;
    req.requestId = nextQueryId();
    req.layerMask = layerMask;
    req.sphereCenter = center;
    req.sphereRadius = radius;
    SyncQueryResponse resp{};
    if (!m->syncMailbox2D()->submitAndWait(req, resp, 1000 * 1000)) {
        return PhysResult::BackendError;
    }
    out = resp.overlaps;
    return resp.status;
}

PhysResult PhysicsWorld2D::overlapBoxSync(const ayt::math::FVector3& center,
                                          const ayt::math::FVector3& halfExtents,
                                          std::vector<BodyHandle>& out, PhysLayerMask layerMask) {
    PhysicsManager* m = _manager;
    if (!m || !m->syncMailbox2D()) return PhysResult::InvalidState;
    SyncQueryRequest req{};
    req.type = SyncQueryType::OverlapBox;
    req.requestId = nextQueryId();
    req.layerMask = layerMask;
    req.boxCenter = center;
    req.boxHalfExtents = halfExtents;
    SyncQueryResponse resp{};
    if (!m->syncMailbox2D()->submitAndWait(req, resp, 1000 * 1000)) {
        return PhysResult::BackendError;
    }
    out = resp.overlaps;
    return resp.status;
}

PhysResult PhysicsWorld2D::setRigidbodyVelocity(BodyHandle h, const ayt::math::FVector3& v) {
    PhysicsManager* m = _manager;
    if (!m || !m->commandQueue2D()) return PhysResult::InvalidState;
    if (!isValidHandle(h)) return PhysResult::InvalidParam;
    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::SetRigidbodyVelocity;
    cmd.body = h;
    cmd.u.vec4.x = v.x; cmd.u.vec4.y = v.y; cmd.u.vec4.z = v.z;
    if (!m->commandQueue2D()->tryPush(cmd)) return PhysResult::QueueFull;
    return PhysResult::Ok;
}

PhysResult PhysicsWorld2D::setGravityScale(BodyHandle h, float scale) {
    PhysicsManager* m = _manager;
    if (!m || !m->commandQueue2D()) return PhysResult::InvalidState;
    if (!isValidHandle(h)) return PhysResult::InvalidParam;
    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::SetGravityScale;
    cmd.body = h;
    cmd.u.vec4.x = scale;
    if (!m->commandQueue2D()->tryPush(cmd)) return PhysResult::QueueFull;
    return PhysResult::Ok;
}

void PhysicsWorld2D::wakeAll() {
    PhysicsManager* m = _manager;
    if (!m) return;
    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::WakeAll;
    (void)m->commandQueue2D()->tryPush(cmd);
}

void PhysicsWorld2D::sleepAll() {
    PhysicsManager* m = _manager;
    if (!m) return;
    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::SleepAll;
    (void)m->commandQueue2D()->tryPush(cmd);
}

} // namespace ayt::physics
