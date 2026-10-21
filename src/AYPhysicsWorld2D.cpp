#include "AYPhysics/PhysicsWorld2D.h"

#include "AYPhysics/PhysicsManager.h"

#include <cmath>
#include <new>
#include <utility>

namespace ayt::physics {

template <typename Handle>
Handle PhysicsWorld2D::mintHandle(uint32_t& indexSlot, uint32_t& genSlot) noexcept {
    // F-A — mirror of PhysicsWorld3D::mintHandle. See the 3D version for the
    // full rationale: capping by descriptor().maxBodies prevents minting a
    // handle the backend's NotFound gate will silently reject.
    const uint32_t maxBodies = _manager ? _manager->descriptor().maxBodies : 0u;
    const uint32_t nextIndex = indexSlot + 1u;
    if (nextIndex >= maxBodies) {
        return InvalidBodyHandle;  // aliased to 0u for all Handle types
    }
    const uint32_t freshGen = bumpGeneration(genSlot);
    indexSlot = nextIndex;
    genSlot   = freshGen;
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
    PhysicsDebugQuery debug; debug.type=SyncQueryType::Raycast; debug.dimension=PhysicsDimension::TwoD;
    debug.queryId=cmd.queryId; debug.layerMask=layerMask;
    debug.ray=ray;
    (void)m->debugDraw().track(debug);
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
    PhysicsDebugQuery debug; debug.type=SyncQueryType::OverlapSphere; debug.dimension=PhysicsDimension::TwoD;
    debug.queryId=cmd.queryId; debug.layerMask=layerMask;
    debug.center=center; debug.radius=radius;
    (void)m->debugDraw().track(debug);
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
    PhysicsDebugQuery debug; debug.type=SyncQueryType::OverlapBox; debug.dimension=PhysicsDimension::TwoD;
    debug.queryId=cmd.queryId; debug.layerMask=layerMask;
    debug.center=center; debug.halfExtents=halfExtents;
    (void)m->debugDraw().track(debug);
    return cmd.queryId;
}

PhysResult PhysicsWorld2D::raycastSync(const ayt::math::Ray& ray, RaycastHit& outHit,
                                       PhysLayerMask layerMask) {
    outHit = {};
    const auto finite = [](const auto& value) {
        return std::isfinite(value.x) && std::isfinite(value.y)
            && std::isfinite(value.z);
    };
    if (!finite(ray.origin) || !finite(ray.dir)) return PhysResult::InvalidParam;
    PhysicsManager* m = _manager;
    if (!m || !m->isRunning() || !m->syncMailbox2D()) return PhysResult::InvalidState;
    PhysicsDebugQuery debug; debug.type = SyncQueryType::Raycast;
    debug.dimension = PhysicsDimension::TwoD;
    debug.ray = ray;
    SyncQueryRequest req{};
    req.type = SyncQueryType::Raycast;
    req.requestId = nextQueryId();
    req.layerMask = layerMask;
    req.ray = ray;
    debug.queryId=req.requestId; debug.layerMask=layerMask;
    SyncQueryResponse resp{};
    if (!m->syncMailbox2D()->submitAndWait(req, resp, 1000 * 1000)) {
        (void)m->debugDraw().record(debug, PhysResult::BackendError, false);
        return PhysResult::BackendError;
    }
    if (resp.status == PhysResult::Ok) outHit = resp.firstHit;
    (void)m->debugDraw().record(debug, resp.status, outHit.hit, &outHit);
    return resp.status;
}

PhysResult PhysicsWorld2D::raycastAllSync(
    const RaycastAll2D& query, std::vector<RaycastHit>& out) {
    out.clear();
    try {
        const auto finite = [](const auto& value) {
            return std::isfinite(value.x) && std::isfinite(value.y)
                && std::isfinite(value.z);
        };
        const float lengthSq = query.displacement.x * query.displacement.x
            + query.displacement.y * query.displacement.y;
        if (!finite(query.origin) || !finite(query.displacement)
            || !std::isfinite(lengthSq) || lengthSq < 1.0e-12f
            || lengthSq > 10000.0f * 10000.0f
            || query.ignoredBodies.size() > 64u
            || query.maxHits == 0u || query.maxHits > 1024u) {
            return PhysResult::InvalidParam;
        }
        PhysicsManager* manager = _manager;
        if (!manager || !manager->isRunning() || !manager->syncMailbox2D()) {
            return PhysResult::InvalidState;
        }
        SyncQueryRequest request{};
        request.type = SyncQueryType::RaycastAll2D;
        request.requestId = nextQueryId();
        request.raycastAll2D = query;
        PhysicsDebugQuery debug;
        debug.type = SyncQueryType::RaycastAll2D;
        debug.dimension = PhysicsDimension::TwoD;
        debug.queryId = request.requestId;
        debug.layerMask = query.layerMask;
        debug.ray = ayt::math::Ray(query.origin, query.displacement);
        debug.rayDistance = std::sqrt(lengthSq);
        SyncQueryResponse response{};
        if (!manager->syncMailbox2D()->submitAndWait(
                request, response, 1000 * 1000)) {
            (void)manager->debugDraw().record(
                debug, PhysResult::BackendError, false);
            return PhysResult::BackendError;
        }
        if (response.status == PhysResult::Ok) {
            out = std::move(response.rayHits);
        }
        (void)manager->debugDraw().record(
            debug, response.status, !out.empty(),
            out.empty() ? nullptr : &out.front());
        return response.status;
    } catch (const std::bad_alloc&) {
        out.clear();
        return PhysResult::NoMemory;
    }
}

PhysResult PhysicsWorld2D::sphereSweepSync(const SphereSweep& sweep,
                                           RaycastHit& outHit) {
    outHit = {};
    const auto finite = [](const auto& value) {
        return std::isfinite(value.x) && std::isfinite(value.y)
            && std::isfinite(value.z);
    };
    const float travelSq = sweep.displacement.x * sweep.displacement.x
        + sweep.displacement.y * sweep.displacement.y;
    if (!finite(sweep.origin) || !finite(sweep.displacement)
        || !std::isfinite(travelSq) || travelSq < 1.0e-12f
        || travelSq > 10000.0f * 10000.0f
        || !std::isfinite(sweep.radius) || sweep.radius <= 0.0f
        || sweep.radius > 1000.0f || sweep.ignoredBodies.size() > 64u) {
        return PhysResult::InvalidParam;
    }
    PhysicsManager* manager = _manager;
    if (!manager || !manager->isRunning() || !manager->syncMailbox2D()) {
        return PhysResult::InvalidState;
    }
    SyncQueryRequest request{};
    request.type = SyncQueryType::SphereSweep;
    request.requestId = nextQueryId();
    request.sphereSweep = sweep;
    PhysicsDebugQuery debug;
    debug.type = SyncQueryType::SphereSweep;
    debug.dimension = PhysicsDimension::TwoD;
    debug.queryId = request.requestId;
    debug.layerMask = sweep.layerMask;
    debug.center = sweep.origin;
    debug.displacement = sweep.displacement;
    debug.radius = sweep.radius;
    SyncQueryResponse response{};
    if (!manager->syncMailbox2D()->submitAndWait(
            request, response, 1000 * 1000)) {
        (void)manager->debugDraw().record(
            debug, PhysResult::BackendError, false);
        return PhysResult::BackendError;
    }
    if (response.status == PhysResult::Ok) outHit = response.firstHit;
    (void)manager->debugDraw().record(
        debug, response.status, outHit.hit, &outHit);
    return response.status;
}

PhysResult PhysicsWorld2D::sphereSweepAllSync(
    const SphereSweep& sweep, std::vector<RaycastHit>& out,
    uint32_t maxHits) {
    out.clear();
    try {
        const auto finite = [](const auto& value) {
            return std::isfinite(value.x) && std::isfinite(value.y)
                && std::isfinite(value.z);
        };
        const float travelSq = sweep.displacement.x * sweep.displacement.x
            + sweep.displacement.y * sweep.displacement.y;
        if (!finite(sweep.origin) || !finite(sweep.displacement)
            || !std::isfinite(travelSq) || travelSq < 1.0e-12f
            || travelSq > 10000.0f * 10000.0f
            || !std::isfinite(sweep.radius) || sweep.radius <= 0.0f
            || sweep.radius > 1000.0f || sweep.ignoredBodies.size() > 64u
            || maxHits == 0u || maxHits > 1024u) {
            return PhysResult::InvalidParam;
        }
        PhysicsManager* manager = _manager;
        if (!manager || !manager->isRunning() || !manager->syncMailbox2D()) {
            return PhysResult::InvalidState;
        }
        SyncQueryRequest request{};
        request.type = SyncQueryType::SphereSweepAll2D;
        request.requestId = nextQueryId();
        request.sphereSweep = sweep;
        request.maxHits = maxHits;
        PhysicsDebugQuery debug;
        debug.type = SyncQueryType::SphereSweepAll2D;
        debug.dimension = PhysicsDimension::TwoD;
        debug.queryId = request.requestId;
        debug.layerMask = sweep.layerMask;
        debug.center = sweep.origin;
        debug.displacement = sweep.displacement;
        debug.radius = sweep.radius;
        SyncQueryResponse response{};
        if (!manager->syncMailbox2D()->submitAndWait(
                request, response, 1000 * 1000)) {
            (void)manager->debugDraw().record(
                debug, PhysResult::BackendError, false);
            return PhysResult::BackendError;
        }
        if (response.status == PhysResult::Ok) out = std::move(response.rayHits);
        (void)manager->debugDraw().record(
            debug, response.status, !out.empty(),
            out.empty() ? nullptr : &out.front());
        return response.status;
    } catch (const std::bad_alloc&) {
        out.clear();
        return PhysResult::NoMemory;
    }
}

PhysResult PhysicsWorld2D::boxSweepSync(const BoxSweep2D& sweep,
                                        RaycastHit& outHit) {
    outHit = {};
    const auto finite = [](const auto& value) {
        return std::isfinite(value.x) && std::isfinite(value.y)
            && std::isfinite(value.z);
    };
    const float travelSq = sweep.displacement.x * sweep.displacement.x
        + sweep.displacement.y * sweep.displacement.y;
    if (!finite(sweep.origin) || !finite(sweep.displacement)
        || !finite(sweep.halfExtents)
        || !std::isfinite(travelSq) || travelSq < 1.0e-12f
        || travelSq > 10000.0f * 10000.0f
        || sweep.halfExtents.x <= 0.0f || sweep.halfExtents.y <= 0.0f
        || sweep.halfExtents.x > 1000.0f
        || sweep.halfExtents.y > 1000.0f
        || sweep.ignoredBodies.size() > 64u) {
        return PhysResult::InvalidParam;
    }
    PhysicsManager* manager = _manager;
    if (!manager || !manager->isRunning() || !manager->syncMailbox2D()) {
        return PhysResult::InvalidState;
    }
    SyncQueryRequest request{};
    request.type = SyncQueryType::BoxSweep2D;
    request.requestId = nextQueryId();
    request.boxSweep2D = sweep;
    PhysicsDebugQuery debug;
    debug.type = SyncQueryType::BoxSweep2D;
    debug.dimension = PhysicsDimension::TwoD;
    debug.queryId = request.requestId;
    debug.layerMask = sweep.layerMask;
    debug.center = sweep.origin;
    debug.displacement = sweep.displacement;
    debug.halfExtents = sweep.halfExtents;
    SyncQueryResponse response{};
    if (!manager->syncMailbox2D()->submitAndWait(
            request, response, 1000 * 1000)) {
        (void)manager->debugDraw().record(
            debug, PhysResult::BackendError, false);
        return PhysResult::BackendError;
    }
    if (response.status == PhysResult::Ok) outHit = response.firstHit;
    (void)manager->debugDraw().record(
        debug, response.status, outHit.hit, &outHit);
    return response.status;
}

PhysResult PhysicsWorld2D::boxSweepAllSync(
    const BoxSweep2D& sweep, std::vector<RaycastHit>& out,
    uint32_t maxHits) {
    out.clear();
    try {
        const auto finite = [](const auto& value) {
            return std::isfinite(value.x) && std::isfinite(value.y)
                && std::isfinite(value.z);
        };
        const float travelSq = sweep.displacement.x * sweep.displacement.x
            + sweep.displacement.y * sweep.displacement.y;
        if (!finite(sweep.origin) || !finite(sweep.displacement)
            || !finite(sweep.halfExtents)
            || !std::isfinite(travelSq) || travelSq < 1.0e-12f
            || travelSq > 10000.0f * 10000.0f
            || sweep.halfExtents.x <= 0.0f || sweep.halfExtents.y <= 0.0f
            || sweep.halfExtents.x > 1000.0f
            || sweep.halfExtents.y > 1000.0f
            || sweep.ignoredBodies.size() > 64u
            || maxHits == 0u || maxHits > 1024u) {
            return PhysResult::InvalidParam;
        }
        PhysicsManager* manager = _manager;
        if (!manager || !manager->isRunning() || !manager->syncMailbox2D()) {
            return PhysResult::InvalidState;
        }
        SyncQueryRequest request{};
        request.type = SyncQueryType::BoxSweepAll2D;
        request.requestId = nextQueryId();
        request.boxSweep2D = sweep;
        request.maxHits = maxHits;
        PhysicsDebugQuery debug;
        debug.type = SyncQueryType::BoxSweepAll2D;
        debug.dimension = PhysicsDimension::TwoD;
        debug.queryId = request.requestId;
        debug.layerMask = sweep.layerMask;
        debug.center = sweep.origin;
        debug.displacement = sweep.displacement;
        debug.halfExtents = sweep.halfExtents;
        SyncQueryResponse response{};
        if (!manager->syncMailbox2D()->submitAndWait(
                request, response, 1000 * 1000)) {
            (void)manager->debugDraw().record(
                debug, PhysResult::BackendError, false);
            return PhysResult::BackendError;
        }
        if (response.status == PhysResult::Ok) {
            out = std::move(response.rayHits);
        }
        (void)manager->debugDraw().record(
            debug, response.status, !out.empty(),
            out.empty() ? nullptr : &out.front());
        return response.status;
    } catch (const std::bad_alloc&) {
        out.clear();
        return PhysResult::NoMemory;
    }
}

PhysResult PhysicsWorld2D::orientedBoxSweepSync(
    const OrientedBoxSweep2D& sweep, RaycastHit& outHit) {
    outHit = {};
    try {
        const auto finite = [](const auto& value) {
            return std::isfinite(value.x) && std::isfinite(value.y)
                && std::isfinite(value.z);
        };
        const float travelSq = sweep.displacement.x * sweep.displacement.x
            + sweep.displacement.y * sweep.displacement.y;
        if (!finite(sweep.origin) || !finite(sweep.displacement)
            || !finite(sweep.halfExtents)
            || !std::isfinite(sweep.angleRadians)
            || !std::isfinite(travelSq) || travelSq < 1.0e-12f
            || travelSq > 10000.0f * 10000.0f
            || sweep.halfExtents.x <= 0.0f || sweep.halfExtents.y <= 0.0f
            || sweep.halfExtents.x > 1000.0f
            || sweep.halfExtents.y > 1000.0f
            || sweep.ignoredBodies.size() > 64u) {
            return PhysResult::InvalidParam;
        }
        PhysicsManager* manager = _manager;
        if (!manager || !manager->isRunning() || !manager->syncMailbox2D()) {
            return PhysResult::InvalidState;
        }
        SyncQueryRequest request{};
        request.type = SyncQueryType::OrientedBoxSweep2D;
        request.requestId = nextQueryId();
        request.orientedBoxSweep2D = sweep;
        PhysicsDebugQuery debug;
        debug.type = SyncQueryType::OrientedBoxSweep2D;
        debug.dimension = PhysicsDimension::TwoD;
        debug.queryId = request.requestId;
        debug.layerMask = sweep.layerMask;
        debug.center = sweep.origin;
        debug.displacement = sweep.displacement;
        debug.halfExtents = sweep.halfExtents;
        debug.rotation = ayt::math::FQuaternion::fromAxisAngle(
            {0.0f, 0.0f, 1.0f}, sweep.angleRadians);
        SyncQueryResponse response{};
        if (!manager->syncMailbox2D()->submitAndWait(
                request, response, 1000 * 1000)) {
            (void)manager->debugDraw().record(
                debug, PhysResult::BackendError, false);
            return PhysResult::BackendError;
        }
        if (response.status == PhysResult::Ok) outHit = response.firstHit;
        (void)manager->debugDraw().record(
            debug, response.status, outHit.hit, &outHit);
        return response.status;
    } catch (...) {
        outHit = {};
        return PhysResult::NoMemory;
    }
}

PhysResult PhysicsWorld2D::orientedBoxSweepAllSync(
    const OrientedBoxSweep2D& sweep, std::vector<RaycastHit>& out,
    uint32_t maxHits) {
    out.clear();
    try {
        const auto finite = [](const auto& value) {
            return std::isfinite(value.x) && std::isfinite(value.y)
                && std::isfinite(value.z);
        };
        const float travelSq = sweep.displacement.x * sweep.displacement.x
            + sweep.displacement.y * sweep.displacement.y;
        if (!finite(sweep.origin) || !finite(sweep.displacement)
            || !finite(sweep.halfExtents)
            || !std::isfinite(sweep.angleRadians)
            || !std::isfinite(travelSq) || travelSq < 1.0e-12f
            || travelSq > 10000.0f * 10000.0f
            || sweep.halfExtents.x <= 0.0f || sweep.halfExtents.y <= 0.0f
            || sweep.halfExtents.x > 1000.0f
            || sweep.halfExtents.y > 1000.0f
            || sweep.ignoredBodies.size() > 64u
            || maxHits == 0u || maxHits > 1024u) {
            return PhysResult::InvalidParam;
        }
        PhysicsManager* manager = _manager;
        if (!manager || !manager->isRunning() || !manager->syncMailbox2D()) {
            return PhysResult::InvalidState;
        }
        SyncQueryRequest request{};
        request.type = SyncQueryType::OrientedBoxSweepAll2D;
        request.requestId = nextQueryId();
        request.orientedBoxSweep2D = sweep;
        request.maxHits = maxHits;
        PhysicsDebugQuery debug;
        debug.type = SyncQueryType::OrientedBoxSweepAll2D;
        debug.dimension = PhysicsDimension::TwoD;
        debug.queryId = request.requestId;
        debug.layerMask = sweep.layerMask;
        debug.center = sweep.origin;
        debug.displacement = sweep.displacement;
        debug.halfExtents = sweep.halfExtents;
        debug.rotation = ayt::math::FQuaternion::fromAxisAngle(
            {0.0f, 0.0f, 1.0f}, sweep.angleRadians);
        SyncQueryResponse response{};
        if (!manager->syncMailbox2D()->submitAndWait(
                request, response, 1000 * 1000)) {
            (void)manager->debugDraw().record(
                debug, PhysResult::BackendError, false);
            return PhysResult::BackendError;
        }
        if (response.status == PhysResult::Ok) out = std::move(response.rayHits);
        (void)manager->debugDraw().record(
            debug, response.status, !out.empty(),
            out.empty() ? nullptr : &out.front());
        return response.status;
    } catch (const std::bad_alloc&) {
        out.clear();
        return PhysResult::NoMemory;
    }
}

PhysResult PhysicsWorld2D::capsuleSweepSync(const CapsuleSweep2D& sweep,
                                            RaycastHit& outHit) {
    outHit = {};
    const auto finite = [](const auto& value) {
        return std::isfinite(value.x) && std::isfinite(value.y)
            && std::isfinite(value.z);
    };
    const float travelSq = sweep.displacement.x * sweep.displacement.x
        + sweep.displacement.y * sweep.displacement.y;
    if (!finite(sweep.origin) || !finite(sweep.displacement)
        || !std::isfinite(travelSq) || travelSq < 1.0e-12f
        || travelSq > 10000.0f * 10000.0f
        || !std::isfinite(sweep.radius) || sweep.radius <= 0.0f
        || sweep.radius > 1000.0f || !std::isfinite(sweep.height)
        || sweep.height < 2.0f * sweep.radius || sweep.height > 2000.0f
        || sweep.ignoredBodies.size() > 64u) {
        return PhysResult::InvalidParam;
    }
    PhysicsManager* manager = _manager;
    if (!manager || !manager->isRunning() || !manager->syncMailbox2D()) {
        return PhysResult::InvalidState;
    }
    SyncQueryRequest request{};
    request.type = SyncQueryType::CapsuleSweep2D;
    request.requestId = nextQueryId();
    request.capsuleSweep2D = sweep;
    PhysicsDebugQuery debug;
    debug.type = SyncQueryType::CapsuleSweep2D;
    debug.dimension = PhysicsDimension::TwoD;
    debug.queryId = request.requestId;
    debug.layerMask = sweep.layerMask;
    debug.center = sweep.origin;
    debug.displacement = sweep.displacement;
    debug.radius = sweep.radius;
    debug.height = sweep.height;
    SyncQueryResponse response{};
    if (!manager->syncMailbox2D()->submitAndWait(
            request, response, 1000 * 1000)) {
        (void)manager->debugDraw().record(
            debug, PhysResult::BackendError, false);
        return PhysResult::BackendError;
    }
    if (response.status == PhysResult::Ok) outHit = response.firstHit;
    (void)manager->debugDraw().record(
        debug, response.status, outHit.hit, &outHit);
    return response.status;
}

PhysResult PhysicsWorld2D::capsuleSweepAllSync(
    const CapsuleSweep2D& sweep, std::vector<RaycastHit>& out,
    uint32_t maxHits) {
    out.clear();
    try {
        const auto finite = [](const auto& value) {
            return std::isfinite(value.x) && std::isfinite(value.y)
                && std::isfinite(value.z);
        };
        const float travelSq = sweep.displacement.x * sweep.displacement.x
            + sweep.displacement.y * sweep.displacement.y;
        if (!finite(sweep.origin) || !finite(sweep.displacement)
            || !std::isfinite(travelSq) || travelSq < 1.0e-12f
            || travelSq > 10000.0f * 10000.0f
            || !std::isfinite(sweep.radius) || sweep.radius <= 0.0f
            || sweep.radius > 1000.0f || !std::isfinite(sweep.height)
            || sweep.height < 2.0f * sweep.radius
            || sweep.height > 2000.0f
            || sweep.ignoredBodies.size() > 64u
            || maxHits == 0u || maxHits > 1024u) {
            return PhysResult::InvalidParam;
        }
        PhysicsManager* manager = _manager;
        if (!manager || !manager->isRunning() || !manager->syncMailbox2D()) {
            return PhysResult::InvalidState;
        }
        SyncQueryRequest request{};
        request.type = SyncQueryType::CapsuleSweepAll2D;
        request.requestId = nextQueryId();
        request.capsuleSweep2D = sweep;
        request.maxHits = maxHits;
        PhysicsDebugQuery debug;
        debug.type = SyncQueryType::CapsuleSweepAll2D;
        debug.dimension = PhysicsDimension::TwoD;
        debug.queryId = request.requestId;
        debug.layerMask = sweep.layerMask;
        debug.center = sweep.origin;
        debug.displacement = sweep.displacement;
        debug.radius = sweep.radius;
        debug.height = sweep.height;
        SyncQueryResponse response{};
        if (!manager->syncMailbox2D()->submitAndWait(
                request, response, 1000 * 1000)) {
            (void)manager->debugDraw().record(
                debug, PhysResult::BackendError, false);
            return PhysResult::BackendError;
        }
        if (response.status == PhysResult::Ok) out = std::move(response.rayHits);
        (void)manager->debugDraw().record(
            debug, response.status, !out.empty(),
            out.empty() ? nullptr : &out.front());
        return response.status;
    } catch (const std::bad_alloc&) {
        out.clear();
        return PhysResult::NoMemory;
    }
}

PhysResult PhysicsWorld2D::overlapCapsuleSync(
    const CapsuleOverlap2D& query, std::vector<BodyHandle>& out) {
    out.clear();
    try {
    if (!std::isfinite(query.center.x) || !std::isfinite(query.center.y)
        || !std::isfinite(query.center.z)
        || !std::isfinite(query.radius) || query.radius <= 0.0f
        || query.radius > 1000.0f || !std::isfinite(query.height)
        || query.height < 2.0f * query.radius || query.height > 2000.0f
        || query.ignoredBodies.size() > 64u) {
        return PhysResult::InvalidParam;
    }
    PhysicsManager* manager = _manager;
    if (!manager || !manager->isRunning() || !manager->syncMailbox2D()) {
        return PhysResult::InvalidState;
    }
    SyncQueryRequest request{};
    request.type = SyncQueryType::CapsuleOverlap2D;
    request.requestId = nextQueryId();
    request.capsuleOverlap2D = query;
    PhysicsDebugQuery debug;
    debug.type = SyncQueryType::CapsuleOverlap2D;
    debug.dimension = PhysicsDimension::TwoD;
    debug.queryId = request.requestId;
    debug.layerMask = query.layerMask;
    debug.center = query.center;
    debug.radius = query.radius;
    debug.height = query.height;
    SyncQueryResponse response{};
    if (!manager->syncMailbox2D()->submitAndWait(
            request, response, 1000 * 1000)) {
        (void)manager->debugDraw().record(
            debug, PhysResult::BackendError, false);
        return PhysResult::BackendError;
    }
    if (response.status == PhysResult::Ok) {
        out = std::move(response.overlaps);
    }
    (void)manager->debugDraw().record(
        debug, response.status, !out.empty(), nullptr, &out);
    return response.status;
    } catch (const std::bad_alloc&) {
        out.clear();
        return PhysResult::NoMemory;
    }
}

PhysResult PhysicsWorld2D::overlapCapsuleCollidersSync(
    const CapsuleOverlap2D& query,
    std::vector<ColliderOverlapHit2D>& out, uint32_t maxHits) {
    out.clear();
    try {
        if (!std::isfinite(query.center.x) || !std::isfinite(query.center.y)
            || !std::isfinite(query.center.z)
            || !std::isfinite(query.radius) || query.radius <= 0.0f
            || query.radius > 1000.0f || !std::isfinite(query.height)
            || query.height < 2.0f * query.radius
            || query.height > 2000.0f
            || query.ignoredBodies.size() > 64u
            || maxHits == 0u || maxHits > 1024u) {
            return PhysResult::InvalidParam;
        }
        PhysicsManager* manager = _manager;
        if (!manager || !manager->isRunning() || !manager->syncMailbox2D()) {
            return PhysResult::InvalidState;
        }
        SyncQueryRequest request{};
        request.type = SyncQueryType::CapsuleColliderOverlap2D;
        request.requestId = nextQueryId();
        request.capsuleOverlap2D = query;
        request.maxHits = maxHits;
        PhysicsDebugQuery debug;
        debug.type = SyncQueryType::CapsuleColliderOverlap2D;
        debug.dimension = PhysicsDimension::TwoD;
        debug.queryId = request.requestId;
        debug.layerMask = query.layerMask;
        debug.center = query.center;
        debug.radius = query.radius;
        debug.height = query.height;
        SyncQueryResponse response{};
        if (!manager->syncMailbox2D()->submitAndWait(
                request, response, 1000 * 1000)) {
            (void)manager->debugDraw().record(
                debug, PhysResult::BackendError, false);
            return PhysResult::BackendError;
        }
        if (response.status == PhysResult::Ok) {
            out = std::move(response.colliderOverlaps);
        }
        (void)manager->debugDraw().record(
            debug, response.status, !out.empty());
        return response.status;
    } catch (const std::bad_alloc&) {
        out.clear();
        return PhysResult::NoMemory;
    }
}

PhysResult PhysicsWorld2D::overlapOrientedBoxSync(
    const OrientedBoxOverlap2D& query, std::vector<BodyHandle>& out) {
    out.clear();
    try {
        const auto finite = [](const auto& value) {
            return std::isfinite(value.x) && std::isfinite(value.y)
                && std::isfinite(value.z);
        };
        if (!finite(query.center) || !finite(query.halfExtents)
            || query.halfExtents.x <= 0.0f
            || query.halfExtents.y <= 0.0f
            || query.halfExtents.x > 1000.0f
            || query.halfExtents.y > 1000.0f
            || !std::isfinite(query.angleRadians)
            || query.ignoredBodies.size() > 64u) {
            return PhysResult::InvalidParam;
        }
        PhysicsManager* manager = _manager;
        if (!manager || !manager->isRunning() || !manager->syncMailbox2D()) {
            return PhysResult::InvalidState;
        }
        SyncQueryRequest request{};
        request.type = SyncQueryType::OrientedBoxOverlap2D;
        request.requestId = nextQueryId();
        request.orientedBoxOverlap2D = query;
        PhysicsDebugQuery debug;
        debug.type = SyncQueryType::OrientedBoxOverlap2D;
        debug.dimension = PhysicsDimension::TwoD;
        debug.queryId = request.requestId;
        debug.layerMask = query.layerMask;
        debug.center = query.center;
        debug.halfExtents = query.halfExtents;
        debug.rotation = ayt::math::FQuaternion::fromAxisAngle(
            {0.0f, 0.0f, 1.0f}, query.angleRadians);
        SyncQueryResponse response{};
        if (!manager->syncMailbox2D()->submitAndWait(
                request, response, 1000 * 1000)) {
            (void)manager->debugDraw().record(
                debug, PhysResult::BackendError, false);
            return PhysResult::BackendError;
        }
        if (response.status == PhysResult::Ok) {
            out = std::move(response.overlaps);
        }
        (void)manager->debugDraw().record(
            debug, response.status, !out.empty(), nullptr, &out);
        return response.status;
    } catch (const std::bad_alloc&) {
        out.clear();
        return PhysResult::NoMemory;
    }
}

PhysResult PhysicsWorld2D::overlapOrientedBoxCollidersSync(
    const OrientedBoxOverlap2D& query,
    std::vector<ColliderOverlapHit2D>& out, uint32_t maxHits) {
    out.clear();
    try {
        const auto finite = [](const auto& value) {
            return std::isfinite(value.x) && std::isfinite(value.y)
                && std::isfinite(value.z);
        };
        if (!finite(query.center) || !finite(query.halfExtents)
            || query.halfExtents.x <= 0.0f
            || query.halfExtents.y <= 0.0f
            || query.halfExtents.x > 1000.0f
            || query.halfExtents.y > 1000.0f
            || !std::isfinite(query.angleRadians)
            || query.ignoredBodies.size() > 64u
            || maxHits == 0u || maxHits > 1024u) {
            return PhysResult::InvalidParam;
        }
        PhysicsManager* manager = _manager;
        if (!manager || !manager->isRunning() || !manager->syncMailbox2D()) {
            return PhysResult::InvalidState;
        }
        SyncQueryRequest request{};
        request.type = SyncQueryType::OrientedBoxColliderOverlap2D;
        request.requestId = nextQueryId();
        request.orientedBoxOverlap2D = query;
        request.maxHits = maxHits;
        PhysicsDebugQuery debug;
        debug.type = SyncQueryType::OrientedBoxColliderOverlap2D;
        debug.dimension = PhysicsDimension::TwoD;
        debug.queryId = request.requestId;
        debug.layerMask = query.layerMask;
        debug.center = query.center;
        debug.halfExtents = query.halfExtents;
        debug.rotation = ayt::math::FQuaternion::fromAxisAngle(
            {0.0f, 0.0f, 1.0f}, query.angleRadians);
        SyncQueryResponse response{};
        if (!manager->syncMailbox2D()->submitAndWait(
                request, response, 1000 * 1000)) {
            (void)manager->debugDraw().record(
                debug, PhysResult::BackendError, false);
            return PhysResult::BackendError;
        }
        if (response.status == PhysResult::Ok) {
            out = std::move(response.colliderOverlaps);
        }
        (void)manager->debugDraw().record(
            debug, response.status, !out.empty());
        return response.status;
    } catch (const std::bad_alloc&) {
        out.clear();
        return PhysResult::NoMemory;
    }
}

PhysResult PhysicsWorld2D::overlapSphereSync(const ayt::math::FVector3& center, float radius,
                                             std::vector<BodyHandle>& out, PhysLayerMask layerMask) {
    out.clear();
    if (!std::isfinite(center.x) || !std::isfinite(center.y)
        || !std::isfinite(center.z) || !std::isfinite(radius)
        || radius <= 0.0f) return PhysResult::InvalidParam;
    PhysicsManager* m = _manager;
    if (!m || !m->isRunning() || !m->syncMailbox2D()) return PhysResult::InvalidState;
    PhysicsDebugQuery debug; debug.type = SyncQueryType::OverlapSphere;
    debug.dimension = PhysicsDimension::TwoD;
    debug.center = center; debug.radius = radius;
    SyncQueryRequest req{};
    req.type = SyncQueryType::OverlapSphere;
    req.requestId = nextQueryId();
    req.layerMask = layerMask;
    req.sphereCenter = center;
    req.sphereRadius = radius;
    debug.queryId=req.requestId; debug.layerMask=layerMask;
    SyncQueryResponse resp{};
    if (!m->syncMailbox2D()->submitAndWait(req, resp, 1000 * 1000)) {
        (void)m->debugDraw().record(debug, PhysResult::BackendError, false);
        return PhysResult::BackendError;
    }
    if (resp.status == PhysResult::Ok) out = std::move(resp.overlaps);
    (void)m->debugDraw().record(debug, resp.status, !out.empty(),nullptr,&out);
    return resp.status;
}

PhysResult PhysicsWorld2D::overlapBoxSync(const ayt::math::FVector3& center,
                                          const ayt::math::FVector3& halfExtents,
                                          std::vector<BodyHandle>& out, PhysLayerMask layerMask) {
    out.clear();
    const auto finite = [](const auto& value) {
        return std::isfinite(value.x) && std::isfinite(value.y)
            && std::isfinite(value.z);
    };
    if (!finite(center) || !finite(halfExtents)
        || halfExtents.x <= 0.0f || halfExtents.y <= 0.0f) {
        return PhysResult::InvalidParam;
    }
    PhysicsManager* m = _manager;
    if (!m || !m->isRunning() || !m->syncMailbox2D()) return PhysResult::InvalidState;
    PhysicsDebugQuery debug; debug.type = SyncQueryType::OverlapBox;
    debug.dimension = PhysicsDimension::TwoD;
    debug.center = center; debug.halfExtents = halfExtents;
    SyncQueryRequest req{};
    req.type = SyncQueryType::OverlapBox;
    req.requestId = nextQueryId();
    req.layerMask = layerMask;
    req.boxCenter = center;
    req.boxHalfExtents = halfExtents;
    debug.queryId=req.requestId; debug.layerMask=layerMask;
    SyncQueryResponse resp{};
    if (!m->syncMailbox2D()->submitAndWait(req, resp, 1000 * 1000)) {
        (void)m->debugDraw().record(debug, PhysResult::BackendError, false);
        return PhysResult::BackendError;
    }
    if (resp.status == PhysResult::Ok) out = std::move(resp.overlaps);
    (void)m->debugDraw().record(debug, resp.status, !out.empty(),nullptr,&out);
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

PhysResult PhysicsWorld2D::setMass(BodyHandle h, float mass) {
    PhysicsManager* m = _manager;
    if (!m || !m->commandQueue2D()) return PhysResult::InvalidState;
    // F-P3: reject non-finite or non-positive mass synchronously — Inf/NaN
    // slip past the bare `!(mass > 0.0f)` check (Inf > 0 is true).
    if (!isValidHandle(h) || !std::isfinite(mass) || !(mass > 0.0f)) {
        return PhysResult::InvalidParam;
    }
    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::SetMass;
    cmd.body = h;
    cmd.u.vec4.x = mass;
    if (!m->commandQueue2D()->tryPush(cmd)) return PhysResult::QueueFull;
    return PhysResult::Ok;
}

PhysResult PhysicsWorld2D::setMaterial(ColliderHandle h, float friction, float restitution) {
    PhysicsManager* m = _manager;
    if (!m || !m->commandQueue2D()) return PhysResult::InvalidState;
    // F-P3: reject NaN/Inf/negative synchronously — backends would happily
    // take them otherwise.
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
