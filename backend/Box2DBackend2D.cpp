#include "Box2DBackend2D.h"

// R2.5: this is the ONLY TU that may include <box2d/...> headers.
#include <box2d/box2d.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <vector>

namespace ayt::physics {

namespace {

constexpr float kRayCastMaxDistance = 10000.0f;

inline void* handleToUserData(BodyHandle h) noexcept {
    return reinterpret_cast<void*>(static_cast<uintptr_t>(h));
}

inline BodyHandle userDataToHandle(void* userData) noexcept {
    if (userData == nullptr) return InvalidBodyHandle;
    return static_cast<BodyHandle>(reinterpret_cast<uintptr_t>(userData));
}

inline b2BodyType toB2BodyType(BodyType t) noexcept {
    switch (t) {
    case BodyType::Static:    return b2_staticBody;
    case BodyType::Dynamic:   return b2_dynamicBody;
    case BodyType::Kinematic: return b2_kinematicBody;
    }
    return b2_staticBody;
}

inline float quatToAngleZ(const ayt::math::FQuaternion& q) {
    const ayt::math::FVector3 euler = q.toEulerAngles();
    return euler.z;
}

inline ayt::math::FQuaternion angleZToQuat(float angle) {
    return ayt::math::FQuaternion::fromEulerAngles(
        ayt::math::FVector3(0.0f, 0.0f, angle));
}

inline b2Filter makeB2Filter(PhysLayer layer, PhysLayerMask collideMask) {
    b2Filter filter = b2DefaultFilter();
    const uint64_t category =
        (layer < 64u) ? (1ull << static_cast<uint64_t>(layer)) : 1ull;
    filter.categoryBits = category;
    filter.maskBits     = static_cast<uint64_t>(collideMask);
    return filter;
}

inline b2Vec2 toB2Vec2(const ayt::math::FVector3& v) {
    return b2Vec2{v.x, v.y};
}

inline b2Vec2 toB2Vec2(const ayt::math::FVector2& v) {
    return b2Vec2{v.x, v.y};
}

BodyHandle resolveBodyHandle(b2BodyId id,
                             const std::vector<b2BodyId>& bodyIdByIndex,
                             const std::vector<uint16_t>& bodyGeneration,
                             uint32_t maxBodies) {
    if (!b2Body_IsValid(id)) return InvalidBodyHandle;
    const BodyHandle h = userDataToHandle(b2Body_GetUserData(id));
    if (h == InvalidBodyHandle) return InvalidBodyHandle;
    const uint32_t idx = handleIndex(h);
    if (idx == 0u || idx >= maxBodies) return InvalidBodyHandle;
    if (bodyGeneration[idx] != handleGeneration(h)) return InvalidBodyHandle;
    if (!B2_ID_EQUALS(bodyIdByIndex[idx], id)) return InvalidBodyHandle;
    return h;
}

}  // namespace

// =============================================================================
// Impl (Pimpl)
// =============================================================================
struct Box2DBackend2D::Impl {
    b2WorldId worldId = B2_ZERO_INIT;

    std::vector<b2BodyId>  bodyIdByIndex;
    std::vector<uint16_t>  bodyGeneration;
    std::vector<uint16_t>  colliderGeneration;
    std::vector<b2ShapeId> shapeIdByIndex;
    // R10: creation-time mass override (RigidbodyDesc.mass). Box2D computes
    // body mass from shape density at collider attach, so the override is
    // applied there via GetMassData->SetMassData (keeps the shape-computed
    // inertia, swaps mass). 0 = no override.
    std::vector<float>     bodyMassOverride;
    std::vector<uint16_t>  jointGeneration;
    std::vector<b2JointId> jointIdByIndex;

    // Per-body collision filter (A5): layer + collideMask from RigidbodyDesc.
    std::vector<PhysLayer>     bodyLayer;
    std::vector<PhysLayerMask> bodyCollideMask;

    // Dense live-body list keeps snapshot/WakeAll/SleepAll proportional to
    // actual bodies instead of the configured handle-space capacity.
    std::vector<uint32_t> liveBodyIndices;
    std::vector<uint32_t> liveBodyPosition;
    std::vector<uint8_t>  alwaysSyncByIndex;

    mutable std::mutex           asyncQueryMu;
    std::vector<QueryResult>       asyncQueryQueue;

    mutable std::mutex           collisionEventMu;
    std::vector<CollisionEvent>    collisionEventQueue;

    uint32_t maxBodies = 0;
};

Box2DBackend2D::Box2DBackend2D() = default;
Box2DBackend2D::~Box2DBackend2D() { stop(); }

bool Box2DBackend2D::init2D(const PhysicsBackendDescriptor& desc) {
    if (_initialized) return false;  // reject double-init (B15)
    if (!_impl) _impl = std::make_unique<Impl>();

    const uint32_t maxBodies = desc.maxBodies > 0u ? desc.maxBodies : 1024u;
    _impl->maxBodies = maxBodies;
    _maxSubSteps     = desc.maxSubSteps > 0 ? desc.maxSubSteps : 4;

    _impl->bodyIdByIndex.assign(maxBodies, B2_ZERO_INIT);
    _impl->bodyGeneration.assign(maxBodies, 0u);
    _impl->colliderGeneration.assign(maxBodies, 0u);
    _impl->shapeIdByIndex.assign(maxBodies, B2_ZERO_INIT);
    _impl->bodyMassOverride.assign(maxBodies, 0.0f);
    _impl->jointGeneration.assign(maxBodies, 0u);
    _impl->jointIdByIndex.assign(maxBodies, B2_ZERO_INIT);
    _impl->bodyLayer.assign(maxBodies, 0u);
    _impl->bodyCollideMask.assign(maxBodies, 0xFFFFFFFFu);
    _impl->liveBodyIndices.clear();
    _impl->liveBodyIndices.reserve(std::min<uint32_t>(maxBodies, 1024u));
    _impl->liveBodyPosition.assign(maxBodies, std::numeric_limits<uint32_t>::max());
    _impl->alwaysSyncByIndex.assign(maxBodies, 0u);

    b2WorldDef worldDef = b2DefaultWorldDef();
    worldDef.gravity    = toB2Vec2(_gravity);
    _impl->worldId      = b2CreateWorld(&worldDef);
    if (!b2World_IsValid(_impl->worldId)) {
        _impl.reset();
        return false;
    }

    _initialized = true;
    return true;
}

void Box2DBackend2D::setGravity(const ayt::math::FVector2& g) {
    _gravity = g;
    if (_impl && b2World_IsValid(_impl->worldId)) {
        b2World_SetGravity(_impl->worldId, toB2Vec2(g));
    }
}

bool Box2DBackend2D::start(const PhysicsBackendInfo& /*info*/) {
    if (!_initialized || !_impl || !b2World_IsValid(_impl->worldId)) return false;
    _running = true;
    return true;
}

void Box2DBackend2D::stop() {
    if (!_running && !_initialized) return;
    _running = false;
    if (_impl && b2World_IsValid(_impl->worldId)) {
        b2DestroyWorld(_impl->worldId);
        _impl->worldId = B2_ZERO_INIT;
    }
    _initialized = false;
}

void Box2DBackend2D::step(float deltaTime) {
    ++_stepCount;

    // A6: lockstep gate — mirrors JoltBackend3D::step (§10.2).
    if (isLockstepActive() || _lockstepForcedActive) {
        ++_lockstepRefusedCount;
        return;
    }

    if (!_running || !_impl || !b2World_IsValid(_impl->worldId)) return;

    float dt = deltaTime;
    if (dt <= 0.0f)         dt = 1.0f / 60.0f;
    if (dt > 1.0f / 30.0f)  dt = 1.0f / 30.0f;
    if (dt < 1.0f / 240.0f) dt = 1.0f / 240.0f;

    b2World_Step(_impl->worldId, dt, _maxSubSteps);

    // A1+A2: drain contact + sensor events into the collision event queue.
    // Box2D event arrays are transient — only valid until the next step, and
    // may reference shapes destroyed since the last step (async destroy +
    // pending step in the same worker iteration). Validate before deref —
    // box2d's own sample pattern (b2Shape_IsValid precedes b2Shape_GetBody);
    // an invalid id yields InvalidBodyHandle and drops the event.
    auto bodyOf = [this](b2ShapeId sid) -> BodyHandle {
        if (!b2Shape_IsValid(sid)) return InvalidBodyHandle;
        return resolveBodyHandle(b2Shape_GetBody(sid),
            _impl->bodyIdByIndex, _impl->bodyGeneration, _impl->maxBodies);
    };
    {
        b2ContactEvents ce = b2World_GetContactEvents(_impl->worldId);
        std::lock_guard<std::mutex> lk(_impl->collisionEventMu);
        for (int i = 0; i < ce.beginCount; ++i) {
            const b2ContactBeginTouchEvent& e = ce.beginEvents[i];
            CollisionEvent ev{};
            ev.kind  = 0;  // enter
            ev.bodyA  = bodyOf(e.shapeIdA);
            ev.bodyB  = bodyOf(e.shapeIdB);
            ev.normal = ayt::math::FVector3(
                e.manifold.normal.x, e.manifold.normal.y, 0.0f);
            _impl->collisionEventQueue.push_back(ev);
            ++_collisionEventCount;
        }
        for (int i = 0; i < ce.endCount; ++i) {
            const b2ContactEndTouchEvent& e = ce.endEvents[i];
            CollisionEvent ev{};
            ev.kind  = 2;  // exit
            ev.bodyA  = bodyOf(e.shapeIdA);
            ev.bodyB  = bodyOf(e.shapeIdB);
            _impl->collisionEventQueue.push_back(ev);
            ++_collisionEventCount;
        }
    }
    {
        b2SensorEvents se = b2World_GetSensorEvents(_impl->worldId);
        std::lock_guard<std::mutex> lk(_impl->collisionEventMu);
        for (int i = 0; i < se.beginCount; ++i) {
            const b2SensorBeginTouchEvent& e = se.beginEvents[i];
            CollisionEvent ev{};
            ev.kind  = 3;  // sensor enter
            ev.bodyA  = bodyOf(e.sensorShapeId);
            ev.bodyB  = bodyOf(e.visitorShapeId);
            _impl->collisionEventQueue.push_back(ev);
            ++_collisionEventCount;
        }
        for (int i = 0; i < se.endCount; ++i) {
            const b2SensorEndTouchEvent& e = se.endEvents[i];
            CollisionEvent ev{};
            ev.kind  = 4;  // sensor exit
            ev.bodyA  = bodyOf(e.sensorShapeId);
            ev.bodyB  = bodyOf(e.visitorShapeId);
            _impl->collisionEventQueue.push_back(ev);
            ++_collisionEventCount;
        }
    }
}

void Box2DBackend2D::publishSnapshot(PhysFrameSnapshot& outSnapshot) {
    if (!_impl || !b2World_IsValid(_impl->worldId)) return;

    const size_t baseTransformCount = outSnapshot.transforms.size();
    outSnapshot.transforms.reserve(baseTransformCount + _impl->liveBodyIndices.size());
    _lastSnapshotBodyVisitCount = 0;

    auto emitBody = [&](uint32_t idx) {
        if (idx == 0u || idx >= _impl->maxBodies) return;
        if (_impl->bodyGeneration[idx] == 0u) return;
        const b2BodyId id = _impl->bodyIdByIndex[idx];
        if (!b2Body_IsValid(id)) return;

        const bool awake = b2Body_IsAwake(id);
        const bool always = _impl->alwaysSyncByIndex[idx] != 0u;
        if (!awake && !always) return;

        const BodyHandle h = makeHandle(idx, _impl->bodyGeneration[idx]);
        const b2Vec2 pos = b2Body_GetPosition(id);
        const b2Rot  rot = b2Body_GetRotation(id);
        const b2Vec2 lv  = b2Body_GetLinearVelocity(id);
        const float  av  = b2Body_GetAngularVelocity(id);

        BodyTransform bt{};
        bt.body     = h;
        bt.position = ayt::math::FVector3(pos.x, pos.y, 0.0f);
        bt.rotation = angleZToQuat(b2Rot_GetAngle(rot));
        bt.linearVelocity =
            ayt::math::FVector3(lv.x, lv.y, 0.0f);
        bt.angularVelocity = ayt::math::FVector3(0.0f, 0.0f, av);
        bt.dimension       = PhysicsDimension::TwoD;
        bt.flags           = awake ? 0u : 1u;
        outSnapshot.transforms.push_back(bt);
    };

    for (uint32_t idx : _impl->liveBodyIndices) {
        ++_lastSnapshotBodyVisitCount;
        emitBody(idx);
    }

    // Drain collision/sensor events.
    {
        std::lock_guard<std::mutex> lk(_impl->collisionEventMu);
        outSnapshot.collisionEvents.insert(outSnapshot.collisionEvents.end(),
            _impl->collisionEventQueue.begin(), _impl->collisionEventQueue.end());
        _impl->collisionEventQueue.clear();
    }

    {
        std::lock_guard<std::mutex> lk(_impl->asyncQueryMu);
        outSnapshot.queryResults.insert(outSnapshot.queryResults.end(),
            _impl->asyncQueryQueue.begin(), _impl->asyncQueryQueue.end());
        _impl->asyncQueryQueue.clear();
    }
}

void Box2DBackend2D::execute(const PhysicsCommand& cmd,
                             const PhysicsCreatePayload* createPayload) {
    ++_commandCount;
    if (!_impl || !b2World_IsValid(_impl->worldId)) return;

    using CT = PhysicsCommandType;

    switch (cmd.type) {
    case CT::Step:
        return;

    case CT::CreateRigidbody: {
        if (!createPayload) { ++_notFoundCount; return; }
        const RigidbodyDesc& d = createPayload->rigidDesc;
        const uint32_t idx = handleIndex(cmd.body);
        if (idx == 0u || idx >= _impl->maxBodies) { ++_notFoundCount; return; }
        if (_impl->bodyGeneration[idx] != 0u) { ++_notFoundCount; return; }

        b2BodyDef bodyDef = b2DefaultBodyDef();
        bodyDef.type      = toB2BodyType(d.type);
        bodyDef.position  = b2Vec2{d.position.x, d.position.y};
        bodyDef.rotation  = b2MakeRot(quatToAngleZ(d.rotation));
        bodyDef.linearVelocity =
            b2Vec2{d.linearVelocity.x, d.linearVelocity.y};
        bodyDef.angularVelocity   = d.angularVelocity.z;
        bodyDef.linearDamping     = d.linearDamping;
        bodyDef.angularDamping    = d.angularDamping;
        bodyDef.gravityScale      = d.gravityScale;  // R6: creation-time gravity multiplier
        bodyDef.isAwake           = true;
        bodyDef.enableSleep       = true;
        bodyDef.isBullet          = d.enableCCD;  // A4: CCD wiring
        bodyDef.fixedRotation     = d.fixedRotation;  // R2: lock angular DOF
        bodyDef.userData          = handleToUserData(cmd.body);

        const b2BodyId bodyId = b2CreateBody(_impl->worldId, &bodyDef);
        if (!b2Body_IsValid(bodyId)) { ++_notFoundCount; return; }

        _impl->bodyIdByIndex[idx]   = bodyId;
        _impl->bodyGeneration[idx]  = static_cast<uint16_t>(handleGeneration(cmd.body));
        // A5: stash per-body filter so CreateCollider can build the right b2Filter.
        _impl->bodyLayer[idx]        = d.layer;
        _impl->bodyCollideMask[idx]  = d.collideMask;
        _impl->liveBodyPosition[idx] =
            static_cast<uint32_t>(_impl->liveBodyIndices.size());
        _impl->liveBodyIndices.push_back(idx);
        _impl->alwaysSyncByIndex[idx] = d.alwaysSync ? 1u : 0u;
        // R10: creation-time mass override — applied at first collider attach
        // (see CreateCollider). Dynamic only: b2Body_SetMassData mutates
        // mass/inertia on any body type, which corrupts kinematic bodies
        // (Box2D owns their mass state; overriding it breaks sensors).
        _impl->bodyMassOverride[idx] =
            d.type == BodyType::Dynamic ? d.mass : 0.0f;
        return;
    }

    case CT::DestroyRigidbody: {
        const uint32_t idx = handleIndex(cmd.body);
        if (idx == 0u || idx >= _impl->maxBodies) { ++_notFoundCount; return; }
        if (_impl->bodyGeneration[idx] != handleGeneration(cmd.body)) {
            ++_notFoundCount;
            return;
        }
        const b2BodyId id = _impl->bodyIdByIndex[idx];
        if (b2Body_IsValid(id)) {
            b2DestroyBody(id);
        }
        _impl->bodyGeneration[idx] = 0u;
        _impl->bodyIdByIndex[idx]  = B2_ZERO_INIT;
        _impl->bodyLayer[idx]       = 0u;
        _impl->bodyCollideMask[idx] = 0xFFFFFFFFu;
        _impl->alwaysSyncByIndex[idx] = 0u;

        const uint32_t livePosition = _impl->liveBodyPosition[idx];
        if (livePosition < _impl->liveBodyIndices.size()) {
            const uint32_t movedIndex = _impl->liveBodyIndices.back();
            _impl->liveBodyIndices[livePosition] = movedIndex;
            _impl->liveBodyPosition[movedIndex] = livePosition;
            _impl->liveBodyIndices.pop_back();
        }
        _impl->liveBodyPosition[idx] = std::numeric_limits<uint32_t>::max();
        return;
    }

    case CT::SetRigidbodyTransform: {
        const uint32_t idx = handleIndex(cmd.body);
        if (idx == 0u || idx >= _impl->maxBodies) { ++_notFoundCount; return; }
        if (_impl->bodyGeneration[idx] != handleGeneration(cmd.body)) {
            ++_notFoundCount;
            return;
        }
        const b2BodyId id = _impl->bodyIdByIndex[idx];
        if (!b2Body_IsValid(id)) return;
        b2Body_SetTransform(id,
            b2Vec2{cmd.u.xform.px, cmd.u.xform.py},
            b2MakeRot(quatToAngleZ(ayt::math::FQuaternion(
                cmd.u.xform.qx, cmd.u.xform.qy, cmd.u.xform.qz, cmd.u.xform.qw))));
        return;
    }

    case CT::SetRigidbodyCollideMask: {
        // P3: update the per-body collide mask and re-apply it to every shape
        // currently attached to the body (so existing colliders pick up the new
        // filter immediately, and any future collider inherits it).
        const uint32_t idx = handleIndex(cmd.body);
        if (idx == 0u || idx >= _impl->maxBodies) { ++_notFoundCount; return; }
        if (_impl->bodyGeneration[idx] != handleGeneration(cmd.body)) {
            ++_notFoundCount;
            return;
        }
        _impl->bodyCollideMask[idx] = cmd.layerMask;
        const b2Filter filter =
            makeB2Filter(_impl->bodyLayer[idx], _impl->bodyCollideMask[idx]);
        const b2BodyId bodyId = _impl->bodyIdByIndex[idx];
        if (!b2Body_IsValid(bodyId)) return;
        for (uint32_t sIdx = 1u; sIdx < _impl->maxBodies; ++sIdx) {
            const b2ShapeId shapeId = _impl->shapeIdByIndex[sIdx];
            if (!b2Shape_IsValid(shapeId)) continue;
            if (!B2_ID_EQUALS(b2Shape_GetBody(shapeId), bodyId)) continue;
            b2Shape_SetFilter(shapeId, filter);
        }
        return;
    }

    case CT::SetRigidbodyVelocity: {
        // R2: set linear velocity directly. Wakes the body so a sleeping body
        // actually starts moving this step.
        const uint32_t idx = handleIndex(cmd.body);
        if (idx == 0u || idx >= _impl->maxBodies) { ++_notFoundCount; return; }
        if (_impl->bodyGeneration[idx] != handleGeneration(cmd.body)) {
            ++_notFoundCount;
            return;
        }
        const b2BodyId bodyId = _impl->bodyIdByIndex[idx];
        if (!b2Body_IsValid(bodyId)) { ++_notFoundCount; return; }
        b2Body_SetLinearVelocity(bodyId, b2Vec2{cmd.u.vec4.x, cmd.u.vec4.y});
        b2Body_SetAwake(bodyId, true);
        return;
    }

    case CT::SetGravityScale: {
        // R6: per-body gravity multiplier (0 = float, 1 = world default).
        const uint32_t idx = handleIndex(cmd.body);
        if (idx == 0u || idx >= _impl->maxBodies) { ++_notFoundCount; return; }
        if (_impl->bodyGeneration[idx] != handleGeneration(cmd.body)) {
            ++_notFoundCount;
            return;
        }
        const b2BodyId bodyId = _impl->bodyIdByIndex[idx];
        if (!b2Body_IsValid(bodyId)) { ++_notFoundCount; return; }
        b2Body_SetGravityScale(bodyId, cmd.u.vec4.x);
        b2Body_SetAwake(bodyId, true);
        return;
    }

    case CT::ApplyTorque: {
        // R6: Z-axis torque (2D spin).
        const uint32_t idx = handleIndex(cmd.body);
        if (idx == 0u || idx >= _impl->maxBodies) { ++_notFoundCount; return; }
        if (_impl->bodyGeneration[idx] != handleGeneration(cmd.body)) {
            ++_notFoundCount;
            return;
        }
        const b2BodyId id = _impl->bodyIdByIndex[idx];
        if (!b2Body_IsValid(id)) return;
        b2Body_ApplyTorque(id, cmd.u.vec4.z, true);
        return;
    }

    case CT::ApplyAngularImpulse: {
        // R6: Z-axis angular impulse (2D spin kick).
        const uint32_t idx = handleIndex(cmd.body);
        if (idx == 0u || idx >= _impl->maxBodies) { ++_notFoundCount; return; }
        if (_impl->bodyGeneration[idx] != handleGeneration(cmd.body)) {
            ++_notFoundCount;
            return;
        }
        const b2BodyId id = _impl->bodyIdByIndex[idx];
        if (!b2Body_IsValid(id)) return;
        b2Body_ApplyAngularImpulse(id, cmd.u.vec4.z, true);
        return;
    }

    case CT::ApplyForce: {
        const uint32_t idx = handleIndex(cmd.body);
        if (idx == 0u || idx >= _impl->maxBodies) { ++_notFoundCount; return; }
        if (_impl->bodyGeneration[idx] != handleGeneration(cmd.body)) {
            ++_notFoundCount;
            return;
        }
        const b2BodyId id = _impl->bodyIdByIndex[idx];
        if (!b2Body_IsValid(id)) return;
        b2Body_ApplyForceToCenter(id, b2Vec2{cmd.u.vec4.x, cmd.u.vec4.y}, true);
        return;
    }

    case CT::SetMass: {
        // R10: runtime mass override. Keeps the computed center + rotational
        // inertia, swaps only the mass; wakes the body so the change applies
        // immediately.
        const uint32_t idx = handleIndex(cmd.body);
        if (idx == 0u || idx >= _impl->maxBodies) { ++_notFoundCount; return; }
        if (_impl->bodyGeneration[idx] != handleGeneration(cmd.body)) {
            ++_notFoundCount;
            return;
        }
        if (!(cmd.u.vec4.x > 0.0f)) { ++_notFoundCount; return; }
        const b2BodyId bodyId = _impl->bodyIdByIndex[idx];
        if (!b2Body_IsValid(bodyId)) { ++_notFoundCount; return; }
        // b2Body_SetMassData writes mass/inertia on any body type; kinematic
        // and static bodies have no meaningful mass (Box2D owns it) — dynamic
        // only, matching the creation-path override.
        if (b2Body_GetType(bodyId) != b2_dynamicBody) { ++_notFoundCount; return; }
        b2MassData md = b2Body_GetMassData(bodyId);
        md.mass = cmd.u.vec4.x;
        b2Body_SetMassData(bodyId, md);
        b2Body_SetAwake(bodyId, true);
        return;
    }

    case CT::SetMaterial: {
        // R10: runtime per-collider friction / restitution (Box2D keeps these
        // on the shape).
        const uint32_t idx = handleIndex(cmd.collider);
        if (idx == 0u || idx >= _impl->maxBodies) { ++_notFoundCount; return; }
        if (_impl->colliderGeneration[idx] != handleGeneration(cmd.collider)) {
            ++_notFoundCount;
            return;
        }
        const b2ShapeId shapeId = _impl->shapeIdByIndex[idx];
        if (!b2Shape_IsValid(shapeId)) { ++_notFoundCount; return; }
        b2Shape_SetFriction(shapeId, cmd.u.vec4.x);
        b2Shape_SetRestitution(shapeId, cmd.u.vec4.y);
        return;
    }

    case CT::ApplyImpulse: {
        const uint32_t idx = handleIndex(cmd.body);
        if (idx == 0u || idx >= _impl->maxBodies) { ++_notFoundCount; return; }
        if (_impl->bodyGeneration[idx] != handleGeneration(cmd.body)) {
            ++_notFoundCount;
            return;
        }
        const b2BodyId id = _impl->bodyIdByIndex[idx];
        if (!b2Body_IsValid(id)) return;
        b2Body_ApplyLinearImpulseToCenter(id, b2Vec2{cmd.u.vec4.x, cmd.u.vec4.y}, true);
        return;
    }

    case CT::CreateCollider: {
        if (!createPayload) { ++_notFoundCount; return; }
        const ColliderDesc& d = createPayload->colliderDesc;
        const uint32_t idx = handleIndex(cmd.collider);
        if (idx == 0u || idx >= _impl->maxBodies) { ++_notFoundCount; return; }
        if (_impl->colliderGeneration[idx] != 0u) { ++_notFoundCount; return; }

        const uint32_t bodyIdx = handleIndex(d.body);
        if (bodyIdx == 0u || bodyIdx >= _impl->maxBodies) { ++_notFoundCount; return; }
        if (_impl->bodyGeneration[bodyIdx] != handleGeneration(d.body)) {
            ++_notFoundCount;
            return;
        }
        const b2BodyId bodyId = _impl->bodyIdByIndex[bodyIdx];
        if (!b2Body_IsValid(bodyId)) { ++_notFoundCount; return; }

        b2ShapeDef shapeDef = b2DefaultShapeDef();
        shapeDef.density    = d.material.density > 0.0f ? d.material.density : 1.0f;
        shapeDef.material.friction    = d.material.friction;
        shapeDef.material.restitution = d.material.restitution;
        // A5: pull the owning body's layer/mask so PhysDefaultLayer works in 2D.
        shapeDef.filter               = makeB2Filter(_impl->bodyLayer[bodyIdx],
                                                     _impl->bodyCollideMask[bodyIdx]);
        shapeDef.isSensor               = d.isTrigger;
        shapeDef.enableContactEvents    = true;
        shapeDef.enableSensorEvents      = true;  // A2: surface sensor events on all shapes

        // R10: local-space shape offset (x/y; z ignored in 2D).
        const b2Vec2 offset2D{d.offset.x, d.offset.y};
        b2ShapeId shapeId = B2_ZERO_INIT;
        switch (d.shape) {
        case ColliderShape::Box: {
            const b2Polygon poly =
                b2MakeOffsetBox(d.halfExtents.x, d.halfExtents.y, offset2D, b2Rot{1.0f, 0.0f});
            shapeId = b2CreatePolygonShape(bodyId, &shapeDef, &poly);
            break;
        }
        case ColliderShape::Sphere: {
            b2Circle circle{};
            circle.center = offset2D;
            circle.radius = d.radius;
            shapeId       = b2CreateCircleShape(bodyId, &shapeDef, &circle);
            break;
        }
        case ColliderShape::Capsule: {
            const float halfHeight = std::max(0.0f, d.height * 0.5f - d.radius);
            b2Capsule capsule{};
            capsule.center1 = b2Add(offset2D, b2Vec2{0.0f, -halfHeight});
            capsule.center2 = b2Add(offset2D, b2Vec2{0.0f, halfHeight});
            capsule.radius  = d.radius;
            shapeId         = b2CreateCapsuleShape(bodyId, &shapeDef, &capsule);
            break;
        }
        case ColliderShape::ConvexHull: {
            // R5: 2D convex polygon from a point cloud. Project hullPoints
            // (FVector3) onto the XY plane, compute the convex hull, and
            // feed it to b2CreatePolygonShape. Box2D caps the hull at
            // B2_MAX_POLYGON_VERTICES (8); b2ComputeHull welds close/collinear
            // points but returns count < 3 on failure (too few points, all
            // collinear, or the reduced hull would exceed 8 vertices).
            // TriangleMesh / Heightfield are 3D-only and stay rejected here.
            if (!d.shapeData || d.shapeData->hullPoints.size() < 3u) {
                ++_notFoundCount;
                return;
            }
            const auto& src = d.shapeData->hullPoints;
            std::vector<b2Vec2> pts(src.size());
            for (size_t i = 0; i < src.size(); ++i) {
                pts[i] = b2Vec2{src[i].x, src[i].y};
            }
            const b2Hull hull =
                b2ComputeHull(pts.data(), static_cast<int>(pts.size()));
            if (hull.count < 3) {
                ++_notFoundCount;
                return;
            }
            const b2Polygon poly =
                b2MakeOffsetPolygon(&hull, offset2D, b2Rot{1.0f, 0.0f});
            shapeId = b2CreatePolygonShape(bodyId, &shapeDef, &poly);
            break;
        }
        default:
            ++_notFoundCount;
            return;
        }

        if (!b2Shape_IsValid(shapeId)) {
            ++_notFoundCount;
            return;
        }
        _impl->shapeIdByIndex[idx]      = shapeId;
        _impl->colliderGeneration[idx]  = static_cast<uint16_t>(handleGeneration(cmd.collider));

        // R10: apply the body's creation-time mass override now that the shape
        // has computed density-based mass/inertia. Box2D's SetMassData swaps
        // mass and keeps the given inertia — read the shape-derived inertia
        // back and re-apply it under the override mass.
        if (_impl->bodyMassOverride[bodyIdx] > 0.0f) {
            b2MassData md = b2Body_GetMassData(bodyId);
            md.mass = _impl->bodyMassOverride[bodyIdx];
            b2Body_SetMassData(bodyId, md);
        }
        return;
    }

    case CT::DestroyCollider: {
        const uint32_t idx = handleIndex(cmd.collider);
        if (idx == 0u || idx >= _impl->maxBodies) { ++_notFoundCount; return; }
        if (_impl->colliderGeneration[idx] != handleGeneration(cmd.collider)) {
            ++_notFoundCount;
            return;
        }
        const b2ShapeId shapeId = _impl->shapeIdByIndex[idx];
        if (b2Shape_IsValid(shapeId)) {
            b2DestroyShape(shapeId, true);
        }
        _impl->colliderGeneration[idx] = 0u;
        _impl->shapeIdByIndex[idx]       = B2_ZERO_INIT;
        return;
    }

    case CT::CreateJoint: {
        if (!createPayload) { ++_notFoundCount; return; }
        const JointDesc& d = createPayload->jointDesc;
        const uint32_t idx = handleIndex(cmd.joint);
        if (idx == 0u || idx >= _impl->maxBodies) { ++_notFoundCount; return; }
        if (_impl->jointGeneration[idx] != 0u) { ++_notFoundCount; return; }

        const uint32_t aIdx = handleIndex(d.bodyA);
        const uint32_t bIdx = handleIndex(d.bodyB);
        if (aIdx == 0u || bIdx == 0u || aIdx >= _impl->maxBodies || bIdx >= _impl->maxBodies) {
            ++_notFoundCount;
            return;
        }
        if (_impl->bodyGeneration[aIdx] == 0u || _impl->bodyGeneration[bIdx] == 0u) {
            ++_notFoundCount;
            return;
        }
        const b2BodyId bodyA = _impl->bodyIdByIndex[aIdx];
        const b2BodyId bodyB = _impl->bodyIdByIndex[bIdx];
        if (!b2Body_IsValid(bodyA) || !b2Body_IsValid(bodyB)) {
            ++_notFoundCount;
            return;
        }

        const b2Vec2 worldA{d.anchorA.x, d.anchorA.y};
        const b2Vec2 worldB{d.anchorB.x, d.anchorB.y};

        b2JointId jointId = B2_ZERO_INIT;
        switch (d.type) {
        case JointType::Fixed: {
            b2WeldJointDef jd = b2DefaultWeldJointDef();
            jd.bodyIdA        = bodyA;
            jd.bodyIdB        = bodyB;
            jd.localAnchorA   = b2Body_GetLocalPoint(bodyA, worldA);
            jd.localAnchorB   = b2Body_GetLocalPoint(bodyB, worldB);
            jd.referenceAngle = b2Rot_GetAngle(b2Body_GetRotation(bodyB)) -
                                b2Rot_GetAngle(b2Body_GetRotation(bodyA));
            // A7: stiffness→angularHertz (0 = max stiffness = rigid weld).
            if (d.stiffness > 0.0f) {
                jd.angularHertz          = d.stiffness;
                jd.angularDampingRatio   = d.damping > 0.0f ? d.damping : 1.0f;
            }
            jointId           = b2CreateWeldJoint(_impl->worldId, &jd);
            break;
        }
        case JointType::Hinge: {
            b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
            jd.bodyIdA            = bodyA;
            jd.bodyIdB            = bodyB;
            jd.localAnchorA       = b2Body_GetLocalPoint(bodyA, worldA);
            jd.localAnchorB       = b2Body_GetLocalPoint(bodyB, worldB);
            // A7: spring stiffness/damping on the hinge axis.
            if (d.stiffness > 0.0f) {
                jd.enableSpring  = true;
                jd.hertz         = d.stiffness;
                jd.dampingRatio  = d.damping > 0.0f ? d.damping : 1.0f;
            }
            jointId               = b2CreateRevoluteJoint(_impl->worldId, &jd);
            break;
        }
        case JointType::Distance: {
            b2DistanceJointDef jd = b2DefaultDistanceJointDef();
            jd.bodyIdA            = bodyA;
            jd.bodyIdB            = bodyB;
            jd.localAnchorA       = b2Body_GetLocalPoint(bodyA, worldA);
            jd.localAnchorB       = b2Body_GetLocalPoint(bodyB, worldB);
            const b2Vec2 delta    = b2Sub(worldB, worldA);
            jd.length             = b2Length(delta);
            if (d.maxDistance > 0.0f) jd.length = d.maxDistance;
            // A7: limits + spring.
            if (d.minDistance > 0.0f || d.maxDistance > 0.0f) {
                jd.enableLimit = true;
                jd.minLength   = d.minDistance > 0.0f ? d.minDistance : 0.0f;
                jd.maxLength   = d.maxDistance > 0.0f ? d.maxDistance : jd.length;
            }
            if (d.stiffness > 0.0f) {
                jd.enableSpring  = true;
                jd.hertz         = d.stiffness;
                jd.dampingRatio  = d.damping > 0.0f ? d.damping : 1.0f;
            }
            jointId               = b2CreateDistanceJoint(_impl->worldId, &jd);
            break;
        }
        case JointType::Spring: {
            // A8: Spring = Distance joint with spring always enabled.
            b2DistanceJointDef jd = b2DefaultDistanceJointDef();
            jd.bodyIdA            = bodyA;
            jd.bodyIdB            = bodyB;
            jd.localAnchorA       = b2Body_GetLocalPoint(bodyA, worldA);
            jd.localAnchorB       = b2Body_GetLocalPoint(bodyB, worldB);
            const b2Vec2 delta     = b2Sub(worldB, worldA);
            jd.length              = b2Length(delta);
            if (d.maxDistance > 0.0f) jd.length = d.maxDistance;
            jd.enableSpring  = true;
            jd.hertz         = d.stiffness > 0.0f ? d.stiffness : 4.0f;
            jd.dampingRatio  = d.damping > 0.0f ? d.damping : 0.5f;
            jointId               = b2CreateDistanceJoint(_impl->worldId, &jd);
            break;
        }
        case JointType::Slider: {
            b2PrismaticJointDef jd = b2DefaultPrismaticJointDef();
            jd.bodyIdA             = bodyA;
            jd.bodyIdB             = bodyB;
            jd.localAnchorA        = b2Body_GetLocalPoint(bodyA, worldA);
            jd.localAnchorB        = b2Body_GetLocalPoint(bodyB, worldB);
            jd.localAxisA          = b2Body_GetLocalVector(bodyA, b2Vec2{d.axisA.x, d.axisA.y});
            // A7: spring on the translation axis.
            if (d.stiffness > 0.0f) {
                jd.enableSpring = true;
                jd.hertz        = d.stiffness;
                jd.dampingRatio = d.damping > 0.0f ? d.damping : 1.0f;
            }
            jointId                = b2CreatePrismaticJoint(_impl->worldId, &jd);
            break;
        }
        case JointType::Point: {
            // R5: 2D Point = tether joint (design.md §14.3.3). A motor joint
            // drives bodyB toward a target offset relative to bodyA. With
            // bodyA static, this pulls bodyB to the world-space target
            // (anchorB) on every step. (The mouse joint was considered but
            // rejects this use: it pins bodyB's anchor AT the target at
            // creation, so it holds bodyB at its start position and only
            // drags when the target is moved each frame via SetTarget.)
            // Cone is a 3D-only concept and stays rejected here.
            b2MotorJointDef jd = b2DefaultMotorJointDef();
            jd.bodyIdA      = bodyA;
            jd.bodyIdB      = bodyB;
            // Target = anchorB expressed in bodyA's local frame.
            jd.linearOffset = b2Body_GetLocalPoint(bodyA, worldB);
            // Preserve the current relative angle so the joint doesn't spin B.
            jd.angularOffset = b2Rot_GetAngle(b2Body_GetRotation(bodyB)) -
                               b2Rot_GetAngle(b2Body_GetRotation(bodyA));
            jd.maxTorque = 0.0f;  // do not constrain rotation
            const float massB = b2Body_GetMass(bodyB);
            jd.maxForce = (massB > 0.0f) ? (1000.0f * massB) : 1.0e4f;
            // stiffness -> correctionFactor (higher = snappier); clamp to [0,1].
            jd.correctionFactor = d.stiffness > 0.0f
                                  ? (d.stiffness < 1.0f ? d.stiffness : 1.0f)
                                  : 0.5f;
            jointId = b2CreateMotorJoint(_impl->worldId, &jd);
            break;
        }
        default:
            ++_notFoundCount;
            return;
        }

        if (!b2Joint_IsValid(jointId)) { ++_notFoundCount; return; }
        _impl->jointIdByIndex[idx]   = jointId;
        _impl->jointGeneration[idx] = static_cast<uint16_t>(handleGeneration(cmd.joint));
        return;
    }

    case CT::DestroyJoint: {
        const uint32_t idx = handleIndex(cmd.joint);
        if (idx == 0u || idx >= _impl->maxBodies) { ++_notFoundCount; return; }
        if (_impl->jointGeneration[idx] != handleGeneration(cmd.joint)) {
            ++_notFoundCount;
            return;
        }
        const b2JointId jointId = _impl->jointIdByIndex[idx];
        if (b2Joint_IsValid(jointId)) {
            b2DestroyJoint(jointId);
        }
        _impl->jointGeneration[idx] = 0u;
        _impl->jointIdByIndex[idx]  = B2_ZERO_INIT;
        return;
    }

    case CT::RaycastAsync: {
        b2QueryFilter filter = b2DefaultQueryFilter();
        filter.maskBits      = static_cast<uint64_t>(cmd.layerMask);
        filter.categoryBits  = ~0ull;  // P3: query as "all categories" so it hits any shape whose mask accepts >=1 category (b2ShouldQueryCollide checks shape.maskBits & query.categoryBits)

        const b2Vec2 origin{cmd.u.ray.ox, cmd.u.ray.oy};
        b2Vec2 dir{cmd.u.ray.dx, cmd.u.ray.dy};
        const float len = b2Length(dir);
        if (len > 1.0e-8f) {
            dir = b2MulSV(kRayCastMaxDistance / len, dir);
        } else {
            dir = b2Vec2{0.0f, kRayCastMaxDistance};
        }

        const b2RayResult hit =
            b2World_CastRayClosest(_impl->worldId, origin, dir, filter);

        QueryResult qr{};
        qr.queryId = cmd.queryId;
        if (hit.hit && b2Shape_IsValid(hit.shapeId)) {
            qr.hitCount = 1;
            RaycastHit& rh = qr.hits.emplace_back();
            rh.hit         = true;
            const b2BodyId bodyId = b2Shape_GetBody(hit.shapeId);
            rh.body        = resolveBodyHandle(bodyId, _impl->bodyIdByIndex,
                                               _impl->bodyGeneration, _impl->maxBodies);
            rh.point       = ayt::math::FVector3(hit.point.x, hit.point.y, 0.0f);
            rh.normal      = ayt::math::FVector3(hit.normal.x, hit.normal.y, 0.0f);
            rh.distance    = hit.fraction * kRayCastMaxDistance;
        }
        std::lock_guard<std::mutex> lk(_impl->asyncQueryMu);
        _impl->asyncQueryQueue.push_back(std::move(qr));
        return;
    }

    case CT::OverlapSphereAsync: {
        // A3: 2D overlap = AABB query around the sphere's AABB.
        b2QueryFilter filter = b2DefaultQueryFilter();
        filter.maskBits      = static_cast<uint64_t>(cmd.layerMask);
        filter.categoryBits  = ~0ull;  // P3: query as "all categories" so it hits any shape whose mask accepts >=1 category (b2ShouldQueryCollide checks shape.maskBits & query.categoryBits)
        const b2Vec2 center{cmd.u.sphere.cx, cmd.u.sphere.cy};
        const float  r       = cmd.u.sphere.radius;
        b2AABB aabb{};
        aabb.lowerBound = b2Vec2{center.x - r, center.y - r};
        aabb.upperBound = b2Vec2{center.x + r, center.y + r};

        QueryResult qr{};
        qr.queryId = cmd.queryId;
        struct Ctx { Box2DBackend2D::Impl* impl; std::vector<RaycastHit>* out; };
        Ctx ctx{_impl.get(), &qr.hits};
        auto cb = [](b2ShapeId shapeId, void* userData) -> bool {
            Ctx* c = static_cast<Ctx*>(userData);
            const b2BodyId bid = b2Shape_GetBody(shapeId);
            const BodyHandle h = resolveBodyHandle(bid, c->impl->bodyIdByIndex,
                c->impl->bodyGeneration, c->impl->maxBodies);
            if (h != InvalidBodyHandle) {
                RaycastHit& rh = c->out->emplace_back();
                rh.hit  = true;
                rh.body = h;
            }
            return true;  // continue querying
        };
        b2World_OverlapAABB(_impl->worldId, aabb, filter, cb, &ctx);
        qr.hitCount = static_cast<uint32_t>(qr.hits.size());
        std::lock_guard<std::mutex> lk(_impl->asyncQueryMu);
        _impl->asyncQueryQueue.push_back(std::move(qr));
        return;
    }

    case CT::OverlapBoxAsync: {
        // R1: 2D box overlap as a real axis-aligned rectangle (independent hx/hy),
        // not the old square proxy that reused the sphere payload.
        b2QueryFilter filter = b2DefaultQueryFilter();
        filter.maskBits      = static_cast<uint64_t>(cmd.layerMask);
        filter.categoryBits  = ~0ull;  // P3: query as "all categories" so it hits any shape whose mask accepts >=1 category (b2ShouldQueryCollide checks shape.maskBits & query.categoryBits)
        const b2Vec2 center{cmd.u.box.cx, cmd.u.box.cy};
        const float  hx = cmd.u.box.hx >= 0.0f ? cmd.u.box.hx : 0.0f;
        const float  hy = cmd.u.box.hy >= 0.0f ? cmd.u.box.hy : 0.0f;
        b2AABB aabb{};
        aabb.lowerBound = b2Vec2{center.x - hx, center.y - hy};
        aabb.upperBound = b2Vec2{center.x + hx, center.y + hy};

        QueryResult qr{};
        qr.queryId = cmd.queryId;
        struct Ctx { Box2DBackend2D::Impl* impl; std::vector<RaycastHit>* out; };
        Ctx ctx{_impl.get(), &qr.hits};
        auto cb = [](b2ShapeId shapeId, void* userData) -> bool {
            Ctx* c = static_cast<Ctx*>(userData);
            const b2BodyId bid = b2Shape_GetBody(shapeId);
            const BodyHandle h = resolveBodyHandle(bid, c->impl->bodyIdByIndex,
                c->impl->bodyGeneration, c->impl->maxBodies);
            if (h != InvalidBodyHandle) {
                RaycastHit& rh = c->out->emplace_back();
                rh.hit  = true;
                rh.body = h;
            }
            return true;
        };
        b2World_OverlapAABB(_impl->worldId, aabb, filter, cb, &ctx);
        qr.hitCount = static_cast<uint32_t>(qr.hits.size());
        std::lock_guard<std::mutex> lk(_impl->asyncQueryMu);
        _impl->asyncQueryQueue.push_back(std::move(qr));
        return;
    }

    case CT::WakeAll: {
        for (uint32_t idx : _impl->liveBodyIndices) {
            const b2BodyId id = _impl->bodyIdByIndex[idx];
            if (b2Body_IsValid(id)) b2Body_SetAwake(id, true);
        }
        return;
    }

    case CT::SleepAll: {
        for (uint32_t idx : _impl->liveBodyIndices) {
            const b2BodyId id = _impl->bodyIdByIndex[idx];
            if (b2Body_IsValid(id)) b2Body_SetAwake(id, false);
        }
        return;
    }

    case CT::SetColliderShape:
        return;
    }

    ++_notFoundCount;}

void Box2DBackend2D::executeSync(const SyncQueryRequest& request,
                                 SyncQueryResponse& outResponse) {
    outResponse.requestId = request.requestId;
    outResponse.status    = PhysResult::Ok;

    if (!_impl || !b2World_IsValid(_impl->worldId)) {
        outResponse.status = PhysResult::BackendError;
        return;
    }

    switch (request.type) {
    case SyncQueryType::Raycast: {
        b2QueryFilter filter = b2DefaultQueryFilter();
        filter.maskBits      = static_cast<uint64_t>(request.layerMask);
        filter.categoryBits  = ~0ull;  // P3: query as "all categories" (see async path)

        const b2Vec2 origin{request.ray.origin.x, request.ray.origin.y};
        b2Vec2 dir{request.ray.dir.x, request.ray.dir.y};
        const float len = b2Length(dir);
        if (len > 1.0e-8f) {
            dir = b2MulSV(kRayCastMaxDistance / len, dir);
        } else {
            dir = b2Vec2{0.0f, kRayCastMaxDistance};
        }

        const b2RayResult hit =
            b2World_CastRayClosest(_impl->worldId, origin, dir, filter);
        if (hit.hit && b2Shape_IsValid(hit.shapeId)) {
            outResponse.firstHit.hit  = true;
            const b2BodyId bodyId     = b2Shape_GetBody(hit.shapeId);
            outResponse.firstHit.body = resolveBodyHandle(bodyId, _impl->bodyIdByIndex,
                                                          _impl->bodyGeneration, _impl->maxBodies);
            outResponse.firstHit.point =
                ayt::math::FVector3(hit.point.x, hit.point.y, 0.0f);
            outResponse.firstHit.normal =
                ayt::math::FVector3(hit.normal.x, hit.normal.y, 0.0f);
            outResponse.firstHit.distance = hit.fraction * kRayCastMaxDistance;
        }
        return;
    }
    case SyncQueryType::OverlapSphere: {
        b2QueryFilter filter = b2DefaultQueryFilter();
        filter.maskBits      = static_cast<uint64_t>(request.layerMask);
        filter.categoryBits  = ~0ull;  // P3: query as "all categories" (see async path)
        const b2Vec2 center{request.sphereCenter.x, request.sphereCenter.y};
        const float  r       = request.sphereRadius;
        b2AABB aabb{};
        aabb.lowerBound = b2Vec2{center.x - r, center.y - r};
        aabb.upperBound = b2Vec2{center.x + r, center.y + r};

        struct Ctx { Box2DBackend2D::Impl* impl; std::vector<BodyHandle>* out; };
        Ctx ctx{_impl.get(), &outResponse.overlaps};
        auto cb = [](b2ShapeId shapeId, void* userData) -> bool {
            Ctx* c = static_cast<Ctx*>(userData);
            const b2BodyId bid = b2Shape_GetBody(shapeId);
            const BodyHandle h = resolveBodyHandle(bid, c->impl->bodyIdByIndex,
                c->impl->bodyGeneration, c->impl->maxBodies);
            if (h != InvalidBodyHandle) c->out->push_back(h);
            return true;
        };
        b2World_OverlapAABB(_impl->worldId, aabb, filter, cb, &ctx);
        return;
    }
    case SyncQueryType::OverlapBox: {
        // R1: real axis-aligned rectangle (independent hx/hy) from the box request
        // fields, not the old sphere-payload square proxy.
        b2QueryFilter filter = b2DefaultQueryFilter();
        filter.maskBits      = static_cast<uint64_t>(request.layerMask);
        filter.categoryBits  = ~0ull;  // P3: query as "all categories" (see async path)
        const b2Vec2 center{request.boxCenter.x, request.boxCenter.y};
        const float  hx = request.boxHalfExtents.x >= 0.0f ? request.boxHalfExtents.x : 0.0f;
        const float  hy = request.boxHalfExtents.y >= 0.0f ? request.boxHalfExtents.y : 0.0f;
        b2AABB aabb{};
        aabb.lowerBound = b2Vec2{center.x - hx, center.y - hy};
        aabb.upperBound = b2Vec2{center.x + hx, center.y + hy};

        struct Ctx { Box2DBackend2D::Impl* impl; std::vector<BodyHandle>* out; };
        Ctx ctx{_impl.get(), &outResponse.overlaps};
        auto cb = [](b2ShapeId shapeId, void* userData) -> bool {
            Ctx* c = static_cast<Ctx*>(userData);
            const b2BodyId bid = b2Shape_GetBody(shapeId);
            const BodyHandle h = resolveBodyHandle(bid, c->impl->bodyIdByIndex,
                c->impl->bodyGeneration, c->impl->maxBodies);
            if (h != InvalidBodyHandle) c->out->push_back(h);
            return true;
        };
        b2World_OverlapAABB(_impl->worldId, aabb, filter, cb, &ctx);
        return;
    }
    }
}

PhysicsBackendInfo Box2DBackend2D::describe() const {
    PhysicsBackendInfo info{};
    info.name       = "Box2D (R2.5)";
    info.realDevice = true;
    info.maxBodies  = _impl ? _impl->maxBodies : 0u;
    if (_impl && b2World_IsValid(_impl->worldId)) {
        const b2Counters c = b2World_GetCounters(_impl->worldId);
        info.maxColliders = static_cast<uint32_t>(c.shapeCount);
        info.maxJoints    = static_cast<uint32_t>(c.jointCount);
    }
    return info;
}

} // namespace ayt::physics
