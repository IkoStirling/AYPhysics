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
ColliderHandle resolveColliderHandle(b2ShapeId id,
                                     const std::vector<b2ShapeId>& shapes,
                                     const std::vector<uint16_t>& generations,
                                     uint32_t maxBodies) {
    if(!b2Shape_IsValid(id))return InvalidColliderHandle;
    const auto handle=static_cast<ColliderHandle>(
        reinterpret_cast<uintptr_t>(b2Shape_GetUserData(id)));
    const auto index=handleIndex(handle);
    if(!index || index>=maxBodies || generations[index]!=handleGeneration(handle) ||
       !B2_ID_EQUALS(shapes[index],id))return InvalidColliderHandle;
    return handle;
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
    std::vector<ayt::math::FVector3> desiredVelocity;
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

    // F-P1: Box2DBackend2D shares a single index pool across bodies, colliders,
    // and joints. Every per-collider / per-joint slot array (colliderGeneration,
    // shapeIdByIndex, jointGeneration, jointIdByIndex) is sized by `maxBodies`,
    // so the *combined* cap on (bodies + colliders + joints) is maxBodies.
    // The manager clamps info2D.maxColliders to maxBodies to honour this; see
    // PhysicsManager::create. Future work: separate index pools per §6.4.
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
    _impl->desiredVelocity.assign(maxBodies, {});
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
            if(e.manifold.pointCount) {const auto p=e.manifold.points[0].point;ev.pointA={p.x,p.y,0};ev.pointB=ev.pointA;}
            ev.kind  = 0;  // enter
            ev.bodyA  = bodyOf(e.shapeIdA);
            ev.bodyB  = bodyOf(e.shapeIdB);
            ev.normal = ayt::math::FVector3(
                e.manifold.normal.x, e.manifold.normal.y, 0.0f);
            if(ev.bodyA) ev.layerA=_impl->bodyLayer[handleIndex(ev.bodyA)];
            _impl->collisionEventQueue.push_back(ev);
            ++_collisionEventCount;
        }
        for (int i = 0; i < ce.endCount; ++i) {
            const b2ContactEndTouchEvent& e = ce.endEvents[i];
            CollisionEvent ev{};
            ev.kind  = 2;  // exit
            ev.bodyA  = bodyOf(e.shapeIdA);
            ev.bodyB  = bodyOf(e.shapeIdB);
            if(ev.bodyA) ev.layerA=_impl->bodyLayer[handleIndex(ev.bodyA)];
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
            if(ev.bodyA) ev.layerA=_impl->bodyLayer[handleIndex(ev.bodyA)];
            _impl->collisionEventQueue.push_back(ev);
            ++_collisionEventCount;
        }
        for (int i = 0; i < se.endCount; ++i) {
            const b2SensorEndTouchEvent& e = se.endEvents[i];
            CollisionEvent ev{};
            ev.kind  = 4;  // sensor exit
            ev.bodyA  = bodyOf(e.sensorShapeId);
            ev.bodyB  = bodyOf(e.visitorShapeId);
            if(ev.bodyA) ev.layerA=_impl->bodyLayer[handleIndex(ev.bodyA)];
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

void Box2DBackend2D::appendDebugSnapshot(PhysicsDebugSnapshot& out) {
    if(!_impl || !b2World_IsValid(_impl->worldId)) return;
    using V=ayt::math::FVector3;
    auto cv=[](b2Vec2 p)->V{return {p.x,p.y,0};};
    auto owner=[&](b2BodyId id){return resolveBodyHandle(id,_impl->bodyIdByIndex,_impl->bodyGeneration,_impl->maxBodies);};
    for(auto i:_impl->liveBodyIndices) {
        const auto id=_impl->bodyIdByIndex[i]; if(!b2Body_IsValid(id)) continue;
        PhysicsDebugRecord r; r.dimension=PhysicsDimension::TwoD; r.category=PhysicsDebugCategory::Motion;
        r.body=owner(id); r.layer=_impl->bodyLayer[i]; r.anchor=cv(b2Body_GetPosition(id)); r.velocity=cv(b2Body_GetLinearVelocity(id));r.desiredVelocity=_impl->desiredVelocity[i];
        r.label=b2Body_IsAwake(id)?"Velocity (awake)":"Velocity (sleeping/static)";
        r.lines.push_back({r.anchor,r.anchor+r.velocity*.25f,0xff00ff00}); out.append(r);
        if(!(out.categories&(1u<<2))) continue;
        const int capacity=std::min(b2Body_GetContactCapacity(id),1024);
        std::vector<b2ContactData> contacts(capacity);
        const int count=b2Body_GetContactData(id,contacts.data(),capacity);
        for(int n=0;n<count;++n) {
            const auto& c=contacts[n]; const auto a=owner(b2Shape_GetBody(c.shapeIdA)),b=owner(b2Shape_GetBody(c.shapeIdB));
            if(a!=r.body) continue;
            for(int p=0;p<c.manifold.pointCount;++p) {
                PhysicsDebugRecord contact; contact.dimension=PhysicsDimension::TwoD; contact.category=PhysicsDebugCategory::Contact;
                contact.body=a; contact.otherBody=b; contact.layer=r.layer; contact.label="Contact stay (solved)";
                const auto& point=c.manifold.points[p]; contact.anchor=cv(point.point);contact.hitPoint=contact.anchor;contact.normal=cv(c.manifold.normal);
                contact.impulse=point.totalNormalImpulse; contact.impulseAvailable=true;
                contact.lines.push_back({contact.anchor,contact.anchor+cv(c.manifold.normal)*.5f,0xff00ffff});
                contact.lines.push_back({contact.anchor,contact.anchor+cv(c.manifold.normal)*std::min(contact.impulse,10.f),0xffff00ff});
                out.append(std::move(contact));
            }
        }
    }
    for(uint32_t i=1;i<_impl->maxBodies;++i) {
        if(!(out.categories&1u)) break;
        const auto id=_impl->shapeIdByIndex[i]; if(!_impl->colliderGeneration[i] || !b2Shape_IsValid(id)) continue;
        const auto body=b2Shape_GetBody(id);
        PhysicsDebugRecord r; r.dimension=PhysicsDimension::TwoD; r.category=PhysicsDebugCategory::Collider;
        r.body=owner(body); r.collider=makeHandle(i,_impl->colliderGeneration[i]); r.layer=_impl->bodyLayer[handleIndex(r.body)];
        r.anchor=cv(b2Body_GetPosition(body)); r.label=b2Shape_IsSensor(id)?"Native sensor":"Native collider";
        const uint32_t color=b2Shape_IsSensor(id)?0xffffff00:0xff00c8ff;
        auto world=[&](b2Vec2 p){return cv(b2Body_GetWorldPoint(body,p));};
        auto circle=[&](b2Vec2 center,float radius) {
            const auto c=world(center);
            for(int n=0;n<32;++n) {const float a=n*6.2831853f/32,b=(n+1)*6.2831853f/32;
                r.lines.push_back({c+V{std::cos(a)*radius,std::sin(a)*radius,0},c+V{std::cos(b)*radius,std::sin(b)*radius,0},color});}
        };
        switch(b2Shape_GetType(id)) {
        case b2_circleShape: {auto c=b2Shape_GetCircle(id); circle(c.center,c.radius); break;}
        case b2_capsuleShape: {
            auto c=b2Shape_GetCapsule(id); auto axis=b2Normalize(b2Sub(c.center2,c.center1)); const b2Vec2 normal{-axis.y*c.radius,axis.x*c.radius};
            // Native capsule boundary: semicircle at each end, joined by the two tangents.
            const float angle=std::atan2(axis.y,axis.x);
            for(int end=0;end<2;++end) for(int n=0;n<16;++n) {
                const float a=angle+(end?-.5f:.5f)*3.14159265f+n*3.14159265f/16,b=a+3.14159265f/16;
                const auto c0=end?c.center2:c.center1;
                r.lines.push_back({world(b2Add(c0,{std::cos(a)*c.radius,std::sin(a)*c.radius})),world(b2Add(c0,{std::cos(b)*c.radius,std::sin(b)*c.radius})),color});
            }
            for(float sign:{-1.f,1.f}) r.lines.push_back({world(b2Add(c.center1,b2MulSV(sign,normal))),world(b2Add(c.center2,b2MulSV(sign,normal))),color});
            break;
        }
        case b2_polygonShape: {auto p=b2Shape_GetPolygon(id); for(int n=0;n<p.count;++n) r.lines.push_back({world(p.vertices[n]),world(p.vertices[(n+1)%p.count]),color}); break;}
        case b2_segmentShape: {auto p=b2Shape_GetSegment(id); r.lines.push_back({world(p.point1),world(p.point2),color}); break;}
        case b2_chainSegmentShape: {auto p=b2Shape_GetChainSegment(id).segment; r.lines.push_back({world(p.point1),world(p.point2),color}); break;}
        default: break;
        }
        out.append(std::move(r));
    }
    for(uint32_t i=1;i<_impl->maxBodies;++i) {
        if(!(out.categories&(1u<<5))) break;
        const auto id=_impl->jointIdByIndex[i]; if(!_impl->jointGeneration[i] || !b2Joint_IsValid(id)) continue;
        const auto a=b2Joint_GetBodyA(id),b=b2Joint_GetBodyB(id);
        PhysicsDebugRecord r; r.dimension=PhysicsDimension::TwoD; r.category=PhysicsDebugCategory::Joint;
        r.body=owner(a);r.layer=_impl->bodyLayer[handleIndex(r.body)]; r.otherBody=owner(b); r.queryId=makeHandle(i,_impl->jointGeneration[i]); r.label="Native joint anchors/axis";
        r.anchor=cv(b2Body_GetWorldPoint(a,b2Joint_GetLocalAnchorA(id))); const auto end=cv(b2Body_GetWorldPoint(b,b2Joint_GetLocalAnchorB(id)));
        const auto axis=cv(b2Body_GetWorldVector(a,b2Joint_GetLocalAxisA(id)));
        r.lines={{r.anchor,end,0xffffff00},{r.anchor,r.anchor+axis*.5f,0xff0000ff},{end,end+V{0,0,.3f},0xffff0000}};
        if(b2Joint_GetType(id)==b2_distanceJoint) {
            const auto delta=end-r.anchor; const auto dir=delta.lengthSq()>1e-10f?delta.normalize():V{1,0,0};
            r.lines.push_back({r.anchor+dir*b2DistanceJoint_GetMinLength(id),r.anchor+dir*b2DistanceJoint_GetMaxLength(id),0xffff00ff});
        }
        if(b2Joint_GetType(id)==b2_prismaticJoint && b2PrismaticJoint_IsLimitEnabled(id)) {
            r.lines.push_back({r.anchor+axis*b2PrismaticJoint_GetLowerLimit(id),r.anchor+axis*b2PrismaticJoint_GetUpperLimit(id),0xffff00ff});
            r.label+=" translation="+std::to_string(b2PrismaticJoint_GetTranslation(id));
        }
        if(b2Joint_GetType(id)==b2_revoluteJoint && b2RevoluteJoint_IsLimitEnabled(id)) {
            const float low=b2RevoluteJoint_GetLowerLimit(id),high=b2RevoluteJoint_GetUpperLimit(id);
            r.label+=" angle="+std::to_string(b2RevoluteJoint_GetAngle(id));
            for(int n=0;n<32;++n) {const float reference=b2Rot_GetAngle(b2Body_GetRotation(a))+b2Joint_GetReferenceAngle(id);
            const float x=reference+low+(high-low)*n/32.f,y=reference+low+(high-low)*(n+1)/32.f;
                r.lines.push_back({r.anchor+V{std::cos(x)*.4f,std::sin(x)*.4f,0},r.anchor+V{std::cos(y)*.4f,std::sin(y)*.4f,0},0xffff00ff});}
        }
        out.append(std::move(r));
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
        _impl->desiredVelocity[idx]=d.linearVelocity;
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
        _impl->desiredVelocity[idx]={cmd.u.vec4.x,cmd.u.vec4.y,0};
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
        // F-P3: param-out-of-range path. Mass must be a positive finite float;
        // zero / negative / NaN / Inf are caller bugs, not stale handles.
        if (!std::isfinite(cmd.u.vec4.x) || !(cmd.u.vec4.x > 0.0f)) {
            ++_invalidParamCount;
            return;
        }
        const b2BodyId bodyId = _impl->bodyIdByIndex[idx];
        if (!b2Body_IsValid(bodyId)) { ++_notFoundCount; return; }
        // b2Body_SetMassData writes mass/inertia on any body type; kinematic
        // and static bodies have no meaningful mass (Box2D owns it) — dynamic
        // only, matching the creation-path override. F-P3: this is also a
        // param-shaped rejection (wrong body type for the requested change),
        // not a missing handle.
        if (b2Body_GetType(bodyId) != b2_dynamicBody) { ++_invalidParamCount; return; }
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
        // F-P3: friction / restitution must be finite non-negative. Box2D's
        // own setter would happily take NaN and propagate it; reject here so
        // bad-call diagnostics surface as _invalidParamCount.
        if (!std::isfinite(cmd.u.vec4.x) || cmd.u.vec4.x < 0.0f ||
            !std::isfinite(cmd.u.vec4.y) || cmd.u.vec4.y < 0.0f) {
            ++_invalidParamCount;
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
        shapeDef.userData = handleToUserData(cmd.collider);
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

    case CT::RaycastAsync:
    case CT::OverlapSphereAsync:
    case CT::OverlapBoxAsync: {
        SyncQueryRequest req; req.requestId=cmd.queryId; req.layerMask=cmd.layerMask;
        if(cmd.type==CT::RaycastAsync) { req.type=SyncQueryType::Raycast;
            req.ray.origin={cmd.u.ray.ox,cmd.u.ray.oy,cmd.u.ray.oz}; req.ray.dir={cmd.u.ray.dx,cmd.u.ray.dy,cmd.u.ray.dz};
        } else if(cmd.type==CT::OverlapSphereAsync) {req.type=SyncQueryType::OverlapSphere; req.sphereCenter={cmd.u.sphere.cx,cmd.u.sphere.cy,cmd.u.sphere.cz};req.sphereRadius=cmd.u.sphere.radius;
        } else { req.type=SyncQueryType::OverlapBox; req.boxCenter={cmd.u.box.cx,cmd.u.box.cy,cmd.u.box.cz}; req.boxHalfExtents={cmd.u.box.hx,cmd.u.box.hy,cmd.u.box.hz}; }
        SyncQueryResponse response; executeSync(req,response);
        QueryResult result; result.queryId=cmd.queryId; result.status=response.status;
        result.dimension=PhysicsDimension::TwoD;
        if(response.firstHit.hit) result.hits.push_back(response.firstHit);
        for(auto body:response.overlaps) {RaycastHit hit; hit.hit=true;hit.body=body;result.hits.push_back(hit);}
        result.hitCount=static_cast<uint32_t>(result.hits.size());result.completedAtUs=ayt::time::Clock::performanceNowUs();
        std::lock_guard<std::mutex> lock(_impl->asyncQueryMu); _impl->asyncQueryQueue.push_back(std::move(result)); return;
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
    case SyncQueryType::SphereSweep: outResponse.status = PhysResult::Unsupported; return;
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
            outResponse.firstHit.collider = resolveColliderHandle(hit.shapeId,
                _impl->shapeIdByIndex,_impl->colliderGeneration,_impl->maxBodies);
            outResponse.firstHit.point =
                ayt::math::FVector3(hit.point.x, hit.point.y, 0.0f);
            outResponse.firstHit.normal =
                ayt::math::FVector3(hit.normal.x, hit.normal.y, 0.0f);
            outResponse.firstHit.distance = hit.fraction * kRayCastMaxDistance;
        }
        return;
    }
    case SyncQueryType::OverlapSphere:
    case SyncQueryType::OverlapBox: {
        const bool sphere=request.type==SyncQueryType::OverlapSphere;
        const auto c=sphere?request.sphereCenter:request.boxCenter;
        const auto h=request.boxHalfExtents; const auto radius=request.sphereRadius;
        if(!std::isfinite(c.x)||!std::isfinite(c.y)||(sphere?(!std::isfinite(radius)||radius<=0):(!std::isfinite(h.x)||!std::isfinite(h.y)||h.x<=0||h.y<=0))) {outResponse.status=PhysResult::InvalidParam;return;}
        b2Vec2 points[4]={{c.x,c.y},{c.x,c.y},{c.x,c.y},{c.x,c.y}};
        if(!sphere) {points[0]={c.x-h.x,c.y-h.y};points[1]={c.x+h.x,c.y-h.y};points[2]={c.x+h.x,c.y+h.y};points[3]={c.x-h.x,c.y+h.y};}
        const auto proxy=b2MakeProxy(points,sphere?1:4,sphere?radius:0);
        b2QueryFilter filter=b2DefaultQueryFilter(); filter.maskBits=request.layerMask;filter.categoryBits=~0ull;
        struct Context {Impl* impl;std::vector<BodyHandle>* out;};Context context{_impl.get(),&outResponse.overlaps};
        auto callback=[](b2ShapeId shape,void* data)->bool {auto& c=*static_cast<Context*>(data);
            const auto h=resolveBodyHandle(b2Shape_GetBody(shape),c.impl->bodyIdByIndex,c.impl->bodyGeneration,c.impl->maxBodies);
            if(h && std::find(c.out->begin(),c.out->end(),h)==c.out->end()) c.out->push_back(h);return true;};
        b2World_OverlapShape(_impl->worldId,&proxy,filter,callback,&context);return;
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
