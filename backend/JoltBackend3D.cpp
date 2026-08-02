#include "JoltBackend3D.h"

// R1.5b (B-7): this is the ONLY TU that may include <Jolt/...> headers
// (design.md §17.8 item 10). All Jolt types live behind class Impl (Pimpl)
// so the public header stays Jolt-free. R1.5a added the minimum bootstrap
// (Factory + RegisterTypes); R1.5b wires PhysicsSystem + bodies / colliders
// / joints + ContactListener + layer filters + lockstep gate.

#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/Memory.h>          // RegisterDefaultAllocator
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Math/Vec3.h>
#include <Jolt/Math/Quat.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceTable.h>
#include <Jolt/Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterTable.h>
#include <Jolt/Physics/Collision/ObjectLayerPairFilterTable.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>  // R2.0a
#include <Jolt/Physics/Collision/Shape/MeshShape.h>        // R2.0a
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h> // R2.0a
#include <Jolt/Math/Float3.h>                              // R2.0a (MeshShape VertexList = Array<Float3>)
#include <Jolt/Physics/Collision/CollisionGroup.h>
#include <Jolt/Physics/Collision/GroupFilter.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/CollisionCollector.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyManager.h>
#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Body/BodyLockMulti.h>
#include <Jolt/Physics/Constraints/Constraint.h>
#include <Jolt/Physics/Body/MotionType.h>
#include <Jolt/Physics/Body/MotionQuality.h>
#include <Jolt/Physics/EActivation.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
#include <Jolt/Physics/Constraints/DistanceConstraint.h>

#include <atomic>
#include <algorithm>
#include <mutex>
#include <thread>
#include <vector>
#include <unordered_map>

namespace ayt::physics {

// =============================================================================
// Jolt-side helpers (TU-private)
// =============================================================================
namespace {

// Broadphase layers (design.md §17.3: NON_MOVING / MOVING).
constexpr JPH::BroadPhaseLayer::Type BP_NON_MOVING = 0;
constexpr JPH::BroadPhaseLayer::Type BP_MOVING     = 1;
constexpr uint32_t                    BP_NUM_LAYERS = 2;

// Object layers map 1:1 to PhysLayer (uint16_t). We cap at 32 to keep the
// filter tables tiny; user-defined layers start at 16 (see PhysDefaultLayer).
constexpr uint32_t kMaxObjectLayers = 32;

struct BPLayerInterface final : public JPH::BroadPhaseLayerInterfaceTable {
    BPLayerInterface()
        : BroadPhaseLayerInterfaceTable(kMaxObjectLayers, BP_NUM_LAYERS) {
        for (uint32_t i = 0; i < kMaxObjectLayers; ++i) {
            MapObjectToBroadPhaseLayer(i, JPH::BroadPhaseLayer(BP_MOVING));
        }
    }
};

struct ObjLayerPairFilter final : public JPH::ObjectLayerPairFilterTable {
    // ObjectLayerPairFilterTable defaults to ALL PAIRS DISABLED (zero-filled
    // bit table). AYPhysics does fine-grained filtering via collideMask on
    // CollisionGroup, so the object-layer filter must allow every pair by
    // default — otherwise no 3D body ever collides (bodies fall through ground).
    ObjLayerPairFilter() : JPH::ObjectLayerPairFilterTable(kMaxObjectLayers) {
        for (uint32_t i = 0; i < kMaxObjectLayers; ++i) {
            for (uint32_t j = 0; j < kMaxObjectLayers; ++j) {
                EnableCollision(static_cast<JPH::ObjectLayer>(i),
                                 static_cast<JPH::ObjectLayer>(j));
            }
        }
    }
};

// ObjectVsBroadPhaseLayerFilterTable ctor signature (Jolt 5.5):
//   ObjectVsBroadPhaseLayerFilterTable(BPLayerInterface&, numBPLayers,
//                                       ObjectLayerPairFilter&, numObjLayers).
// It derives its entries from the ObjectLayerPairFilter above, so now that
// every object-layer pair is enabled, every object-vs-broadphase pair is too.
struct ObjVsBPLayerFilter final : public JPH::ObjectVsBroadPhaseLayerFilterTable {
    ObjVsBPLayerFilter(const BPLayerInterface& bpIface, const ObjLayerPairFilter& pairFilter)
        : JPH::ObjectVsBroadPhaseLayerFilterTable(
            bpIface, BP_NUM_LAYERS, pairFilter, kMaxObjectLayers) {}
};

// R9: Box2D-style bidirectional layer/mask filter for CollisionGroup.
// Encoding (mirrors PhysicsWorld2D / b2Filter):
//   GroupID    = PhysLayer     (category / "who I am")
//   SubGroupID = PhysLayerMask (collide mask / "who I hit")
// Two bodies collide iff each mask includes the other's layer bit.
class LayerMaskGroupFilter final : public JPH::GroupFilter {
public:
    bool CanCollide(const JPH::CollisionGroup& a,
                    const JPH::CollisionGroup& b) const override {
        const uint32_t layerA = a.GetGroupID();
        const uint32_t layerB = b.GetGroupID();
        if (layerA >= 32u || layerB >= 32u) return false;
        const uint32_t maskA = a.GetSubGroupID();
        const uint32_t maskB = b.GetSubGroupID();
        const bool aWantsB = (maskA & (1u << layerB)) != 0u;
        const bool bWantsA = (maskB & (1u << layerA)) != 0u;
        return aWantsB && bWantsA;
    }
};

// ContactListener: emits enter/stay/exit events into the backend's queue.
// OnContactRemoved receives a SubShapeIDPair that *does* expose Body1ID/Body2ID
// (Jolt/Physics/Collision/Shape/SubShapeIDPair.h:46-48) so we CAN resolve the
// bodies here even though the wider Body objects are unavailable.
class JoltContactListener final : public JPH::ContactListener {
public:
    explicit JoltContactListener(class JoltBackend3D* owner) : _owner(owner) {}

    void OnContactAdded(const JPH::Body& inBody1, const JPH::Body& inBody2,
                        const JPH::ContactManifold& inManifold,
                        JPH::ContactSettings& /*ioSettings*/) override {
        CollisionEvent ev{};
        ev.kind = 0;  // enter
        ev.bodyA = _owner->handleFromBodyId(inBody1.GetID());
        ev.bodyB = _owner->handleFromBodyId(inBody2.GetID());
        const JPH::Vec3 wp = inManifold.mBaseOffset;
        ev.pointA = ayt::math::FVector3(static_cast<float>(wp.GetX()),
                                        static_cast<float>(wp.GetY()),
                                        static_cast<float>(wp.GetZ()));
        ev.pointB = ev.pointA;
        const JPH::Vec3 n = inManifold.mWorldSpaceNormal;
        ev.normal = ayt::math::FVector3(static_cast<float>(n.GetX()),
                                        static_cast<float>(n.GetY()),
                                        static_cast<float>(n.GetZ()));
        ev.impulse = 0.0f;
        _owner->pushCollisionEvent(ev);
    }

    void OnContactPersisted(const JPH::Body& inBody1, const JPH::Body& inBody2,
                            const JPH::ContactManifold& inManifold,
                            JPH::ContactSettings& /*ioSettings*/) override {
        CollisionEvent ev{};
        ev.kind = 1;  // stay
        ev.bodyA = _owner->handleFromBodyId(inBody1.GetID());
        ev.bodyB = _owner->handleFromBodyId(inBody2.GetID());
        const JPH::Vec3 wp = inManifold.mBaseOffset;
        ev.pointA = ayt::math::FVector3(static_cast<float>(wp.GetX()),
                                        static_cast<float>(wp.GetY()),
                                        static_cast<float>(wp.GetZ()));
        ev.pointB = ev.pointA;
        const JPH::Vec3 n = inManifold.mWorldSpaceNormal;
        ev.normal = ayt::math::FVector3(static_cast<float>(n.GetX()),
                                        static_cast<float>(n.GetY()),
                                        static_cast<float>(n.GetZ()));
        ev.impulse = 0.0f;
        _owner->pushCollisionEvent(ev);
    }

    void OnContactRemoved(const JPH::SubShapeIDPair& inSubShapePair) override {
        CollisionEvent ev{};
        ev.kind = 2;  // exit
        ev.bodyA = _owner->handleFromBodyId(inSubShapePair.GetBody1ID());
        ev.bodyB = _owner->handleFromBodyId(inSubShapePair.GetBody2ID());
        ev.impulse = 0.0f;
        _owner->pushCollisionEvent(ev);
    }

private:
    JoltBackend3D* _owner = nullptr;
};

// =============================================================================
// Shape factory (R1.5b: Box / Sphere / Capsule; R2.0a: +ConvexHull/TriangleMesh/Heightfield)
// =============================================================================
JPH::RefConst<JPH::Shape> makeShape(const ColliderDesc& desc) {
    using namespace JPH;
    switch (desc.shape) {
    case ColliderShape::Box: {
        BoxShapeSettings s(Vec3(desc.halfExtents.x, desc.halfExtents.y, desc.halfExtents.z));
        auto r = s.Create();
        return r.IsValid() ? r.Get() : nullptr;
    }
    case ColliderShape::Sphere: {
        SphereShapeSettings s(desc.radius);
        auto r = s.Create();
        return r.IsValid() ? r.Get() : nullptr;
    }
    case ColliderShape::Capsule: {
        float halfHeight = desc.height * 0.5f - desc.radius;
        if (halfHeight < 0.0f) halfHeight = 0.0f;
        CapsuleShapeSettings s(halfHeight, desc.radius);
        auto r = s.Create();
        return r.IsValid() ? r.Get() : nullptr;
    }
    // ---- R2.0a: advanced shapes (cooking inputs via ColliderShapeData) ----
    case ColliderShape::ConvexHull: {
        const ColliderShapeData* sd = desc.shapeData.get();
        if (!sd || sd->hullPoints.empty()) return nullptr;
        // JPH::ConvexHullShape::cMaxPointsInHull = 256 (Jolt 5.5.0).
        if (sd->hullPoints.size() > 256u) return nullptr;
        Array<Vec3> pts((int)sd->hullPoints.size());
        for (size_t i = 0; i < sd->hullPoints.size(); ++i) {
            pts[i] = Vec3(sd->hullPoints[i].x, sd->hullPoints[i].y, sd->hullPoints[i].z);
        }
        ConvexHullShapeSettings settings(pts.begin(), (int)pts.size());
        auto r = settings.Create();
        if (!r.IsValid()) return nullptr;
        return r.Get();
    }
    case ColliderShape::TriangleMesh: {
        const ColliderShapeData* sd = desc.shapeData.get();
        if (!sd || sd->meshVertices.empty()) return nullptr;
        if (sd->meshIndices.size() < 3u || (sd->meshIndices.size() % 3u) != 0u) return nullptr;
        // Convert FVector3 -> Float3 (Jolt's compact vertex type for mesh shape).
        VertexList vl;
        vl.reserve(sd->meshVertices.size());
        for (const auto& v : sd->meshVertices) {
            vl.push_back(Float3(v.x, v.y, v.z));
        }
        IndexedTriangleList tl;
        tl.reserve(sd->meshIndices.size() / 3u);
        for (size_t i = 0; i + 2u < sd->meshIndices.size(); i += 3u) {
            // Jolt IndexedTriangle ctor: (i1, i2, i3, materialIndex, userData)
            // materialIndex defaults to 0 (no material list on our side yet).
            tl.push_back(IndexedTriangle(
                sd->meshIndices[i + 0],
                sd->meshIndices[i + 1],
                sd->meshIndices[i + 2],
                /*materialIndex*/ 0u,
                /*userData*/ 0u));
        }
        MeshShapeSettings settings(std::move(vl), std::move(tl));
        auto r = settings.Create();
        if (!r.IsValid()) return nullptr;
        return r.Get();
    }
    case ColliderShape::Heightfield: {
        const ColliderShapeData* sd = desc.shapeData.get();
        if (!sd || sd->heightGridN < 4u) return nullptr;
        // Jolt requires sampleCount / mBlockSize >= 2 (default mBlockSize = 2),
        // and heightSamples.size() == N*N exactly.
        const size_t expected = (size_t)sd->heightGridN * (size_t)sd->heightGridN;
        if (sd->heightSamples.size() != expected) return nullptr;
        HeightFieldShapeSettings settings(
            sd->heightSamples.data(),
            Vec3::sZero(),  // offset (R2.0a: identity; future R2.5+ can offset)
            Vec3(1.0f, 1.0f, 1.0f),  // scale (R2.0a: unit; future R2.5+ user-tunable)
            sd->heightGridN);
        auto r = settings.Create();
        if (!r.IsValid()) return nullptr;
        return r.Get();
    }
    default:
        return nullptr;
    }
}

inline JPH::ObjectLayer toObjectLayer(PhysLayer layer) noexcept {
    return static_cast<JPH::ObjectLayer>(layer < kMaxObjectLayers ? layer : 0u);
}

inline JPH::EMotionType toMotionType(BodyType t) noexcept {
    switch (t) {
    case BodyType::Static:    return JPH::EMotionType::Static;
    case BodyType::Dynamic:   return JPH::EMotionType::Dynamic;
    case BodyType::Kinematic: return JPH::EMotionType::Kinematic;
    }
    return JPH::EMotionType::Static;
}

constexpr float kRayCastMaxDistance = 10000.0f;

// AYMath Ray::dir is normalized; Jolt expects direction magnitude = cast length.
JPH::RRayCast makeJoltRayCast(const ayt::math::FVector3& origin,
                              const ayt::math::FVector3& dir,
                              float maxDistance,
                              float& outScaledLength) {
    JPH::Vec3 jDir(dir.x, dir.y, dir.z);
    const float len = jDir.Length();
    outScaledLength = maxDistance;
    if (len > 1.0e-8f) {
        jDir *= maxDistance / len;
    } else {
        jDir = JPH::Vec3(0.0f, 0.0f, maxDistance);
    }
    return JPH::RRayCast(
        JPH::RVec3(origin.x, origin.y, origin.z),
        jDir);
}

// Jolt rejects null shapes for every motion type. R1.5b uses a tiny
// placeholder until CreateCollider attaches the real shape via SetShape.
JPH::RefConst<JPH::Shape> makePlaceholderShape(
    std::vector<JPH::RefConst<JPH::Shape>>& shapeCache) {
    JPH::SphereShapeSettings settings(0.001f);
    const JPH::ShapeSettings::ShapeResult result = settings.Create();
    if (!result.IsValid()) {
        return nullptr;
    }
    shapeCache.push_back(result.Get());
    return result.Get();
}

// Per-body-side lock helper: write-lock a body, return Body& via callback.
// We use BodyLockWrite (not BodyLockRead) because ConstraintSettings::Create
// takes non-const Body& per Jolt's API. The "lock" only blocks other threads
// from mutating the body; we read it inside the callback.
template <typename Fn>
bool withBody(const JPH::BodyLockInterface& bli, JPH::BodyID id, Fn&& fn) {
    JPH::BodyLockWrite lock(bli, id);
    if (lock.Succeeded()) {
        fn(lock.GetBody());
        return true;
    }
    return false;
}

}  // anonymous namespace

// =============================================================================
// Impl (Pimpl)
// =============================================================================
struct JoltBackend3D::Impl {
    bool joltRegistered = false;

    std::unique_ptr<JPH::TempAllocatorImpl>       tempAllocator;
    std::unique_ptr<JPH::JobSystemThreadPool>     jobSystem;
    // BPLayerInterface / ObjLayerPairFilter / ObjVsBPLayerFilter wrap Jolt
    // Array / vector types whose constructors route through JPH::Allocate.
    // They MUST be allocated AFTER JPH::RegisterDefaultAllocator() runs, so
    // they live as unique_ptr<...> and we construct them inside Impl() body.
    std::unique_ptr<BPLayerInterface>             bpLayerInterface;
    std::unique_ptr<ObjLayerPairFilter>           objLayerPairFilter;
    std::unique_ptr<ObjVsBPLayerFilter>           objVsBpFilter;
    // R9: shared GroupFilter for all bodies (Ref keeps it alive while any
    // CollisionGroup still points at it).
    JPH::Ref<LayerMaskGroupFilter>                layerMaskGroupFilter;
    std::unique_ptr<JPH::PhysicsSystem>           physicsSystem;
    std::unique_ptr<JoltContactListener>          contactListener;
    JPH::BodyManager*                             bodyManager = nullptr;

    // Handle tables (sized at init3D; never resized).
    std::vector<JPH::BodyID>      bodyIdByIndex;
    std::vector<uint16_t>         bodyGeneration;
    std::vector<PhysLayer>        bodyLayer;        // R9: category for GroupID
    std::vector<PhysLayerMask>    bodyCollideMask;  // R9: mask for SubGroupID
    std::vector<uint16_t>         colliderGeneration;
    std::vector<uint16_t>         jointGeneration;
    std::vector<JPH::BodyID>      jointBodyA;
    std::vector<JPH::BodyID>      jointBodyB;
    std::vector<JPH::Constraint*>   jointConstraintByIndex;

    // Shape retention (collider shapes cached until backend destruction).
    std::vector<JPH::RefConst<JPH::Shape>> shapeCache;

    mutable std::mutex                            collisionEventMu;
    std::vector<CollisionEvent>                   collisionEventQueue;
    mutable std::mutex                            asyncQueryMu;
    std::vector<QueryResult>                      asyncQueryQueue;

    std::vector<uint32_t>                         alwaysSyncSet;

    uint32_t                                      maxBodies = 0;

    Impl() {
        // R1.5b critical landmine: vcpkg's Jolt 5.5.0 static lib exports
        // JPH::Allocate as a NULL function pointer (BSS zero-init). The
        // RegisterDefaultAllocator() symbol may not be pulled in by the
        // linker (only its declaration lives in the header — nothing in
        // vcpkg's Jolt internals calls it). Belt-and-suspenders: assign
        // directly so we never depend on Jolt's allocator registration
        // being pulled in.
        if (JPH::Allocate == nullptr) {
            JPH::Allocate        = [](size_t s) -> void* { return std::malloc(s); };
            JPH::Free            = [](void* p)         { std::free(p); };
            JPH::AlignedAllocate = [](size_t s, size_t a) -> void* {
#if defined(_WIN32)
                return _aligned_malloc(s, a);
#else
                void* b = nullptr;
                posix_memalign(&b, a, s);
                return b;
#endif
            };
            JPH::AlignedFree     = [](void* p) {
#if defined(_WIN32)
                _aligned_free(p);
#else
                std::free(p);
#endif
            };
        }
        // Also try the official registration in case RegisterDefaultAllocator
        // got pulled in (cheap idempotent re-assignment if already wired).
        JPH::RegisterDefaultAllocator();

        if (JPH::Factory::sInstance == nullptr) {
            JPH::Factory::sInstance = new JPH::Factory();
        }
        JPH::RegisterTypes();
        joltRegistered = true;

        // Filters — allocated NOW (after RegisterDefaultAllocator) so the
        // BroadPhaseLayerInterfaceTable ctor's internal Array resize goes
        // through JPH::Allocate (already wired).
        bpLayerInterface   = std::make_unique<BPLayerInterface>();
        objLayerPairFilter = std::make_unique<ObjLayerPairFilter>();
        objVsBpFilter      = std::make_unique<ObjVsBPLayerFilter>(*bpLayerInterface, *objLayerPairFilter);
        layerMaskGroupFilter = new LayerMaskGroupFilter();
    }

    ~Impl() {
        if (physicsSystem) {
            for (JPH::Constraint* c : jointConstraintByIndex) {
                if (c != nullptr) {
                    physicsSystem->RemoveConstraint(c);
                }
            }
        }
        jointConstraintByIndex.clear();
        // PhysicsSystem dtor calls ExitPhysicsSystem internally.
        physicsSystem.reset();
        contactListener.reset();
        jobSystem.reset();
        tempAllocator.reset();

        shapeCache.clear();
        objVsBpFilter.reset();
        objLayerPairFilter.reset();

        if (joltRegistered) {
            JPH::UnregisterTypes();
            joltRegistered = false;
        }
    }
};

// =============================================================================
// BodyID <-> BodyHandle translation (called by ContactListener too)
// =============================================================================
BodyHandle JoltBackend3D::handleFromBodyId(const JPH::BodyID& id) {
    if (!_impl || id.IsInvalid() || !_impl->physicsSystem) {
        return InvalidBodyHandle;
    }

    JPH::BodyInterface& bi = _impl->physicsSystem->GetBodyInterfaceNoLock();
    const uint64_t userdata = bi.GetUserData(id);
    if (userdata == 0u) {
        return InvalidBodyHandle;
    }

    const BodyHandle h = static_cast<BodyHandle>(userdata);
    const uint32_t idx = handleIndex(h);
    if (idx == 0u || idx >= _impl->maxBodies) {
        return InvalidBodyHandle;
    }
    if (_impl->bodyGeneration[idx] != handleGeneration(h)) {
        return InvalidBodyHandle;
    }
    if (_impl->bodyIdByIndex[idx] != id) {
        return InvalidBodyHandle;
    }
    return h;
}

void JoltBackend3D::pushCollisionEvent(const CollisionEvent& ev) {
    if (!_impl) return;
    std::lock_guard<std::mutex> lk(_impl->collisionEventMu);
    _impl->collisionEventQueue.push_back(ev);
    ++_collisionEventCount;
}

// =============================================================================
// Lifecycle
// =============================================================================
JoltBackend3D::JoltBackend3D()  = default;
JoltBackend3D::~JoltBackend3D() { stop(); }

bool JoltBackend3D::init3D(const PhysicsBackendDescriptor& desc) {
    if (!_impl) _impl = std::make_unique<Impl>();

    const uint32_t maxBodies = desc.maxBodies > 0u ? desc.maxBodies : 1024u;
    _impl->maxBodies = maxBodies;

    _impl->bodyIdByIndex.assign(maxBodies, JPH::BodyID());
    _impl->bodyGeneration.assign(maxBodies, 0u);
    _impl->bodyLayer.assign(maxBodies, 0u);
    _impl->bodyCollideMask.assign(maxBodies, 0xFFFFFFFFu);
    _impl->colliderGeneration.assign(maxBodies, 0u);
    _impl->jointGeneration.assign(maxBodies, 0u);
    _impl->jointBodyA.assign(maxBodies, JPH::BodyID());
    _impl->jointBodyB.assign(maxBodies, JPH::BodyID());
    _impl->jointConstraintByIndex.assign(maxBodies, nullptr);
    _impl->alwaysSyncSet.reserve(64);

    // PhysicsSystem + TempAllocator + JobSystem.
    const uint32_t tempBytes = desc.tempAllocatorBytes > 0u
        ? desc.tempAllocatorBytes
        : (10u * 1024u * 1024u);
    _impl->tempAllocator = std::make_unique<JPH::TempAllocatorImpl>(tempBytes);
    const uint32_t hwConc =
        std::max(1u, static_cast<uint32_t>(std::thread::hardware_concurrency()));
    uint32_t workerCount = 0u;
    if (desc.jobWorkerCount > 0u) {
        workerCount = std::min(desc.jobWorkerCount, hwConc);
    } else if (hwConc > 2u) {
        workerCount = hwConc - 2u;
    } else {
        workerCount = 1u;
    }
    workerCount = std::max(1u, workerCount);

    // Match Jolt sample defaults (HelloWorld / UnitTests): large fixed job
    // pool + small barrier count; only thread count scales with hardware.
    constexpr uint32_t kMaxPhysicsJobs     = 2048u;
    constexpr uint32_t kMaxPhysicsBarriers   = 8u;
    _impl->jobSystem = std::make_unique<JPH::JobSystemThreadPool>(
        kMaxPhysicsJobs, kMaxPhysicsBarriers, static_cast<int>(workerCount));

    // numBodyMutexes: 0 = auto-detect (power-of-2 in [1, 64]).
    _impl->physicsSystem = std::make_unique<JPH::PhysicsSystem>();
    _impl->physicsSystem->Init(
        maxBodies,
        /*numBodyMutexes*/ 0,
        std::max(desc.maxBodyPairs, maxBodies * 4u),
        std::max(desc.maxContactConstraints, maxBodies * 4u),
        *_impl->bpLayerInterface,
        *_impl->objVsBpFilter,
        *_impl->objLayerPairFilter);

    _impl->physicsSystem->SetGravity(JPH::Vec3(_gravity.x, _gravity.y, _gravity.z));

    _initialized = true;
    return true;
}

void JoltBackend3D::setGravity(const ayt::math::FVector3& g) {
    _gravity = g;
    if (_impl && _impl->physicsSystem) {
        _impl->physicsSystem->SetGravity(JPH::Vec3(g.x, g.y, g.z));
    }
}

bool JoltBackend3D::start(const PhysicsBackendInfo& /*info*/) {
    if (!_initialized || !_impl || !_impl->physicsSystem) return false;
    _impl->contactListener = std::make_unique<JoltContactListener>(this);
    _impl->physicsSystem->SetContactListener(_impl->contactListener.get());
    _running = true;
    return true;
}

void JoltBackend3D::stop() {
    if (!_running) return;
    _running = false;
}

// =============================================================================
// Step (lockstep gate + Jolt Update)
// =============================================================================
void JoltBackend3D::step(float deltaTime) {
    ++_stepCount;
    if (isLockstepActive() || _lockstepForcedActive) {
        ++_lockstepRefusedCount;
        return;
    }
    if (!_running || !_impl || !_impl->physicsSystem) return;

    float dt = deltaTime;
    if (dt <= 0.0f)         dt = 1.0f / 60.0f;
    if (dt > 1.0f / 30.0f)  dt = 1.0f / 30.0f;
    if (dt < 1.0f / 240.0f) dt = 1.0f / 240.0f;

    // Update signature: (deltaTime, inCollisionSteps, tempAlloc, jobSys).
    _impl->physicsSystem->Update(dt, /*collisionSteps*/ 1,
                                 _impl->tempAllocator.get(),
                                 _impl->jobSystem.get());
}

// =============================================================================
// publishSnapshot
// =============================================================================
void JoltBackend3D::publishSnapshot(PhysFrameSnapshot& outSnapshot) {
    if (!_impl || !_impl->physicsSystem) return;
    const JPH::BodyLockInterface& bli = _impl->physicsSystem->GetBodyLockInterface();

    outSnapshot.transforms.clear();
    outSnapshot.transforms.reserve(_impl->physicsSystem->GetNumBodies());

    auto emitBody = [&](JPH::BodyID id) {
        JPH::BodyLockRead lock(bli, id);
        if (!lock.Succeeded()) return;
        const JPH::Body& body = lock.GetBody();
        const JPH::RVec3 pos = body.GetPosition();
        const JPH::Quat  rot = body.GetRotation();
        const JPH::Vec3  lv  = body.GetLinearVelocity();
        const JPH::Vec3  av  = body.GetAngularVelocity();
        BodyHandle h = handleFromBodyId(id);
        if (h == InvalidBodyHandle) return;
        BodyTransform bt{};
        bt.body = h;
        bt.position = ayt::math::FVector3(static_cast<float>(pos.GetX()),
                                          static_cast<float>(pos.GetY()),
                                          static_cast<float>(pos.GetZ()));
        bt.rotation = ayt::math::FQuaternion(rot.GetX(), rot.GetY(),
                                             rot.GetZ(), rot.GetW());
        bt.linearVelocity  = ayt::math::FVector3(lv.GetX(), lv.GetY(), lv.GetZ());
        bt.angularVelocity = ayt::math::FVector3(av.GetX(), av.GetY(), av.GetZ());
        outSnapshot.transforms.push_back(bt);
    };

    // Iterate active rigid bodies. Jolt's "active" list covers Awake bodies
    // (incl. kinematic). Sleeping bodies omitted unless they have alwaysSync.
    JPH::BodyIDVector activeIds;
    _impl->physicsSystem->GetActiveBodies(JPH::EBodyType::RigidBody, activeIds);
    for (JPH::BodyID id : activeIds) emitBody(id);

    // alwaysSync set: emit sleeping bodies too.
    for (uint32_t idx : _impl->alwaysSyncSet) {
        if (idx == 0u || idx >= _impl->bodyGeneration.size()) continue;
        if (_impl->bodyGeneration[idx] == 0u) continue;
        const JPH::BodyID id = _impl->bodyIdByIndex[idx];
        if (id.IsInvalid()) continue;
        emitBody(id);
    }

    // Drain collision events.
    {
        std::lock_guard<std::mutex> lk(_impl->collisionEventMu);
        outSnapshot.collisionEvents.insert(outSnapshot.collisionEvents.end(),
            _impl->collisionEventQueue.begin(), _impl->collisionEventQueue.end());
        _impl->collisionEventQueue.clear();
    }
    // Drain async query results.
    {
        std::lock_guard<std::mutex> lk(_impl->asyncQueryMu);
        outSnapshot.queryResults.insert(outSnapshot.queryResults.end(),
            _impl->asyncQueryQueue.begin(), _impl->asyncQueryQueue.end());
        _impl->asyncQueryQueue.clear();
    }
}

// =============================================================================
// execute — dispatch a PhysicsCommand (physics thread only)
// =============================================================================
void JoltBackend3D::execute(const PhysicsCommand& cmd,
                            const PhysicsCreatePayload* createPayload) {
    ++_commandCount;
    if (!_impl || !_impl->physicsSystem) return;

    using CT = PhysicsCommandType;
    JPH::BodyInterface&               bi  = _impl->physicsSystem->GetBodyInterfaceNoLock();
    const JPH::BodyLockInterface&     bli = _impl->physicsSystem->GetBodyLockInterface();

    switch (cmd.type) {
    case CT::Step:
        return;

    case CT::CreateRigidbody: {
        if (!createPayload) { ++_notFoundCount; return; }
        const RigidbodyDesc& d = createPayload->rigidDesc;
        const uint32_t idx = handleIndex(cmd.body);
        if (idx == 0u || idx >= _impl->maxBodies) { ++_notFoundCount; return; }
        if (_impl->bodyGeneration[idx] != 0u)    { ++_notFoundCount; return; }

        JPH::BodyCreationSettings bcs(
            static_cast<const JPH::Shape*>(nullptr),
            JPH::RVec3(d.position.x, d.position.y, d.position.z),
            JPH::Quat(d.rotation.x, d.rotation.y, d.rotation.z, d.rotation.w),
            toMotionType(d.type),
            toObjectLayer(d.layer));
        // R9: GroupID=layer, SubGroupID=collideMask, filtered by LayerMaskGroupFilter
        // (Box2D-style bidirectional mask). Previously stored mask as GroupID with
        // a null filter, so collideMask was never actually applied.
        bcs.mCollisionGroup = JPH::CollisionGroup(
            _impl->layerMaskGroupFilter.GetPtr(),
            static_cast<JPH::CollisionGroup::GroupID>(d.layer),
            static_cast<JPH::CollisionGroup::SubGroupID>(d.collideMask));
        bcs.mLinearVelocity  = JPH::Vec3(d.linearVelocity.x,  d.linearVelocity.y,  d.linearVelocity.z);
        bcs.mAngularVelocity = JPH::Vec3(d.angularVelocity.x, d.angularVelocity.y, d.angularVelocity.z);
        bcs.mFriction        = d.material.friction;
        bcs.mRestitution     = d.material.restitution;
        bcs.mLinearDamping   = d.linearDamping;
        bcs.mAngularDamping  = d.angularDamping;
        bcs.mMotionQuality   = d.enableCCD ? JPH::EMotionQuality::LinearCast
                                            : JPH::EMotionQuality::Discrete;
        bcs.mAllowSleeping   = true;

        JPH::RefConst<JPH::Shape> placeholder =
            makePlaceholderShape(_impl->shapeCache);
        if (placeholder == nullptr) {
            ++_notFoundCount;
            return;
        }
        bcs.SetShape(placeholder.GetPtr());
        bcs.mOverrideMassProperties =
            JPH::EOverrideMassProperties::MassAndInertiaProvided;
        if (d.type == BodyType::Dynamic || d.type == BodyType::Kinematic) {
            bcs.mMassPropertiesOverride.mMass = d.mass > 0.0f ? d.mass : 1.0f;
        } else {
            bcs.mMassPropertiesOverride.mMass = 0.0f;
        }
        bcs.mMassPropertiesOverride.mInertia = JPH::Mat44::sIdentity();

        JPH::Body* body = bi.CreateBody(bcs);
        if (body == nullptr) { ++_notFoundCount; return; }
        const JPH::BodyID id = body->GetID();
        bi.AddBody(id, JPH::EActivation::Activate);
        bi.SetUserData(id, static_cast<uint64_t>(cmd.body));
        _impl->bodyIdByIndex[idx] = id;
        _impl->bodyGeneration[idx] = static_cast<uint16_t>(handleGeneration(cmd.body));
        _impl->bodyLayer[idx]       = d.layer;
        _impl->bodyCollideMask[idx] = d.collideMask;
        if (d.alwaysSync) _impl->alwaysSyncSet.push_back(idx);
        return;
    }

    case CT::DestroyRigidbody: {
        const uint32_t idx = handleIndex(cmd.body);
        if (idx == 0u || idx >= _impl->maxBodies) { ++_notFoundCount; return; }
        if (_impl->bodyGeneration[idx] != handleGeneration(cmd.body)) { ++_notFoundCount; return; }
        const JPH::BodyID id = _impl->bodyIdByIndex[idx];
        if (!id.IsInvalid()) {
            bi.RemoveBody(id);
            bi.DestroyBody(id);
        }
        _impl->bodyGeneration[idx] = 0u;
        _impl->bodyIdByIndex[idx]  = JPH::BodyID();
        _impl->bodyLayer[idx]       = 0u;
        _impl->bodyCollideMask[idx] = 0xFFFFFFFFu;
        auto it = std::find(_impl->alwaysSyncSet.begin(), _impl->alwaysSyncSet.end(), idx);
        if (it != _impl->alwaysSyncSet.end()) _impl->alwaysSyncSet.erase(it);
        return;
    }

    case CT::SetRigidbodyTransform: {
        const uint32_t idx = handleIndex(cmd.body);
        if (idx == 0u || idx >= _impl->maxBodies) { ++_notFoundCount; return; }
        if (_impl->bodyGeneration[idx] != handleGeneration(cmd.body)) { ++_notFoundCount; return; }
        const JPH::BodyID id = _impl->bodyIdByIndex[idx];
        if (id.IsInvalid()) return;
        bi.SetPositionAndRotation(id,
            JPH::RVec3(cmd.u.xform.px, cmd.u.xform.py, cmd.u.xform.pz),
            JPH::Quat(cmd.u.xform.qx, cmd.u.xform.qy, cmd.u.xform.qz, cmd.u.xform.qw),
            JPH::EActivation::Activate);
        return;
    }

    case CT::ApplyForce:
    case CT::ApplyImpulse: {
        const uint32_t idx = handleIndex(cmd.body);
        if (idx == 0u || idx >= _impl->maxBodies) { ++_notFoundCount; return; }
        if (_impl->bodyGeneration[idx] != handleGeneration(cmd.body)) { ++_notFoundCount; return; }
        const JPH::BodyID id = _impl->bodyIdByIndex[idx];
        if (id.IsInvalid()) return;
        if (cmd.type == CT::ApplyForce) {
            bi.AddForce(id, JPH::Vec3(cmd.u.vec4.x, cmd.u.vec4.y, cmd.u.vec4.z),
                        JPH::EActivation::Activate);
        } else {
            bi.AddImpulse(id, JPH::Vec3(cmd.u.vec4.x, cmd.u.vec4.y, cmd.u.vec4.z));
            bi.ActivateBody(id);
        }
        return;
    }

    case CT::CreateCollider: {
        if (!createPayload) { ++_notFoundCount; return; }
        const ColliderDesc& d = createPayload->colliderDesc;
        const uint32_t idx = handleIndex(cmd.collider);
        if (idx == 0u || idx >= _impl->maxBodies) { ++_notFoundCount; return; }
        if (_impl->colliderGeneration[idx] != 0u)  { ++_notFoundCount; return; }

        const uint32_t bodyIdx = handleIndex(d.body);
        if (bodyIdx == 0u || bodyIdx >= _impl->maxBodies) { ++_notFoundCount; return; }
        if (_impl->bodyGeneration[bodyIdx] != handleGeneration(d.body)) {
            ++_notFoundCount;
            return;
        }
        const JPH::BodyID bodyId = _impl->bodyIdByIndex[bodyIdx];
        if (bodyId.IsInvalid()) { ++_notFoundCount; return; }

        JPH::RefConst<JPH::Shape> shape = makeShape(d);
        if (shape == nullptr) { ++_notFoundCount; return; }

        // R2.0a: JPH::MeshShape and JPH::HeightFieldShape both override
        // MustBeStatic() = true (Jolt asserts otherwise). Verify the body
        // is actually a Static motion type before we SetShape() — reject
        // with NotFound (not a crash) if the caller asked for a static-only
        // shape on a dynamic / kinematic body.
        if (d.shape == ColliderShape::TriangleMesh ||
            d.shape == ColliderShape::Heightfield) {
            const JPH::BodyID bId = _impl->bodyIdByIndex[bodyIdx];
            if (!bId.IsInvalid()) {
                const JPH::BodyLockInterface& bli =
                    _impl->physicsSystem->GetBodyLockInterface();
                JPH::BodyLockRead lock(bli, bId);
                if (!lock.Succeeded() ||
                    lock.GetBody().GetMotionType() != JPH::EMotionType::Static) {
                    ++_notFoundCount;
                    return;
                }
            }
        }

        _impl->shapeCache.push_back(shape);
        bi.SetShape(bodyId, shape.GetPtr(), /*inUpdateMassProperties*/ false,
                    JPH::EActivation::Activate);
        _impl->colliderGeneration[idx] = static_cast<uint16_t>(handleGeneration(cmd.collider));
        return;
    }

    case CT::DestroyCollider: {
        const uint32_t idx = handleIndex(cmd.collider);
        if (idx == 0u || idx >= _impl->maxBodies) { ++_notFoundCount; return; }
        if (_impl->colliderGeneration[idx] != handleGeneration(cmd.collider)) { ++_notFoundCount; return; }
        _impl->colliderGeneration[idx] = 0u;
        return;
    }

    case CT::CreateJoint: {
        if (!createPayload) { ++_notFoundCount; return; }
        const JointDesc& d = createPayload->jointDesc;
        const uint32_t idx = handleIndex(cmd.joint);
        if (idx == 0u || idx >= _impl->maxBodies) { ++_notFoundCount; return; }
        if (_impl->jointGeneration[idx] != 0u)  { ++_notFoundCount; return; }

        const uint32_t aIdx = handleIndex(d.bodyA);
        const uint32_t bIdx = handleIndex(d.bodyB);
        if (aIdx == 0u || bIdx == 0u || aIdx >= _impl->maxBodies || bIdx >= _impl->maxBodies) {
            ++_notFoundCount; return;
        }
        if (_impl->bodyGeneration[aIdx] == 0u || _impl->bodyGeneration[bIdx] == 0u) {
            ++_notFoundCount; return;
        }
        const JPH::BodyID aId = _impl->bodyIdByIndex[aIdx];
        const JPH::BodyID bId = _impl->bodyIdByIndex[bIdx];
        if (aId.IsInvalid() || bId.IsInvalid()) { ++_notFoundCount; return; }

        JPH::BodyID bodyIds[2] = { aId, bId };
        JPH::BodyLockMultiWrite bodyLock(bli, bodyIds, 2);
        JPH::Body* aBody = bodyLock.GetBody(0);
        JPH::Body* bBody = bodyLock.GetBody(1);
        if (aBody == nullptr || bBody == nullptr) {
            ++_notFoundCount;
            return;
        }

        JPH::TwoBodyConstraint* c = nullptr;
        switch (d.type) {
        case JointType::Fixed: {
            JPH::FixedConstraintSettings s;
            s.mSpace = JPH::EConstraintSpace::WorldSpace;
            s.mAutoDetectPoint = true;
            c = s.Create(*aBody, *bBody);
            break;
        }
        case JointType::Hinge: {
            JPH::HingeConstraintSettings s;
            s.mPoint1     = JPH::Vec3(d.anchorA.x, d.anchorA.y, d.anchorA.z);
            s.mPoint2     = JPH::Vec3(d.anchorB.x, d.anchorB.y, d.anchorB.z);
            s.mHingeAxis1 = JPH::Vec3(d.axisA.x, d.axisA.y, d.axisA.z);
            s.mHingeAxis2 = JPH::Vec3(d.axisB.x, d.axisB.y, d.axisB.z);
            c = s.Create(*aBody, *bBody);
            break;
        }
        case JointType::Distance: {
            JPH::DistanceConstraintSettings s;
            s.mPoint1     = JPH::Vec3(d.anchorA.x, d.anchorA.y, d.anchorA.z);
            s.mPoint2     = JPH::Vec3(d.anchorB.x, d.anchorB.y, d.anchorB.z);
            s.mMinDistance = d.minDistance;
            s.mMaxDistance = d.maxDistance;
            c = s.Create(*aBody, *bBody);
            break;
        }
        default:
            break;
        }

        if (c == nullptr) { ++_notFoundCount; return; }
        _impl->physicsSystem->AddConstraint(c);
        _impl->jointBodyA[idx]      = aId;
        _impl->jointBodyB[idx]      = bId;
        _impl->jointConstraintByIndex[idx] = c;
        _impl->jointGeneration[idx] = static_cast<uint16_t>(handleGeneration(cmd.joint));
        return;
    }

    case CT::DestroyJoint: {
        const uint32_t idx = handleIndex(cmd.joint);
        if (idx == 0u || idx >= _impl->maxBodies) { ++_notFoundCount; return; }
        if (_impl->jointGeneration[idx] != handleGeneration(cmd.joint)) { ++_notFoundCount; return; }
        if (JPH::Constraint* c = _impl->jointConstraintByIndex[idx]) {
            _impl->physicsSystem->RemoveConstraint(c);
            _impl->jointConstraintByIndex[idx] = nullptr;
        }
        _impl->jointGeneration[idx] = 0u;
        _impl->jointBodyA[idx] = JPH::BodyID();
        _impl->jointBodyB[idx] = JPH::BodyID();
        return;
    }

    case CT::RaycastAsync: {
        float rayLength = kRayCastMaxDistance;
        const ayt::math::FVector3 origin(cmd.u.ray.ox, cmd.u.ray.oy, cmd.u.ray.oz);
        const ayt::math::FVector3 dir(cmd.u.ray.dx, cmd.u.ray.dy, cmd.u.ray.dz);
        const JPH::RRayCast ray = makeJoltRayCast(origin, dir, kRayCastMaxDistance, rayLength);
        JPH::RayCastResult hit;
        const bool didHit = _impl->physicsSystem->GetNarrowPhaseQuery().CastRay(ray, hit);
        QueryResult qr{};
        qr.queryId = cmd.queryId;
        if (didHit) {
            qr.hitCount = 1;
            RaycastHit& rh = qr.hits.emplace_back();
            rh.hit      = true;
            rh.body     = handleFromBodyId(hit.mBodyID);
            const float worldDist = hit.mFraction * rayLength;
            rh.point = ayt::math::FVector3(
                origin.x + dir.x * worldDist,
                origin.y + dir.y * worldDist,
                origin.z + dir.z * worldDist);
            rh.normal   = ayt::math::FVector3(-dir.x, -dir.y, -dir.z);
            rh.distance = worldDist;
        }
        std::lock_guard<std::mutex> lk(_impl->asyncQueryMu);
        _impl->asyncQueryQueue.push_back(std::move(qr));
        return;
    }

    case CT::OverlapSphereAsync: {
        struct CollectorCB : public JPH::CollideShapeBodyCollector {
            uint32_t hitCount = 0;
            void AddHit(const JPH::BodyID& /*inID*/) override { ++hitCount; }
        } cb;
        _impl->physicsSystem->GetBroadPhaseQuery().CollideSphere(
            JPH::Vec3(cmd.u.sphere.cx, cmd.u.sphere.cy, cmd.u.sphere.cz),
            cmd.u.sphere.radius, cb);
        QueryResult qr{};
        qr.queryId = cmd.queryId;
        qr.hitCount = cb.hitCount;
        std::lock_guard<std::mutex> lk(_impl->asyncQueryMu);
        _impl->asyncQueryQueue.push_back(std::move(qr));
        return;
    }

    case CT::OverlapBoxAsync: {
        // R1.5b: command packs only center (R1 limit); emit empty result.
        std::lock_guard<std::mutex> lk(_impl->asyncQueryMu);
        QueryResult qr{};
        qr.queryId = cmd.queryId;
        qr.hitCount = 0;
        _impl->asyncQueryQueue.push_back(std::move(qr));
        return;
    }

    case CT::WakeAll: {
        JPH::BodyIDVector all;
        _impl->physicsSystem->GetBodies(all);
        for (JPH::BodyID id : all) bi.ActivateBody(id);
        return;
    }

    case CT::SleepAll: {
        JPH::BodyIDVector all;
        _impl->physicsSystem->GetBodies(all);
        for (JPH::BodyID id : all) bi.DeactivateBody(id);
        return;
    }

    case CT::SetRigidbodyCollideMask: {
        // R9: runtime collide-mask toggle (symmetric with PhysicsWorld2D / Box2D).
        // Updates SubGroupID on the body's CollisionGroup so LayerMaskGroupFilter
        // picks up the new mask on the next contact validation.
        const uint32_t idx = handleIndex(cmd.body);
        if (idx == 0u || idx >= _impl->maxBodies) { ++_notFoundCount; return; }
        if (_impl->bodyGeneration[idx] != handleGeneration(cmd.body)) {
            ++_notFoundCount;
            return;
        }
        _impl->bodyCollideMask[idx] = cmd.layerMask;
        const JPH::BodyID id = _impl->bodyIdByIndex[idx];
        if (id.IsInvalid()) { ++_notFoundCount; return; }
        const PhysLayer layer = _impl->bodyLayer[idx];
        const bool ok = withBody(bli, id, [&](JPH::Body& body) {
            JPH::CollisionGroup g = body.GetCollisionGroup();
            g.SetGroupFilter(_impl->layerMaskGroupFilter.GetPtr());
            g.SetGroupID(static_cast<JPH::CollisionGroup::GroupID>(layer));
            g.SetSubGroupID(static_cast<JPH::CollisionGroup::SubGroupID>(cmd.layerMask));
            body.SetCollisionGroup(g);
        });
        if (!ok) { ++_notFoundCount; return; }
        bi.ActivateBody(id);
        return;
    }

    case CT::SetColliderShape:
        return;
    }

    ++_notFoundCount;
}

// =============================================================================
// executeSync
// =============================================================================
void JoltBackend3D::executeSync(const SyncQueryRequest& request,
                                SyncQueryResponse& outResponse) {
    outResponse.requestId = request.requestId;
    outResponse.status    = PhysResult::Ok;

    if (!_impl || !_impl->physicsSystem) {
        outResponse.status = PhysResult::BackendError;
        return;
    }

    switch (request.type) {
    case SyncQueryType::Raycast: {
        float rayLength = kRayCastMaxDistance;
        const JPH::RRayCast ray = makeJoltRayCast(
            request.ray.origin, request.ray.dir, kRayCastMaxDistance, rayLength);
        JPH::RayCastResult hit;
        if (_impl->physicsSystem->GetNarrowPhaseQuery().CastRay(ray, hit)) {
            outResponse.firstHit.hit  = true;
            outResponse.firstHit.body = handleFromBodyId(hit.mBodyID);
            const float worldDist = hit.mFraction * rayLength;
            outResponse.firstHit.point = ayt::math::FVector3(
                request.ray.origin.x + request.ray.dir.x * worldDist,
                request.ray.origin.y + request.ray.dir.y * worldDist,
                request.ray.origin.z + request.ray.dir.z * worldDist);
            outResponse.firstHit.normal = ayt::math::FVector3(
                -request.ray.dir.x, -request.ray.dir.y, -request.ray.dir.z);
            outResponse.firstHit.distance = worldDist;
        }
        return;
    }
    case SyncQueryType::OverlapSphere: {
        struct SphereCollector : public JPH::CollideShapeBodyCollector {
            std::vector<BodyHandle> outHandles;
            void AddHit(const JPH::BodyID& inID) override {
                BodyHandle h = owner->handleFromBodyId(inID);
                if (h != InvalidBodyHandle) outHandles.push_back(h);
            }
            JoltBackend3D* owner = nullptr;
        } cb;
        cb.owner = this;
        _impl->physicsSystem->GetBroadPhaseQuery().CollideSphere(
            JPH::Vec3(request.sphereCenter.x, request.sphereCenter.y, request.sphereCenter.z),
            request.sphereRadius, cb);
        outResponse.overlaps = std::move(cb.outHandles);
        return;
    }
    case SyncQueryType::OverlapBox: {
        // R1.5b: command packed only center; clear overlaps (R2 widens).
        outResponse.overlaps.clear();
        return;
    }
    }
}

PhysicsBackendInfo JoltBackend3D::describe() const {
    PhysicsBackendInfo info{};
    info.name       = "Jolt (R1.5b real)";
    info.realDevice = true;
    info.maxBodies  = _impl ? _impl->maxBodies : 0u;
    return info;
}

} // namespace ayt::physics