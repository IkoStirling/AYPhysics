#pragma once
// AYPhysics/PhysicsTypes.h - public types for AYPhysics
//
// Conventions:
//   - All types live in namespace ayt::physics
//   - Handles are packed uint32 (index 20 bits + generation 12 bits), invalid=0
//   - PhysResult covers all failure modes; PhysResult::Ok == 0
//   - Compact PhysicsCommand (see AYPhysics/PhysicsCommandQueue.h) does NOT embed these descriptors;
//     create payloads live in PhysicsCreatePool out-of-band.

#include "AYPhysics/PhysicsHandles.h"

#include <cstdint>
#include <memory>
#include <vector>

#include "AYMath/MathTypes.h"
#include "AYMath/MathGeometry.h"
#include <AYTime/Duration.h>

namespace ayt::physics {

// =============================================================================
// Error model (§7.2)
// =============================================================================

enum class PhysResult : uint8_t {
    Ok              = 0,
    InvalidParam    = 1,
    NoMemory        = 2,
    AlreadyExists   = 3,
    NotFound        = 4,
    InvalidState    = 5,
    BackendError    = 6,
    OutOfRange      = 7,
    Unsupported     = 8,
    QueueFull       = 9,
};

const char* toString(PhysResult r) noexcept;

// =============================================================================
// Layers & masks (§7.3)
// =============================================================================

using PhysLayer    = uint16_t;
using PhysLayerMask = uint32_t;

enum class PhysDefaultLayer : uint16_t {
    Static    = 0,
    Dynamic   = 1,
    Character = 2,
    Trigger   = 3,
    Debris    = 4,
    // user-defined layers 16+
};

// =============================================================================
// Material (§7.4)
// =============================================================================

struct PhysMaterial {
    float friction    = 0.5f;
    float restitution = 0.0f;
    float density     = 1.0f;
};

// =============================================================================
// Body / collider / joint descriptors (§7.5)
// =============================================================================

enum class BodyType : uint8_t { Static, Dynamic, Kinematic };

struct RigidbodyDesc {
    BodyType       type           = BodyType::Dynamic;
    PhysLayer      layer          = 0;
    PhysLayerMask  collideMask    = 0xFFFFFFFFu;
    ayt::math::FVector3    position{};
    ayt::math::FQuaternion rotation{};
    ayt::math::FVector3    linearVelocity{};
    ayt::math::FVector3    angularVelocity{};
    float          mass           = 1.0f;
    float          linearDamping  = 0.05f;
    float          angularDamping = 0.05f;
    float          gravityScale   = 1.0f;  // R6: per-body gravity multiplier (1 = world default)
    PhysMaterial   material{};
    bool           alwaysSync     = false;  // appears in snapshot even when sleeping
    bool           enableCCD      = false;
    bool           fixedRotation  = false;  // R2: lock angular DOF (character controllers, top-down)
};

enum class ColliderShape : uint8_t {
    Box          = 0,
    Sphere       = 1,
    Capsule      = 2,
    ConvexHull   = 3,  // R2.0a
    TriangleMesh = 4,  // R2.0a
    Heightfield  = 5,  // R2.0a
};

// R2.0a: cooking inputs for advanced collider shapes.
// Held by std::shared_ptr<const ColliderShapeData> on ColliderDesc so that
// (a) copy through PhysicsCreatePool is O(1) refcount,
// (b) multiple colliders can share one cooked payload,
// (c) the payload is immutable at the physics layer.
// Backends reject malformed payloads (see JoltBackend3D makeShape).
struct ColliderShapeData {
    std::vector<ayt::math::FVector3> hullPoints;       // ConvexHull only; size <= 256
    std::vector<ayt::math::FVector3> meshVertices;     // TriangleMesh only
    std::vector<uint32_t>            meshIndices;      // TriangleMesh only; size % 3 == 0; CCW order
    std::vector<float>               heightSamples;    // Heightfield only; row-major N*N
    uint32_t                         heightGridN = 0;  // Heightfield only; N >= 4
};

struct ColliderDesc {
    BodyHandle     body  = InvalidBodyHandle;
    ColliderShape  shape = ColliderShape::Box;
    ayt::math::FVector3   halfExtents{0.5f, 0.5f, 0.5f};  // Box
    float          radius = 0.5f;                          // Sphere / Capsule
    float          height = 1.0f;                          // Capsule
    PhysMaterial   material{};
    bool           isTrigger = false;
    // R2.0a: required for ConvexHull / TriangleMesh / Heightfield; null otherwise.
    // Conversion to backend-native types (JPH::Vec3 / JPH::Float3 / raw float*)
    // happens inside the backend TU — never leaks Jolt types into this header.
    std::shared_ptr<const ColliderShapeData> shapeData;
};

enum class JointType : uint8_t {
    Hinge    = 0,
    Fixed    = 1,
    Distance = 2,
    Spring   = 3,
    Slider   = 4,
    Point    = 5,
    Cone     = 6,
};

struct JointDesc {
    JointType      type     = JointType::Fixed;
    BodyHandle     bodyA    = InvalidBodyHandle;
    BodyHandle     bodyB    = InvalidBodyHandle;
    ayt::math::FVector3   anchorA{};
    ayt::math::FVector3   anchorB{};
    ayt::math::FVector3   axisA{0.0f, 1.0f, 0.0f};
    ayt::math::FVector3   axisB{0.0f, 1.0f, 0.0f};
    float          minDistance = 0.0f;
    float          maxDistance = 0.0f;
    float          stiffness   = 0.0f;
    float          damping     = 0.0f;
};

// =============================================================================
// Ray / hit (§9 query)
// =============================================================================

struct RaycastHit {
    bool       hit      = false;
    BodyHandle body     = InvalidBodyHandle;
    ayt::math::FVector3 point{};
    ayt::math::FVector3 normal{};
    float      distance = 0.0f;
};

// =============================================================================
// Snapshot structures (§5.3 sparse active-body)
// =============================================================================

enum class PhysicsDimension : uint8_t {
    ThreeD = 0,
    TwoD = 1
};

struct BodyTransform {
    BodyHandle body = InvalidBodyHandle;
    ayt::math::FVector3    position{};
    ayt::math::FQuaternion rotation{};
    ayt::math::FVector3    linearVelocity{};
    ayt::math::FVector3    angularVelocity{};
    PhysicsDimension dimension = PhysicsDimension::ThreeD;
    uint8_t     flags = 0;  // bit0 = wasSleepingThisFrame (optional diagnostic)
};

struct CollisionEvent {
    BodyHandle bodyA = InvalidBodyHandle;
    BodyHandle bodyB = InvalidBodyHandle;
    ayt::math::FVector3 pointA{};
    ayt::math::FVector3 pointB{};
    ayt::math::FVector3 normal{};
    float      impulse = 0.0f;
    uint8_t    kind    = 0;  // 0=enter, 1=stay, 2=exit
};

struct QueryResult {
    uint32_t   queryId = 0;
    uint32_t   hitCount = 0;
    std::vector<RaycastHit> hits;          // raycast: first hit; overlap: many
};

struct PhysFrameSnapshot {
    uint64_t frameIndex   = 0;
    float    stepSeconds  = 0.0f;
    std::vector<BodyTransform> transforms;     // sparse: Awake ∪ AlwaysSync
    std::vector<CollisionEvent> collisionEvents;
    std::vector<QueryResult>    queryResults;  // async queries, keyed by queryId
};

// =============================================================================
// Sync-query request (§5.4 mailbox)
// =============================================================================

enum class SyncQueryType : uint8_t { Raycast, OverlapSphere, OverlapBox };

struct SyncQueryRequest {
    SyncQueryType type = SyncQueryType::Raycast;
    uint32_t      requestId = 0;
    ayt::math::Ray     ray{};
    ayt::math::FVector3 sphereCenter{};
    float             sphereRadius = 0.0f;
    ayt::math::FVector3 boxCenter{};
    ayt::math::FVector3 boxHalfExtents{};
    ayt::math::FQuaternion boxRotation{};
    PhysLayerMask  layerMask = 0xFFFFFFFFu;
};

struct SyncQueryResponse {
    uint32_t requestId = 0;
    PhysResult status = PhysResult::Ok;
    RaycastHit firstHit;       // for raycast
    std::vector<BodyHandle> overlaps;  // for overlap
};

// =============================================================================
// Backend descriptor (§9)
// =============================================================================

enum class BackendKind : uint8_t {
    Null          = 0,
    Mock          = 1,
    DefaultJolt   = 2,
    DefaultBox2D  = 3,
};

struct PhysicsBackendDescriptor {
    BackendKind kind3D = BackendKind::DefaultJolt;
    BackendKind kind2D = BackendKind::Null;
    uint32_t    jobWorkerCount          = 0;
    uint32_t    commandQueueCapacity    = 1024;
    uint32_t    createPoolCapacity      = 256;
    uint32_t    maxDrainPerTick         = 4096;
    uint32_t    maxSyncQueriesPerFrame  = 64;
    uint32_t    maxBodies               = 65536;
    uint32_t    maxBodyPairs            = 65536;
    uint32_t    maxContactConstraints   = 10240;
    uint32_t    tempAllocatorBytes      = 10u * 1024u * 1024u;
    float       fixedDeltaTime          = 1.0f / 60.0f;
    int         maxSubSteps             = 4;
    ayt::time::Duration fixedStepTimeout = ayt::time::Duration::fromSeconds(1);
    bool        syncVelocitiesInSnapshot = true;
    ayt::math::FVector3 gravity3D{0.0f, -9.81f, 0.0f};
    ayt::math::FVector2 gravity2D{0.0f, -9.81f};
};

struct PhysicsBackendInfo {
    const char* name = nullptr;
    uint32_t    maxBodies = 0;
    uint32_t    maxColliders = 0;
    uint32_t    maxJoints = 0;
    bool        realDevice = false;
};

// =============================================================================
// Lockstep stub (§10.2)
// =============================================================================

bool isLockstepActive() noexcept;

} // namespace ayt::physics
