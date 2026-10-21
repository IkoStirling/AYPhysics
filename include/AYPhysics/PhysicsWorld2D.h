#pragma once
// AYPhysics/PhysicsWorld2D.h - 2D world public API (§9, R2.5 Box2D)

#include "AYPhysics/PhysicsTypes.h"
#include "AYPhysics/PhysicsHandles.h"

#include <cstdint>
#include <vector>

namespace ayt::physics {

class PhysicsManager;
class PhysicsWorld2D {
public:
    /** Queue creation on a running manager. Ok publishes a handle for a body
     * created during the next step; rejection clears outHandle and does not
     * consume a handle index or generation. */
    PhysResult createRigidbody(const RigidbodyDesc& desc, BodyHandle& outHandle);
    PhysResult destroyRigidbody(BodyHandle h);
    PhysResult setRigidbodyTransform(BodyHandle h, const ayt::math::FVector3& p,
                                     const ayt::math::FQuaternion& r);
    PhysResult applyForce(BodyHandle h, const ayt::math::FVector3& f);
    PhysResult applyImpulse(BodyHandle h, const ayt::math::FVector3& impulse,
                            const ayt::math::FVector3& point);
    // R6: Z-axis torque / angular impulse (2D spin).
    PhysResult applyTorque(BodyHandle h, float torque);
    PhysResult applyAngularImpulse(BodyHandle h, float angularImpulse);

    // P3: runtime toggle of a body's collide mask. Used by the bridge character
    // controller to phase through one-way platforms while rising. The new mask
    // replaces RigidbodyDesc::collideMask for all shapes on the body.
    PhysResult setRigidbodyCollideMask(BodyHandle h, PhysLayerMask newMask);

    /** Queue collider creation with the same admission and handle rules as bodies. */
    PhysResult createCollider(const ColliderDesc& desc, ColliderHandle& outHandle);
    PhysResult destroyCollider(ColliderHandle h);

    /** Queue joint creation with the same admission and handle rules as bodies. */
    PhysResult createJoint(const JointDesc& desc, JointHandle& outHandle);
    PhysResult destroyJoint(JointHandle h);

    /** Queue a legacy 2D ray. All origin/direction components must be finite.
     * A zero XY direction keeps the legacy upward fallback. Returns a nonzero
     * query ID on enqueue; zero means invalid geometry, stopped manager or full
     * queue. Consume the result from a later PhysFrameSnapshot. */
    uint32_t   raycastAsync(const ayt::math::Ray& ray, PhysLayerMask layerMask = 0xFFFFFFFFu);
    /** Queue a body-only XY circle overlap. Center must be finite and radius
     * positive/finite. Zero indicates rejection; results arrive in a later
     * PhysFrameSnapshot. */
    uint32_t   overlapSphereAsync(const ayt::math::FVector3& center, float radius,
                                  PhysLayerMask layerMask = 0xFFFFFFFFu);
    /** Queue a body-only axis-aligned XY box overlap. Center and half-extents
     * must be finite, with positive XY half-extents. Zero indicates rejection;
     * results arrive in a later PhysFrameSnapshot. */
    uint32_t   overlapBoxAsync(const ayt::math::FVector3& center,
                               const ayt::math::FVector3& halfExtents,
                               PhysLayerMask layerMask = 0xFFFFFFFFu);

    /**
     * @brief Blocks for the closest 2D physics hit along a ray.
     * @note Configured manager debugDraw records the actual completed query.
     * @param ray Finite origin and direction; Box2D evaluates the XY plane.
     * A zero XY direction retains the legacy upward fallback.
     * @param outHit Receives the completed response. On PhysResult::Ok, inspect
     * RaycastHit::hit to distinguish a miss from a collision.
     * @param layerMask Physics collision layers eligible for the query.
     * @return Ok when the query completed, InvalidParam for non-finite input,
     * InvalidState when the manager is stopped, BackendError on timeout, or a
     * backend-reported PhysResult.
     * @framephase
     * Use for queries that require an immediate answer. Prefer async queries
     * when the result can be consumed in a later physics frame.
     * @threading
     * Call from the game/simulation thread, never from the physics worker; this
     * method waits for that worker to service the query mailbox.
     * @failure
     * outHit clears on failure and miss.
     */
    PhysResult raycastSync(const ayt::math::Ray& ray, RaycastHit& outHit,
                           PhysLayerMask layerMask = 0xFFFFFFFFu);
    /**
     * Collect all exact collider hits along a finite XY segment.
     * Origin/displacement must be finite; XY travel is [0.000001,10000].
     * Layer, sensor and up to 64 ignored-body filters apply. maxHits is
     * 1..1024; overflow returns OutOfRange with no partial list. Results are
     * sorted by distance, then body and collider. Output clears on failure or
     * miss. Null 2D returns Unsupported. Call from the game thread only.
     */
    PhysResult raycastAllSync(const RaycastAll2D& query,
                              std::vector<RaycastHit>& out);
    /**
     * Sweep a circle through the XY world and return its closest accepted hit.
     * Uses SphereSweep::origin/displacement XY; Z is ignored by Box2D. Radius
     * must be in (0, 1000], XY travel in [0.000001, 10000], and ignoredBodies may
     * contain at most 64 handles. Initial overlap has zero center travel and
     * a zero normal. Sensors are excluded unless includeSensors is true.
     * The output is cleared on every failure or miss. Like raycastSync, this
     * waits for the physics worker and must not be called from that worker.
     */
    PhysResult sphereSweepSync(const SphereSweep& sweep, RaycastHit& outHit);
    /**
     * @brief Collect every exact collider touched by a finite XY circle cast.
     * Uses SphereSweep geometry and filters; maxHits is 1..1024. Results sort
     * by center travel, body and collider. A valid miss is empty Ok; errors
     * clear output. Null 2D returns Unsupported. Waits for the worker.
     */
    PhysResult sphereSweepAllSync(const SphereSweep& sweep,
                                  std::vector<RaycastHit>& out,
                                  uint32_t maxHits = 64u);
    /**
     * @brief Sweep an axis-aligned XY box and return its closest accepted hit.
     * @param sweep Finite center, travel and positive XY half-extents; each
     * extent is at most 1000, XY travel is [0.000001, 10000]. At most 64 body
     * handles may be ignored. Z is validated but ignored by Box2D.
     * @param outHit Cleared on failure or miss. Distance is center travel;
     * starting contact reports zero. Sensors require includeSensors.
     * @return Ok for a completed query, InvalidParam for a bad request,
     * InvalidState when the manager is unavailable, BackendError on timeout,
     * or Unsupported on the Null 2D backend.
     * @threading Call from the game/simulation thread; waits for the physics
     * worker and must never be called from that worker.
     */
    PhysResult boxSweepSync(const BoxSweep2D& sweep, RaycastHit& outHit);
    /**
     * @brief Return all exact colliders touched by an axis-aligned XY box.
     * Uses BoxSweep2D geometry/filters; maxHits is 1..1024. Results are
     * sorted by center travel distance, body and collider. A completed miss
     * returns Ok with an empty vector. Any error, including OutOfRange when
     * the result limit is exceeded, clears output. Null 2D is Unsupported.
     * Call on the game thread; this waits for the physics worker.
     */
    PhysResult boxSweepAllSync(const BoxSweep2D& sweep,
                               std::vector<RaycastHit>& out,
                               uint32_t maxHits = 64u);
    /**
     * @brief Cast a fixed-angle rotated XY rectangle to its closest hit.
     * Finite center, displacement, angle and positive XY half-extents are
     * required. Each extent is at most 1000; XY travel is [0.000001,10000].
     * Layer, sensor and up to 64 ignored-body filters apply. Z is validated
     * but ignored. Distance is box-center travel, including zero on initial
     * overlap. Output is cleared on failure or miss; Null 2D is Unsupported.
     * Call on the game thread; the query waits for the physics worker.
     */
    PhysResult orientedBoxSweepSync(const OrientedBoxSweep2D& sweep,
                                    RaycastHit& outHit);
    /**
     * @brief Collect all exact hits from a fixed-angle rotated box cast.
     * Geometry and filters match orientedBoxSweepSync; maxHits is 1..1024.
     * Results sort by travel/body/collider; errors clear output, including
     * OutOfRange. Null 2D returns Unsupported. Waits for the worker.
     */
    PhysResult orientedBoxSweepAllSync(const OrientedBoxSweep2D& sweep,
                                       std::vector<RaycastHit>& out,
                                       uint32_t maxHits = 64u);
    /**
     * @brief Sweep an upright XY capsule and return the closest accepted hit.
     * @param sweep Finite center and travel; radius (0,1000], total height
     * [2*radius,2000], XY travel [0.000001,10000]. At most 64 body handles
     * may be ignored. Z is validated but ignored by Box2D.
     * @param outHit Cleared on failure or miss. Distance is capsule-center
     * travel; starting contact reports zero. Sensors require includeSensors.
     * @return Ok for a completed query, InvalidParam for a bad request,
     * InvalidState when the manager is unavailable, BackendError on timeout,
     * or Unsupported on the Null 2D backend.
     * @threading Call from the game/simulation thread; waits for the physics
     * worker and must never be called from that worker.
     */
    PhysResult capsuleSweepSync(const CapsuleSweep2D& sweep, RaycastHit& outHit);
    /**
     * @brief Collect all colliders touched by an upright XY capsule cast.
     * Geometry and filters match capsuleSweepSync; maxHits is 1..1024.
     * Results sort by travel/body/collider; errors clear output, including
     * OutOfRange. Null 2D returns Unsupported. Waits for the worker.
     */
    PhysResult capsuleSweepAllSync(const CapsuleSweep2D& sweep,
                                   std::vector<RaycastHit>& out,
                                   uint32_t maxHits = 64u);
    /**
     * @brief Find bodies overlapping an upright XY capsule at one center.
     * @param query Finite center, radius (0,1000], total height
     * [2*radius,2000], up to 64 ignored handles. Z is ignored by Box2D.
     * @param out Cleared on failure or miss; successful results are unique
     * generation-checked body handles in ascending order.
     * @return Ok even for an empty result, InvalidParam for a bad request,
     * InvalidState when the manager is unavailable, BackendError on timeout,
     * NoMemory on response allocation failure, or Unsupported on Null 2D.
     * @threading Call from the game/simulation thread; waits for the physics
     * worker and must never be called from that worker.
     */
    PhysResult overlapCapsuleSync(const CapsuleOverlap2D& query,
                                  std::vector<BodyHandle>& out);
    /**
     * @brief Return every exact collider overlapping an upright XY capsule.
     * A capsule with height == 2*radius is a circle. Geometry/filters match
     * overlapCapsuleSync; maxHits is 1..1024. Results sort by body/collider.
     * Errors, including OutOfRange, clear output; Null 2D is Unsupported.
     * Contact point and penetration depth are not reported. Waits for worker.
     */
    PhysResult overlapCapsuleCollidersSync(const CapsuleOverlap2D& query,
        std::vector<ColliderOverlapHit2D>& out, uint32_t maxHits = 64u);
    /**
     * Find bodies inside a counterclockwise rotated XY rectangle.
     * Finite center, angle and half-extents are required; XY extents are in
     * (0,1000]. Z is validated but ignored by Box2D. Layer, sensor and up to
     * 64 ignored-body filters apply. Results are unique ascending handles.
     * Output is cleared on failure or miss. Null 2D returns Unsupported.
     * Call from the game thread; this waits for the physics worker.
     */
    PhysResult overlapOrientedBoxSync(const OrientedBoxOverlap2D& query,
                                      std::vector<BodyHandle>& out);
    /**
     * @brief Return every exact collider inside a rotated XY rectangle.
     * Angle zero is an axis-aligned box. Geometry/filters match the body-only
     * overlap query; maxHits is 1..1024. Results sort by body/collider.
     * Errors clear output; Null 2D is Unsupported. No contact geometry is
     * inferred. Call from the game thread; waits for the physics worker.
     */
    PhysResult overlapOrientedBoxCollidersSync(
        const OrientedBoxOverlap2D& query,
        std::vector<ColliderOverlapHit2D>& out, uint32_t maxHits = 64u);
    /** Legacy body-only XY circle overlap. Finite center and positive finite
     * radius are required. Clears output on failure; a stopped manager returns
     * InvalidState. Valid Null 2D queries return Ok with empty output. */
    PhysResult overlapSphereSync(const ayt::math::FVector3& center, float radius,
                                 std::vector<BodyHandle>& out,
                                 PhysLayerMask layerMask = 0xFFFFFFFFu);
    // R1: axis-aligned box overlap (independent half-extents; no rotation).
    /** Legacy body-only axis-aligned XY box overlap. Center and half-extents
     * must be finite, with positive XY half-extents. Clears output on failure;
     * a stopped manager returns InvalidState. Valid Null 2D is Ok/empty. */
    PhysResult overlapBoxSync(const ayt::math::FVector3& center,
                              const ayt::math::FVector3& halfExtents,
                              std::vector<BodyHandle>& out,
                              PhysLayerMask layerMask = 0xFFFFFFFFu);

    // R2: set a dynamic/kinematic body's linear velocity directly (bypasses force/impulse).
    PhysResult setRigidbodyVelocity(BodyHandle h, const ayt::math::FVector3& v);
    // R6: runtime per-body gravity multiplier (0 = float, 1 = world default).
    PhysResult setGravityScale(BodyHandle h, float scale);
    // R10: runtime mass override (must be > 0; wakes the body).
    PhysResult setMass(BodyHandle h, float mass);
    // R10: runtime per-collider friction / restitution override.
    PhysResult setMaterial(ColliderHandle h, float friction, float restitution);

    void wakeAll();
    void sleepAll();

private:
    PhysicsWorld2D() = default;
    friend class PhysicsManager;
    PhysicsManager* _manager = nullptr;

    uint32_t _nextBodyIndex     = 0;
    uint32_t _nextBodyGen       = 0;
    uint32_t _nextColliderIndex = 0;
    uint32_t _nextColliderGen   = 0;
    uint32_t _nextJointIndex    = 0;
    uint32_t _nextJointGen      = 0;
    uint32_t _nextQueryId       = 0;

    template <typename Handle>
    Handle mintHandle(uint32_t& indexSlot, uint32_t& genSlot) noexcept;

    uint32_t nextQueryId() noexcept;
};

} // namespace ayt::physics
