#pragma once
// AYPhysicsWorld3D.h - 3D world public API (§9)
//
// Game-thread enqueue. All mutators return PhysResult and (for create*) write a
// new handle into an out-parameter. On failure the out-param is set to Invalid.
// Use query dual-path (§5.1): *Async returns queryId (result next fetchResults),
// *Sync blocks on the mailbox (same-frame).

#include "AYPhysicsTypes.h"
#include "AYPhysicsHandles.h"

#include <cstdint>
#include <vector>

namespace ayt::physics {

class PhysicsManager;
class PhysicsWorld3D {
public:
    PhysResult createRigidbody(const RigidbodyDesc& desc, BodyHandle& outHandle);
    PhysResult    destroyRigidbody(BodyHandle h);
    PhysResult    setRigidbodyTransform(BodyHandle h, const ayt::math::FVector3& p, const ayt::math::FQuaternion& r);
    PhysResult    applyForce(BodyHandle h, const ayt::math::FVector3& f);
    PhysResult    applyImpulse(BodyHandle h, const ayt::math::FVector3& impulse, const ayt::math::FVector3& point);

    // R9: runtime toggle of a body's collide mask (symmetric with PhysicsWorld2D).
    // Replaces RigidbodyDesc::collideMask for the body's CollisionGroup SubGroupID;
    // LayerMaskGroupFilter then re-evaluates contacts on the next step.
    PhysResult    setRigidbodyCollideMask(BodyHandle h, PhysLayerMask newMask);

    PhysResult    createCollider(const ColliderDesc& desc, ColliderHandle& outHandle);
    PhysResult    destroyCollider(ColliderHandle h);

    PhysResult    createJoint(const JointDesc& desc, JointHandle& outHandle);
    PhysResult    destroyJoint(JointHandle h);

    // Async queries (result in next fetchResults() under returned queryId).
    uint32_t      raycastAsync(const ayt::math::Ray& ray, PhysLayerMask layerMask = 0xFFFFFFFFu);
    uint32_t      overlapSphereAsync(const ayt::math::FVector3& center, float radius, PhysLayerMask layerMask);
    uint32_t      overlapBoxAsync(const ayt::math::FVector3& center, const ayt::math::FVector3& halfExtents,
                                  const ayt::math::FQuaternion& rot, PhysLayerMask layerMask);

    // Sync queries (block game thread on mailbox).
    PhysResult    raycastSync(const ayt::math::Ray& ray, RaycastHit& outHit, PhysLayerMask layerMask = 0xFFFFFFFFu);
    PhysResult    overlapSphereSync(const ayt::math::FVector3& center, float radius,
                                    std::vector<BodyHandle>& out, PhysLayerMask layerMask);

    void wakeAll();
    void sleepAll();

private:
    PhysicsWorld3D() = default;
    friend class PhysicsManager;
    // Back-pointer to owning manager; lifetime guaranteed by PhysicsManager.
    PhysicsManager* _manager = nullptr;

    // Per-instance handle mints (§7.1). One pair per kind keeps BodyHandle /
    // ColliderHandle / JointHandle in independent name spaces so a body and
    // a collider can share a numeric handle without ambiguity at the API layer.
    // Game thread only — no atomic needed (single producer on the World).
    // Generation wraps 4095 -> 1 (0 reserved for invalid).
    uint32_t _nextBodyIndex      = 0;
    uint32_t _nextBodyGen        = 0;
    uint32_t _nextColliderIndex  = 0;
    uint32_t _nextColliderGen    = 0;
    uint32_t _nextJointIndex     = 0;
    uint32_t _nextJointGen       = 0;

    // Monotonic query id (also game-thread-only — opaque to backend). Must be
    // != 0 (the API contract returns 0 on error).
    uint32_t _nextQueryId        = 0;

    // Helper: mint a fresh (idx, gen) handle of any kind.
    template <typename Handle>
    Handle mintHandle(uint32_t& indexSlot, uint32_t& genSlot) noexcept;

    uint32_t nextQueryId() noexcept;
};

} // namespace ayt::physics