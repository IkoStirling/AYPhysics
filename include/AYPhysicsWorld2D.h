#pragma once
// AYPhysicsWorld2D.h - 2D world public API (§9, R2.5 Box2D)

#include "AYPhysicsTypes.h"
#include "AYPhysicsHandles.h"

#include <cstdint>
#include <vector>

namespace ayt::physics {

class PhysicsManager;
class PhysicsWorld2D {
public:
    PhysResult createRigidbody(const RigidbodyDesc& desc, BodyHandle& outHandle);
    PhysResult destroyRigidbody(BodyHandle h);
    PhysResult setRigidbodyTransform(BodyHandle h, const ayt::math::FVector3& p,
                                     const ayt::math::FQuaternion& r);
    PhysResult applyForce(BodyHandle h, const ayt::math::FVector3& f);
    PhysResult applyImpulse(BodyHandle h, const ayt::math::FVector3& impulse,
                            const ayt::math::FVector3& point);

    // P3: runtime toggle of a body's collide mask. Used by the bridge character
    // controller to phase through one-way platforms while rising. The new mask
    // replaces RigidbodyDesc::collideMask for all shapes on the body.
    PhysResult setRigidbodyCollideMask(BodyHandle h, PhysLayerMask newMask);

    PhysResult createCollider(const ColliderDesc& desc, ColliderHandle& outHandle);
    PhysResult destroyCollider(ColliderHandle h);

    PhysResult createJoint(const JointDesc& desc, JointHandle& outHandle);
    PhysResult destroyJoint(JointHandle h);

    uint32_t   raycastAsync(const ayt::math::Ray& ray, PhysLayerMask layerMask = 0xFFFFFFFFu);
    uint32_t   overlapSphereAsync(const ayt::math::FVector3& center, float radius,
                                  PhysLayerMask layerMask = 0xFFFFFFFFu);
    // R1: axis-aligned box overlap (independent half-extents; no rotation).
    uint32_t   overlapBoxAsync(const ayt::math::FVector3& center,
                               const ayt::math::FVector3& halfExtents,
                               PhysLayerMask layerMask = 0xFFFFFFFFu);

    PhysResult raycastSync(const ayt::math::Ray& ray, RaycastHit& outHit,
                           PhysLayerMask layerMask = 0xFFFFFFFFu);
    PhysResult overlapSphereSync(const ayt::math::FVector3& center, float radius,
                                 std::vector<BodyHandle>& out,
                                 PhysLayerMask layerMask = 0xFFFFFFFFu);
    // R1: axis-aligned box overlap (independent half-extents; no rotation).
    PhysResult overlapBoxSync(const ayt::math::FVector3& center,
                              const ayt::math::FVector3& halfExtents,
                              std::vector<BodyHandle>& out,
                              PhysLayerMask layerMask = 0xFFFFFFFFu);

    // R2: set a dynamic/kinematic body's linear velocity directly (bypasses force/impulse).
    PhysResult setRigidbodyVelocity(BodyHandle h, const ayt::math::FVector3& v);

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
