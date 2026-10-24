#pragma once
// AYPhysics/IPhysicsQuery.h — narrow Host-facing physics query facade (§6 progressive)
//
// Gameplay should prefer host->physicsQuery() over PhysicsManager::* when only
// raycasts / overlaps are needed. Mutators stay on PhysicsManager / worlds.

#include "AYPhysics/PhysicsTypes.h"
#include "AYPhysics/PhysicsHandles.h"

#include "AYMath/MathGeometry.h"
#include "AYMath/MathTypes.h"

#include <vector>

namespace ayt::physics
{

class IPhysicsQuery {
public:
    virtual ~IPhysicsQuery() = default;

    /// False until a running PhysicsManager is attached and ready.
    virtual bool isReady() const = 0;

    /** Query on the game thread; waits for the physics-thread mailbox.
     * Jolt returns the exact generation-checked collider within a compound,
     * world-space surface normal and distance (up to 10000 world units).
     * A successful no-hit and every failure clear outHit. Finite origin and
     * nonzero finite direction are required;
     * the backend normalizes it. layerMask filters targets, not their collide masks.
     * Returned handles belong to this manager and expire on collider/body destruction. */
    virtual PhysResult raycastSync(const ayt::math::Ray& ray,
                                   RaycastHit& outHit,
                                   PhysLayerMask layerMask = 0xFFFFFFFFu) = 0;

    /// Return the nearest hit except one generation-checked body handle.
    /// Backends without this query return Unsupported and clear outHit.
    virtual PhysResult raycastSyncIgnoringBody(const ayt::math::Ray& ray,
        RaycastHit& outHit, PhysLayerMask layerMask, BodyHandle ignoredBody) {
        if (ignoredBody == InvalidBodyHandle) return raycastSync(ray, outHit, layerMask);
        outHit = {};
        return PhysResult::Unsupported;
    }

    /// Physics-thread sphere cast. Jolt reports exact collider and center travel distance.
    /// Initial penetration reports distance 0; failures clear outHit. Unsupported in Null/Mock/2D.
    virtual PhysResult sphereSweepSync(const SphereSweep&, RaycastHit& outHit) {
        outHit = {}; return PhysResult::Unsupported;
    }
    /// Finite center and positive finite radius are required. Failure and
    /// a successful empty result clear out. Returns body handles only.
    virtual PhysResult overlapSphereSync(const ayt::math::FVector3& center,
                                         float radius,
                                         std::vector<BodyHandle>& out,
                                         PhysLayerMask layerMask = 0xFFFFFFFFu) = 0;

    /// Current-world oriented 3D box overlap. Returns body handles only.
    /// Failure and a successful empty result clear out. Backends without this
    /// query return Unsupported so callers can reject placement safely.
    virtual PhysResult overlapBoxSync(const ayt::math::FVector3&,
                                      const ayt::math::FVector3&,
                                      const ayt::math::FQuaternion&,
                                      std::vector<BodyHandle>& out,
                                      PhysLayerMask = 0xFFFFFFFFu) {
        out.clear(); return PhysResult::Unsupported;
    }
};

} // namespace ayt::physics
