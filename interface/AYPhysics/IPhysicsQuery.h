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

    /// False until a PhysicsManager is attached and ready.
    virtual bool isReady() const = 0;

    /** Query on the game thread; waits for the physics-thread mailbox.
     * Jolt returns the exact generation-checked collider within a compound,
     * world-space surface normal and distance (up to 10000 world units).
     * A successful no-hit has hit=false. Finite nonzero direction is required;
     * the backend normalizes it. layerMask filters targets, not their collide masks.
     * Returned handles belong to this manager and expire on collider/body destruction. */
    virtual PhysResult raycastSync(const ayt::math::Ray& ray,
                                   RaycastHit& outHit,
                                   PhysLayerMask layerMask = 0xFFFFFFFFu) = 0;

    virtual PhysResult overlapSphereSync(const ayt::math::FVector3& center,
                                         float radius,
                                         std::vector<BodyHandle>& out,
                                         PhysLayerMask layerMask = 0xFFFFFFFFu) = 0;
};

} // namespace ayt::physics
