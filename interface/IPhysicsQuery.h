#pragma once
// IPhysicsQuery.h — narrow Host-facing physics query facade (§6 progressive)
//
// Gameplay should prefer host->physicsQuery() over PhysicsManager::* when only
// raycasts / overlaps are needed. Mutators stay on PhysicsManager / worlds.

#include "AYPhysicsTypes.h"
#include "AYPhysicsHandles.h"

#include "aymath/MathGeometry.h"
#include "aymath/MathTypes.h"

#include <vector>

namespace ayt::physics
{

class IPhysicsQuery {
public:
    virtual ~IPhysicsQuery() = default;

    /// False until a PhysicsManager is attached and ready.
    virtual bool isReady() const = 0;

    virtual PhysResult raycastSync(const ayt::math::Ray& ray,
                                   RaycastHit& outHit,
                                   PhysLayerMask layerMask = 0xFFFFFFFFu) = 0;

    virtual PhysResult overlapSphereSync(const ayt::math::FVector3& center,
                                         float radius,
                                         std::vector<BodyHandle>& out,
                                         PhysLayerMask layerMask = 0xFFFFFFFFu) = 0;
};

} // namespace ayt::physics
