#include "AYPhysics/PhysicsQueryAdapter.h"

#include "AYPhysics/PhysicsManager.h"
#include "AYPhysics/PhysicsWorld3D.h"

namespace ayt::physics
{

bool PhysicsQueryAdapter::isReady() const
{
    return _manager != nullptr && _manager->world3D() != nullptr;
}

PhysResult PhysicsQueryAdapter::raycastSync(const ayt::math::Ray& ray,
                                            RaycastHit& outHit,
                                            PhysLayerMask layerMask)
{
    if (!isReady()) {
        return PhysResult::InvalidState;
    }
    return _manager->world3D()->raycastSync(ray, outHit, layerMask);
}

PhysResult PhysicsQueryAdapter::overlapSphereSync(const ayt::math::FVector3& center,
                                                  float radius,
                                                  std::vector<BodyHandle>& out,
                                                  PhysLayerMask layerMask)
{
    if (!isReady()) {
        return PhysResult::InvalidState;
    }
    return _manager->world3D()->overlapSphereSync(center, radius, out, layerMask);
}

} // namespace ayt::physics
