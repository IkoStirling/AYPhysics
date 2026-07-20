#include "AYPhysicsWorld2D.h"

#include "AYPhysicsManager.h"

namespace ayt::physics {

PhysResult PhysicsWorld2D::createRigidbody(const RigidbodyDesc& desc, BodyHandle& outHandle) {
    outHandle = InvalidBodyHandle;
    (void)desc;
    return PhysResult::Unsupported;  // R1.5 gate
}

PhysResult PhysicsWorld2D::destroyRigidbody(BodyHandle h) {
    if (!isValidHandle(h)) return PhysResult::InvalidParam;
    return PhysResult::Unsupported;
}

PhysResult PhysicsWorld2D::createCollider(const ColliderDesc& desc, ColliderHandle& outHandle) {
    outHandle = InvalidColliderHandle;
    (void)desc;
    return PhysResult::Unsupported;
}

PhysResult PhysicsWorld2D::destroyCollider(ColliderHandle h) {
    if (!isValidHandle(h)) return PhysResult::InvalidParam;
    return PhysResult::Unsupported;
}

PhysResult PhysicsWorld2D::raycastSync(const ayt::math::Ray& ray, RaycastHit& outHit,
                                       PhysLayerMask layerMask) {
    (void)ray; (void)outHit; (void)layerMask;
    return PhysResult::Unsupported;
}

} // namespace ayt::physics