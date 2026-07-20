#pragma once
// AYPhysicsWorld2D.h - 2D world public API (§9, R1 placeholder)
//
// R1 ships NullBackend2D only; design §4.2 2D decision (Box2D vs Jolt-2D) is
// a R1.5 gate. Until then all create/destroy/query operations are no-ops that
// return PhysResult::Unsupported on real work and Ok on no-op creates.

#include "AYPhysicsTypes.h"
#include "AYPhysicsHandles.h"

#include <vector>

namespace ayt::physics {

class PhysicsManager;
class PhysicsWorld2D {
public:
    // R1.5+: backend-supplied impl; for now returns Unsupported when would-be work.
    PhysResult createRigidbody(const RigidbodyDesc& desc, BodyHandle& outHandle);  // 2D desc subset (pos.x,pos.y)
    PhysResult destroyRigidbody(BodyHandle h);
    PhysResult createCollider(const ColliderDesc& desc, ColliderHandle& outHandle);
    PhysResult destroyCollider(ColliderHandle h);
    PhysResult raycastSync(const ayt::math::Ray& ray, RaycastHit& outHit, PhysLayerMask layerMask);

private:
    PhysicsWorld2D() = default;
    friend class PhysicsManager;
    PhysicsManager* _manager = nullptr;
};

} // namespace ayt::physics