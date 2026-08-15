#pragma once
// AYPhysics/PhysicsQueryAdapter.h — IPhysicsQuery over PhysicsManager::world3D()

#include "AYPhysics/IPhysicsQuery.h"

namespace ayt::physics
{

class PhysicsManager;

class PhysicsQueryAdapter final : public IPhysicsQuery {
public:
    void bind(PhysicsManager* manager) noexcept { _manager = manager; }
    PhysicsManager* manager() const noexcept { return _manager; }

    bool isReady() const override;

    PhysResult raycastSync(const ayt::math::Ray& ray,
                           RaycastHit& outHit,
                           PhysLayerMask layerMask) override;

    PhysResult overlapSphereSync(const ayt::math::FVector3& center,
                                 float radius,
                                 std::vector<BodyHandle>& out,
                                 PhysLayerMask layerMask) override;

private:
    PhysicsManager* _manager = nullptr;
};

} // namespace ayt::physics
