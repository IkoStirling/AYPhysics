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
    PhysResult raycastSyncIgnoringBody(const ayt::math::Ray& ray,
        RaycastHit& outHit, PhysLayerMask layerMask, BodyHandle ignoredBody) override;
    PhysResult raycastAll3DSync(const RaycastAll3D& query,
        std::vector<RaycastHit>& out) override;

    PhysResult sphereSweepSync(const SphereSweep& sweep, RaycastHit& outHit) override;

    PhysResult overlapSphereSync(const ayt::math::FVector3& center,
                                 float radius,
                                 std::vector<BodyHandle>& out,
                                 PhysLayerMask layerMask) override;
    PhysResult overlapBoxSync(const ayt::math::FVector3& center,
                              const ayt::math::FVector3& halfExtents,
                              const ayt::math::FQuaternion& rotation,
                              std::vector<BodyHandle>& out,
                              PhysLayerMask layerMask) override;

private:
    PhysicsManager* _manager = nullptr;
};

} // namespace ayt::physics
