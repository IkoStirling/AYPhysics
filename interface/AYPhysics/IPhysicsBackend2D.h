#pragma once
// AYPhysics/IPhysicsBackend2D.h - 2D backend interface (§6.4)
//
// 2D-specific; mirror of IPhysicsBackend3D with FVector2 / 2D joints.
// R1 ships NullBackend2D only; Box2DBackend2D is gated on §4.2 decision.

#include "AYPhysics/IPhysicsBackend.h"

namespace ayt::physics {

class IPhysicsBackend2D : public IPhysicsBackend {
public:
    virtual bool init2D(const PhysicsBackendDescriptor& desc) = 0;
    virtual void setGravity(const ayt::math::FVector2& g) = 0;
};

} // namespace ayt::physics