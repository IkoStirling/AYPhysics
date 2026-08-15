#pragma once
// AYPhysics/IPhysicsBackend3D.h - 3D backend interface (§6.3)
//
// 3D-specific mutators are called from the physics thread only.
// World3D already allocated the handle (index + generation); backend binds
// its native body to that handle slot.

#include "AYPhysics/IPhysicsBackend.h"

namespace ayt::physics {

class IPhysicsBackend3D : public IPhysicsBackend {
public:
    // Initialise 3D world parameters (gravity, sub-step, capacity, etc.).
    virtual bool init3D(const PhysicsBackendDescriptor& desc) = 0;

    // Set / query gravity.
    virtual void setGravity(const ayt::math::FVector3& g) = 0;

    // 3D-specific mutators (called from execute() after decoding PhysicsCommand).
    // Implementations decode the compact cmd and look up the create payload via
    // createSlot when type == Create*.

    // Handle generation bookkeeping (§17.4) — backend maintains a generation table
    // sized to maxBodies and verifies every mutator before touching native body.
};

} // namespace ayt::physics