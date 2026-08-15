#pragma once
// AYPhysics.h v0.4.0 - umbrella include for AYPhysics (Jolt 3D + Box2D 2D + Box2DBridge 2D)

#include "AYPhysics/PhysicsTypes.h"
#include "AYPhysics/PhysicsHandles.h"
#include "AYPhysics/PhysicsCommandQueue.h"
#include "AYPhysics/PhysicsWorld3D.h"
#include "AYPhysics/PhysicsWorld2D.h"
#include "AYPhysics/PhysicsManager.h"
#include "AYPhysics/PhysicsSubSystem.h"
#include "AYPhysics/IPhysicsQuery.h"
// IPhysicsBackend* are reachable via the Manager; do not include here to
// avoid dragging backend-specific headers into consumers.
// #include "AYPhysics/IPhysicsBackend.h"
// #include "AYPhysics/IPhysicsBackend3D.h"
// #include "AYPhysics/IPhysicsBackend2D.h"
// #include "AYPhysics/PhysicsScene.h"

// Effects placeholders (R3+): no .cpp until F-1/F-2/F-3.
// #include "AYPhysicsCloth.h"
// #include "AYPhysicsFluid.h"
// #include "AYPhysicsParticle.h"

// Test-only inspection: include explicitly in test TUs only.
// #include "AYPhysics/PhysicsBackendTestAccess.h"