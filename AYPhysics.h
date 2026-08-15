#pragma once
// AYPhysics.h v0.4.0 - umbrella include for AYPhysics (Jolt 3D + Box2D 2D + Box2DBridge 2D)

#include "AYPhysicsTypes.h"
#include "AYPhysicsHandles.h"
#include "AYPhysicsCommandQueue.h"
#include "AYPhysicsWorld3D.h"
#include "AYPhysicsWorld2D.h"
#include "AYPhysicsManager.h"
#include "AYPhysicsSubSystem.h"
#include "IPhysicsQuery.h"
// IPhysicsBackend* are reachable via the Manager; do not include here to
// avoid dragging backend-specific headers into consumers.
// #include "IPhysicsBackend.h"
// #include "IPhysicsBackend3D.h"
// #include "IPhysicsBackend2D.h"
// #include "PhysicsScene.h"

// Effects placeholders (R3+): no .cpp until F-1/F-2/F-3.
// #include "AYPhysicsCloth.h"
// #include "AYPhysicsFluid.h"
// #include "AYPhysicsParticle.h"

// Test-only inspection: include explicitly in test TUs only.
// #include "AYPhysicsBackendTestAccess.h"