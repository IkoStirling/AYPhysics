#pragma once
// AYPhysics/PhysicsBackendTestAccess.h - test-only inspection (mirror AYAudio)
//
// Provides read-only views into MockBackend3D state for unit tests.
// Production code MUST NOT include this header; see CLAUDE.md "新增模块要求".

#include "AYPhysics/PhysicsTypes.h"
#include "AYPhysics/PhysicsCommandQueue.h"

#include <vector>

namespace ayt::physics {

class MockBackend3D;  // forward declaration for setCurrentMock parameter type

namespace testaccess {

// Returns a snapshot of every compact command MockBackend3D has executed since
// start() or last reset. Order = execution order. Tests verify queue drain order.
const std::vector<PhysicsCommand>& mockBackendCommands();

// Returns create-pool payload sequence captured by MockBackend3D.
const std::vector<PhysicsCreatePayload>& mockBackendCreatePayloads();

// Returns the last queryId issued by raycastAsync/overlapAsync on the given world.
uint32_t lastIssuedQueryId();

// Reset all captured state (call at start of each test).
void reset();

// Internal: called by PhysicsManager when Mock backend is selected; do not call.
// Parameter type uses the global MockBackend3D (defined in backend/MockBackend3D.h).
void setCurrentMock(::ayt::physics::MockBackend3D* m);

} // namespace testaccess
} // namespace ayt::physics