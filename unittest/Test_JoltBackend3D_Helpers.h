#pragma once
// Test_JoltBackend3D_Helpers.h - shared helpers for JoltBackend3D test TUs.
//
// Header-only inline helpers shared between Test_JoltBackend3D.cpp (R1.5b
// baseline) and Test_JoltBackend3D_AdvancedShapes.cpp (R2.0a). All helpers
// are inline so each TU gets its own copy and stays MSVC-include-order-safe
// per CLAUDE.md "every Test_*.cpp is an independent TU".
//
// Gating: helpers are always available (no AYPHYSICS_HAS_JOLT guard) so the
// Mock-only round-trip case can use them too.

#include "AYPhysicsManager.h"
#include "AYPhysicsWorld3D.h"
#include "AYPhysicsTypes.h"

#include "AYTime/Clock.h"
#include "AYPlatform/Thread.h"

#include <memory>
#include <cstdint>

namespace ayt::physics {
namespace test_helpers {

// Spin briefly so the physics thread drains enqueued commands / publishes the
// next snapshot. Mirrors Test_MockBackend3D.cpp helpers. AYTime v1.2 value-type
// rules: Clock::performanceNowUs() static method, no IClock*.
inline void waitForDrain(PhysicsManager& m, int ms) {
    const uint64_t startUs = ayt::time::Clock::performanceNowUs();
    while (ayt::time::Clock::performanceNowUs() - startUs <
           static_cast<uint64_t>(ms) * 1000u) {
        (void)m.fetchResults();
        ayt::platform::Thread::sleep(0.001f);
    }
}

// Convenience: create a manager wired to JoltBackend3D with sane defaults.
// Used by all real-backend tests; guarded so it compiles in Jolt-OFF builds.
#if defined(AYPHYSICS_HAS_JOLT)
inline std::unique_ptr<PhysicsManager> makeJoltMgr(uint32_t maxBodies = 256u) {
    PhysicsBackendDescriptor desc;
    desc.kind3D                 = BackendKind::DefaultJolt;
    desc.commandQueueCapacity   = 1024;
    desc.createPoolCapacity     = 256;
    desc.maxBodies              = maxBodies;
    desc.maxBodyPairs           = maxBodies * 4u;
    desc.maxContactConstraints  = maxBodies * 4u;
    desc.gravity3D              = ayt::math::FVector3(0.0f, -9.81f, 0.0f);
    return PhysicsManager::create(desc);
}
#endif

} // namespace test_helpers
} // namespace ayt::physics