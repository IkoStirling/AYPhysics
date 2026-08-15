#pragma once
// Test_Box2DBackend2D_Helpers.h - shared helpers for Box2DBackend2D tests.

#include "AYPhysics/PhysicsManager.h"
#include "AYPhysics/PhysicsWorld2D.h"
#include "AYPhysics/PhysicsTypes.h"

#include "AYTime/Clock.h"
#include "AYPlatform/Thread.h"

#include <cstdint>
#include <memory>

namespace ayt::physics {
namespace test_helpers {

inline void waitForDrain2D(PhysicsManager& m, int ms) {
    const uint64_t startUs = ayt::time::Clock::performanceNowUs();
    while (ayt::time::Clock::performanceNowUs() - startUs <
           static_cast<uint64_t>(ms) * 1000u) {
        (void)m.fetchResults();
        ayt::platform::Thread::sleep(0.001f);
    }
}

#if defined(AYPHYSICS_HAS_BOX2D)
inline std::unique_ptr<PhysicsManager> makeBox2DMgr(uint32_t maxBodies = 256u) {
    PhysicsBackendDescriptor desc;
    desc.kind3D               = BackendKind::Null;
    desc.kind2D               = BackendKind::DefaultBox2D;
    desc.commandQueueCapacity = 1024;
    desc.createPoolCapacity   = 256;
    desc.maxBodies            = maxBodies;
    desc.gravity2D            = ayt::math::FVector2(0.0f, -9.81f);
    return PhysicsManager::create(desc);
}
#endif

} // namespace test_helpers
} // namespace ayt::physics
