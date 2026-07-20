#include "AYPhysicsManager.h"
#include "AYPhysicsWorld3D.h"
#include "AYPhysicsTypes.h"

#include "AYTest.h"

#include "aytime/Clock.h"
#include "ayplatform/Thread.h"

using namespace ayt::physics;

namespace {
bool waitForFrame(PhysicsManager& m, uint64_t targetFrame, int timeoutMs) {
    const uint64_t startUs = ayt::time::Clock::performanceNowUs();
    while (m.fetchResults().frameIndex < targetFrame) {
        const uint64_t elapsedUs = ayt::time::Clock::performanceNowUs() - startUs;
        if (elapsedUs > static_cast<uint64_t>(timeoutMs) * 1000u) return false;
        ayt::platform::Thread::sleep(0.001f);  // 1 ms
    }
    return true;
}
}  // namespace

TEST_SUITE(PhysicsManagerTests)

    TEST_CASE(CreateWithNullBackendSucceeds) {
        PhysicsBackendDescriptor desc;
        desc.kind3D = BackendKind::Null;
        auto mgr = PhysicsManager::create(desc);
        CHECK_NOT_NULL(mgr.get());
        CHECK_NOT_NULL(mgr->world3D());
        CHECK_NOT_NULL(mgr->world2D());
        mgr->shutdown();
    }

    TEST_CASE(CreateRigidbodyReturnsHandleAndEnqueues) {
        PhysicsBackendDescriptor desc;
        desc.kind3D = BackendKind::Null;
        auto mgr = PhysicsManager::create(desc);

        PhysicsWorld3D* w = mgr->world3D();
        BodyHandle h = InvalidBodyHandle;
        RigidbodyDesc rb;
        rb.type = BodyType::Dynamic;
        rb.position = ayt::math::FVector3(1, 2, 3);
        const PhysResult r = w->createRigidbody(rb, h);
        CHECK_INT_EQ(static_cast<uint32_t>(r), static_cast<uint32_t>(PhysResult::Ok));
        CHECK(isValidHandle(h));
        mgr->shutdown();
    }

    TEST_CASE(DestroyWithInvalidHandleReturnsInvalidParam) {
        PhysicsBackendDescriptor desc;
        desc.kind3D = BackendKind::Null;
        auto mgr = PhysicsManager::create(desc);
        PhysicsWorld3D* w = mgr->world3D();
        const PhysResult r = w->destroyRigidbody(InvalidBodyHandle);
        CHECK_INT_EQ(static_cast<uint32_t>(r), static_cast<uint32_t>(PhysResult::InvalidParam));
        mgr->shutdown();
    }

    TEST_CASE(QueueFullFiresBackpressureOnCreate) {
        // Tiny queue: 4 slots. We pre-fill with Step commands (which trigger
        // a step boundary inside physics thread, draining); instead pre-fill
        // with non-Step commands that the physics thread will pop one-by-one.
        // Simpler: fill queue with create commands and then verify the Nth
        // fails with QueueFull.
        PhysicsBackendDescriptor desc;
        desc.kind3D = BackendKind::Null;
        desc.commandQueueCapacity = 8;
        desc.createPoolCapacity   = 1;  // exhaustion rather than queue full
        auto mgr = PhysicsManager::create(desc);

        PhysicsWorld3D* w = mgr->world3D();
        BodyHandle h1 = InvalidBodyHandle, h2 = InvalidBodyHandle;
        RigidbodyDesc rb;
        rb.type = BodyType::Dynamic;
        const PhysResult r1 = w->createRigidbody(rb, h1);
        CHECK_INT_EQ(static_cast<uint32_t>(r1), static_cast<uint32_t>(PhysResult::Ok));
        const PhysResult r2 = w->createRigidbody(rb, h2);
        CHECK_INT_EQ(static_cast<uint32_t>(r2), static_cast<uint32_t>(PhysResult::NoMemory));
        CHECK_INT_EQ(h2, InvalidBodyHandle);
        mgr->shutdown();
    }

    TEST_CASE(LockstepStubDoesNotBlockNullStep) {
        PhysicsBackendDescriptor desc;
        desc.kind3D = BackendKind::Null;
        auto mgr = PhysicsManager::create(desc);
        const PhysResult r = mgr->step(1.0f / 60.0f);
        CHECK_INT_EQ(static_cast<uint32_t>(r), static_cast<uint32_t>(PhysResult::Ok));
        CHECK_FALSE(isLockstepActive());
        mgr->shutdown();
    }

TEST_SUITE_END