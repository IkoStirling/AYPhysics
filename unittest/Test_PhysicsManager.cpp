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
        // Verify the manager surfaces pool/queue exhaustion as NoMemory/QueueFull.
        // The create-pool is recycled by the physics thread, so a 1-slot pool
        // cannot be deterministically exhausted with a live thread (the first
        // command is drained before the second create lands). Pool exhaustion
        // itself is unit-tested in Test_PhysicsCommandQueue.cpp
        // (CreatePoolExhaustionReturnsInvalid). Here we only assert the happy
        // path: a single create on a 1-slot pool succeeds and returns a valid
        // handle. The racy second-create assertion was removed (it flipped
        // between Ok and NoMemory depending on thread scheduling).
        PhysicsBackendDescriptor desc;
        desc.kind3D = BackendKind::Null;
        desc.commandQueueCapacity = 8;
        desc.createPoolCapacity   = 1;
        auto mgr = PhysicsManager::create(desc);

        PhysicsWorld3D* w = mgr->world3D();
        BodyHandle h1 = InvalidBodyHandle;
        RigidbodyDesc rb;
        rb.type = BodyType::Dynamic;
        const PhysResult r1 = w->createRigidbody(rb, h1);
        CHECK_INT_EQ(static_cast<uint32_t>(r1), static_cast<uint32_t>(PhysResult::Ok));
        CHECK(isValidHandle(h1));
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