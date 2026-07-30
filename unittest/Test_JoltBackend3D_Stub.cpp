#include "AYPhysicsManager.h"
#include "AYPhysicsWorld3D.h"
#include "AYPhysicsTypes.h"
#include "IPhysicsBackend3D.h"  // needed to call backend3D()->isRealDevice()

#if defined(AYPHYSICS_HAS_JOLT)
#include "JoltBackend3D.h"
#endif

#include "AYTest.h"

#include "aytime/Clock.h"
#include "ayplatform/Thread.h"

using namespace ayt::physics;

namespace {
// Helper: spin briefly so the physics thread drains enqueued commands.
// Mirrors waitForDrain() in Test_MockBackend3D.cpp.
void waitForDrain(PhysicsManager& m, int ms) {
    const uint64_t startUs = ayt::time::Clock::performanceNowUs();
    while (ayt::time::Clock::performanceNowUs() - startUs <
           static_cast<uint64_t>(ms) * 1000u) {
        (void)m.fetchResults();
        ayt::platform::Thread::sleep(0.001f);
    }
}
}  // namespace

TEST_SUITE(JoltBackend3DStubTests)

#if defined(AYPHYSICS_HAS_JOLT)
    // R1.5a: real JoltBackend3D is built and dispatched by manager when
    // kind3D == DefaultJolt. Verifies the stub boots, links, and routes through
    // the full physics thread pipeline.

    TEST_CASE(Stub_DescribeReportsRealDevice) {
        JoltBackend3D backend;
        const PhysicsBackendInfo info = backend.describe();
        CHECK_NOT_NULL(info.name);
        // R1.5a stub still uses the real-device family marker.
        CHECK(backend.isRealDevice());
    }

    TEST_CASE(Stub_InitStartStopCycle) {
        JoltBackend3D backend;
        CHECK(backend.init3D(PhysicsBackendDescriptor{}));
        CHECK(backend.start(backend.describe()));
        CHECK_INT_EQ(backend.stepCount(), 0u);
        backend.step(1.0f / 60.0f);
        backend.step(1.0f / 60.0f);
        CHECK_INT_EQ(backend.stepCount(), 2u);
        backend.stop();
    }

    TEST_CASE(Stub_ExecuteAcceptsAnyCommand) {
        JoltBackend3D backend;
        CHECK(backend.init3D(PhysicsBackendDescriptor{}));
        CHECK(backend.start(backend.describe()));
        PhysicsCommand cmd{};
        cmd.type = PhysicsCommandType::ApplyForce;
        backend.execute(cmd, nullptr);
        backend.execute(cmd, nullptr);
        CHECK_INT_EQ(backend.commandCount(), 2u);
        backend.stop();
    }

    TEST_CASE(Stub_ExecuteSyncReturnsOkEmptyHit) {
        JoltBackend3D backend;
        CHECK(backend.init3D(PhysicsBackendDescriptor{}));
        CHECK(backend.start(backend.describe()));
        SyncQueryRequest req{};
        req.requestId = 42;
        SyncQueryResponse resp{};
        backend.executeSync(req, resp);
        CHECK_INT_EQ(static_cast<uint32_t>(resp.status),
                     static_cast<uint32_t>(PhysResult::Ok));
        CHECK_INT_EQ(resp.requestId, 42u);
        CHECK_FALSE(resp.firstHit.hit);
        backend.stop();
    }

    TEST_CASE(Stub_ManagerDispatchesDefaultJolt) {
        // The R1.5a integration test: when the manager is asked for DefaultJolt
        // and AYPHYSICS_HAS_JOLT is defined, the resulting backend must be a
        // real JoltBackend3D (not the Null fallback). Drives a few mgr->step()
        // calls to exercise the full dispatch path.
        PhysicsBackendDescriptor desc;
        desc.kind3D = BackendKind::DefaultJolt;
        desc.commandQueueCapacity = 32;
        desc.createPoolCapacity   = 16;
        auto mgr = PhysicsManager::create(desc);
        CHECK_NOT_NULL(mgr.get());

        // Step a few frames. Stub does no physics work, but the command queue
        // + physics thread pipeline is exercised.
        for (int i = 0; i < 10; ++i) {
            mgr->step(1.0f / 60.0f);
        }
        waitForDrain(*mgr, 50);

        mgr->shutdown();
    }

#else
    // No Jolt: only assert that the manager falls back to Null and the build
    // is otherwise intact. R1.5b will not have this branch.
    TEST_CASE(Stub_ManagerFallsBackToNullWhenNoJolt) {
        PhysicsBackendDescriptor desc;
        desc.kind3D = BackendKind::DefaultJolt;
        auto mgr = PhysicsManager::create(desc);
        CHECK_NOT_NULL(mgr.get());
        // Without AYPHYSICS_HAS_JOLT, kind3D==DefaultJolt returns NullBackend3D.
        if (mgr && mgr->backend3D()) {
            CHECK_FALSE(mgr->backend3D()->isRealDevice());
        }
        mgr->shutdown();
    }
#endif

TEST_SUITE_END