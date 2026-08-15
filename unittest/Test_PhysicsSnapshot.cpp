#include "AYPhysics/PhysicsManager.h"
#include "AYPhysics/PhysicsWorld3D.h"
#include "AYPhysics/PhysicsTypes.h"

#include "AYTest.h"

#include "AYTime/Clock.h"
#include "AYPlatform/Thread.h"

using namespace ayt::physics;

namespace {
// Block until front-snapshot frameIndex advances, with timeout.
bool waitForFrame(PhysicsManager& m, uint64_t targetFrame, int timeoutMs) {
    const uint64_t startUs = ayt::time::Clock::performanceNowUs();
    while (m.fetchResults().frameIndex < targetFrame) {
        const uint64_t elapsedUs = ayt::time::Clock::performanceNowUs() - startUs;
        if (elapsedUs > static_cast<uint64_t>(timeoutMs) * 1000u) {
            return false;
        }
        ayt::platform::Thread::sleep(0.001f);  // 1 ms
    }
    return true;
}
}  // namespace

TEST_SUITE(PhysicsSnapshotTests)

    TEST_CASE(SnapshotStartsEmptyInNullMode) {
        PhysicsBackendDescriptor desc;
        desc.kind3D = BackendKind::Null;
        auto mgr = PhysicsManager::create(desc);
        CHECK_NOT_NULL(mgr.get());
        mgr->step(1.0f / 60.0f);
        const PhysFrameSnapshot& s = mgr->fetchResults();
        CHECK_INT_EQ(s.frameIndex, 0u);  // before any drain
        CHECK_INT_EQ(static_cast<uint32_t>(s.transforms.size()), 0u);
        CHECK_INT_EQ(static_cast<uint32_t>(s.collisionEvents.size()), 0u);
        mgr->shutdown();
    }

    TEST_CASE(StepAdvancesFrameIndex) {
        PhysicsBackendDescriptor desc;
        desc.kind3D = BackendKind::Null;
        auto mgr = PhysicsManager::create(desc);
        CHECK_NOT_NULL(mgr.get());

        mgr->step(1.0f / 60.0f);
        CHECK_TRUE(waitForFrame(*mgr, 1, 2000));
        mgr->step(1.0f / 60.0f);
        CHECK_TRUE(waitForFrame(*mgr, 2, 2000));
        mgr->shutdown();
    }

    TEST_CASE(SparseSnapshotNeverIndexesByHandle) {
        // §5.3: snapshot is a flat vector; no transforms[handleIndex] usage.
        PhysicsBackendDescriptor desc;
        desc.kind3D = BackendKind::Null;
        auto mgr = PhysicsManager::create(desc);
        CHECK_NOT_NULL(mgr.get());

        PhysicsWorld3D* w = mgr->world3D();
        CHECK_NOT_NULL(w);

        BodyHandle h = InvalidBodyHandle;
        RigidbodyDesc rb;
        rb.type = BodyType::Dynamic;
        rb.position = ayt::math::FVector3(0, 10, 0);
        CHECK_INT_EQ(static_cast<uint32_t>(w->createRigidbody(rb, h)), static_cast<uint32_t>(PhysResult::Ok));
        CHECK(isValidHandle(h));

        mgr->step(1.0f / 60.0f);
        CHECK_TRUE(waitForFrame(*mgr, 1, 2000));

        const PhysFrameSnapshot& s = mgr->fetchResults();
        // Null backend emits no transforms; verify sparse shape: either empty
        // or BodyTransforms present without implicit array-of-handles structure.
        for (const auto& t : s.transforms) {
            CHECK(isValidHandle(t.body));   // every transform has a real handle
        }
        mgr->shutdown();
    }

TEST_SUITE_END