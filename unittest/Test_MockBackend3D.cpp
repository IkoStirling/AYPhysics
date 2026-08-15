#include "AYPhysics/PhysicsManager.h"
#include "AYPhysics/PhysicsWorld3D.h"
#include "AYPhysics/PhysicsBackendTestAccess.h"
#include "AYPhysics/PhysicsTypes.h"

#include "AYTest.h"

#include "AYTime/Clock.h"
#include "AYPlatform/Thread.h"

using namespace ayt::physics;

namespace {
// Helper: spin briefly so the physics thread drains enqueued commands.
void waitForDrain(PhysicsManager& m, int ms) {
    const uint64_t startUs = ayt::time::Clock::performanceNowUs();
    while (ayt::time::Clock::performanceNowUs() - startUs <
           static_cast<uint64_t>(ms) * 1000u) {
        (void)m.fetchResults();
        ayt::platform::Thread::sleep(0.001f);  // 1 ms
    }
}
}  // namespace

TEST_SUITE(MockBackend3DTests)

    TEST_CASE(MockCapturesCompactCommandSequence) {
        testaccess::reset();

        PhysicsBackendDescriptor desc;
        desc.kind3D = BackendKind::Mock;
        desc.commandQueueCapacity = 32;
        desc.createPoolCapacity   = 16;
        auto mgr = PhysicsManager::create(desc);
        CHECK_NOT_NULL(mgr.get());

        PhysicsWorld3D* w = mgr->world3D();
        CHECK_NOT_NULL(w);

        BodyHandle h = InvalidBodyHandle;
        RigidbodyDesc rb;
        rb.type = BodyType::Dynamic;
        rb.mass = 7.5f;
        const PhysResult r = w->createRigidbody(rb, h);
        CHECK_INT_EQ(static_cast<uint32_t>(r), static_cast<uint32_t>(PhysResult::Ok));
        CHECK(isValidHandle(h));

        // Give the physics thread a moment to drain + step.
        waitForDrain(*mgr, 50);

        const auto& cmds = testaccess::mockBackendCommands();
        CHECK(cmds.size() >= 1u);
        if (!cmds.empty()) {
            CHECK(cmds[0].type == PhysicsCommandType::CreateRigidbody);
            CHECK_INT_EQ(handleIndex(cmds[0].body), handleIndex(h));
        }

        const auto& payloads = testaccess::mockBackendCreatePayloads();
        CHECK(payloads.size() >= 1u);
        if (!payloads.empty()) {
            CHECK(payloads[0].kind == PhysicsCreatePayload::Kind::Rigidbody);
            CHECK_FLOAT_EQ(payloads[0].rigidDesc.mass, 7.5f, 1e-6f);
        }

        mgr->shutdown();
    }

    TEST_CASE(MockTracksLastQueryId) {
        testaccess::reset();

        PhysicsBackendDescriptor desc;
        desc.kind3D = BackendKind::Mock;
        desc.commandQueueCapacity = 32;
        desc.createPoolCapacity   = 16;
        auto mgr = PhysicsManager::create(desc);
        CHECK_NOT_NULL(mgr.get());

        PhysicsWorld3D* w = mgr->world3D();
        uint32_t qid = w->raycastAsync(ayt::math::Ray(ayt::math::FVector3(0,0,0),
                                                      ayt::math::FVector3(1,0,0)));
        CHECK(qid != 0u);

        waitForDrain(*mgr, 50);

        CHECK_INT_EQ(testaccess::lastIssuedQueryId(), qid);
        mgr->shutdown();
    }

TEST_SUITE_END