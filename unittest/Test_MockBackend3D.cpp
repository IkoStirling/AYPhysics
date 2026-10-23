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

    TEST_CASE(TransferRigidbodyUsesOnePayloadAndOneQueueSlot) {
        PhysicsBackendDescriptor desc;
        desc.kind3D=BackendKind::Mock;
        auto mgr=PhysicsManager::create(desc);
        CHECK_NOT_NULL(mgr.get());
        auto* world=mgr->world3D();
        BodyHandle body=InvalidBodyHandle;
        CHECK(world->createRigidbody(RigidbodyDesc{},body)==PhysResult::Ok);
        CHECK(mgr->stepAndWait(1.f/60.f)==PhysResult::Ok);
        testaccess::reset();
        CHECK(world->transferRigidbody(body,{10,2,3},{0,0,0,1},
            {1,2,3},{4,5,6},3,1u<<3)==PhysResult::Ok);
        CHECK(mgr->stepAndWait(1.f/60.f)==PhysResult::Ok);
        const auto& commands=testaccess::mockBackendCommands();
        const auto& payloads=testaccess::mockBackendCreatePayloads();
        CHECK(commands.size()==1);CHECK(payloads.size()==1);
        if(commands.size()==1 && payloads.size()==1) {
            CHECK(commands[0].type==PhysicsCommandType::TransferRigidbody3D);
            CHECK(commands[0].body==body);
            CHECK(payloads[0].kind==PhysicsCreatePayload::Kind::BodyTransfer3D);
            const auto& transfer=payloads[0].bodyTransfer3D;
            CHECK_FLOAT_EQ(transfer.position.x,10,1e-5f);
            CHECK_FLOAT_EQ(transfer.linearVelocity.z,3,1e-5f);
            CHECK_FLOAT_EQ(transfer.angularVelocity.y,5,1e-5f);
            CHECK(transfer.layer==3);CHECK(transfer.collideMask==(1u<<3));
        }
        testaccess::reset();
        CHECK(world->transferRigidbody(body,{},{0,0,0,0},{},{},3,1u<<3)
              ==PhysResult::InvalidParam);
        CHECK(world->transferRigidbody(body,{},{0,0,0,1},{},{},32,0)
              ==PhysResult::InvalidParam);
        CHECK(testaccess::mockBackendCommands().empty());
        mgr->shutdown();
    }

TEST_SUITE_END
