#include "NullBackend3D.h"
#include "AYPhysics/PhysicsTypes.h"

#include "AYTest.h"

using namespace ayt::physics;

TEST_SUITE(NullBackend3DTests)

    TEST_CASE(DescribeReportsNull) {
        NullBackend3D backend;
        const PhysicsBackendInfo info = backend.describe();
        CHECK_NOT_NULL(info.name);
        CHECK_FALSE(backend.isRealDevice());
    }

    TEST_CASE(StepIncrementsCounter) {
        NullBackend3D backend;
        backend.init3D(PhysicsBackendDescriptor{});
        backend.start(backend.describe());
        CHECK_INT_EQ(backend.stepCount(), 0u);
        backend.step(1.0f / 60.0f);
        backend.step(1.0f / 60.0f);
        CHECK_INT_EQ(backend.stepCount(), 2u);
        backend.stop();
    }

    TEST_CASE(ExecuteAcceptsAnyCommand) {
        NullBackend3D backend;
        backend.start(backend.describe());
        PhysicsCommand cmd{};
        cmd.type = PhysicsCommandType::ApplyForce;
        backend.execute(cmd, nullptr);
        backend.execute(cmd, nullptr);
        CHECK_INT_EQ(backend.commandCount(), 2u);
        backend.stop();
    }

    TEST_CASE(ExecuteSyncReturnsOkEmptyHit) {
        NullBackend3D backend;
        SyncQueryRequest req{};
        req.requestId = 42;
        SyncQueryResponse resp{};
        backend.executeSync(req, resp);
        CHECK_INT_EQ(static_cast<uint32_t>(resp.status), static_cast<uint32_t>(PhysResult::Ok));
        CHECK_INT_EQ(resp.requestId, 42u);
        CHECK_FALSE(resp.firstHit.hit);
    }

TEST_SUITE_END