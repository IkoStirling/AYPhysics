#include "AYPhysics/PhysicsCommandQueue.h"

#include "AYTest.h"

#include "AYPlatform/Thread.h"

using namespace ayt::physics;

TEST_SUITE(PhysicsCommandQueueTests)

    TEST_CASE(PhysicsCommandStaysWithinBudget) {
        // §17.1 hard cap.
        CHECK_TRUE(sizeof(PhysicsCommand) <= 64);
    }

    TEST_CASE(InitializeRoundsToPowerOfTwo) {
        PhysicsCommandQueue q;
        CHECK(q.initialize(20));
        CHECK_INT_EQ(q.capacity(), 32u);  // rounded up
        q.shutdown();
    }

    TEST_CASE(InitializeRejectsZeroCapacity) {
        PhysicsCommandQueue q;
        CHECK_FALSE(q.initialize(0));
    }

    TEST_CASE(PushPopRoundTrips) {
        PhysicsCommandQueue q;
        q.initialize(8);
        PhysicsCommand in{};
        in.type = PhysicsCommandType::ApplyForce;
        in.body = makeHandle(7, 1);
        in.u.vec4.x = 1.0f; in.u.vec4.y = 2.0f; in.u.vec4.z = 3.0f;
        CHECK(q.tryPush(in));

        PhysicsCommand out{};
        CHECK(q.tryPop(out));
        CHECK(out.type == PhysicsCommandType::ApplyForce);
        CHECK_INT_EQ(handleIndex(out.body), 7u);
        CHECK_FLOAT_EQ(out.u.vec4.x, 1.0f, 1e-6f);
        CHECK_FLOAT_EQ(out.u.vec4.y, 2.0f, 1e-6f);
        CHECK(q.tryPop(out) == false);  // empty
        q.shutdown();
    }

    TEST_CASE(FullQueueRejectsPush) {
        PhysicsCommandQueue q;
        q.initialize(4);
        PhysicsCommand cmd{};
        cmd.type = PhysicsCommandType::Step;
        cmd.u.step.deltaTime = 0.016f;
        for (int i = 0; i < 4; ++i) CHECK(q.tryPush(cmd));
        CHECK_FALSE(q.tryPush(cmd));  // 5th rejected
        CHECK_INT_EQ(q.approximateDepth(), 4u);
        q.shutdown();
    }

    TEST_CASE(CreatePoolAllocateAndTakeRoundTrip) {
        PhysicsCreatePool pool;
        CHECK(pool.initialize(4));
        PhysicsCreatePayload payload{};
        payload.kind = PhysicsCreatePayload::Kind::Rigidbody;
        payload.rigidDesc.mass = 2.5f;
        const CreateSlotId slotId = pool.allocate(payload);
        CHECK(slotId != InvalidCreateSlotId);

        PhysicsCreatePayload taken{};
        CHECK(pool.take(slotId, taken));
        CHECK(taken.kind == PhysicsCreatePayload::Kind::Rigidbody);
        CHECK_FLOAT_EQ(taken.rigidDesc.mass, 2.5f, 1e-6f);
        pool.shutdown();
    }

    TEST_CASE(CreatePoolExhaustionReturnsInvalid) {
        PhysicsCreatePool pool;
        pool.initialize(2);
        PhysicsCreatePayload p{};
        const CreateSlotId s1 = pool.allocate(p);
        const CreateSlotId s2 = pool.allocate(p);
        const CreateSlotId s3 = pool.allocate(p);
        CHECK(s1 != InvalidCreateSlotId);
        CHECK(s2 != InvalidCreateSlotId);
        CHECK_INT_EQ(s3, InvalidCreateSlotId);
        pool.shutdown();
    }

    TEST_CASE(CrossThreadProducerConsumerRoundTrips) {
        // TSan-friendly smoke; 200 items pushed by one thread and popped by another.
        PhysicsCommandQueue q;
        q.initialize(64);

        std::thread producer([&]() {
            for (int i = 0; i < 200; ++i) {
                PhysicsCommand c{};
                c.type = PhysicsCommandType::Step;
                c.u.step.deltaTime = static_cast<float>(i) * 0.001f;
                while (!q.tryPush(c)) ayt::platform::Thread::yield();
            }
        });
        std::thread consumer([&]() {
            int seen = 0;
            while (seen < 200) {
                PhysicsCommand c{};
                if (q.tryPop(c)) ++seen;
                else ayt::platform::Thread::yield();
            }
        });
        producer.join();
        consumer.join();
        CHECK_INT_EQ(q.approximateDepth(), 0u);
        q.shutdown();
    }

TEST_SUITE_END