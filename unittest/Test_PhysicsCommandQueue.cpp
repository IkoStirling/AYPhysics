#include "AYPhysics/PhysicsCommandQueue.h"
#include "AYPhysics/IPhysicsBackend.h"

#include "AYTest.h"

#include "AYPlatform/Thread.h"

#include <atomic>
#include <new>
#include <thread>

using namespace ayt::physics;

namespace {
class MailboxVectorBackend final : public IPhysicsBackend {
public:
    std::atomic<bool> failNext{false};
    bool start(const PhysicsBackendInfo&) override { return true; }
    void stop() override {}
    void step(float) override {}
    void publishSnapshot(PhysFrameSnapshot&) override {}
    void execute(const PhysicsCommand&, const PhysicsCreatePayload*) override {}
    void executeSync(const SyncQueryRequest&, SyncQueryResponse& response) override {
        response.overlaps = {17u, 23u};
        if (failNext.exchange(false)) {
            throw std::bad_alloc{};
        }
    }
    PhysicsBackendInfo describe() const override { return {}; }
    bool isRealDevice() const override { return false; }
};
}

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

    TEST_CASE(SyncMailboxTransfersVectorsAndReusesSlotAfterBackendFailure) {
        SyncQueryMailbox mailbox;
        CHECK_TRUE(mailbox.initialize(1u));
        MailboxVectorBackend backend;
        std::atomic<bool> stop{false};
        std::thread worker([&] {
            while (!stop.load(std::memory_order_acquire)) {
                mailbox.serviceAll(&backend);
                ayt::platform::Thread::yield();
            }
        });
        SyncQueryRequest request{};
        request.requestId = 41u;
        SyncQueryResponse response{};
        CHECK_TRUE(mailbox.submitAndWait(request, response, 1000 * 1000));
        CHECK_TRUE(response.status == PhysResult::Ok);
        CHECK_INT_EQ(response.overlaps.size(), 2u);
        CHECK_INT_EQ(response.overlaps.front(), 17u);

        backend.failNext = true;
        request.requestId = 42u;
        CHECK_TRUE(mailbox.submitAndWait(request, response, 1000 * 1000));
        CHECK_TRUE(response.status == PhysResult::NoMemory);
        CHECK_TRUE(response.overlaps.empty());

        request.requestId = 43u;
        CHECK_TRUE(mailbox.submitAndWait(request, response, 1000 * 1000));
        CHECK_TRUE(response.status == PhysResult::Ok);
        CHECK_INT_EQ(response.overlaps.size(), 2u);
        stop.store(true, std::memory_order_release);
        worker.join();
        mailbox.shutdown();
    }

TEST_SUITE_END
