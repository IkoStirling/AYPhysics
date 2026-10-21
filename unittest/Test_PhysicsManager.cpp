#include "AYPhysics/PhysicsManager.h"
#include "AYPhysics/PhysicsWorld3D.h"
#include "AYPhysics/PhysicsWorld2D.h"
#include "AYPhysics/PhysicsTypes.h"
#include "AYPhysics/PhysicsCommandQueue.h"
#include "AYPhysics/PhysicsQueryAdapter.h"

#include "AYTest.h"

#include "AYTime/Clock.h"
#include "AYPlatform/Thread.h"

#include <limits>

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

    TEST_CASE(CreateBackpressureDoesNotConsumeTwoOrThreeDimensionalHandles) {
        PhysicsBackendDescriptor desc;
        desc.kind3D = BackendKind::Null;
        desc.kind2D = BackendKind::Null;
        desc.maxBodies = 3u;
        desc.createPoolCapacity = 1u;
        auto mgr = PhysicsManager::create(desc);
        CHECK_NOT_NULL(mgr.get());
        PhysicsWorld2D* world2D = mgr->world2D();
        PhysicsWorld3D* world3D = mgr->world3D();
        PhysicsCreatePayload held{};
        const CreateSlotId slot = mgr->createPool()->allocate(held);
        CHECK_TRUE(slot != InvalidCreateSlotId);
        RigidbodyDesc bodyDesc;
        ColliderDesc colliderDesc;
        JointDesc jointDesc;
        for (int attempt = 0; attempt < 5; ++attempt) {
            BodyHandle body2D = makeHandle(42u, 1u);
            BodyHandle body3D = makeHandle(42u, 1u);
            ColliderHandle collider2D = makeHandle(42u, 1u);
            ColliderHandle collider3D = makeHandle(42u, 1u);
            JointHandle joint2D = makeHandle(42u, 1u);
            JointHandle joint3D = makeHandle(42u, 1u);
            CHECK_TRUE(world2D->createRigidbody(bodyDesc, body2D)
                == PhysResult::NoMemory);
            CHECK_TRUE(world3D->createRigidbody(bodyDesc, body3D)
                == PhysResult::NoMemory);
            CHECK_TRUE(world2D->createCollider(colliderDesc, collider2D)
                == PhysResult::NoMemory);
            CHECK_TRUE(world3D->createCollider(colliderDesc, collider3D)
                == PhysResult::NoMemory);
            CHECK_TRUE(world2D->createJoint(jointDesc, joint2D)
                == PhysResult::NoMemory);
            CHECK_TRUE(world3D->createJoint(jointDesc, joint3D)
                == PhysResult::NoMemory);
            CHECK_INT_EQ(body2D, InvalidBodyHandle);
            CHECK_INT_EQ(body3D, InvalidBodyHandle);
            CHECK_INT_EQ(collider2D, InvalidColliderHandle);
            CHECK_INT_EQ(collider3D, InvalidColliderHandle);
            CHECK_INT_EQ(joint2D, InvalidJointHandle);
            CHECK_INT_EQ(joint3D, InvalidJointHandle);
        }
        CHECK_TRUE(mgr->createPool()->take(slot, held));

        BodyHandle body2D = InvalidBodyHandle;
        BodyHandle body3D = InvalidBodyHandle;
        CHECK_TRUE(world2D->createRigidbody(bodyDesc, body2D) == PhysResult::Ok);
        CHECK_INT_EQ(handleIndex(body2D), 1u);
        CHECK_TRUE(mgr->stepAndWait(1.0f / 60.0f) == PhysResult::Ok);
        CHECK_TRUE(world3D->createRigidbody(bodyDesc, body3D) == PhysResult::Ok);
        CHECK_INT_EQ(handleIndex(body3D), 1u);
        CHECK_TRUE(mgr->stepAndWait(1.0f / 60.0f) == PhysResult::Ok);

        ColliderHandle collider2D = InvalidColliderHandle;
        ColliderHandle collider3D = InvalidColliderHandle;
        colliderDesc.body = body2D;
        CHECK_TRUE(world2D->createCollider(colliderDesc, collider2D)
            == PhysResult::Ok);
        CHECK_INT_EQ(handleIndex(collider2D), 1u);
        CHECK_TRUE(mgr->stepAndWait(1.0f / 60.0f) == PhysResult::Ok);
        colliderDesc.body = body3D;
        CHECK_TRUE(world3D->createCollider(colliderDesc, collider3D)
            == PhysResult::Ok);
        CHECK_INT_EQ(handleIndex(collider3D), 1u);
        CHECK_TRUE(mgr->stepAndWait(1.0f / 60.0f) == PhysResult::Ok);

        JointHandle joint2D = InvalidJointHandle;
        JointHandle joint3D = InvalidJointHandle;
        CHECK_TRUE(world2D->createJoint(jointDesc, joint2D) == PhysResult::Ok);
        CHECK_INT_EQ(handleIndex(joint2D), 1u);
        CHECK_TRUE(mgr->stepAndWait(1.0f / 60.0f) == PhysResult::Ok);
        CHECK_TRUE(world3D->createJoint(jointDesc, joint3D) == PhysResult::Ok);
        CHECK_INT_EQ(handleIndex(joint3D), 1u);
        mgr->shutdown();
        CHECK_TRUE(world2D->createRigidbody(bodyDesc, body2D)
            == PhysResult::InvalidState);
        CHECK_INT_EQ(body2D, InvalidBodyHandle);
    }

    TEST_CASE(ThreeDimensionalLegacyQueriesValidateAndClearOutputs) {
        PhysicsBackendDescriptor desc;
        desc.kind3D = BackendKind::Null;
        auto mgr = PhysicsManager::create(desc);
        CHECK_NOT_NULL(mgr.get());
        PhysicsWorld3D* world = mgr->world3D();
        PhysicsQueryAdapter facade;
        CHECK_FALSE(facade.isReady());
        ayt::math::Ray ray;
        ray.origin = {0.0f, 0.0f, 0.0f};
        ray.dir = {1.0f, 0.0f, 0.0f};
        const float nan = (std::numeric_limits<float>::quiet_NaN)();
        RaycastHit hit{};
        hit.hit = true;
        hit.body = makeHandle(5u, 1u);
        ray.origin.x = nan;
        CHECK_TRUE(world->raycastSync(ray, hit) == PhysResult::InvalidParam);
        CHECK_FALSE(hit.hit);
        hit.hit = true;
        CHECK_TRUE(facade.raycastSync(ray, hit, 0xFFFFFFFFu)
            == PhysResult::InvalidState);
        CHECK_FALSE(hit.hit);
        CHECK_INT_EQ(world->raycastAsync(ray), 0u);
        ray.origin.x = 0.0f;
        ray.dir = {0.0f, 0.0f, 0.0f};
        CHECK_INT_EQ(world->raycastAsync(ray), 0u);
        ray.dir = {1.0f, 0.0f, 0.0f};

        std::vector<BodyHandle> bodies{makeHandle(7u, 1u)};
        CHECK_TRUE(facade.overlapSphereSync({0.0f, 0.0f, 0.0f},
            1.0f, bodies, 0xFFFFFFFFu) == PhysResult::InvalidState);
        CHECK_TRUE(bodies.empty());
        bodies.push_back(makeHandle(7u, 1u));
        CHECK_TRUE(world->overlapSphereSync({nan, 0.0f, 0.0f},
            1.0f, bodies, 0xFFFFFFFFu) == PhysResult::InvalidParam);
        CHECK_TRUE(bodies.empty());
        CHECK_INT_EQ(world->overlapSphereAsync({0.0f, 0.0f, 0.0f},
            0.0f, 0xFFFFFFFFu), 0u);
        auto rotation = ayt::math::FQuaternion::identity();
        CHECK_INT_EQ(world->overlapBoxAsync({0.0f, 0.0f, 0.0f},
            {0.0f, 1.0f, 1.0f}, rotation, 0xFFFFFFFFu), 0u);
        rotation = ayt::math::FQuaternion(0.0f, 0.0f, 0.0f, 0.0f);
        CHECK_INT_EQ(world->overlapBoxAsync({0.0f, 0.0f, 0.0f},
            {1.0f, 1.0f, 1.0f}, rotation, 0xFFFFFFFFu), 0u);

        facade.bind(mgr.get());
        CHECK_TRUE(facade.isReady());

        hit.hit = true;
        CHECK_TRUE(world->raycastSync(ray, hit) == PhysResult::Ok);
        CHECK_FALSE(hit.hit);
        bodies.push_back(makeHandle(7u, 1u));
        CHECK_TRUE(world->overlapSphereSync({0.0f, 0.0f, 0.0f},
            1.0f, bodies, 0xFFFFFFFFu) == PhysResult::Ok);
        CHECK_TRUE(bodies.empty());
        mgr->shutdown();
        CHECK_FALSE(facade.isReady());
        hit.hit = true;
        CHECK_TRUE(world->raycastSync(ray, hit) == PhysResult::InvalidState);
        CHECK_FALSE(hit.hit);
        bodies.push_back(makeHandle(7u, 1u));
        CHECK_TRUE(world->overlapSphereSync({0.0f, 0.0f, 0.0f},
            1.0f, bodies, 0xFFFFFFFFu) == PhysResult::InvalidState);
        CHECK_TRUE(bodies.empty());
        hit.hit = true;
        CHECK_TRUE(facade.raycastSync(ray, hit, 0xFFFFFFFFu)
            == PhysResult::InvalidState);
        CHECK_FALSE(hit.hit);
        bodies.push_back(makeHandle(7u, 1u));
        CHECK_TRUE(facade.overlapSphereSync({0.0f, 0.0f, 0.0f},
            1.0f, bodies, 0xFFFFFFFFu) == PhysResult::InvalidState);
        CHECK_TRUE(bodies.empty());
        CHECK_INT_EQ(world->raycastAsync(ray), 0u);
        CHECK_INT_EQ(world->overlapSphereAsync({0.0f, 0.0f, 0.0f},
            1.0f, 0xFFFFFFFFu), 0u);
        CHECK_INT_EQ(world->overlapBoxAsync({0.0f, 0.0f, 0.0f},
            {1.0f, 1.0f, 1.0f}, ayt::math::FQuaternion::identity(),
            0xFFFFFFFFu), 0u);
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

    TEST_CASE(StepAndWaitPublishesCompletedSnapshot) {
        PhysicsBackendDescriptor d;
        d.kind3D = BackendKind::Mock;
        auto mgr = PhysicsManager::create(d);
        CHECK_NOT_NULL(mgr.get());

        const float dt = 1.0f / 120.0f;
        const PhysResult r = mgr->stepAndWait(
            dt, ayt::time::Duration::fromSeconds(1));
        CHECK_INT_EQ(static_cast<uint32_t>(r),
                     static_cast<uint32_t>(PhysResult::Ok));

        const PhysFrameSnapshot& snapshot = mgr->fetchResults();
        CHECK_INT_EQ(static_cast<int>(snapshot.frameIndex), 1);
        CHECK_FLOAT_EQ(snapshot.stepSeconds, dt, 0.00001f);
        mgr->shutdown();
    }

    TEST_CASE(StepAndWaitRejectsStoppedManager) {
        PhysicsBackendDescriptor d;
        d.kind3D = BackendKind::Null;
        auto mgr = PhysicsManager::create(d);
        CHECK_NOT_NULL(mgr.get());
        mgr->shutdown();

        const PhysResult r = mgr->stepAndWait(1.0f / 60.0f);
        CHECK_INT_EQ(static_cast<uint32_t>(r),
                     static_cast<uint32_t>(PhysResult::InvalidState));
    }

    // F-A regression — mintHandle caps index at descriptor().maxBodies. Before
    // the fix, the game-thread counter was unbounded; once it crossed the
    // backend's slot-table size, createRigidbody returned Ok with a handle
    // whose index was > maxBodies and the backend silently NotFound'd every
    // mutator. Caller walked away with a phantom body. This test exhausts
    // the cap on a small manager and asserts the (N+1)th create fails with
    // OutOfRange + outHandle = Invalid, instead of returning Ok with a junk
    // handle.
    TEST_CASE(CreateRigidbodyOverflowReturnsOutOfRange) {
        PhysicsBackendDescriptor desc;
        desc.kind3D = BackendKind::Null;
        desc.maxBodies = 4u;  // 4 legal indices: 1..3 (index 0 reserved)
        auto mgr = PhysicsManager::create(desc);
        PhysicsWorld3D* w = mgr->world3D();

        // First 3 should succeed (index 1, 2, 3).
        for (uint32_t i = 0; i < 3u; ++i) {
            BodyHandle h = InvalidBodyHandle;
            RigidbodyDesc rb;
            rb.type = BodyType::Dynamic;
            const PhysResult r = w->createRigidbody(rb, h);
            CHECK_INT_EQ(static_cast<uint32_t>(r), static_cast<uint32_t>(PhysResult::Ok));
            CHECK(isValidHandle(h));
        }
        // 4th overflows the cap.
        BodyHandle hOver = makeHandle(123u, 5u);  // sentinel — must be overwritten
        RigidbodyDesc rbOver;
        rbOver.type = BodyType::Dynamic;
        const PhysResult rOver = w->createRigidbody(rbOver, hOver);
        CHECK_INT_EQ(static_cast<uint32_t>(rOver), static_cast<uint32_t>(PhysResult::OutOfRange));
        CHECK_INT_EQ(hOver, InvalidBodyHandle);
        mgr->shutdown();
    }

    // F-A — same overflow guard on the 2D world + the 3D createCollider / createJoint
    // paths. Verify the cap is enforced uniformly across all 3 create kinds and
    // across both worlds.
    TEST_CASE(CreateColliderAndJointOverflowReturnsOutOfRange) {
        PhysicsBackendDescriptor desc;
        desc.kind3D = BackendKind::Null;
        desc.maxBodies = 4u;
        auto mgr = PhysicsManager::create(desc);
        PhysicsWorld3D* w3 = mgr->world3D();
        PhysicsWorld2D* w2 = mgr->world2D();

        BodyHandle h = InvalidBodyHandle;
        RigidbodyDesc rb;
        rb.type = BodyType::Static;
        CHECK_INT_EQ(static_cast<uint32_t>(w2->createRigidbody(rb, h)),
                     static_cast<uint32_t>(PhysResult::Ok));
        CHECK(isValidHandle(h));

        // Same cap on 2D's own collider counter (per-world). maxBodies=4 means
        // 3 colliders can be created on the 2D world.
        for (uint32_t i = 0; i < 3u; ++i) {
            ColliderHandle ch = InvalidColliderHandle;
            ColliderDesc cd;
            cd.body = h; cd.shape = ColliderShape::Box;
            cd.halfExtents = ayt::math::FVector3(0.1f, 0.1f, 0.0f);
            const PhysResult r = w2->createCollider(cd, ch);
            CHECK_INT_EQ(static_cast<uint32_t>(r), static_cast<uint32_t>(PhysResult::Ok));
            CHECK(isValidHandle(ch));
        }
        ColliderHandle chOver = makeHandle(99u, 1u);
        ColliderDesc cdOver;
        cdOver.body = h; cdOver.shape = ColliderShape::Box;
        cdOver.halfExtents = ayt::math::FVector3(0.1f, 0.1f, 0.0f);
        const PhysResult rOver = w2->createCollider(cdOver, chOver);
        CHECK_INT_EQ(static_cast<uint32_t>(rOver), static_cast<uint32_t>(PhysResult::OutOfRange));
        CHECK_INT_EQ(chOver, InvalidColliderHandle);

        mgr->shutdown();
    }

TEST_SUITE_END
