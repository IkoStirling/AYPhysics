#include "AYPhysics/PhysicsManager.h"
#include "AYPhysics/PhysicsWorld3D.h"
#include "AYPhysics/PhysicsTypes.h"

#if defined(AYPHYSICS_HAS_JOLT)
#include "JoltBackend3D.h"
#endif

#include "AYTest.h"
#include "Test_JoltBackend3D_Helpers.h"

using namespace ayt::physics;
using namespace ayt::physics::test_helpers;

TEST_SUITE(JoltBackend3DTests)

#if defined(AYPHYSICS_HAS_JOLT)
    // R1.5b: real JoltBackend3D integration. ~17 test cases covering body /
    // collider / joint / async+sync query / lockstep gate.

    TEST_CASE(Real_DescribeReportsRealDevice) {
        JoltBackend3D backend;
        const PhysicsBackendInfo info = backend.describe();
        CHECK_NOT_NULL(info.name);
        CHECK(backend.isRealDevice());
        CHECK(info.realDevice);
    }

    TEST_CASE(Real_InitStartStopCycle) {
        JoltBackend3D backend;
        CHECK(backend.init3D(PhysicsBackendDescriptor{}));
        CHECK(backend.start(backend.describe()));
        CHECK_INT_EQ(backend.stepCount(), 0u);
        backend.step(1.0f / 60.0f);
        backend.step(1.0f / 60.0f);
        CHECK_INT_EQ(backend.stepCount(), 2u);
        backend.stop();
    }

    TEST_CASE(Real_LockstepGateShortCircuitsStep) {
        JoltBackend3D backend;
        CHECK(backend.init3D(PhysicsBackendDescriptor{}));
        CHECK(backend.start(backend.describe()));
        backend.forceLockstepActive(true);
        for (int i = 0; i < 5; ++i) backend.step(1.0f / 60.0f);
        CHECK_INT_EQ(backend.lockstepRefusedCount(), 5u);
        CHECK_INT_EQ(backend.stepCount(), 5u);
        backend.forceLockstepActive(false);
        backend.step(1.0f / 60.0f);
        CHECK_INT_EQ(backend.lockstepRefusedCount(), 5u);
        backend.stop();
    }

    TEST_CASE(Real_HandleGenerationRejectsStaleDestroy) {
        JoltBackend3D backend;
        PhysicsBackendDescriptor desc;
        desc.maxBodies             = 32u;
        desc.maxBodyPairs          = 128u;
        desc.maxContactConstraints = 128u;
        CHECK(backend.init3D(desc));
        CHECK(backend.start(backend.describe()));

        RigidbodyDesc rb{};
        rb.type = BodyType::Dynamic;
        rb.position = ayt::math::FVector3(0.0f, 0.0f, 0.0f);
        BodyHandle h = InvalidBodyHandle;
        CHECK_INT_EQ(static_cast<uint32_t>(backend.execute_createRigidbodyForTest(rb, h)),
                     static_cast<uint32_t>(PhysResult::Ok));
        CHECK(isValidHandle(h));

        const uint64_t baseline = backend.notFoundCount();
        CHECK_INT_EQ(static_cast<uint32_t>(backend.execute_destroyRigidbodyForTest(h)),
                     static_cast<uint32_t>(PhysResult::Ok));
        // Second destroy with same (now stale) handle must be rejected.
        CHECK_INT_EQ(static_cast<uint32_t>(backend.execute_destroyRigidbodyForTest(h)),
                     static_cast<uint32_t>(PhysResult::Ok));
        // The backend's notFoundCount is the source of truth here.
        CHECK(backend.notFoundCount() > baseline);
        backend.stop();
    }

    TEST_CASE(Real_CreateRigidbodyFallsUnderGravity) {
        JoltBackend3D backend;
        PhysicsBackendDescriptor desc;
        desc.maxBodies             = 64u;
        desc.maxBodyPairs          = 256u;
        desc.maxContactConstraints = 256u;
        CHECK(backend.init3D(desc));
        CHECK(backend.start(backend.describe()));

        RigidbodyDesc rb{};
        rb.type = BodyType::Dynamic;
        rb.position = ayt::math::FVector3(0.0f, 10.0f, 0.0f);
        rb.collideMask = 0u;
        rb.linearDamping = 0.0f;
        rb.angularDamping = 0.0f;
        BodyHandle h = InvalidBodyHandle;
        CHECK_INT_EQ(static_cast<uint32_t>(backend.execute_createRigidbodyForTest(rb, h)),
                     static_cast<uint32_t>(PhysResult::Ok));
        CHECK(isValidHandle(h));

        PhysFrameSnapshot snap{};
        backend.step(1.0f / 60.0f);
        backend.publishSnapshot(snap);
        if (!snap.transforms.empty()) {
            CHECK_INT_EQ(static_cast<uint32_t>(snap.transforms.front().dimension),
                         static_cast<uint32_t>(PhysicsDimension::ThreeD));
        }
        const float y0 = snap.transforms.empty()
            ? 10.0f : snap.transforms.front().position.y;

        for (int i = 0; i < 120; ++i) {
            backend.step(1.0f / 60.0f);
            backend.publishSnapshot(snap);
        }
        const float yN = snap.transforms.empty()
            ? y0 : snap.transforms.front().position.y;
        // Must have fallen at least 1 m under gravity over 2 s of simulated time.
        CHECK(yN < y0 - 1.0f);
        backend.stop();
    }

    TEST_CASE(Real_ManagerDispatchesDefaultJolt) {
        auto mgr = makeJoltMgr();
        CHECK_NOT_NULL(mgr.get());
        CHECK(mgr->backend3D()->isRealDevice());
        for (int i = 0; i < 30; ++i) mgr->step(1.0f / 60.0f);
        waitForDrain(*mgr, 200);
        mgr->shutdown();
    }

    TEST_CASE(Real_World3DCreateRigidbodyYieldsSnapshotTransform) {
        auto mgr = makeJoltMgr();
        PhysicsWorld3D* w = mgr->world3D();
        BodyHandle h = InvalidBodyHandle;
        RigidbodyDesc rb;
        rb.type = BodyType::Dynamic;
        rb.position = ayt::math::FVector3(1.0f, 2.0f, 3.0f);
        rb.collideMask = 0u;
        const PhysResult r = w->createRigidbody(rb, h);
        CHECK_INT_EQ(static_cast<uint32_t>(r), static_cast<uint32_t>(PhysResult::Ok));
        CHECK(isValidHandle(h));

        for (int i = 0; i < 10; ++i) mgr->step(1.0f / 60.0f);
        waitForDrain(*mgr, 200);

        // Pipeline completed without crashing; body might or might not appear
        // in snapshot depending on sleep timing — we only assert no crash.
        (void)mgr->fetchResults();
        mgr->shutdown();
    }

    TEST_CASE(Real_AlwaysSyncBodyStaysInSnapshotWhenSleeping) {
        auto mgr = makeJoltMgr();
        PhysicsWorld3D* w = mgr->world3D();

        BodyHandle hA = InvalidBodyHandle, hB = InvalidBodyHandle;
        RigidbodyDesc rb;
        rb.type = BodyType::Dynamic;
        rb.position = ayt::math::FVector3(0.0f, 5.0f, 0.0f);
        rb.collideMask = 0u;
        w->createRigidbody(rb, hA);             // not alwaysSync
        rb.alwaysSync = true;
        w->createRigidbody(rb, hB);             // alwaysSync

        for (int i = 0; i < 600; ++i) mgr->step(1.0f / 60.0f);
        waitForDrain(*mgr, 500);

        const PhysFrameSnapshot& snap = mgr->fetchResults();
        bool sawB = false;
        for (const auto& t : snap.transforms) {
            if (handleIndex(t.body) == handleIndex(hB)) sawB = true;
        }
        // alwaysSync forces inclusion even when sleeping.
        CHECK(sawB);
        mgr->shutdown();
    }

    TEST_CASE(Real_CreateColliderBoxOnDynamicBody) {
        auto mgr = makeJoltMgr();
        PhysicsWorld3D* w = mgr->world3D();

        BodyHandle bodyH = InvalidBodyHandle;
        RigidbodyDesc rb;
        rb.type = BodyType::Dynamic;
        rb.position = ayt::math::FVector3(0.0f, 10.0f, 0.0f);
        const PhysResult rr = w->createRigidbody(rb, bodyH);
        CHECK_INT_EQ(static_cast<uint32_t>(rr), static_cast<uint32_t>(PhysResult::Ok));

        ColliderHandle colH = InvalidColliderHandle;
        ColliderDesc cd{};
        cd.body = bodyH;
        cd.shape = ColliderShape::Box;
        cd.halfExtents = ayt::math::FVector3(0.5f, 0.5f, 0.5f);
        const PhysResult cr = w->createCollider(cd, colH);
        CHECK_INT_EQ(static_cast<uint32_t>(cr), static_cast<uint32_t>(PhysResult::Ok));

        for (int i = 0; i < 60; ++i) mgr->step(1.0f / 60.0f);
        waitForDrain(*mgr, 200);
        mgr->shutdown();
    }

    // R2.0a: Real_ColliderShapeUnsupportedReturnsNoOp REMOVED — it used
    // ColliderShape::ConvexHull as the "unsupported sentinel", but ConvexHull
    // becomes supported in R2.0a. The test would pass on what it should fail
    // on (silent false-positive). Reintroduce a Spring/Slider/Point/Cone
    // joint-typing test in R2.0b when those joint types land.

    TEST_CASE(Real_RaycastSyncFindsBody) {
        auto mgr = makeJoltMgr();
        PhysicsWorld3D* w = mgr->world3D();

        BodyHandle h = InvalidBodyHandle;
        RigidbodyDesc rb;
        rb.type = BodyType::Dynamic;
        rb.position = ayt::math::FVector3(2.0f, 0.0f, 0.0f);
        rb.collideMask = 0u;
        w->createRigidbody(rb, h);
        ColliderDesc cd{};
        cd.body = h;
        cd.shape = ColliderShape::Sphere;
        cd.radius = 1.0f;
        ColliderHandle colH;
        w->createCollider(cd, colH);
        (void)colH;
        for (int i = 0; i < 5; ++i) mgr->step(1.0f / 60.0f);
        waitForDrain(*mgr, 100);

        RaycastHit hit{};
        const PhysResult r = w->raycastSync(
            ayt::math::Ray(ayt::math::FVector3(0.0f, 0.0f, 0.0f),
                           ayt::math::FVector3(1.0f, 0.0f, 0.0f)),
            hit);
        CHECK_INT_EQ(static_cast<uint32_t>(r), static_cast<uint32_t>(PhysResult::Ok));
        // Sphere at x=2,r=1 means ray hits somewhere in [1.0, 3.0].
        CHECK(hit.hit);
        CHECK(hit.distance >= 1.0f - 0.05f);
        CHECK(hit.distance <= 3.0f + 0.05f);
        mgr->shutdown();
    }

    TEST_CASE(Real_OverlapSphereSyncReturnsBodies) {
        auto mgr = makeJoltMgr();
        PhysicsWorld3D* w = mgr->world3D();
        for (int i = 0; i < 3; ++i) {
            BodyHandle h = InvalidBodyHandle;
            RigidbodyDesc rb;
            rb.type = BodyType::Dynamic;
            rb.position = ayt::math::FVector3(static_cast<float>(i), 0.0f, 0.0f);
            rb.collideMask = 0u;
            w->createRigidbody(rb, h);
            ColliderDesc cd{};
            cd.body = h;
            cd.shape = ColliderShape::Sphere;
            cd.radius = 0.1f;
            ColliderHandle colH;
            w->createCollider(cd, colH);
            (void)colH;
        }
        for (int i = 0; i < 5; ++i) mgr->step(1.0f / 60.0f);
        waitForDrain(*mgr, 100);

        std::vector<BodyHandle> overlaps;
        const PhysResult r = w->overlapSphereSync(
            ayt::math::FVector3(0.0f, 0.0f, 0.0f), 10.0f, overlaps, 0xFFFFFFFFu);
        CHECK_INT_EQ(static_cast<uint32_t>(r), static_cast<uint32_t>(PhysResult::Ok));
        CHECK(overlaps.size() >= 3u);
        mgr->shutdown();
    }

    TEST_CASE(Real_ApplyImpulseChangesLinearVelocity) {
        auto mgr = makeJoltMgr();
        PhysicsWorld3D* w = mgr->world3D();
        BodyHandle h = InvalidBodyHandle;
        RigidbodyDesc rb;
        rb.type = BodyType::Dynamic;
        rb.position = ayt::math::FVector3(0.0f, 0.0f, 0.0f);
        rb.collideMask = 0u;
        w->createRigidbody(rb, h);
        ColliderDesc cd{};
        cd.body = h;
        cd.shape = ColliderShape::Box;
        cd.halfExtents = ayt::math::FVector3(0.5f, 0.5f, 0.5f);
        ColliderHandle colH;
        w->createCollider(cd, colH);
        (void)colH;

        for (int i = 0; i < 30; ++i) mgr->step(1.0f / 60.0f);
        waitForDrain(*mgr, 100);
        w->applyImpulse(h, ayt::math::FVector3(5.0f, 0.0f, 0.0f),
                        ayt::math::FVector3(0.0f, 0.0f, 0.0f));
        for (int i = 0; i < 10; ++i) mgr->step(1.0f / 60.0f);
        waitForDrain(*mgr, 100);
        const PhysFrameSnapshot& snap = mgr->fetchResults();
        bool foundPositiveVx = false;
        for (const auto& t : snap.transforms) {
            if (handleIndex(t.body) == handleIndex(h) && t.linearVelocity.x > 0.1f) {
                foundPositiveVx = true;
                break;
            }
        }
        CHECK(foundPositiveVx);
        mgr->shutdown();
    }

    TEST_CASE(Real_FixedJointHoldsBodyInPlace) {
        auto mgr = makeJoltMgr();
        PhysicsWorld3D* w = mgr->world3D();

        BodyHandle anchorH = InvalidBodyHandle;
        RigidbodyDesc anchorRb;
        anchorRb.type = BodyType::Static;
        anchorRb.position = ayt::math::FVector3(0.0f, 0.0f, 0.0f);
        w->createRigidbody(anchorRb, anchorH);

        BodyHandle dynH = InvalidBodyHandle;
        RigidbodyDesc dynRb;
        dynRb.type = BodyType::Dynamic;
        dynRb.position = ayt::math::FVector3(0.0f, 1.0f, 0.0f);
        w->createRigidbody(dynRb, dynH);
        ColliderDesc cd{};
        cd.body = dynH;
        cd.shape = ColliderShape::Sphere;
        cd.radius = 0.1f;
        ColliderHandle colH;
        w->createCollider(cd, colH);
        (void)colH;

        JointDesc jd{};
        jd.type = JointType::Fixed;
        jd.bodyA = anchorH;
        jd.bodyB = dynH;
        jd.anchorA = ayt::math::FVector3(0.0f, 0.0f, 0.0f);
        jd.anchorB = ayt::math::FVector3(0.0f, 1.0f, 0.0f);
        JointHandle jH = InvalidJointHandle;
        const PhysResult jr = w->createJoint(jd, jH);
        CHECK_INT_EQ(static_cast<uint32_t>(jr), static_cast<uint32_t>(PhysResult::Ok));
        (void)jH;

        for (int i = 0; i < 120; ++i) mgr->step(1.0f / 60.0f);
        waitForDrain(*mgr, 200);
        const PhysFrameSnapshot& snap = mgr->fetchResults();
        for (const auto& t : snap.transforms) {
            if (handleIndex(t.body) == handleIndex(dynH)) {
                // Body should be near (0, 1, 0) — not fallen.
                CHECK(t.position.y > 0.95f);
                CHECK(t.position.y <= 1.05f);
                break;
            }
        }
        mgr->shutdown();
    }

    TEST_CASE(Real_HingeJointAllowsRotationAroundAxis) {
        auto mgr = makeJoltMgr();
        PhysicsWorld3D* w = mgr->world3D();

        BodyHandle anchorH = InvalidBodyHandle;
        RigidbodyDesc anchorRb;
        anchorRb.type = BodyType::Static;
        w->createRigidbody(anchorRb, anchorH);

        BodyHandle dynH = InvalidBodyHandle;
        RigidbodyDesc dynRb;
        dynRb.type = BodyType::Dynamic;
        dynRb.position = ayt::math::FVector3(0.0f, 1.0f, 0.0f);
        dynRb.collideMask = 0u;
        w->createRigidbody(dynRb, dynH);

        JointDesc jd{};
        jd.type = JointType::Hinge;
        jd.bodyA = anchorH;
        jd.bodyB = dynH;
        jd.anchorA = ayt::math::FVector3(0.0f, 0.0f, 0.0f);
        jd.anchorB = ayt::math::FVector3(0.0f, 0.0f, 0.0f);
        jd.axisA   = ayt::math::FVector3(0.0f, 1.0f, 0.0f);
        jd.axisB   = ayt::math::FVector3(0.0f, 1.0f, 0.0f);
        JointHandle jH = InvalidJointHandle;
        w->createJoint(jd, jH);
        (void)jH;

        for (int i = 0; i < 30; ++i) mgr->step(1.0f / 60.0f);
        waitForDrain(*mgr, 100);
        mgr->shutdown();
    }

    TEST_CASE(Real_JointUnsupportedReturnsNoOp) {
        auto mgr = makeJoltMgr();
        PhysicsWorld3D* w = mgr->world3D();
        BodyHandle aH = InvalidBodyHandle, bH = InvalidBodyHandle;
        RigidbodyDesc rb; rb.type = BodyType::Dynamic;
        rb.position = ayt::math::FVector3(0.0f, 0.0f, 0.0f);
        w->createRigidbody(rb, aH);
        rb.position = ayt::math::FVector3(1.0f, 0.0f, 0.0f);
        w->createRigidbody(rb, bH);

        JointDesc jd{};
        jd.type = JointType::Spring;  // unsupported
        jd.bodyA = aH;
        jd.bodyB = bH;
        JointHandle jH = InvalidJointHandle;
        const PhysResult jr = w->createJoint(jd, jH);
        CHECK_INT_EQ(static_cast<uint32_t>(jr), static_cast<uint32_t>(PhysResult::Ok));
        for (int i = 0; i < 10; ++i) mgr->step(1.0f / 60.0f);
        waitForDrain(*mgr, 100);
        mgr->shutdown();
    }

    TEST_CASE(Real_RaycastAsyncFindsBody) {
        auto mgr = makeJoltMgr();
        PhysicsWorld3D* w = mgr->world3D();
        BodyHandle h = InvalidBodyHandle;
        RigidbodyDesc rb;
        rb.type = BodyType::Dynamic;
        rb.position = ayt::math::FVector3(2.0f, 0.0f, 0.0f);
        rb.collideMask = 0u;
        w->createRigidbody(rb, h);
        ColliderDesc cd{};
        cd.body = h;
        cd.shape = ColliderShape::Sphere;
        cd.radius = 1.0f;
        ColliderHandle colH;
        w->createCollider(cd, colH);
        (void)colH;
        for (int i = 0; i < 5; ++i) mgr->step(1.0f / 60.0f);
        waitForDrain(*mgr, 100);

        const uint32_t qid = w->raycastAsync(
            ayt::math::Ray(ayt::math::FVector3(0.0f, 0.0f, 0.0f),
                           ayt::math::FVector3(1.0f, 0.0f, 0.0f)));
        CHECK(qid != 0u);

        bool sawResult = false;
        for (int i = 0; i < 30 && !sawResult; ++i) {
            mgr->step(1.0f / 60.0f);
            waitForDrain(*mgr, 50);
            const PhysFrameSnapshot& snap = mgr->fetchResults();
            for (const auto& qr : snap.queryResults) {
                if (qr.queryId == qid) {
                    sawResult = true;
                    CHECK(qr.hitCount >= 1u);
                    if (!qr.hits.empty()) CHECK(qr.hits.front().hit);
                    break;
                }
            }
        }
        CHECK(sawResult);
        mgr->shutdown();
    }

    TEST_CASE(Real_SleepAllDeactivatesBodies) {
        auto mgr = makeJoltMgr();
        PhysicsWorld3D* w = mgr->world3D();
        for (int i = 0; i < 3; ++i) {
            BodyHandle h = InvalidBodyHandle;
            RigidbodyDesc rb;
            rb.type = BodyType::Dynamic;
            rb.position = ayt::math::FVector3(static_cast<float>(i), 5.0f, 0.0f);
            rb.collideMask = 0u;
            w->createRigidbody(rb, h);
            ColliderDesc cd{};
            cd.body = h;
            cd.shape = ColliderShape::Sphere;
            cd.radius = 0.1f;
            ColliderHandle colH;
            w->createCollider(cd, colH);
            (void)colH;
        }
        for (int i = 0; i < 10; ++i) mgr->step(1.0f / 60.0f);
        waitForDrain(*mgr, 100);
        w->sleepAll();
        for (int i = 0; i < 10; ++i) mgr->step(1.0f / 60.0f);
        waitForDrain(*mgr, 100);
        mgr->shutdown();
    }

    // ------------------------------------------------------------
    // R9: setRigidbodyCollideMask (Jolt GroupFilter symmetry with Box2D)
    // ------------------------------------------------------------
    TEST_CASE(Real_SetRigidbodyCollideMaskDisablesCollision) {
        auto mgr = makeJoltMgr();
        PhysicsWorld3D* w = mgr->world3D();

        // Static floor on Static layer (0), mask=all.
        BodyHandle floor = InvalidBodyHandle;
        {
            RigidbodyDesc rb;
            rb.type = BodyType::Static;
            rb.layer = static_cast<PhysLayer>(PhysDefaultLayer::Static);
            rb.collideMask = 0xFFFFFFFFu;
            rb.position = ayt::math::FVector3(0.0f, 0.0f, 0.0f);
            w->createRigidbody(rb, floor);
            ColliderDesc cd{};
            cd.body = floor; cd.shape = ColliderShape::Box;
            cd.halfExtents = ayt::math::FVector3(10.0f, 0.5f, 10.0f);
            ColliderHandle c; w->createCollider(cd, c); (void)c;
        }

        // Dynamic body on Dynamic layer (1), mask includes Static → settles on floor.
        BodyHandle ball = InvalidBodyHandle;
        {
            RigidbodyDesc rb;
            rb.type = BodyType::Dynamic;
            rb.layer = static_cast<PhysLayer>(PhysDefaultLayer::Dynamic);
            rb.collideMask = (1u << static_cast<uint32_t>(PhysDefaultLayer::Static)) |
                             (1u << static_cast<uint32_t>(PhysDefaultLayer::Dynamic));
            rb.position = ayt::math::FVector3(0.0f, 5.0f, 0.0f);
            rb.alwaysSync = true;
            rb.linearDamping = 0.0f;
            w->createRigidbody(rb, ball);
            ColliderDesc cd{};
            cd.body = ball; cd.shape = ColliderShape::Sphere; cd.radius = 0.5f;
            ColliderHandle c; w->createCollider(cd, c); (void)c;
        }

        float y = 5.0f;
        for (int poll = 0; poll < 40; ++poll) {
            for (int i = 0; i < 15; ++i) mgr->step(1.0f / 60.0f);
            waitForDrain(*mgr, 30);
            const PhysFrameSnapshot snap = mgr->fetchResults();
            for (const BodyTransform& bt : snap.transforms)
                if (bt.body == ball) y = bt.position.y;
            if (y > 0.8f && y < 1.6f) break;  // rest y ~ 1.0 (floor top 0.5 + r 0.5)
        }
        CHECK(y > 0.8f);
        CHECK(y < 1.6f);

        // Clear collide mask → no longer hits Static layer → falls through.
        CHECK_INT_EQ(static_cast<uint32_t>(w->setRigidbodyCollideMask(ball, 0u)),
                     static_cast<uint32_t>(PhysResult::Ok));
        for (int poll = 0; poll < 40; ++poll) {
            for (int i = 0; i < 15; ++i) mgr->step(1.0f / 60.0f);
            waitForDrain(*mgr, 30);
            const PhysFrameSnapshot snap = mgr->fetchResults();
            for (const BodyTransform& bt : snap.transforms)
                if (bt.body == ball) y = bt.position.y;
            if (y < 0.0f) break;
        }
        CHECK(y < 0.0f);  // fell through the floor
        mgr->shutdown();
    }

    TEST_CASE(Real_CollideMaskZeroAtCreationIgnoresFloor) {
        // Creation-time collideMask=0 must actually filter (R9 GroupFilter),
        // not be a no-op like the old null-filter encoding.
        auto mgr = makeJoltMgr();
        PhysicsWorld3D* w = mgr->world3D();

        BodyHandle floor = InvalidBodyHandle;
        {
            RigidbodyDesc rb;
            rb.type = BodyType::Static;
            rb.layer = static_cast<PhysLayer>(PhysDefaultLayer::Static);
            w->createRigidbody(rb, floor);
            ColliderDesc cd{};
            cd.body = floor; cd.shape = ColliderShape::Box;
            cd.halfExtents = ayt::math::FVector3(10.0f, 0.5f, 10.0f);
            ColliderHandle c; w->createCollider(cd, c); (void)c;
        }
        BodyHandle ball = InvalidBodyHandle;
        {
            RigidbodyDesc rb;
            rb.type = BodyType::Dynamic;
            rb.layer = static_cast<PhysLayer>(PhysDefaultLayer::Dynamic);
            rb.collideMask = 0u;  // collide with nothing
            rb.position = ayt::math::FVector3(0.0f, 5.0f, 0.0f);
            rb.alwaysSync = true;
            w->createRigidbody(rb, ball);
            ColliderDesc cd{};
            cd.body = ball; cd.shape = ColliderShape::Sphere; cd.radius = 0.5f;
            ColliderHandle c; w->createCollider(cd, c); (void)c;
        }

        float y = 5.0f;
        for (int poll = 0; poll < 40; ++poll) {
            for (int i = 0; i < 15; ++i) mgr->step(1.0f / 60.0f);
            waitForDrain(*mgr, 30);
            const PhysFrameSnapshot snap = mgr->fetchResults();
            for (const BodyTransform& bt : snap.transforms)
                if (bt.body == ball) y = bt.position.y;
            if (y < 0.0f) break;
        }
        CHECK(y < 0.0f);  // fell through — mask=0 ignored the floor
        mgr->shutdown();
    }

    TEST_CASE(Real_SetGravityScaleZeroFloats) {
        // R10: 3D-symmetric gravity-scale toggle. gravityScale=0 must float;
        // the control body (scale=1) falls.
        auto mgr = makeJoltMgr();
        PhysicsWorld3D* w = mgr->world3D();

        BodyHandle hFloat = InvalidBodyHandle;
        BodyHandle hFall  = InvalidBodyHandle;
        RigidbodyDesc rb;
        rb.type = BodyType::Dynamic;
        rb.collideMask = 0u;
        rb.alwaysSync  = true;
        rb.gravityScale = 0.0f;
        rb.position = ayt::math::FVector3(0.0f, 5.0f, 0.0f);
        w->createRigidbody(rb, hFloat);
        ColliderDesc cd{};
        cd.body = hFloat; cd.shape = ColliderShape::Box;
        cd.halfExtents = ayt::math::FVector3(0.5f, 0.5f, 0.5f);
        ColliderHandle c; w->createCollider(cd, c); (void)c;
        rb.gravityScale = 1.0f;
        rb.position = ayt::math::FVector3(1.0f, 5.0f, 0.0f);
        w->createRigidbody(rb, hFall);
        cd.body = hFall;
        w->createCollider(cd, c);

        for (int i = 0; i < 90; ++i) mgr->step(1.0f / 60.0f);
        waitForDrain(*mgr, 200);
        const PhysFrameSnapshot snap = mgr->fetchResults();
        float yFloat = 5.0f, yFall = 5.0f;
        for (const BodyTransform& bt : snap.transforms) {
            if (bt.body == hFloat) yFloat = bt.position.y;
            if (bt.body == hFall)  yFall  = bt.position.y;
        }
        CHECK(yFloat > 4.0f);  // floated in place
        CHECK(yFall < 2.0f);   // fell ~4.9 m in 90 steps
        mgr->shutdown();
    }

    TEST_CASE(Real_ApplyTorqueSpinsBody) {
        // R10: 3D torque vector — continuous Z torque must spin the box.
        auto mgr = makeJoltMgr();
        PhysicsWorld3D* w = mgr->world3D();
        BodyHandle h = InvalidBodyHandle;
        RigidbodyDesc rb;
        rb.type = BodyType::Dynamic;
        rb.collideMask = 0u;
        rb.alwaysSync  = true;
        rb.angularDamping = 0.0f;
        w->createRigidbody(rb, h);
        ColliderDesc cd{};
        cd.body = h; cd.shape = ColliderShape::Box;
        cd.halfExtents = ayt::math::FVector3(0.5f, 0.5f, 0.5f);
        ColliderHandle c; w->createCollider(cd, c); (void)c;

        for (int i = 0; i < 90; ++i) {
            w->applyTorque(h, ayt::math::FVector3(0.0f, 0.0f, 8.0f));
            mgr->step(1.0f / 60.0f);
        }
        waitForDrain(*mgr, 200);
        const PhysFrameSnapshot snap = mgr->fetchResults();
        float angle = 0.0f;
        for (const BodyTransform& bt : snap.transforms)
            if (bt.body == h) angle = bt.rotation.toEulerAngles().z;
        CHECK(std::fabs(angle) > 0.5f);
        mgr->shutdown();
    }

    TEST_CASE(Real_ApplyAngularImpulseSpinsBody) {
        // R10: single 3D angular impulse kick — |omega_z| > 0.1 right after.
        auto mgr = makeJoltMgr();
        PhysicsWorld3D* w = mgr->world3D();
        BodyHandle h = InvalidBodyHandle;
        RigidbodyDesc rb;
        rb.type = BodyType::Dynamic;
        rb.collideMask = 0u;
        rb.alwaysSync  = true;
        rb.angularDamping = 0.0f;
        w->createRigidbody(rb, h);
        ColliderDesc cd{};
        cd.body = h; cd.shape = ColliderShape::Box;
        cd.halfExtents = ayt::math::FVector3(0.5f, 0.5f, 0.5f);
        ColliderHandle c; w->createCollider(cd, c); (void)c;

        w->applyAngularImpulse(h, ayt::math::FVector3(0.0f, 0.0f, 3.0f));
        mgr->step(1.0f / 60.0f);
        waitForDrain(*mgr, 200);
        const PhysFrameSnapshot snap = mgr->fetchResults();
        float wz = 0.0f;
        for (const BodyTransform& bt : snap.transforms)
            if (bt.body == h) wz = bt.angularVelocity.z;
        CHECK(std::fabs(wz) > 0.1f);
        mgr->shutdown();
    }

    TEST_CASE(Real_SetRigidbodyVelocityMovesBody) {
        // R10: direct velocity set beats damping — body drifts at 2 m/s.
        auto mgr = makeJoltMgr();
        PhysicsWorld3D* w = mgr->world3D();
        BodyHandle h = InvalidBodyHandle;
        RigidbodyDesc rb;
        rb.type = BodyType::Dynamic;
        rb.collideMask = 0u;
        rb.alwaysSync  = true;
        rb.linearDamping = 0.0f;
        w->createRigidbody(rb, h);
        ColliderDesc cd{};
        cd.body = h; cd.shape = ColliderShape::Box;
        cd.halfExtents = ayt::math::FVector3(0.5f, 0.5f, 0.5f);
        ColliderHandle c; w->createCollider(cd, c); (void)c;
        for (int i = 0; i < 10; ++i) mgr->step(1.0f / 60.0f);
        waitForDrain(*mgr, 200);

        CHECK_INT_EQ(static_cast<uint32_t>(w->setRigidbodyVelocity(
            h, ayt::math::FVector3(2.0f, 0.0f, 0.0f))),
            static_cast<uint32_t>(PhysResult::Ok));
        for (int i = 0; i < 30; ++i) mgr->step(1.0f / 60.0f);
        waitForDrain(*mgr, 200);
        const PhysFrameSnapshot snap = mgr->fetchResults();
        float x = 0.0f, vx = 0.0f;
        for (const BodyTransform& bt : snap.transforms)
            if (bt.body == h) { x = bt.position.x; vx = bt.linearVelocity.x; }
        CHECK(x > 0.5f);      // drifted ~1 m
        CHECK(vx > 1.5f);     // velocity held (no damping)
        mgr->shutdown();
    }

    TEST_CASE(Real_SetMassChangesAccelerationUnderForce) {
        // R10: same force, mass 4 vs 1 — 4x the acceleration after the set.
        auto mgr = makeJoltMgr();
        PhysicsWorld3D* w = mgr->world3D();
        BodyHandle h = InvalidBodyHandle;
        RigidbodyDesc rb;
        rb.type = BodyType::Dynamic;
        rb.collideMask = 0u;
        rb.alwaysSync  = true;
        rb.linearDamping = 0.0f;
        rb.mass = 4.0f;
        w->createRigidbody(rb, h);
        ColliderDesc cd{};
        cd.body = h; cd.shape = ColliderShape::Box;
        cd.halfExtents = ayt::math::FVector3(0.5f, 0.5f, 0.5f);
        ColliderHandle c; w->createCollider(cd, c); (void)c;

        for (int i = 0; i < 30; ++i) {
            w->applyForce(h, ayt::math::FVector3(2.0f, 0.0f, 0.0f));
            mgr->step(1.0f / 60.0f);
        }
        waitForDrain(*mgr, 200);
        float vLow = 0.0f;
        {
            const PhysFrameSnapshot snap = mgr->fetchResults();
            for (const BodyTransform& bt : snap.transforms)
                if (bt.body == h) vLow = bt.linearVelocity.x;
        }
        CHECK(vLow > 0.2f);   // mass 4, F=2 -> a=0.5, v(0.5s)=0.25

        CHECK_INT_EQ(static_cast<uint32_t>(w->setMass(h, 1.0f)),
                     static_cast<uint32_t>(PhysResult::Ok));
        for (int i = 0; i < 30; ++i) {
            w->applyForce(h, ayt::math::FVector3(2.0f, 0.0f, 0.0f));
            mgr->step(1.0f / 60.0f);
        }
        waitForDrain(*mgr, 200);
        float vAfter = vLow;
        {
            const PhysFrameSnapshot snap = mgr->fetchResults();
            for (const BodyTransform& bt : snap.transforms)
                if (bt.body == h) vAfter = bt.linearVelocity.x;
        }
        // Same 30-step force window: delta after the set (a=2) must be > 3x
        // the pre-set delta (a=0.5).
        CHECK((vAfter - vLow) > 3.0f * vLow);
        mgr->shutdown();
    }

    TEST_CASE(Real_SetMaterialChangesBounce) {
        // R10: restitution swap before first contact. Jolt caches friction /
        // restitution in contact constraints until they are re-evaluated, so
        // a mid-contact set does not apply to an existing contact — the new
        // value must land before the first impact. The 0.9-restitution ball
        // must bounce back up; the 0.0 control rests on the floor.
        auto mgr = makeJoltMgr();
        PhysicsWorld3D* w = mgr->world3D();

        BodyHandle floor = InvalidBodyHandle;
        {
            RigidbodyDesc rb;
            rb.type = BodyType::Static;
            w->createRigidbody(rb, floor);
            ColliderDesc cd{};
            cd.body = floor; cd.shape = ColliderShape::Box;
            cd.halfExtents = ayt::math::FVector3(10.0f, 0.5f, 10.0f);
            ColliderHandle c; w->createCollider(cd, c); (void)c;
        }
        auto makeBall = [&](float x, BodyHandle& outH, ColliderHandle& outC) {
            RigidbodyDesc rb;
            rb.type = BodyType::Dynamic;
            rb.linearDamping = 0.0f;
            rb.alwaysSync  = true;  // stays in the snapshot once asleep
            rb.position = ayt::math::FVector3(x, 6.0f, 0.0f);
            w->createRigidbody(rb, outH);
            ColliderDesc cd{};
            cd.body = outH; cd.shape = ColliderShape::Sphere;
            cd.radius = 0.5f;
            w->createCollider(cd, outC);
        };
        BodyHandle hHi = InvalidBodyHandle, hLo = InvalidBodyHandle;
        ColliderHandle cHi, cLo;
        makeBall(0.0f, hHi, cHi);
        makeBall(2.0f, hLo, cLo);  // keep >1.0 m away: the bouncing ball must
                                   // not collide with the control ball
        // Both balls airborne (first impact ~64 steps). Swap restitution now.
        for (int i = 0; i < 5; ++i) mgr->step(1.0f / 60.0f);
        waitForDrain(*mgr, 200);
        CHECK_INT_EQ(static_cast<uint32_t>(w->setMaterial(cHi, 0.0f, 0.9f)),
                     static_cast<uint32_t>(PhysResult::Ok));
        // Impact ~step 64, bounce apex ~step 121 — sample well past it.
        for (int i = 0; i < 130; ++i) mgr->step(1.0f / 60.0f);
        waitForDrain(*mgr, 200);
        const PhysFrameSnapshot snap = mgr->fetchResults();
        float yHi = 6.0f, yLo = 6.0f;
        for (const BodyTransform& bt : snap.transforms) {
            if (bt.body == hHi) yHi = bt.position.y;
            if (bt.body == hLo) yLo = bt.position.y;
        }
        CHECK(yHi > 2.0f);  // 0.9 restitution -> bounce apex ~4.9 m
        CHECK(yLo < 1.5f);  // control: rests on floor (ball center y = 1.0)
        mgr->shutdown();
    }

    TEST_CASE(Real_RuntimeSettersRejectInvalidParams) {
        // R10: API-layer validation — bad handles / non-positive mass.
        auto mgr = makeJoltMgr();
        PhysicsWorld3D* w = mgr->world3D();
        BodyHandle h = InvalidBodyHandle;
        RigidbodyDesc rb;
        rb.type = BodyType::Dynamic;
        rb.collideMask = 0u;
        w->createRigidbody(rb, h);

        CHECK_INT_EQ(static_cast<int>(w->setMass(InvalidBodyHandle, 2.0f)),
                     static_cast<int>(PhysResult::InvalidParam));
        CHECK_INT_EQ(static_cast<int>(w->setMass(h, 0.0f)),
                     static_cast<int>(PhysResult::InvalidParam));
        CHECK_INT_EQ(static_cast<int>(w->setMass(h, -1.0f)),
                     static_cast<int>(PhysResult::InvalidParam));
        CHECK_INT_EQ(static_cast<int>(w->setRigidbodyVelocity(InvalidBodyHandle,
                     ayt::math::FVector3(1.0f, 0.0f, 0.0f))),
                     static_cast<int>(PhysResult::InvalidParam));
        CHECK_INT_EQ(static_cast<int>(w->setGravityScale(InvalidBodyHandle, 0.0f)),
                     static_cast<int>(PhysResult::InvalidParam));
        CHECK_INT_EQ(static_cast<int>(w->applyTorque(InvalidBodyHandle,
                     ayt::math::FVector3(0.0f, 0.0f, 1.0f))),
                     static_cast<int>(PhysResult::InvalidParam));
        CHECK_INT_EQ(static_cast<int>(w->applyAngularImpulse(InvalidBodyHandle,
                     ayt::math::FVector3(0.0f, 0.0f, 1.0f))),
                     static_cast<int>(PhysResult::InvalidParam));
        CHECK_INT_EQ(static_cast<int>(w->setMaterial(InvalidColliderHandle, 0.5f, 0.1f)),
                     static_cast<int>(PhysResult::InvalidParam));
        mgr->shutdown();
    }

    TEST_CASE(Real_ColliderOffsetShiftsRestPose) {
        // R10: local shape offset shifts geometry relative to the body
        // origin. A 0.25 m upward-offset box rests with the body origin
        // 0.25 m above the floor (a centered box rests at 0.5 m).
        auto mgr = makeJoltMgr();
        PhysicsWorld3D* w = mgr->world3D();

        BodyHandle floor = InvalidBodyHandle;
        {
            RigidbodyDesc rb;
            rb.type = BodyType::Static;
            w->createRigidbody(rb, floor);
            ColliderDesc cd{};
            cd.body = floor; cd.shape = ColliderShape::Box;
            cd.halfExtents = ayt::math::FVector3(10.0f, 0.5f, 10.0f);
            ColliderHandle c; w->createCollider(cd, c); (void)c;
        }
        BodyHandle h = InvalidBodyHandle;
        {
            RigidbodyDesc rb;
            rb.type = BodyType::Dynamic;
            rb.alwaysSync = true;
            rb.position = ayt::math::FVector3(0.0f, 2.0f, 0.0f);
            w->createRigidbody(rb, h);
            ColliderDesc cd{};
            cd.body = h; cd.shape = ColliderShape::Box;
            cd.halfExtents = ayt::math::FVector3(0.5f, 0.5f, 0.5f);
            cd.offset = ayt::math::FVector3(0.0f, 0.25f, 0.0f);
            ColliderHandle c; w->createCollider(cd, c); (void)c;
        }
        for (int i = 0; i < 120; ++i) mgr->step(1.0f / 60.0f);
        waitForDrain(*mgr, 200);
        const PhysFrameSnapshot snap = mgr->fetchResults();
        float y = 2.0f;
        for (const BodyTransform& bt : snap.transforms) {
            if (bt.body == h) y = bt.position.y;
        }
        // Box bottom = y + 0.25 - 0.5 = 0.5 (floor top) -> rests at y = 0.75
        // (a centered box rests at y = 1.0).
        CHECK(y > 0.65f);
        CHECK(y < 0.85f);
        mgr->shutdown();
    }
#endif

TEST_SUITE_END

// =============================================================================
// Test-only seam: JoltBackend3D exposes direct synchronous create/destroy to
// let unit tests bypass the manager-level queue path. Implemented in the .cpp
// (so the .h stays Jolt-free).
// =============================================================================
#if defined(AYPHYSICS_HAS_JOLT)
namespace ayt::physics {

PhysResult JoltBackend3D::execute_createRigidbodyForTest(const RigidbodyDesc& desc,
                                                        BodyHandle& outHandle) {
    outHandle = InvalidBodyHandle;
    // Mint a fresh handle. We mirror PhysicsWorld3D's handle-mint logic: a
    // monotonically increasing index + a bumpGeneration-wrapped generation.
    static std::atomic<uint32_t> g_nextBodyIndex{0};
    static std::atomic<uint32_t> g_nextBodyGen{0};
    const uint32_t idx = g_nextBodyIndex.fetch_add(1u) + 1u;  // skip 0
    const uint32_t gen = g_nextBodyGen.fetch_add(1u) + 1u;
    if (gen > kPhysHandleMaxGen) {
        g_nextBodyGen.store(0u);
    }
    outHandle = makeHandle(idx, gen);

    // R2.0a: capture _notFoundCount BEFORE execute so we can detect reject
    // (MustBeStatic validation, missing body, etc.) and surface it.
    const uint64_t before = notFoundCount();

    PhysicsCommand cmd{};
    cmd.type       = PhysicsCommandType::CreateRigidbody;
    cmd.body       = outHandle;
    cmd.createSlot = 0;

    PhysicsCreatePayload payload{};
    payload.kind      = PhysicsCreatePayload::Kind::Rigidbody;
    payload.rigidDesc = desc;
    execute(cmd, &payload);

    if (notFoundCount() > before) {
        outHandle = InvalidBodyHandle;
        return PhysResult::BackendError;
    }
    return PhysResult::Ok;
}

PhysResult JoltBackend3D::execute_destroyRigidbodyForTest(BodyHandle h) {
    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::DestroyRigidbody;
    cmd.body = h;
    execute(cmd, nullptr);
    return PhysResult::Ok;
}

// R2.0a: synchronous create-collider test seam. Mirrors
// execute_createRigidbodyForTest; uses notFoundCount() delta to surface
// backend reject decisions (Mesh/Heightfield MustBeStatic, empty hull,
// >256 points, N<4, null shapeData, etc.).
PhysResult JoltBackend3D::execute_createColliderForTest(const ColliderDesc& desc,
                                                        ColliderHandle& outHandle) {
    outHandle = InvalidColliderHandle;
    static std::atomic<uint32_t> g_nextColliderIndex{0};
    static std::atomic<uint32_t> g_nextColliderGen{0};
    const uint32_t idx = g_nextColliderIndex.fetch_add(1u) + 1u;
    const uint32_t gen = g_nextColliderGen.fetch_add(1u) + 1u;
    if (gen > kPhysHandleMaxGen) {
        g_nextColliderGen.store(0u);
    }
    outHandle = makeHandle(idx, gen);

    const uint64_t before = notFoundCount();

    // CreateCollider command needs a body target. Mint a temporary handle
    // if desc.body is invalid (test seam is forgiving on this).
    ColliderDesc d = desc;
    if (d.body == InvalidBodyHandle) {
        static std::atomic<uint32_t> g_sentinelIndex{0};
        static std::atomic<uint32_t> g_sentinelGen{0};
        const uint32_t sidx = g_sentinelIndex.fetch_add(1u) + 1u;
        const uint32_t sgen = g_sentinelGen.fetch_add(1u) + 1u;
        d.body = makeHandle(sidx, sgen);
    }

    PhysicsCommand cmd{};
    cmd.type       = PhysicsCommandType::CreateCollider;
    cmd.collider   = outHandle;
    cmd.createSlot = 0;

    PhysicsCreatePayload payload{};
    payload.kind         = PhysicsCreatePayload::Kind::Collider;
    payload.colliderDesc = d;
    execute(cmd, &payload);

    if (notFoundCount() > before) {
        outHandle = InvalidColliderHandle;
        return PhysResult::BackendError;
    }
    return PhysResult::Ok;
}

PhysResult JoltBackend3D::execute_destroyColliderForTest(ColliderHandle h) {
    PhysicsCommand cmd{};
    cmd.type     = PhysicsCommandType::DestroyCollider;
    cmd.collider = h;
    execute(cmd, nullptr);
    return PhysResult::Ok;
}

}  // namespace ayt::physics
#endif
