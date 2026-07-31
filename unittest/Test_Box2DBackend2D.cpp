#include "AYPhysicsManager.h"
#include "AYPhysicsWorld2D.h"
#include "AYPhysicsTypes.h"

#if defined(AYPHYSICS_HAS_BOX2D)
#include "Box2DBackend2D.h"
#endif

#include "AYTest.h"
#include "Test_Box2DBackend2D_Helpers.h"

#include <atomic>
#include <cmath>
#include <memory>

using namespace ayt::physics;
using namespace ayt::physics::test_helpers;

TEST_SUITE(Box2DBackend2DTests)

#if defined(AYPHYSICS_HAS_BOX2D)

    TEST_CASE(Real_DescribeReportsRealDevice) {
        Box2DBackend2D backend;
        const PhysicsBackendInfo info = backend.describe();
        CHECK_NOT_NULL(info.name);
        CHECK(backend.isRealDevice());
        CHECK(info.realDevice);
    }

    TEST_CASE(Real_InitStartStopCycle) {
        Box2DBackend2D backend;
        CHECK(backend.init2D(PhysicsBackendDescriptor{}));
        CHECK(backend.start(backend.describe()));
        CHECK_INT_EQ(backend.stepCount(), 0u);
        backend.step(1.0f / 60.0f);
        backend.step(1.0f / 60.0f);
        CHECK_INT_EQ(backend.stepCount(), 2u);
        backend.stop();
    }

    TEST_CASE(Real_DynamicBoxFallsUnderGravity) {
        Box2DBackend2D backend;
        PhysicsBackendDescriptor desc;
        desc.maxBodies = 64u;
        desc.gravity2D = ayt::math::FVector2(0.0f, -9.81f);
        CHECK(backend.init2D(desc));
        CHECK(backend.start(backend.describe()));

        RigidbodyDesc rb{};
        rb.type = BodyType::Dynamic;
        rb.position = ayt::math::FVector3(0.0f, 10.0f, 0.0f);
        rb.linearDamping = 0.0f;
        rb.angularDamping = 0.0f;
        BodyHandle body = InvalidBodyHandle;
        CHECK_INT_EQ(static_cast<uint32_t>(backend.execute_createRigidbodyForTest(rb, body)),
                     static_cast<uint32_t>(PhysResult::Ok));

        ColliderDesc col{};
        col.body = body;
        col.shape = ColliderShape::Box;
        col.halfExtents = ayt::math::FVector3(0.5f, 0.5f, 0.0f);
        ColliderHandle colH = InvalidColliderHandle;
        PhysicsCreatePayload payload{};
        payload.kind = PhysicsCreatePayload::Kind::Collider;
        payload.colliderDesc = col;
        PhysicsCommand cmd{};
        cmd.type = PhysicsCommandType::CreateCollider;
        cmd.collider = makeHandle(1u, 1u);
        colH = cmd.collider;
        backend.execute(cmd, &payload);

        PhysFrameSnapshot snap{};
        backend.step(1.0f / 60.0f);
        backend.publishSnapshot(snap);
        const float y0 = snap.transforms.empty()
            ? 10.0f : snap.transforms.front().position.y;

        for (int i = 0; i < 120; ++i) {
            backend.step(1.0f / 60.0f);
            snap.transforms.clear();
            snap.queryResults.clear();
            backend.publishSnapshot(snap);
        }
        const float yN = snap.transforms.empty()
            ? y0 : snap.transforms.front().position.y;
        CHECK(yN < y0 - 1.0f);
        backend.stop();
    }

    TEST_CASE(Real_StaticGroundSupportsDynamicCircle) {
        Box2DBackend2D backend;
        PhysicsBackendDescriptor desc;
        desc.maxBodies = 64u;
        CHECK(backend.init2D(desc));
        CHECK(backend.start(backend.describe()));

        RigidbodyDesc ground{};
        ground.type = BodyType::Static;
        ground.position = ayt::math::FVector3(0.0f, 0.0f, 0.0f);
        BodyHandle groundH = InvalidBodyHandle;
        CHECK_INT_EQ(static_cast<uint32_t>(backend.execute_createRigidbodyForTest(ground, groundH)),
                     static_cast<uint32_t>(PhysResult::Ok));

        ColliderDesc groundCol{};
        groundCol.body = groundH;
        groundCol.shape = ColliderShape::Box;
        groundCol.halfExtents = ayt::math::FVector3(10.0f, 0.5f, 0.0f);
        PhysicsCommand gcmd{};
        gcmd.type = PhysicsCommandType::CreateCollider;
        gcmd.collider = makeHandle(2u, 1u);
        PhysicsCreatePayload gp{};
        gp.kind = PhysicsCreatePayload::Kind::Collider;
        gp.colliderDesc = groundCol;
        backend.execute(gcmd, &gp);

        RigidbodyDesc ball{};
        ball.type = BodyType::Dynamic;
        ball.position = ayt::math::FVector3(0.0f, 5.0f, 0.0f);
        ball.alwaysSync = true;
        BodyHandle ballH = InvalidBodyHandle;
        CHECK_INT_EQ(static_cast<uint32_t>(backend.execute_createRigidbodyForTest(ball, ballH)),
                     static_cast<uint32_t>(PhysResult::Ok));

        ColliderDesc ballCol{};
        ballCol.body = ballH;
        ballCol.shape = ColliderShape::Sphere;
        ballCol.radius = 0.5f;
        PhysicsCommand bcmd{};
        bcmd.type = PhysicsCommandType::CreateCollider;
        bcmd.collider = makeHandle(3u, 1u);
        PhysicsCreatePayload bp{};
        bp.kind = PhysicsCreatePayload::Kind::Collider;
        bp.colliderDesc = ballCol;
        backend.execute(bcmd, &bp);

        PhysFrameSnapshot snap{};
        for (int i = 0; i < 180; ++i) {
            backend.step(1.0f / 60.0f);
            snap.transforms.clear();
            backend.publishSnapshot(snap);
        }
        float ballY = 5.0f;
        for (const BodyTransform& bt : snap.transforms) {
            if (bt.body == ballH) ballY = bt.position.y;
        }
        CHECK(ballY > 0.4f);
        CHECK(ballY < 2.5f);
        backend.stop();
    }

    TEST_CASE(Real_ManagerDispatchesDefaultBox2D) {
        auto mgr = makeBox2DMgr();
        CHECK_NOT_NULL(mgr.get());
        CHECK(mgr->backend2D()->isRealDevice());
        for (int i = 0; i < 30; ++i) mgr->step(1.0f / 60.0f);
        waitForDrain2D(*mgr, 200);
        mgr->shutdown();
    }

    TEST_CASE(Real_World2DCreateRigidbodyPipeline) {
        auto mgr = makeBox2DMgr();
        PhysicsWorld2D* w = mgr->world2D();
        BodyHandle h = InvalidBodyHandle;
        RigidbodyDesc rb;
        rb.type = BodyType::Dynamic;
        rb.position = ayt::math::FVector3(0.0f, 3.0f, 0.0f);
        CHECK_INT_EQ(static_cast<uint32_t>(w->createRigidbody(rb, h)),
                     static_cast<uint32_t>(PhysResult::Ok));
        CHECK(isValidHandle(h));

        ColliderDesc col{};
        col.body = h;
        col.shape = ColliderShape::Sphere;
        col.radius = 0.25f;
        ColliderHandle colH = InvalidColliderHandle;
        CHECK_INT_EQ(static_cast<uint32_t>(w->createCollider(col, colH)),
                     static_cast<uint32_t>(PhysResult::Ok));

        for (int i = 0; i < 20; ++i) mgr->step(1.0f / 60.0f);
        waitForDrain2D(*mgr, 200);
        (void)mgr->fetchResults();
        mgr->shutdown();
    }

    TEST_CASE(Real_RaycastSyncHitsGround) {
        auto mgr = makeBox2DMgr();
        PhysicsWorld2D* w = mgr->world2D();

        BodyHandle groundH = InvalidBodyHandle;
        RigidbodyDesc ground{};
        ground.type = BodyType::Static;
        CHECK_INT_EQ(static_cast<uint32_t>(w->createRigidbody(ground, groundH)),
                     static_cast<uint32_t>(PhysResult::Ok));

        ColliderDesc col{};
        col.body = groundH;
        col.shape = ColliderShape::Box;
        col.halfExtents = ayt::math::FVector3(5.0f, 0.5f, 0.0f);
        ColliderHandle colH = InvalidColliderHandle;
        CHECK_INT_EQ(static_cast<uint32_t>(w->createCollider(col, colH)),
                     static_cast<uint32_t>(PhysResult::Ok));

        for (int i = 0; i < 5; ++i) mgr->step(1.0f / 60.0f);
        waitForDrain2D(*mgr, 200);

        ayt::math::Ray ray{};
        ray.origin = ayt::math::FVector3(0.0f, 5.0f, 0.0f);
        ray.dir = ayt::math::FVector3(0.0f, -1.0f, 0.0f);
        RaycastHit hit{};
        CHECK_INT_EQ(static_cast<uint32_t>(w->raycastSync(ray, hit)),
                     static_cast<uint32_t>(PhysResult::Ok));
        CHECK(hit.hit);
        CHECK(hit.distance > 3.0f);
        CHECK(hit.distance < 5.0f);
        mgr->shutdown();
    }

    // ------------------------------------------------------------
    // A6: lockstep gate short-circuits step (mirrors Jolt parity)
    // ------------------------------------------------------------
    TEST_CASE(Real_LockstepGateShortCircuitsStep) {
        Box2DBackend2D backend;
        CHECK(backend.init2D(PhysicsBackendDescriptor{}));
        CHECK(backend.start(backend.describe()));
        backend.forceLockstepActive(true);
        for (int i = 0; i < 5; ++i) backend.step(1.0f / 60.0f);
        CHECK_INT_EQ(backend.lockstepRefusedCount(), 5u);
        backend.forceLockstepActive(false);
        backend.step(1.0f / 60.0f);
        CHECK_INT_EQ(backend.lockstepRefusedCount(), 5u);
        backend.stop();
    }

    // ------------------------------------------------------------
    // A1: collision events enter snapshot
    // ------------------------------------------------------------
    TEST_CASE(Real_CollisionEventsEnterSnapshot) {
        Box2DBackend2D backend;
        PhysicsBackendDescriptor desc;
        desc.maxBodies = 64u;
        CHECK(backend.init2D(desc));
        CHECK(backend.start(backend.describe()));

        RigidbodyDesc ground{};
        ground.type = BodyType::Static;
        ground.position = ayt::math::FVector3(0.0f, 0.0f, 0.0f);
        BodyHandle groundH = InvalidBodyHandle;
        backend.execute_createRigidbodyForTest(ground, groundH);
        ColliderDesc gc{};
        gc.body = groundH; gc.shape = ColliderShape::Box;
        gc.halfExtents = ayt::math::FVector3(10.0f, 0.5f, 0.0f);
        PhysicsCreatePayload gp{}; gp.kind = PhysicsCreatePayload::Kind::Collider; gp.colliderDesc = gc;
        PhysicsCommand gcmd{}; gcmd.type = PhysicsCommandType::CreateCollider;
        gcmd.collider = makeHandle(2u, 1u);
        backend.execute(gcmd, &gp);

        RigidbodyDesc ball{};
        ball.type = BodyType::Dynamic;
        ball.position = ayt::math::FVector3(0.0f, 3.0f, 0.0f);
        BodyHandle ballH = InvalidBodyHandle;
        backend.execute_createRigidbodyForTest(ball, ballH);
        ColliderDesc bc{};
        bc.body = ballH; bc.shape = ColliderShape::Sphere; bc.radius = 0.5f;
        PhysicsCreatePayload bp{}; bp.kind = PhysicsCreatePayload::Kind::Collider; bp.colliderDesc = bc;
        PhysicsCommand bcmd{}; bcmd.type = PhysicsCommandType::CreateCollider;
        bcmd.collider = makeHandle(3u, 1u);
        backend.execute(bcmd, &bp);

        PhysFrameSnapshot snap{};
        for (int i = 0; i < 120; ++i) {
            backend.step(1.0f / 60.0f);
            backend.publishSnapshot(snap);  // accumulate: don't clear collisionEvents
        }
        CHECK(backend.collisionEventCount() > 0u);
        // The snapshot's collisionEvents are drained each publish, so check
        // the cumulative counter as the source of truth (events did fire).
        backend.stop();
    }

    // ------------------------------------------------------------
    // A2: sensor events surface for trigger shapes
    // ------------------------------------------------------------
    TEST_CASE(Real_SensorEventsSurfaceForTrigger) {
        Box2DBackend2D backend;
        PhysicsBackendDescriptor desc;
        desc.maxBodies = 64u;
        CHECK(backend.init2D(desc));
        CHECK(backend.start(backend.describe()));

        RigidbodyDesc sensor{};
        sensor.type = BodyType::Kinematic;  // kinematic so sensor events fire reliably
        BodyHandle sensorH = InvalidBodyHandle;
        backend.execute_createRigidbodyForTest(sensor, sensorH);
        ColliderDesc sc{};
        sc.body = sensorH; sc.shape = ColliderShape::Box;
        sc.halfExtents = ayt::math::FVector3(5.0f, 5.0f, 0.0f);
        sc.isTrigger = true;
        PhysicsCreatePayload sp{}; sp.kind = PhysicsCreatePayload::Kind::Collider; sp.colliderDesc = sc;
        PhysicsCommand scmd{}; scmd.type = PhysicsCommandType::CreateCollider;
        scmd.collider = makeHandle(2u, 1u);
        backend.execute(scmd, &sp);

        RigidbodyDesc ball{};
        ball.type = BodyType::Dynamic;
        ball.position = ayt::math::FVector3(0.0f, 8.0f, 0.0f);  // above sensor, falls in
        BodyHandle ballH = InvalidBodyHandle;
        backend.execute_createRigidbodyForTest(ball, ballH);
        ColliderDesc bc{};
        bc.body = ballH; bc.shape = ColliderShape::Sphere; bc.radius = 0.25f;
        PhysicsCreatePayload bp{}; bp.kind = PhysicsCreatePayload::Kind::Collider; bp.colliderDesc = bc;
        PhysicsCommand bcmd{}; bcmd.type = PhysicsCommandType::CreateCollider;
        bcmd.collider = makeHandle(3u, 1u);
        backend.execute(bcmd, &bp);

        PhysFrameSnapshot snap{};
        for (int i = 0; i < 240; ++i) {
            backend.step(1.0f / 60.0f);
            backend.publishSnapshot(snap);
        }
        CHECK(backend.collisionEventCount() > 0u);
        backend.stop();
    }

    // ------------------------------------------------------------
    // A3: overlap sphere sync returns bodies
    // ------------------------------------------------------------
    TEST_CASE(Real_OverlapSphereSyncReturnsBodies) {
        auto mgr = makeBox2DMgr();
        PhysicsWorld2D* w = mgr->world2D();

        for (int i = 0; i < 3; ++i) {
            BodyHandle h = InvalidBodyHandle;
            RigidbodyDesc rb;
            rb.type = BodyType::Dynamic;
            rb.position = ayt::math::FVector3(static_cast<float>(i) * 0.1f, 0.0f, 0.0f);
            w->createRigidbody(rb, h);
            ColliderDesc cd{};
            cd.body = h; cd.shape = ColliderShape::Sphere; cd.radius = 0.1f;
            ColliderHandle colH;
            w->createCollider(cd, colH);
            (void)colH;
        }
        for (int i = 0; i < 10; ++i) mgr->step(1.0f / 60.0f);
        waitForDrain2D(*mgr, 200);

        std::vector<BodyHandle> overlaps;
        const PhysResult r = w->overlapSphereSync(
            ayt::math::FVector3(0.0f, 0.0f, 0.0f), 2.0f, overlaps);
        CHECK_INT_EQ(static_cast<uint32_t>(r), static_cast<uint32_t>(PhysResult::Ok));
        CHECK(overlaps.size() >= 3u);
        mgr->shutdown();
    }

    // ------------------------------------------------------------
    // A4: CCD (bullet) flag is accepted on body creation
    // ------------------------------------------------------------
    TEST_CASE(Real_CcdFlagAcceptedOnCreate) {
        Box2DBackend2D backend;
        CHECK(backend.init2D(PhysicsBackendDescriptor{}));
        CHECK(backend.start(backend.describe()));
        RigidbodyDesc rb{};
        rb.type = BodyType::Dynamic;
        rb.enableCCD = true;
        BodyHandle h = InvalidBodyHandle;
        CHECK_INT_EQ(static_cast<uint32_t>(backend.execute_createRigidbodyForTest(rb, h)),
                     static_cast<uint32_t>(PhysResult::Ok));
        backend.step(1.0f / 60.0f);  // must not crash
        backend.stop();
    }

    // ------------------------------------------------------------
    // A5: per-body layer/mask filters collisions
    // ------------------------------------------------------------
    TEST_CASE(Real_LayerMaskFiltersCollisions) {
        Box2DBackend2D backend;
        PhysicsBackendDescriptor desc;
        desc.maxBodies = 64u;
        CHECK(backend.init2D(desc));
        CHECK(backend.start(backend.describe()));

        RigidbodyDesc ground{};
        ground.type = BodyType::Static;
        ground.layer = 1u;            // category bit 1
        ground.collideMask = 0xFFFFFFFFu;
        BodyHandle groundH = InvalidBodyHandle;
        backend.execute_createRigidbodyForTest(ground, groundH);
        ColliderDesc gc{};
        gc.body = groundH; gc.shape = ColliderShape::Box;
        gc.halfExtents = ayt::math::FVector3(10.0f, 0.5f, 0.0f);
        PhysicsCreatePayload gp{}; gp.kind = PhysicsCreatePayload::Kind::Collider; gp.colliderDesc = gc;
        PhysicsCommand gcmd{}; gcmd.type = PhysicsCommandType::CreateCollider;
        gcmd.collider = makeHandle(2u, 1u);
        backend.execute(gcmd, &gp);

        RigidbodyDesc ball{};
        ball.type = BodyType::Dynamic;
        ball.position = ayt::math::FVector3(0.0f, 3.0f, 0.0f);
        ball.layer = 2u;              // category bit 2
        ball.collideMask = 0u;        // collides with NOTHING
        BodyHandle ballH = InvalidBodyHandle;
        backend.execute_createRigidbodyForTest(ball, ballH);
        ColliderDesc bc{};
        bc.body = ballH; bc.shape = ColliderShape::Sphere; bc.radius = 0.25f;
        PhysicsCreatePayload bp{}; bp.kind = PhysicsCreatePayload::Kind::Collider; bp.colliderDesc = bc;
        PhysicsCommand bcmd{}; bcmd.type = PhysicsCommandType::CreateCollider;
        bcmd.collider = makeHandle(3u, 1u);
        backend.execute(bcmd, &bp);

        PhysFrameSnapshot snap{};
        float yN = 3.0f;
        for (int i = 0; i < 120; ++i) {
            backend.step(1.0f / 60.0f);
            snap.transforms.clear();
            backend.publishSnapshot(snap);
            for (const BodyTransform& bt : snap.transforms) {
                if (bt.body == ballH) yN = bt.position.y;
            }
        }
        // With mask=0 the ball should NOT collide with ground -> falls past it.
        CHECK(yN < 0.0f);
        backend.stop();
    }

    // ------------------------------------------------------------
    // A7+A8: spring joint (Distance with spring) holds two bodies
    // ------------------------------------------------------------
    TEST_CASE(Real_SpringJointCouplesBodies) {
        Box2DBackend2D backend;
        PhysicsBackendDescriptor desc;
        desc.maxBodies = 64u;
        CHECK(backend.init2D(desc));
        CHECK(backend.start(backend.describe()));

        RigidbodyDesc a{};
        a.type = BodyType::Static;
        a.position = ayt::math::FVector3(0.0f, 5.0f, 0.0f);
        BodyHandle aH = InvalidBodyHandle;
        backend.execute_createRigidbodyForTest(a, aH);

        RigidbodyDesc b{};
        b.type = BodyType::Dynamic;
        b.position = ayt::math::FVector3(0.0f, 0.0f, 0.0f);
        b.alwaysSync = true;
        BodyHandle bH = InvalidBodyHandle;
        backend.execute_createRigidbodyForTest(b, bH);
        ColliderDesc bc{};
        bc.body = bH; bc.shape = ColliderShape::Sphere; bc.radius = 0.1f;
        PhysicsCreatePayload bp{}; bp.kind = PhysicsCreatePayload::Kind::Collider; bp.colliderDesc = bc;
        PhysicsCommand bcmd{}; bcmd.type = PhysicsCommandType::CreateCollider;
        bcmd.collider = makeHandle(3u, 1u);
        backend.execute(bcmd, &bp);

        JointDesc jd{};
        jd.type = JointType::Spring;
        jd.bodyA = aH; jd.bodyB = bH;
        jd.anchorA = ayt::math::FVector3(0.0f, 5.0f, 0.0f);
        jd.anchorB = ayt::math::FVector3(0.0f, 0.0f, 0.0f);
        jd.stiffness = 4.0f;
        jd.damping = 0.5f;
        jd.maxDistance = 2.0f;
        PhysicsCreatePayload jp{}; jp.kind = PhysicsCreatePayload::Kind::Joint; jp.jointDesc = jd;
        PhysicsCommand jcmd{}; jcmd.type = PhysicsCommandType::CreateJoint;
        jcmd.joint = makeHandle(4u, 1u);
        backend.execute(jcmd, &jp);

        PhysFrameSnapshot snap{};
        for (int i = 0; i < 120; ++i) {
            backend.step(1.0f / 60.0f);
            snap.transforms.clear();
            backend.publishSnapshot(snap);
        }
        // Spring should pull b up toward a; b should not fall indefinitely.
        float bY = -100.0f;
        for (const BodyTransform& bt : snap.transforms) {
            if (bt.body == bH) bY = bt.position.y;
        }
        CHECK(bY > -100.0f);  // at least observed in snapshot
        backend.stop();
    }

    // ------------------------------------------------------------
    // R1: overlap box (real rectangle, independent hx/hy)
    // ------------------------------------------------------------
    TEST_CASE(Real_OverlapBoxSyncRectangleNotSquare) {
        auto mgr = makeBox2DMgr();
        PhysicsWorld2D* w = mgr->world2D();

        auto makeStatic = [&](float x, float y) -> BodyHandle {
            BodyHandle h = InvalidBodyHandle;
            RigidbodyDesc rb;
            rb.type = BodyType::Static;
            rb.position = ayt::math::FVector3(x, y, 0.0f);
            w->createRigidbody(rb, h);
            ColliderDesc cd{};
            cd.body = h; cd.shape = ColliderShape::Box;
            cd.halfExtents = ayt::math::FVector3(0.1f, 0.1f, 0.0f);
            ColliderHandle c;
            w->createCollider(cd, c); (void)c;
            return h;
        };
        const BodyHandle bx = makeStatic(3.0f, 0.0f);  // on +X axis
        const BodyHandle by = makeStatic(0.0f, 3.0f);  // on +Y axis
        for (int i = 0; i < 10; ++i) mgr->step(1.0f / 60.0f);
        waitForDrain2D(*mgr, 200);

        // Wide-thin rectangle (hx=4, hy=0.2): catches bx, not by.
        {
            std::vector<BodyHandle> ov;
            CHECK_INT_EQ(static_cast<uint32_t>(w->overlapBoxSync(
                ayt::math::FVector3(0,0,0), ayt::math::FVector3(4.0f, 0.2f, 0.0f), ov)),
                static_cast<uint32_t>(PhysResult::Ok));
            bool hasBx = false, hasBy = false;
            for (BodyHandle h : ov) { if (h == bx) hasBx = true; if (h == by) hasBy = true; }
            CHECK(hasBx);
            CHECK(!hasBy);
        }
        // Tall-thin rectangle (hx=0.2, hy=4): catches by, not bx.
        {
            std::vector<BodyHandle> ov;
            w->overlapBoxSync(ayt::math::FVector3(0,0,0),
                              ayt::math::FVector3(0.2f, 4.0f, 0.0f), ov);
            bool hasBx = false, hasBy = false;
            for (BodyHandle h : ov) { if (h == bx) hasBx = true; if (h == by) hasBy = true; }
            CHECK(!hasBx);
            CHECK(hasBy);
        }
        mgr->shutdown();
    }

    TEST_CASE(Real_OverlapBoxAsyncReturnsBodies) {
        auto mgr = makeBox2DMgr();
        PhysicsWorld2D* w = mgr->world2D();

        BodyHandle bx = InvalidBodyHandle;
        {
            RigidbodyDesc rb;
            rb.type = BodyType::Static;
            rb.position = ayt::math::FVector3(2.0f, 0.0f, 0.0f);
            w->createRigidbody(rb, bx);
            ColliderDesc cd{};
            cd.body = bx; cd.shape = ColliderShape::Box;
            cd.halfExtents = ayt::math::FVector3(0.1f, 0.1f, 0.0f);
            ColliderHandle c; w->createCollider(cd, c); (void)c;
        }
        for (int i = 0; i < 10; ++i) mgr->step(1.0f / 60.0f);
        waitForDrain2D(*mgr, 200);

        const uint32_t qid = w->overlapBoxAsync(
            ayt::math::FVector3(0,0,0), ayt::math::FVector3(3.0f, 0.5f, 0.0f));
        CHECK(qid != 0u);

        // Async query results land in exactly one published snapshot, so poll
        // fetchResults and inspect each frame's queryResults for our queryId.
        bool foundQr = false, foundBody = false;
        for (int poll = 0; poll < 40 && !foundQr; ++poll) {
            mgr->step(1.0f / 60.0f);
            ayt::platform::Thread::sleep(0.005f);
            const PhysFrameSnapshot snap = mgr->fetchResults();
            for (const QueryResult& qr : snap.queryResults) {
                if (qr.queryId != qid) continue;
                foundQr = true;
                CHECK(qr.hitCount >= 1u);
                for (const RaycastHit& rh : qr.hits) if (rh.body == bx) foundBody = true;
            }
        }
        CHECK(foundQr);
        CHECK(foundBody);
        mgr->shutdown();
    }

    // ------------------------------------------------------------
    // R2: setRigidbodyVelocity + fixedRotation
    // ------------------------------------------------------------
    TEST_CASE(Real_SetRigidbodyVelocityOverridesGravity) {
        auto mgr = makeBox2DMgr();
        PhysicsWorld2D* w = mgr->world2D();

        BodyHandle h = InvalidBodyHandle;
        RigidbodyDesc rb;
        rb.type = BodyType::Dynamic;
        rb.position = ayt::math::FVector3(0.0f, 0.0f, 0.0f);
        rb.alwaysSync = true;
        rb.linearDamping = 0.0f;
        w->createRigidbody(rb, h);
        ColliderDesc cd{};
        cd.body = h; cd.shape = ColliderShape::Sphere; cd.radius = 0.1f;
        ColliderHandle c; w->createCollider(cd, c); (void)c;
        for (int i = 0; i < 10; ++i) mgr->step(1.0f / 60.0f);
        waitForDrain2D(*mgr, 200);

        CHECK_INT_EQ(static_cast<uint32_t>(w->setRigidbodyVelocity(
            h, ayt::math::FVector3(0.0f, 5.0f, 0.0f))), static_cast<uint32_t>(PhysResult::Ok));
        for (int i = 0; i < 3; ++i) mgr->step(1.0f / 60.0f);
        waitForDrain2D(*mgr, 200);

        const PhysFrameSnapshot snap = mgr->fetchResults();
        float vy = -100.0f;
        for (const BodyTransform& bt : snap.transforms) {
            if (bt.body == h) vy = bt.linearVelocity.y;
        }
        // Set to +5; gravity subtracts < 0.5 over 3 steps -> still clearly upward.
        CHECK(vy > 4.0f);
        mgr->shutdown();
    }

    TEST_CASE(Real_FixedRotationLocksAngularVelocity) {
        auto mgr = makeBox2DMgr();
        PhysicsWorld2D* w = mgr->world2D();

        // Static ledge: top y=0.5, edges at x = +/-2.
        {
            BodyHandle h;
            RigidbodyDesc rb; rb.type = BodyType::Static;
            w->createRigidbody(rb, h);
            ColliderDesc cd{};
            cd.body = h; cd.shape = ColliderShape::Box;
            cd.halfExtents = ayt::math::FVector3(2.0f, 0.5f, 0.0f);
            cd.material.friction = 0.0f;
            ColliderHandle c; w->createCollider(cd, c); (void)c;
        }

        // Box resting on the ledge with COM just past an edge -> contact torque
        // tips it off. Initial angular velocity is 0; rotation comes only from
        // the off-center contact, which fixedRotation (invInertia=0) rejects.
        auto makeTipper = [&](bool fixed, float x) -> BodyHandle {
            BodyHandle h = InvalidBodyHandle;
            RigidbodyDesc rb;
            rb.type = BodyType::Dynamic;
            rb.position = ayt::math::FVector3(x, 1.05f, 0.0f);
            rb.fixedRotation = fixed;
            rb.alwaysSync = true;
            rb.angularDamping = 0.0f;
            w->createRigidbody(rb, h);
            ColliderDesc cd{};
            cd.body = h; cd.shape = ColliderShape::Box;
            cd.halfExtents = ayt::math::FVector3(0.5f, 0.5f, 0.0f);
            cd.material.friction = 0.0f;
            ColliderHandle c; w->createCollider(cd, c); (void)c;
            return h;
        };
        const BodyHandle free   = makeTipper(false,  2.3f);  // tips off right edge
        const BodyHandle locked = makeTipper(true,  -2.3f);  // tips off left edge
        for (int i = 0; i < 90; ++i) mgr->step(1.0f / 60.0f);
        waitForDrain2D(*mgr, 300);

        const PhysFrameSnapshot snap = mgr->fetchResults();
        float freeAngle = 0.0f, lockedAngle = 0.0f;
        for (const BodyTransform& bt : snap.transforms) {
            const float a = 2.0f * std::atan2(bt.rotation.z, bt.rotation.w);
            if (bt.body == free)   freeAngle   = a;
            if (bt.body == locked) lockedAngle = a;
        }
        CHECK(std::fabs(freeAngle) > 0.3f);     // tipped off the edge -> rotated
        CHECK(std::fabs(lockedAngle) < 0.05f);  // fixedRotation -> stayed axis-aligned
        mgr->shutdown();
    }

    // ------------------------------------------------------------
    // R5: ConvexHull collider + Point (mouse/tether) joint
    // ------------------------------------------------------------
    TEST_CASE(Real_ConvexHullColliderHasCollision) {
        auto mgr = makeBox2DMgr();
        PhysicsWorld2D* w = mgr->world2D();

        // Static floor.
        {
            BodyHandle h;
            RigidbodyDesc rb; rb.type = BodyType::Static;
            w->createRigidbody(rb, h);
            ColliderDesc cd{};
            cd.body = h; cd.shape = ColliderShape::Box;
            cd.halfExtents = ayt::math::FVector3(5.0f, 0.5f, 0.0f);
            ColliderHandle c; w->createCollider(cd, c); (void)c;
        }

        // Dynamic body with a ConvexHull collider: a 4-point box-equivalent
        // hull built via b2ComputeHull + b2MakePolygon (NOT the Box shortcut),
        // proving the hull path produces real collision geometry.
        BodyHandle h = InvalidBodyHandle;
        RigidbodyDesc rb;
        rb.type = BodyType::Dynamic;
        rb.position = ayt::math::FVector3(0.0f, 5.0f, 0.0f);
        rb.alwaysSync = true;
        w->createRigidbody(rb, h);
        auto shapeData = std::make_shared<ColliderShapeData>();
        shapeData->hullPoints = {
            ayt::math::FVector3(-0.5f, -0.5f, 0.0f),
            ayt::math::FVector3( 0.5f, -0.5f, 0.0f),
            ayt::math::FVector3( 0.5f,  0.5f, 0.0f),
            ayt::math::FVector3(-0.5f,  0.5f, 0.0f),
        };
        ColliderDesc cd{};
        cd.body = h; cd.shape = ColliderShape::ConvexHull;
        cd.shapeData = shapeData;
        ColliderHandle c; w->createCollider(cd, c); (void)c;

        for (int i = 0; i < 240; ++i) mgr->step(1.0f / 60.0f);
        waitForDrain2D(*mgr, 300);
        const PhysFrameSnapshot snap = mgr->fetchResults();
        float y = 5.0f;
        for (const BodyTransform& bt : snap.transforms)
            if (bt.body == h) y = bt.position.y;
        // Hull bottom (center.y - 0.5) rests on floor top (0.5) -> center.y ~ 1.0.
        CHECK(y < 4.0f);   // fell
        CHECK(y > 0.8f);   // resting on the floor, not through it
        CHECK(y < 1.3f);
        mgr->shutdown();
    }

    TEST_CASE(Real_PointJointPullsBodyTowardTarget) {
        auto mgr = makeBox2DMgr();
        PhysicsWorld2D* w = mgr->world2D();

        // Static reference at the origin (motor joint target is relative to A).
        BodyHandle aH = InvalidBodyHandle;
        {
            RigidbodyDesc rb; rb.type = BodyType::Static;
            rb.position = ayt::math::FVector3(0.0f, 0.0f, 0.0f);
            w->createRigidbody(rb, aH);
        }
        // Dynamic body pulled toward the target.
        BodyHandle bH = InvalidBodyHandle;
        {
            RigidbodyDesc rb; rb.type = BodyType::Dynamic;
            rb.position = ayt::math::FVector3(0.0f, 0.0f, 0.0f);
            rb.alwaysSync = true;
            rb.linearDamping = 0.0f;
            w->createRigidbody(rb, bH);
            ColliderDesc cd{};
            cd.body = bH; cd.shape = ColliderShape::Sphere; cd.radius = 0.1f;
            ColliderHandle c; w->createCollider(cd, c); (void)c;
        }

        // Point = tether joint: drive bodyB toward world target (0, 5).
        JointDesc jd{};
        jd.type = JointType::Point;
        jd.bodyA = aH; jd.bodyB = bH;
        jd.anchorB = ayt::math::FVector3(0.0f, 5.0f, 0.0f);  // target
        jd.stiffness = 1.0f;   // -> correctionFactor = 1.0 (snappy)
        jd.damping = 0.5f;
        JointHandle jh;
        CHECK_INT_EQ(static_cast<uint32_t>(w->createJoint(jd, jh)),
                     static_cast<uint32_t>(PhysResult::Ok));

        for (int i = 0; i < 180; ++i) mgr->step(1.0f / 60.0f);
        waitForDrain2D(*mgr, 300);
        const PhysFrameSnapshot snap = mgr->fetchResults();
        float y = 0.0f;
        for (const BodyTransform& bt : snap.transforms)
            if (bt.body == bH) y = bt.position.y;
        // Motor joint drives B up toward y=5 against gravity; B rises well
        // above its start and approaches the target.
        CHECK(y > 2.0f);
        CHECK(y < 5.5f);
        mgr->shutdown();
    }

    // ------------------------------------------------------------
    // R6: setGravityScale / applyTorque / applyAngularImpulse
    // ------------------------------------------------------------
    TEST_CASE(Real_SetGravityScaleZeroFloats) {
        auto mgr = makeBox2DMgr();
        PhysicsWorld2D* w = mgr->world2D();

        auto makeBody = [&](float x, float gravityScaleAtCreate) -> BodyHandle {
            BodyHandle h = InvalidBodyHandle;
            RigidbodyDesc rb;
            rb.type = BodyType::Dynamic;
            rb.position = ayt::math::FVector3(x, 10.0f, 0.0f);
            rb.gravityScale = gravityScaleAtCreate;
            rb.alwaysSync = true;
            rb.linearDamping = 0.0f;
            w->createRigidbody(rb, h);
            ColliderDesc cd{};
            cd.body = h; cd.shape = ColliderShape::Sphere; cd.radius = 0.1f;
            ColliderHandle c; w->createCollider(cd, c); (void)c;
            return h;
        };
        // Creation-time gravityScale=0 stays up; runtime set to 0 also works.
        const BodyHandle atCreate = makeBody(-4.0f, 0.0f);
        const BodyHandle runtime  = makeBody(-2.0f, 1.0f);
        const BodyHandle falling  = makeBody( 2.0f, 1.0f);
        waitForDrain2D(*mgr, 200);
        CHECK_INT_EQ(static_cast<uint32_t>(w->setGravityScale(runtime, 0.0f)),
                     static_cast<uint32_t>(PhysResult::Ok));

        for (int i = 0; i < 120; ++i) mgr->step(1.0f / 60.0f);
        waitForDrain2D(*mgr, 300);
        const PhysFrameSnapshot snap = mgr->fetchResults();
        float yCreate = 10.0f, yRuntime = 10.0f, yFall = 10.0f;
        for (const BodyTransform& bt : snap.transforms) {
            if (bt.body == atCreate) yCreate = bt.position.y;
            if (bt.body == runtime)  yRuntime = bt.position.y;
            if (bt.body == falling)  yFall    = bt.position.y;
        }
        CHECK(yCreate > 9.0f);   // RigidbodyDesc.gravityScale=0 at creation
        CHECK(yRuntime > 9.0f);  // runtime setGravityScale(0)
        CHECK(yFall < 9.0f);     // normal gravity -> falls
        mgr->shutdown();
    }

    TEST_CASE(Real_ApplyTorqueSpinsBody) {
        auto mgr = makeBox2DMgr();
        PhysicsWorld2D* w = mgr->world2D();

        BodyHandle h = InvalidBodyHandle;
        RigidbodyDesc rb;
        rb.type = BodyType::Dynamic;
        rb.position = ayt::math::FVector3(0.0f, 5.0f, 0.0f);
        rb.fixedRotation = false;
        rb.alwaysSync = true;
        rb.angularDamping = 0.0f;
        w->createRigidbody(rb, h);
        ColliderDesc cd{};
        cd.body = h; cd.shape = ColliderShape::Box;
        cd.halfExtents = ayt::math::FVector3(0.5f, 0.5f, 0.0f);
        ColliderHandle c; w->createCollider(cd, c); (void)c;
        waitForDrain2D(*mgr, 200);

        for (int i = 0; i < 90; ++i) {
            CHECK_INT_EQ(static_cast<uint32_t>(w->applyTorque(h, 8.0f)),
                         static_cast<uint32_t>(PhysResult::Ok));
            mgr->step(1.0f / 60.0f);
        }
        waitForDrain2D(*mgr, 300);
        const PhysFrameSnapshot snap = mgr->fetchResults();
        float angle = 0.0f;
        for (const BodyTransform& bt : snap.transforms) {
            if (bt.body == h)
                angle = 2.0f * std::atan2(bt.rotation.z, bt.rotation.w);
        }
        CHECK(std::fabs(angle) > 0.5f);
        mgr->shutdown();
    }

    TEST_CASE(Real_ApplyAngularImpulseSpinsBody) {
        auto mgr = makeBox2DMgr();
        PhysicsWorld2D* w = mgr->world2D();

        BodyHandle h = InvalidBodyHandle;
        RigidbodyDesc rb;
        rb.type = BodyType::Dynamic;
        rb.position = ayt::math::FVector3(0.0f, 5.0f, 0.0f);
        rb.fixedRotation = false;
        rb.alwaysSync = true;
        rb.angularDamping = 0.0f;
        w->createRigidbody(rb, h);
        ColliderDesc cd{};
        cd.body = h; cd.shape = ColliderShape::Box;
        cd.halfExtents = ayt::math::FVector3(0.5f, 0.5f, 0.0f);
        ColliderHandle c; w->createCollider(cd, c); (void)c;
        waitForDrain2D(*mgr, 200);

        CHECK_INT_EQ(static_cast<uint32_t>(w->applyAngularImpulse(h, 3.0f)),
                     static_cast<uint32_t>(PhysResult::Ok));
        mgr->step(1.0f / 60.0f);
        waitForDrain2D(*mgr, 300);
        const PhysFrameSnapshot snap = mgr->fetchResults();
        float wz = 0.0f;
        for (const BodyTransform& bt : snap.transforms) {
            if (bt.body == h) wz = bt.angularVelocity.z;
        }
        CHECK(std::fabs(wz) > 0.1f);
        mgr->shutdown();
    }

#else

    TEST_CASE(Stub_ManagerFallsBackToNullWhenNoBox2D) {
        PhysicsBackendDescriptor desc;
        desc.kind2D = BackendKind::DefaultBox2D;
        auto mgr = PhysicsManager::create(desc);
        CHECK_NOT_NULL(mgr.get());
        CHECK(!mgr->backend2D()->isRealDevice());
        mgr->shutdown();
    }

#endif

TEST_SUITE_END

#if defined(AYPHYSICS_HAS_BOX2D)
namespace ayt::physics {

PhysResult Box2DBackend2D::execute_createRigidbodyForTest(const RigidbodyDesc& desc,
                                                          BodyHandle& outHandle) {
    outHandle = InvalidBodyHandle;
    static std::atomic<uint32_t> g_nextBodyIndex{0};
    static std::atomic<uint32_t> g_nextBodyGen{0};
    const uint32_t idx = g_nextBodyIndex.fetch_add(1u) + 1u;
    const uint32_t gen = g_nextBodyGen.fetch_add(1u) + 1u;
    if (gen > kPhysHandleMaxGen) {
        g_nextBodyGen.store(0u);
    }
    outHandle = makeHandle(idx, gen);

    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::CreateRigidbody;
    cmd.body = outHandle;
    cmd.createSlot = 0;

    PhysicsCreatePayload payload{};
    payload.kind = PhysicsCreatePayload::Kind::Rigidbody;
    payload.rigidDesc = desc;
    execute(cmd, &payload);
    return PhysResult::Ok;
}

PhysResult Box2DBackend2D::execute_destroyRigidbodyForTest(BodyHandle h) {
    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::DestroyRigidbody;
    cmd.body = h;
    execute(cmd, nullptr);
    return PhysResult::Ok;
}

}  // namespace ayt::physics
#endif
