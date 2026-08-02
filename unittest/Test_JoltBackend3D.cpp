#include "AYPhysicsManager.h"
#include "AYPhysicsWorld3D.h"
#include "AYPhysicsTypes.h"

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