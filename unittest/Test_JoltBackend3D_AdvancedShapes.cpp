// Test_JoltBackend3D_AdvancedShapes.cpp - R2.0a tests for ConvexHull,
// TriangleMesh, and Heightfield collider shapes. Independent TU per CLAUDE.md
// (each Test_*.cpp is a separate TU; no include-order coupling with the
// baseline Test_JoltBackend3D.cpp).
//
// Test plan (§3.1 of kind-shimmying-swing.md):
//   1-4  : happy-path shapes (settle / collide / Hull-on-Mesh combined stack)
//   5-6  : MustBeStatic validation (rejection without crash)
//   7-10 : malformed-input rejection (empty / oversize / N<4 / null shapeData)
//   11   : Mock-only round-trip proving shared_ptr survives the create pool
//
// IMPORTANT: all Jolt-gated cases use the synchronous `execute_*ForTest`
// seams (Test_JoltBackend3D.cpp), NOT the Manager-level createCollider path.
// Manager::createCollider is async (enqueue + return Ok) and cannot surface
// backend reject reasons — R2.0a reject cases would silently pass on what
// they should fail on. The seams use _notFoundCount delta to surface reject.

#include "AYPhysicsManager.h"
#include "AYPhysicsWorld3D.h"
#include "AYPhysicsBackendTestAccess.h"
#include "AYPhysicsTypes.h"

#if defined(AYPHYSICS_HAS_JOLT)
#include "JoltBackend3D.h"
#endif

#include "AYTest.h"
#include "Test_JoltBackend3D_Helpers.h"

#include <memory>
#include <vector>

using namespace ayt::physics;
using namespace ayt::physics::test_helpers;

namespace {

#if defined(AYPHYSICS_HAS_JOLT)
// Find our body handle in the snapshot's sparse transform list.
const BodyTransform* findTransform(const PhysFrameSnapshot& s, BodyHandle h) {
    for (const auto& t : s.transforms) {
        if (t.body == h) return &t;
    }
    return nullptr;
}

// Create a static body (defaults to a big ground box) via the test seam.
BodyHandle createStaticBody(JoltBackend3D& backend,
                            const ayt::math::FVector3& pos = {},
                            const ayt::math::FVector3& halfExtents = {50.0f, 0.5f, 50.0f}) {
    BodyHandle h = InvalidBodyHandle;
    RigidbodyDesc rb;
    rb.type     = BodyType::Static;
    rb.position = pos;
    (void)backend.execute_createRigidbodyForTest(rb, h);
    ColliderDesc cd{};
    cd.body        = h;
    cd.shape       = ColliderShape::Box;
    cd.halfExtents = halfExtents;
    ColliderHandle ch = InvalidColliderHandle;
    (void)backend.execute_createColliderForTest(cd, ch);
    return h;
}

// Create a dynamic body via the test seam.
BodyHandle createDynamicBody(JoltBackend3D& backend,
                             const ayt::math::FVector3& pos,
                             float mass = 1.0f) {
    BodyHandle h = InvalidBodyHandle;
    RigidbodyDesc rb;
    rb.type     = BodyType::Dynamic;
    rb.position = pos;
    rb.mass     = mass;
    (void)backend.execute_createRigidbodyForTest(rb, h);
    return h;
}

// Step the backend directly + publish a snapshot.
void stepAndPublish(JoltBackend3D& backend, PhysFrameSnapshot& snap, int n) {
    for (int i = 0; i < n; ++i) {
        backend.step(1.0f / 60.0f);
        backend.publishSnapshot(snap);
    }
}
#endif

}  // namespace

TEST_SUITE(JoltBackend3DAdvancedShapesTests)

#if defined(AYPHYSICS_HAS_JOLT)

    // ------------------------------------------------------------
    // 1) ConvexHull basic: 4-vertex tetrahedron drops onto static box.
    // ------------------------------------------------------------
    TEST_CASE(R2a_ConvexHull_BasicSettles) {
        JoltBackend3D backend;
        PhysicsBackendDescriptor desc;
        desc.maxBodies             = 64u;
        desc.maxBodyPairs          = 256u;
        desc.maxContactConstraints = 256u;
        CHECK(backend.init3D(desc));
        CHECK(backend.start(backend.describe()));

        BodyHandle groundH = createStaticBody(backend, ayt::math::FVector3(0, -1.0f, 0),
                                              ayt::math::FVector3(50, 0.5f, 50));
        CHECK(isValidHandle(groundH));

        BodyHandle bodyH = createDynamicBody(backend, ayt::math::FVector3(0, 10.0f, 0));
        CHECK(isValidHandle(bodyH));

        auto sd = std::make_shared<ColliderShapeData>();
        sd->hullPoints = {
            { 0.0f,  0.5f,  0.0f},
            { 0.5f, -0.5f,  0.5f},
            {-0.5f, -0.5f,  0.5f},
            { 0.0f, -0.5f, -0.5f},
        };
        ColliderDesc cd{};
        cd.body      = bodyH;
        cd.shape     = ColliderShape::ConvexHull;
        cd.shapeData = sd;

        ColliderHandle colH = InvalidColliderHandle;
        CHECK_INT_EQ(static_cast<uint32_t>(backend.execute_createColliderForTest(cd, colH)),
                     static_cast<uint32_t>(PhysResult::Ok));

        PhysFrameSnapshot snap{};
        stepAndPublish(backend, snap, 120);

        const BodyTransform* bt = findTransform(snap, bodyH);
        CHECK_NOT_NULL(bt);
        if (bt) {
            CHECK(bt->position.y < 10.0f);   // gravity took effect
            CHECK(bt->position.y > -2.0f);   // didn't fall through ground
        }

        backend.stop();
    }

    // ------------------------------------------------------------
    // 2) TriangleMesh: 2-tri quad as static ground; sphere rests on it.
    // ------------------------------------------------------------
    TEST_CASE(R2a_TriangleMesh_SphereRollsOnFlat) {
        JoltBackend3D backend;
        PhysicsBackendDescriptor desc;
        desc.maxBodies             = 64u;
        desc.maxBodyPairs          = 256u;
        desc.maxContactConstraints = 256u;
        CHECK(backend.init3D(desc));
        CHECK(backend.start(backend.describe()));

        BodyHandle meshH = InvalidBodyHandle;
        RigidbodyDesc rb;
        rb.type = BodyType::Static;
        (void)backend.execute_createRigidbodyForTest(rb, meshH);
        CHECK(isValidHandle(meshH));

        auto sd = std::make_shared<ColliderShapeData>();
        sd->meshVertices = {
            {-10.0f, 0.0f, -10.0f},
            { 10.0f, 0.0f, -10.0f},
            { 10.0f, 0.0f,  10.0f},
            {-10.0f, 0.0f,  10.0f},
        };
        sd->meshIndices = {0, 1, 2, 0, 2, 3};
        ColliderDesc cd{};
        cd.body      = meshH;
        cd.shape     = ColliderShape::TriangleMesh;
        cd.shapeData = sd;
        ColliderHandle colH = InvalidColliderHandle;
        CHECK_INT_EQ(static_cast<uint32_t>(backend.execute_createColliderForTest(cd, colH)),
                     static_cast<uint32_t>(PhysResult::Ok));

        BodyHandle sphereH = createDynamicBody(backend, ayt::math::FVector3(0.0f, 5.0f, 0.0f));
        ColliderDesc scd{};
        scd.body   = sphereH;
        scd.shape  = ColliderShape::Sphere;
        scd.radius = 0.5f;
        ColliderHandle scH = InvalidColliderHandle;
        CHECK_INT_EQ(static_cast<uint32_t>(backend.execute_createColliderForTest(scd, scH)),
                     static_cast<uint32_t>(PhysResult::Ok));

        PhysFrameSnapshot snap{};
        stepAndPublish(backend, snap, 120);

        const BodyTransform* bt = findTransform(snap, sphereH);
        CHECK_NOT_NULL(bt);
        if (bt) {
            CHECK(bt->position.y < 5.0f);   // fell
            CHECK(bt->position.y > 0.0f);   // sphere didn't penetrate mesh
        }

        backend.stop();
    }

    // ------------------------------------------------------------
    // 3) Heightfield: 4x4 grid with central bump; sphere drops and contacts.
    // ------------------------------------------------------------
    TEST_CASE(R2a_HeightField_BasicSettles) {
        JoltBackend3D backend;
        PhysicsBackendDescriptor desc;
        desc.maxBodies             = 64u;
        desc.maxBodyPairs          = 256u;
        desc.maxContactConstraints = 256u;
        CHECK(backend.init3D(desc));
        CHECK(backend.start(backend.describe()));

        BodyHandle hfH = InvalidBodyHandle;
        RigidbodyDesc rb;
        rb.type = BodyType::Static;
        (void)backend.execute_createRigidbodyForTest(rb, hfH);
        CHECK(isValidHandle(hfH));

        auto sd = std::make_shared<ColliderShapeData>();
        sd->heightGridN = 4u;
        sd->heightSamples = {
            0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 1.0f, 0.0f,
            0.0f, 1.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f,
        };
        ColliderDesc cd{};
        cd.body      = hfH;
        cd.shape     = ColliderShape::Heightfield;
        cd.shapeData = sd;
        ColliderHandle colH = InvalidColliderHandle;
        CHECK_INT_EQ(static_cast<uint32_t>(backend.execute_createColliderForTest(cd, colH)),
                     static_cast<uint32_t>(PhysResult::Ok));

        BodyHandle sphereH = createDynamicBody(backend, ayt::math::FVector3(0.5f, 5.0f, 0.5f));
        ColliderDesc scd{};
        scd.body   = sphereH;
        scd.shape  = ColliderShape::Sphere;
        scd.radius = 0.3f;
        ColliderHandle scH = InvalidColliderHandle;
        CHECK_INT_EQ(static_cast<uint32_t>(backend.execute_createColliderForTest(scd, scH)),
                     static_cast<uint32_t>(PhysResult::Ok));

        PhysFrameSnapshot snap{};
        stepAndPublish(backend, snap, 120);

        const BodyTransform* bt = findTransform(snap, sphereH);
        CHECK_NOT_NULL(bt);
        if (bt) {
            CHECK(bt->position.y > 0.5f);   // stopped on bump (y >= 1 + 0.3 radius)
            CHECK(bt->position.y < 5.0f);
        }

        backend.stop();
    }

    // ------------------------------------------------------------
    // 4) ConvexHull dynamic on TriangleMesh static (canonical stack).
    // ------------------------------------------------------------
    TEST_CASE(R2a_HullOnMeshGround) {
        JoltBackend3D backend;
        PhysicsBackendDescriptor desc;
        desc.maxBodies             = 64u;
        desc.maxBodyPairs          = 256u;
        desc.maxContactConstraints = 256u;
        CHECK(backend.init3D(desc));
        CHECK(backend.start(backend.describe()));

        BodyHandle meshH = InvalidBodyHandle;
        RigidbodyDesc rb;
        rb.type = BodyType::Static;
        (void)backend.execute_createRigidbodyForTest(rb, meshH);
        CHECK(isValidHandle(meshH));
        auto msd = std::make_shared<ColliderShapeData>();
        msd->meshVertices = {
            {-10.0f, 0.0f, -10.0f},
            { 10.0f, 0.0f, -10.0f},
            { 10.0f, 0.0f,  10.0f},
            {-10.0f, 0.0f,  10.0f},
        };
        msd->meshIndices = {0, 1, 2, 0, 2, 3};
        ColliderDesc mcd{};
        mcd.body      = meshH;
        mcd.shape     = ColliderShape::TriangleMesh;
        mcd.shapeData = msd;
        ColliderHandle mcH = InvalidColliderHandle;
        CHECK_INT_EQ(static_cast<uint32_t>(backend.execute_createColliderForTest(mcd, mcH)),
                     static_cast<uint32_t>(PhysResult::Ok));

        BodyHandle hullH = createDynamicBody(backend, ayt::math::FVector3(0.0f, 5.0f, 0.0f));
        auto hsd = std::make_shared<ColliderShapeData>();
        hsd->hullPoints = {
            { 0.0f,  0.5f,  0.0f},
            { 0.5f, -0.5f,  0.5f},
            {-0.5f, -0.5f,  0.5f},
            { 0.0f, -0.5f, -0.5f},
        };
        ColliderDesc hcd{};
        hcd.body      = hullH;
        hcd.shape     = ColliderShape::ConvexHull;
        hcd.shapeData = hsd;
        ColliderHandle hcH = InvalidColliderHandle;
        CHECK_INT_EQ(static_cast<uint32_t>(backend.execute_createColliderForTest(hcd, hcH)),
                     static_cast<uint32_t>(PhysResult::Ok));

        PhysFrameSnapshot snap{};
        stepAndPublish(backend, snap, 120);

        const BodyTransform* bt = findTransform(snap, hullH);
        CHECK_NOT_NULL(bt);
        if (bt) {
            CHECK(bt->position.y > 0.0f);   // hull resting on mesh
            CHECK(bt->position.y < 5.0f);
        }

        backend.stop();
    }

    // ------------------------------------------------------------
    // 5) TriangleMesh on Dynamic body: rejected (MustBeStatic).
    // ------------------------------------------------------------
    TEST_CASE(R2a_MeshOnDynamic_Rejected) {
        JoltBackend3D backend;
        PhysicsBackendDescriptor desc;
        desc.maxBodies             = 32u;
        desc.maxBodyPairs          = 128u;
        desc.maxContactConstraints = 128u;
        CHECK(backend.init3D(desc));
        CHECK(backend.start(backend.describe()));

        BodyHandle dynH = createDynamicBody(backend, ayt::math::FVector3(0, 5, 0));

        auto sd = std::make_shared<ColliderShapeData>();
        sd->meshVertices = {{0,0,0}, {1,0,0}, {0,0,1}};
        sd->meshIndices  = {0, 1, 2};
        ColliderDesc cd{};
        cd.body      = dynH;
        cd.shape     = ColliderShape::TriangleMesh;
        cd.shapeData = sd;
        ColliderHandle colH = InvalidColliderHandle;
        CHECK_INT_EQ(static_cast<uint32_t>(backend.execute_createColliderForTest(cd, colH)),
                     static_cast<uint32_t>(PhysResult::BackendError));
        CHECK(!isValidHandle(colH));

        backend.stop();
    }

    // ------------------------------------------------------------
    // 6) Heightfield on Dynamic body: rejected (MustBeStatic).
    // ------------------------------------------------------------
    TEST_CASE(R2a_HeightFieldOnDynamic_Rejected) {
        JoltBackend3D backend;
        PhysicsBackendDescriptor desc;
        desc.maxBodies             = 32u;
        desc.maxBodyPairs          = 128u;
        desc.maxContactConstraints = 128u;
        CHECK(backend.init3D(desc));
        CHECK(backend.start(backend.describe()));

        BodyHandle dynH = createDynamicBody(backend, ayt::math::FVector3(0, 5, 0));

        auto sd = std::make_shared<ColliderShapeData>();
        sd->heightGridN = 4u;
        sd->heightSamples.assign(16u, 0.0f);
        ColliderDesc cd{};
        cd.body      = dynH;
        cd.shape     = ColliderShape::Heightfield;
        cd.shapeData = sd;
        ColliderHandle colH = InvalidColliderHandle;
        CHECK_INT_EQ(static_cast<uint32_t>(backend.execute_createColliderForTest(cd, colH)),
                     static_cast<uint32_t>(PhysResult::BackendError));
        CHECK(!isValidHandle(colH));

        backend.stop();
    }

    // ------------------------------------------------------------
    // 7) ConvexHull with empty hullPoints: rejected.
    // ------------------------------------------------------------
    TEST_CASE(R2a_ConvexHull_EmptyPoints_Rejected) {
        JoltBackend3D backend;
        PhysicsBackendDescriptor desc;
        desc.maxBodies             = 32u;
        CHECK(backend.init3D(desc));
        CHECK(backend.start(backend.describe()));

        BodyHandle dynH = createDynamicBody(backend, ayt::math::FVector3(0, 5, 0));

        auto sd = std::make_shared<ColliderShapeData>();  // hullPoints stays empty
        ColliderDesc cd{};
        cd.body      = dynH;
        cd.shape     = ColliderShape::ConvexHull;
        cd.shapeData = sd;
        ColliderHandle colH = InvalidColliderHandle;
        CHECK_INT_EQ(static_cast<uint32_t>(backend.execute_createColliderForTest(cd, colH)),
                     static_cast<uint32_t>(PhysResult::BackendError));
        CHECK(!isValidHandle(colH));

        backend.stop();
    }

    // ------------------------------------------------------------
    // 8) ConvexHull with 257 points: rejected (cMaxPointsInHull = 256).
    // ------------------------------------------------------------
    TEST_CASE(R2a_ConvexHull_Over256Points_Rejected) {
        JoltBackend3D backend;
        PhysicsBackendDescriptor desc;
        desc.maxBodies             = 32u;
        CHECK(backend.init3D(desc));
        CHECK(backend.start(backend.describe()));

        BodyHandle dynH = createDynamicBody(backend, ayt::math::FVector3(0, 5, 0));

        auto sd = std::make_shared<ColliderShapeData>();
        sd->hullPoints.reserve(257);
        for (int i = 0; i < 257; ++i) {
            sd->hullPoints.push_back({float(i), 0.0f, 0.0f});
        }
        ColliderDesc cd{};
        cd.body      = dynH;
        cd.shape     = ColliderShape::ConvexHull;
        cd.shapeData = sd;
        ColliderHandle colH = InvalidColliderHandle;
        CHECK_INT_EQ(static_cast<uint32_t>(backend.execute_createColliderForTest(cd, colH)),
                     static_cast<uint32_t>(PhysResult::BackendError));
        CHECK(!isValidHandle(colH));

        backend.stop();
    }

    // ------------------------------------------------------------
    // 9) Heightfield with heightGridN < 4: rejected.
    // ------------------------------------------------------------
    TEST_CASE(R2a_HeightField_NLessThan4_Rejected) {
        JoltBackend3D backend;
        PhysicsBackendDescriptor desc;
        desc.maxBodies             = 32u;
        CHECK(backend.init3D(desc));
        CHECK(backend.start(backend.describe()));

        BodyHandle stH = InvalidBodyHandle;
        RigidbodyDesc rb;
        rb.type = BodyType::Static;
        (void)backend.execute_createRigidbodyForTest(rb, stH);

        auto sd = std::make_shared<ColliderShapeData>();
        sd->heightGridN   = 3u;  // Jolt requires N >= 4
        sd->heightSamples.assign(9u, 0.0f);
        ColliderDesc cd{};
        cd.body      = stH;
        cd.shape     = ColliderShape::Heightfield;
        cd.shapeData = sd;
        ColliderHandle colH = InvalidColliderHandle;
        CHECK_INT_EQ(static_cast<uint32_t>(backend.execute_createColliderForTest(cd, colH)),
                     static_cast<uint32_t>(PhysResult::BackendError));
        CHECK(!isValidHandle(colH));

        backend.stop();
    }

    // ------------------------------------------------------------
    // 10) All three advanced shapes with nullptr shapeData: rejected.
    // ------------------------------------------------------------
    TEST_CASE(R2a_AllShapes_NullShapeData_Rejected) {
        JoltBackend3D backend;
        PhysicsBackendDescriptor desc;
        desc.maxBodies             = 32u;
        CHECK(backend.init3D(desc));
        CHECK(backend.start(backend.describe()));

        BodyHandle dynH = createDynamicBody(backend, ayt::math::FVector3(0, 5, 0));

        const ColliderShape dynShapes[] = { ColliderShape::ConvexHull,
                                            ColliderShape::TriangleMesh };
        for (ColliderShape shape : dynShapes) {
            ColliderDesc cd{};
            cd.body      = dynH;
            cd.shape     = shape;
            cd.shapeData = nullptr;
            ColliderHandle colH = InvalidColliderHandle;
            CHECK_INT_EQ(static_cast<uint32_t>(backend.execute_createColliderForTest(cd, colH)),
                         static_cast<uint32_t>(PhysResult::BackendError));
            CHECK(!isValidHandle(colH));
        }

        BodyHandle stH = InvalidBodyHandle;
        RigidbodyDesc srb;
        srb.type = BodyType::Static;
        (void)backend.execute_createRigidbodyForTest(srb, stH);
        ColliderDesc hcd{};
        hcd.body      = stH;
        hcd.shape     = ColliderShape::Heightfield;
        hcd.shapeData = nullptr;
        ColliderHandle hcH = InvalidColliderHandle;
        CHECK_INT_EQ(static_cast<uint32_t>(backend.execute_createColliderForTest(hcd, hcH)),
                     static_cast<uint32_t>(PhysResult::BackendError));
        CHECK(!isValidHandle(hcH));

        backend.stop();
    }

#endif // AYPHYSICS_HAS_JOLT

    // ------------------------------------------------------------
    // 11) Mock round-trip: shared_ptr<ColliderShapeData> survives the
    //     PhysicsCreatePool deep-copy (refcount >= 2 after pool holds it).
    //     Compiles + runs in Jolt-OFF mode (Null backend) — proves the
    //     R2.0a type change doesn't break Jolt-free consumers.
    // ------------------------------------------------------------
    TEST_CASE(R2a_MockBackend_ShapeDataRoundTrip) {
        testaccess::reset();

        PhysicsBackendDescriptor desc;
        desc.kind3D              = BackendKind::Mock;
        desc.commandQueueCapacity = 32;
        desc.createPoolCapacity   = 16;
        auto mgr = PhysicsManager::create(desc);
        CHECK_NOT_NULL(mgr.get());
        if (!mgr) return;

        PhysicsWorld3D* w = mgr->world3D();
        CHECK_NOT_NULL(w);

        BodyHandle bh = InvalidBodyHandle;
        RigidbodyDesc rb;
        rb.type = BodyType::Static;
        w->createRigidbody(rb, bh);
        CHECK(isValidHandle(bh));

        auto sd = std::make_shared<ColliderShapeData>();
        sd->hullPoints = {{1,2,3}, {4,5,6}, {7,8,9}};
        const auto rawBefore = sd.get();

        ColliderDesc cd{};
        cd.body      = bh;
        cd.shape     = ColliderShape::ConvexHull;
        cd.shapeData = sd;
        ColliderHandle ch = InvalidColliderHandle;
        CHECK_INT_EQ(static_cast<uint32_t>(w->createCollider(cd, ch)),
                     static_cast<uint32_t>(PhysResult::Ok));

        CHECK(sd.use_count() >= 2);

        waitForDrain(*mgr, 100);

        const auto& payloads = testaccess::mockBackendCreatePayloads();
        CHECK(!payloads.empty());
        bool found = false;
        for (const auto& p : payloads) {
            if (p.colliderDesc.shape == ColliderShape::ConvexHull &&
                p.colliderDesc.shapeData.get() == rawBefore) {
                found = true;
                break;
            }
        }
        CHECK(found);

        mgr->shutdown();
    }

TEST_SUITE_END