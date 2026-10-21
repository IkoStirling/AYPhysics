#include <AYPhysics/PhysicsManager.h>
#include <AYPhysics/PhysicsWorld2D.h>
#include <AYTest.h>

#include "Test_Box2DBackend2D_Helpers.h"

using namespace ayt::physics;

namespace {
bool addStaticBox(PhysicsWorld2D& world, float y, PhysLayer layer,
                  bool sensor, BodyHandle& body, ColliderHandle& collider) {
    RigidbodyDesc rigid;
    rigid.type = BodyType::Static;
    rigid.layer = layer;
    rigid.position = {0.0f, y, 0.0f};
    if (world.createRigidbody(rigid, body) != PhysResult::Ok) return false;
    ColliderDesc shape;
    shape.body = body;
    shape.shape = ColliderShape::Box;
    shape.halfExtents = {0.5f, 0.2f, 0.0f};
    shape.isTrigger = sensor;
    return world.createCollider(shape, collider) == PhysResult::Ok;
}
} // namespace

TEST_SUITE(ShapeSweepAll2DTests)

#if defined(AYPHYSICS_HAS_BOX2D)
TEST_CASE(CircleCapsuleAndRotatedBoxCollectSortedHits) {
    auto manager = ayt::physics::test_helpers::makeBox2DMgr();
    CHECK_TRUE(manager != nullptr);
    PhysicsWorld2D* world = manager->world2D();
    BodyHandle top = 0u, sensor = 0u, bottom = 0u;
    ColliderHandle topCollider = 0u, sensorCollider = 0u,
                   bottomCollider = 0u;
    CHECK_TRUE(addStaticBox(*world, 5.0f, 3u, false,
        top, topCollider));
    CHECK_TRUE(addStaticBox(*world, 3.0f, 2u, true,
        sensor, sensorCollider));
    CHECK_TRUE(addStaticBox(*world, 0.0f, 3u, false,
        bottom, bottomCollider));
    CHECK_TRUE(manager->stepAndWait(1.0f / 60.0f) == PhysResult::Ok);
    PhysicsDebugDrawOptions options;
    options.enabled = true;
    CHECK_TRUE(manager->debugDraw().configure(options) == PhysResult::Ok);

    SphereSweep circle;
    circle.origin = {0.0f, 7.0f, 4.0f};
    circle.displacement = {0.0f, -10.0f, 2.0f};
    circle.radius = 0.5f;
    std::vector<RaycastHit> hits;
    CHECK_TRUE(world->sphereSweepAllSync(circle, hits) == PhysResult::Ok);
    CHECK_INT_EQ(hits.size(), 2u);
    CHECK_INT_EQ(hits[0].body, top);
    CHECK_INT_EQ(hits[0].collider, topCollider);
    CHECK_INT_EQ(hits[1].body, bottom);
    CHECK_INT_EQ(hits[1].collider, bottomCollider);
    CHECK_TRUE(hits[0].distance < hits[1].distance);
    CHECK_TRUE(hits[0].normal.y > 0.5f);
    CHECK_FLOAT_EQ(hits[0].point.z, 0.0f, 1.0e-6f);
    circle.includeSensors = true;
    CHECK_TRUE(world->sphereSweepAllSync(circle, hits) == PhysResult::Ok);
    CHECK_INT_EQ(hits.size(), 3u);
    CHECK_INT_EQ(hits[1].body, sensor);
    CHECK_INT_EQ(hits[1].collider, sensorCollider);
    circle.ignoredBodies = {top};
    CHECK_TRUE(world->sphereSweepAllSync(circle, hits, 1u)
        == PhysResult::OutOfRange);
    CHECK_TRUE(hits.empty());
    circle.layerMask = 1u << 3u;
    CHECK_TRUE(world->sphereSweepAllSync(circle, hits, 1u) == PhysResult::Ok);
    CHECK_INT_EQ(hits.size(), 1u);
    CHECK_INT_EQ(hits[0].body, bottom);

    CapsuleSweep2D capsule;
    capsule.origin = {0.0f, 7.0f, 0.0f};
    capsule.displacement = {0.0f, -10.0f, 0.0f};
    capsule.radius = 0.3f;
    capsule.height = 2.0f;
    CHECK_TRUE(world->capsuleSweepAllSync(capsule, hits) == PhysResult::Ok);
    CHECK_INT_EQ(hits.size(), 2u);
    CHECK_INT_EQ(hits[0].body, top);
    CHECK_INT_EQ(hits[1].body, bottom);
    capsule.includeSensors = true;
    CHECK_TRUE(world->capsuleSweepAllSync(capsule, hits) == PhysResult::Ok);
    CHECK_INT_EQ(hits.size(), 3u);
    CHECK_INT_EQ(hits[1].body, sensor);
    CHECK_TRUE(world->capsuleSweepAllSync(capsule, hits, 1u)
        == PhysResult::OutOfRange);
    CHECK_TRUE(hits.empty());

    OrientedBoxSweep2D rotated;
    rotated.origin = {0.0f, 7.0f, 0.0f};
    rotated.displacement = {0.0f, -10.0f, 0.0f};
    rotated.halfExtents = {1.0f, 0.2f, 0.0f};
    rotated.angleRadians = 1.57079632679f;
    CHECK_TRUE(world->orientedBoxSweepAllSync(rotated, hits)
        == PhysResult::Ok);
    CHECK_INT_EQ(hits.size(), 2u);
    CHECK_INT_EQ(hits[0].body, top);
    CHECK_INT_EQ(hits[1].body, bottom);
    rotated.includeSensors = true;
    CHECK_TRUE(world->orientedBoxSweepAllSync(rotated, hits)
        == PhysResult::Ok);
    CHECK_INT_EQ(hits.size(), 3u);
    CHECK_INT_EQ(hits[1].body, sensor);
    CHECK_TRUE(world->orientedBoxSweepAllSync(rotated, hits, 1u)
        == PhysResult::OutOfRange);
    CHECK_TRUE(hits.empty());

    const auto records = manager->debugDraw().consumeRecords();
    CHECK_TRUE(records.size() >= 3u);
    CHECK_TRUE(records[0].label == "Circle sweep all");
    CHECK_TRUE(records[records.size() - 1u].label
        == "Rotated box sweep all");
    manager->shutdown();
}
#endif

TEST_CASE(InvalidAndNullQueriesClearResults) {
    PhysicsBackendDescriptor descriptor;
    descriptor.kind3D = BackendKind::Null;
    descriptor.kind2D = BackendKind::Null;
    auto manager = PhysicsManager::create(descriptor);
    CHECK_TRUE(manager != nullptr);
    PhysicsWorld2D* world = manager->world2D();
    std::vector<RaycastHit> hits(1u);

    SphereSweep circle;
    circle.displacement = {0.0f, -5.0f, 0.0f};
    CHECK_TRUE(world->sphereSweepAllSync(circle, hits, 0u)
        == PhysResult::InvalidParam);
    CHECK_TRUE(hits.empty());
    CHECK_TRUE(world->sphereSweepAllSync(circle, hits)
        == PhysResult::Unsupported);

    CapsuleSweep2D capsule;
    capsule.displacement = {0.0f, -5.0f, 0.0f};
    capsule.height = 0.5f;
    CHECK_TRUE(world->capsuleSweepAllSync(capsule, hits)
        == PhysResult::InvalidParam);
    capsule.height = 2.0f;
    CHECK_TRUE(world->capsuleSweepAllSync(capsule, hits)
        == PhysResult::Unsupported);

    OrientedBoxSweep2D rotated;
    rotated.displacement = {0.0f, -5.0f, 0.0f};
    CHECK_TRUE(world->orientedBoxSweepAllSync(rotated, hits, 1025u)
        == PhysResult::InvalidParam);
    CHECK_TRUE(world->orientedBoxSweepAllSync(rotated, hits)
        == PhysResult::Unsupported);
    CHECK_TRUE(hits.empty());
    manager->shutdown();
    CHECK_TRUE(world->sphereSweepAllSync(circle, hits)
        == PhysResult::InvalidState);
    CHECK_TRUE(hits.empty());
}

TEST_SUITE_END
