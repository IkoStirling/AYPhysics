#include <AYPhysics/PhysicsManager.h>
#include <AYPhysics/PhysicsWorld2D.h>
#include <AYTest.h>

#include "Test_Box2DBackend2D_Helpers.h"

using namespace ayt::physics;

TEST_SUITE(ColliderOverlap2DTests)

#if defined(AYPHYSICS_HAS_BOX2D)
TEST_CASE(OverlapReturnsEveryExactColliderWithFilters) {
    auto manager = ayt::physics::test_helpers::makeBox2DMgr();
    CHECK_TRUE(manager != nullptr);
    PhysicsWorld2D* world = manager->world2D();
    RigidbodyDesc rigid;
    rigid.type = BodyType::Static;
    rigid.layer = 3u;
    BodyHandle body = 0u;
    CHECK_TRUE(world->createRigidbody(rigid, body) == PhysResult::Ok);
    ColliderDesc shape;
    shape.body = body;
    shape.shape = ColliderShape::Box;
    shape.halfExtents = {0.25f, 0.25f, 0.0f};
    shape.offset = {-0.5f, 0.0f, 0.0f};
    ColliderHandle left = 0u, right = 0u;
    CHECK_TRUE(world->createCollider(shape, left) == PhysResult::Ok);
    shape.offset.x = 0.5f;
    CHECK_TRUE(world->createCollider(shape, right) == PhysResult::Ok);

    rigid.layer = 2u;
    BodyHandle sensorBody = 0u;
    CHECK_TRUE(world->createRigidbody(rigid, sensorBody) == PhysResult::Ok);
    shape.body = sensorBody;
    shape.offset = {};
    shape.isTrigger = true;
    ColliderHandle sensor = 0u;
    CHECK_TRUE(world->createCollider(shape, sensor) == PhysResult::Ok);
    CHECK_TRUE(manager->stepAndWait(1.0f / 60.0f) == PhysResult::Ok);
    PhysicsDebugDrawOptions options;
    options.enabled = true;
    CHECK_TRUE(manager->debugDraw().configure(options) == PhysResult::Ok);

    OrientedBoxOverlap2D box;
    box.center = {0.0f, 0.0f, 3.0f};
    box.halfExtents = {1.0f, 0.75f, 0.0f};
    box.angleRadians = 0.3f;
    std::vector<ColliderOverlapHit2D> hits;
    CHECK_TRUE(world->overlapOrientedBoxCollidersSync(box, hits)
        == PhysResult::Ok);
    CHECK_INT_EQ(hits.size(), 2u);
    CHECK_INT_EQ(hits[0].body, body);
    CHECK_INT_EQ(hits[1].body, body);
    CHECK_INT_EQ(hits[0].collider, left);
    CHECK_INT_EQ(hits[1].collider, right);
    CHECK_TRUE(world->overlapOrientedBoxCollidersSync(box, hits, 1u)
        == PhysResult::OutOfRange);
    CHECK_TRUE(hits.empty());
    box.includeSensors = true;
    CHECK_TRUE(world->overlapOrientedBoxCollidersSync(box, hits)
        == PhysResult::Ok);
    CHECK_INT_EQ(hits.size(), 3u);
    CHECK_INT_EQ(hits[2].collider, sensor);
    box.layerMask = 1u << 2u;
    CHECK_TRUE(world->overlapOrientedBoxCollidersSync(box, hits)
        == PhysResult::Ok);
    CHECK_INT_EQ(hits.size(), 1u);
    CHECK_INT_EQ(hits[0].body, sensorBody);
    box.layerMask = 0xFFFFFFFFu;
    box.ignoredBodies = {body};
    CHECK_TRUE(world->overlapOrientedBoxCollidersSync(box, hits)
        == PhysResult::Ok);
    CHECK_INT_EQ(hits.size(), 1u);
    CHECK_INT_EQ(hits[0].collider, sensor);

    CapsuleOverlap2D circle;
    circle.center = {0.0f, 0.0f, 0.0f};
    circle.radius = 1.0f;
    circle.height = 2.0f;
    CHECK_TRUE(world->overlapCapsuleCollidersSync(circle, hits)
        == PhysResult::Ok);
    CHECK_INT_EQ(hits.size(), 2u);
    CHECK_INT_EQ(hits[0].collider, left);
    CHECK_INT_EQ(hits[1].collider, right);
    circle.includeSensors = true;
    CHECK_TRUE(world->overlapCapsuleCollidersSync(circle, hits)
        == PhysResult::Ok);
    CHECK_INT_EQ(hits.size(), 3u);
    CHECK_TRUE(world->overlapCapsuleCollidersSync(circle, hits, 2u)
        == PhysResult::OutOfRange);
    CHECK_TRUE(hits.empty());
    circle.ignoredBodies = {body};
    CHECK_TRUE(world->overlapCapsuleCollidersSync(circle, hits)
        == PhysResult::Ok);
    CHECK_INT_EQ(hits.size(), 1u);
    CHECK_INT_EQ(hits[0].collider, sensor);

    const auto records = manager->debugDraw().consumeRecords();
    CHECK_TRUE(records.size() >= 2u);
    CHECK_TRUE(records[0].label == "Overlap rotated box colliders");
    CHECK_TRUE(records[records.size() - 1u].label
        == "Overlap capsule colliders");
    manager->shutdown();
}
#endif

TEST_CASE(InvalidAndNullBackendClearOutput) {
    PhysicsBackendDescriptor descriptor;
    descriptor.kind3D = BackendKind::Null;
    descriptor.kind2D = BackendKind::Null;
    auto manager = PhysicsManager::create(descriptor);
    CHECK_TRUE(manager != nullptr);
    PhysicsWorld2D* world = manager->world2D();
    std::vector<ColliderOverlapHit2D> hits(1u);
    CapsuleOverlap2D capsule;
    CHECK_TRUE(world->overlapCapsuleCollidersSync(capsule, hits, 0u)
        == PhysResult::InvalidParam);
    CHECK_TRUE(hits.empty());
    CHECK_TRUE(world->overlapCapsuleCollidersSync(capsule, hits)
        == PhysResult::Unsupported);
    OrientedBoxOverlap2D box;
    CHECK_TRUE(world->overlapOrientedBoxCollidersSync(box, hits, 1025u)
        == PhysResult::InvalidParam);
    CHECK_TRUE(world->overlapOrientedBoxCollidersSync(box, hits)
        == PhysResult::Unsupported);
    CHECK_TRUE(hits.empty());
    manager->shutdown();
    CHECK_TRUE(world->overlapCapsuleCollidersSync(capsule, hits)
        == PhysResult::InvalidState);
    CHECK_TRUE(hits.empty());
}

TEST_SUITE_END
