#include <AYPhysics/PhysicsManager.h>
#include <AYPhysics/PhysicsWorld2D.h>
#include <AYTest.h>

#include "Test_Box2DBackend2D_Helpers.h"

#include <limits>
#include <vector>

using namespace ayt::physics;

TEST_SUITE(OrientedBoxOverlap2DTests)

#if defined(AYPHYSICS_HAS_BOX2D)
TEST_CASE(RotationFiltersBodiesAndRecordsActualOutline) {
    auto manager = ayt::physics::test_helpers::makeBox2DMgr();
    CHECK_TRUE(manager != nullptr);
    PhysicsWorld2D* world = manager->world2D();
    const auto addBox = [&](float x, float y, PhysLayer layer,
                            bool sensor, BodyHandle& body) {
        RigidbodyDesc rigid;
        rigid.type = BodyType::Static;
        rigid.layer = layer;
        rigid.position = {x, y, 0.0f};
        if (world->createRigidbody(rigid, body) != PhysResult::Ok) return false;
        ColliderDesc shape;
        shape.body = body;
        shape.shape = ColliderShape::Box;
        shape.halfExtents = {0.12f, 0.12f, 0.0f};
        shape.isTrigger = sensor;
        ColliderHandle collider = 0u;
        return world->createCollider(shape, collider) == PhysResult::Ok;
    };
    BodyHandle center = 0u, diagonal = 0u, sensor = 0u, offLine = 0u;
    CHECK_TRUE(addBox(0.0f, 0.0f, 3u, false, center));
    CHECK_TRUE(addBox(1.0f, 1.0f, 3u, false, diagonal));
    CHECK_TRUE(addBox(-1.0f, -1.0f, 2u, true, sensor));
    CHECK_TRUE(addBox(1.0f, -1.0f, 3u, false, offLine));
    ColliderDesc second;
    second.body = center;
    second.shape = ColliderShape::Sphere;
    second.radius = 0.08f;
    ColliderHandle secondCollider = 0u;
    CHECK_TRUE(world->createCollider(second, secondCollider) == PhysResult::Ok);
    CHECK_TRUE(manager->stepAndWait(1.0f / 60.0f) == PhysResult::Ok);

    PhysicsDebugDrawOptions options;
    options.enabled = true;
    CHECK_TRUE(manager->debugDraw().configure(options) == PhysResult::Ok);
    OrientedBoxOverlap2D query;
    query.halfExtents = {1.5f, 0.15f, 0.0f};
    query.angleRadians = 0.78539816339f;
    std::vector<BodyHandle> found{offLine};
    CHECK_TRUE(world->overlapOrientedBoxSync(query, found) == PhysResult::Ok);
    CHECK_INT_EQ(found.size(), 2u);
    CHECK_TRUE(found[0] < found[1]);
    CHECK_TRUE(found[0] == center || found[1] == center);
    CHECK_TRUE(found[0] == diagonal || found[1] == diagonal);
    const auto records = manager->debugDraw().consumeRecords();
    CHECK_INT_EQ(records.size(), 1u);
    CHECK_TRUE(records[0].label == "Overlap rotated box");
    CHECK_TRUE(records[0].lines.size() >= 4u);

    query.angleRadians = 0.0f;
    CHECK_TRUE(world->overlapOrientedBoxSync(query, found) == PhysResult::Ok);
    CHECK_INT_EQ(found.size(), 1u);
    CHECK_INT_EQ(found.front(), center);
    query.angleRadians = 0.78539816339f;
    query.includeSensors = true;
    query.layerMask = 1u << 2u;
    CHECK_TRUE(world->overlapOrientedBoxSync(query, found) == PhysResult::Ok);
    CHECK_INT_EQ(found.size(), 1u);
    CHECK_INT_EQ(found.front(), sensor);
    query.ignoredBodies = {sensor};
    CHECK_TRUE(world->overlapOrientedBoxSync(query, found) == PhysResult::Ok);
    CHECK_TRUE(found.empty());
    manager->shutdown();
}
#endif

TEST_CASE(InvalidNullAndShutdownClearOutput) {
    PhysicsBackendDescriptor descriptor;
    descriptor.kind3D = BackendKind::Null;
    descriptor.kind2D = BackendKind::Null;
    auto manager = PhysicsManager::create(descriptor);
    CHECK_TRUE(manager != nullptr);
    PhysicsWorld2D* world = manager->world2D();
    OrientedBoxOverlap2D query;
    std::vector<BodyHandle> found{123u};
    query.halfExtents.x = 0.0f;
    CHECK_TRUE(world->overlapOrientedBoxSync(query, found)
        == PhysResult::InvalidParam);
    CHECK_TRUE(found.empty());
    query.halfExtents.x = 0.5f;
    query.angleRadians = (std::numeric_limits<float>::infinity)();
    CHECK_TRUE(world->overlapOrientedBoxSync(query, found)
        == PhysResult::InvalidParam);
    query.angleRadians = 0.0f;
    query.ignoredBodies.resize(65u);
    CHECK_TRUE(world->overlapOrientedBoxSync(query, found)
        == PhysResult::InvalidParam);
    query.ignoredBodies.clear();
    CHECK_TRUE(world->overlapOrientedBoxSync(query, found)
        == PhysResult::Unsupported);
    CHECK_TRUE(found.empty());
    manager->shutdown();
    found = {123u};
    CHECK_TRUE(world->overlapOrientedBoxSync(query, found)
        == PhysResult::InvalidState);
    CHECK_TRUE(found.empty());
}

TEST_SUITE_END
