#include <AYPhysics/PhysicsManager.h>
#include <AYPhysics/PhysicsWorld2D.h>
#include <AYTest.h>

#include "Test_Box2DBackend2D_Helpers.h"

#include <limits>
#include <vector>

using namespace ayt::physics;

TEST_SUITE(CapsuleOverlap2DTests)

#if defined(AYPHYSICS_HAS_BOX2D)
TEST_CASE(UniqueBodiesAndFiltersFromRealBox2D) {
    auto manager = ayt::physics::test_helpers::makeBox2DMgr();
    CHECK_TRUE(manager != nullptr);
    PhysicsWorld2D* world = manager->world2D();
    const auto addBox = [&](float y, PhysLayer layer, bool sensor,
                            BodyHandle& body) {
        RigidbodyDesc rigid;
        rigid.type = BodyType::Static;
        rigid.layer = layer;
        rigid.position = {0.0f, y, 0.0f};
        if (world->createRigidbody(rigid, body) != PhysResult::Ok) return false;
        ColliderDesc shape;
        shape.body = body;
        shape.shape = ColliderShape::Box;
        shape.halfExtents = {0.4f, 0.2f, 0.0f};
        shape.isTrigger = sensor;
        ColliderHandle collider = 0u;
        return world->createCollider(shape, collider) == PhysResult::Ok;
    };
    BodyHandle wall = 0u, sensor = 0u, distant = 0u;
    CHECK_TRUE(addBox(0.0f, 3u, false, wall));
    CHECK_TRUE(addBox(0.5f, 2u, true, sensor));
    CHECK_TRUE(addBox(10.0f, 3u, false, distant));
    ColliderDesc second;
    second.body = wall;
    second.shape = ColliderShape::Sphere;
    second.radius = 0.1f;
    second.offset = {0.1f, 0.0f, 0.0f};
    ColliderHandle secondCollider = 0u;
    CHECK_TRUE(world->createCollider(second, secondCollider) == PhysResult::Ok);
    CHECK_TRUE(manager->stepAndWait(1.0f / 60.0f) == PhysResult::Ok);
    PhysicsDebugDrawOptions options;
    options.enabled = true;
    CHECK_TRUE(manager->debugDraw().configure(options) == PhysResult::Ok);

    CapsuleOverlap2D query;
    query.radius = 0.25f;
    query.height = 2.0f;
    std::vector<BodyHandle> found{distant};
    CHECK_TRUE(world->overlapCapsuleSync(query, found) == PhysResult::Ok);
    CHECK_INT_EQ(found.size(), 1u);
    CHECK_INT_EQ(found.front(), wall);
    const auto records = manager->debugDraw().consumeRecords();
    CHECK_INT_EQ(records.size(), 1u);
    CHECK_TRUE(records[0].label == "Overlap capsule");
    CHECK_TRUE(records[0].lines.size() > 30u);
    CHECK_INT_EQ(records[0].hitBodies.size(), 1u);

    query.includeSensors = true;
    CHECK_TRUE(world->overlapCapsuleSync(query, found) == PhysResult::Ok);
    CHECK_INT_EQ(found.size(), 2u);
    CHECK_TRUE(found[0] < found[1]);
    query.layerMask = 1u << 2u;
    CHECK_TRUE(world->overlapCapsuleSync(query, found) == PhysResult::Ok);
    CHECK_INT_EQ(found.size(), 1u);
    CHECK_INT_EQ(found.front(), sensor);
    query.layerMask = 1u << 3u;
    query.ignoredBodies = {wall};
    CHECK_TRUE(world->overlapCapsuleSync(query, found) == PhysResult::Ok);
    CHECK_TRUE(found.empty());
    query.ignoredBodies.clear();
    query.center = {100.0f, 100.0f, 0.0f};
    CHECK_TRUE(world->overlapCapsuleSync(query, found) == PhysResult::Ok);
    CHECK_TRUE(found.empty());
    query.center = {};
    query.height = query.radius * 2.0f;
    CHECK_TRUE(world->overlapCapsuleSync(query, found) == PhysResult::Ok);
    CHECK_INT_EQ(found.size(), 1u);
    CHECK_INT_EQ(found.front(), wall);
    manager->shutdown();
}
#endif

TEST_CASE(InvalidAndUnavailableQueriesClearOutput) {
    PhysicsBackendDescriptor descriptor;
    descriptor.kind3D = BackendKind::Null;
    descriptor.kind2D = BackendKind::Null;
    auto manager = PhysicsManager::create(descriptor);
    CHECK_TRUE(manager != nullptr);
    PhysicsWorld2D* world = manager->world2D();
    CapsuleOverlap2D query;
    std::vector<BodyHandle> found{123u};
    query.radius = 0.0f;
    CHECK_TRUE(world->overlapCapsuleSync(query, found) == PhysResult::InvalidParam);
    CHECK_TRUE(found.empty());
    query.radius = 0.5f;
    query.height = 0.9f;
    CHECK_TRUE(world->overlapCapsuleSync(query, found) == PhysResult::InvalidParam);
    query.height = 2001.0f;
    CHECK_TRUE(world->overlapCapsuleSync(query, found) == PhysResult::InvalidParam);
    query.height = 2.0f;
    query.center.y = (std::numeric_limits<float>::infinity)();
    CHECK_TRUE(world->overlapCapsuleSync(query, found) == PhysResult::InvalidParam);
    query.center = {};
    query.ignoredBodies.resize(65u);
    CHECK_TRUE(world->overlapCapsuleSync(query, found) == PhysResult::InvalidParam);
    query.ignoredBodies.clear();
    PhysicsDebugDrawOptions options;
    options.enabled = true;
    CHECK_TRUE(manager->debugDraw().configure(options) == PhysResult::Ok);
    CHECK_TRUE(world->overlapCapsuleSync(query, found) == PhysResult::Unsupported);
    CHECK_TRUE(found.empty());
    const auto records = manager->debugDraw().consumeRecords();
    CHECK_INT_EQ(records.size(), 1u);
    CHECK_TRUE(records[0].status == PhysResult::Unsupported);
    manager->shutdown();
    found = {123u};
    CHECK_TRUE(world->overlapCapsuleSync(query, found) == PhysResult::InvalidState);
    CHECK_TRUE(found.empty());
}

TEST_SUITE_END
