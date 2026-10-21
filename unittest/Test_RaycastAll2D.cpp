#include <AYPhysics/PhysicsManager.h>
#include <AYPhysics/PhysicsWorld2D.h>
#include <AYTest.h>

#include "Test_Box2DBackend2D_Helpers.h"

#include <limits>
#include <vector>

using namespace ayt::physics;

TEST_SUITE(RaycastAll2DTests)

#if defined(AYPHYSICS_HAS_BOX2D)
TEST_CASE(CollectsExactCollidersInDistanceOrderWithFilters) {
    auto manager = ayt::physics::test_helpers::makeBox2DMgr();
    CHECK_TRUE(manager != nullptr);
    PhysicsWorld2D* world = manager->world2D();
    const auto addBox = [&](float x, PhysLayer layer, bool sensor,
                            BodyHandle& body, ColliderHandle& collider) {
        RigidbodyDesc rigid;
        rigid.type = BodyType::Static;
        rigid.layer = layer;
        rigid.position = {x, 0.0f, 0.0f};
        if (world->createRigidbody(rigid, body) != PhysResult::Ok) return false;
        ColliderDesc shape;
        shape.body = body;
        shape.shape = ColliderShape::Box;
        shape.halfExtents = {0.25f, 0.25f, 0.0f};
        shape.isTrigger = sensor;
        return world->createCollider(shape, collider) == PhysResult::Ok;
    };
    BodyHandle first = 0u, second = 0u, sensor = 0u;
    ColliderHandle firstShape = 0u, secondShape = 0u, sensorShape = 0u;
    CHECK_TRUE(addBox(0.0f, 3u, false, first, firstShape));
    CHECK_TRUE(addBox(2.0f, 3u, false, second, secondShape));
    CHECK_TRUE(addBox(3.0f, 2u, true, sensor, sensorShape));
    ColliderDesc extra;
    extra.body = first;
    extra.shape = ColliderShape::Sphere;
    extra.radius = 0.1f;
    extra.offset = {0.6f, 0.0f, 0.0f};
    ColliderHandle extraShape = 0u;
    CHECK_TRUE(world->createCollider(extra, extraShape) == PhysResult::Ok);
    CHECK_TRUE(manager->stepAndWait(1.0f / 60.0f) == PhysResult::Ok);

    PhysicsDebugDrawOptions options;
    options.enabled = true;
    CHECK_TRUE(manager->debugDraw().configure(options) == PhysResult::Ok);
    RaycastAll2D query;
    query.origin = {-2.0f, 0.0f, 0.0f};
    query.displacement = {6.0f, 0.0f, 0.0f};
    std::vector<RaycastHit> hits{{}};
    CHECK_TRUE(world->raycastAllSync(query, hits) == PhysResult::Ok);
    CHECK_INT_EQ(hits.size(), 3u);
    CHECK_INT_EQ(hits[0].body, first);
    CHECK_INT_EQ(hits[0].collider, firstShape);
    CHECK_INT_EQ(hits[1].body, first);
    CHECK_INT_EQ(hits[1].collider, extraShape);
    CHECK_INT_EQ(hits[2].body, second);
    CHECK_INT_EQ(hits[2].collider, secondShape);
    CHECK_TRUE(hits[0].distance < hits[1].distance);
    CHECK_TRUE(hits[1].distance < hits[2].distance);
    const auto records = manager->debugDraw().consumeRecords();
    CHECK_INT_EQ(records.size(), 1u);
    CHECK_TRUE(records[0].label == "Raycast all");
    CHECK_FLOAT_EQ(records[0].lines[0].from.x, -2.0f, 0.001f);
    CHECK_FLOAT_EQ(records[0].lines[0].to.x, 4.0f, 0.001f);

    query.includeSensors = true;
    query.layerMask = 1u << 2u;
    CHECK_TRUE(world->raycastAllSync(query, hits) == PhysResult::Ok);
    CHECK_INT_EQ(hits.size(), 1u);
    CHECK_INT_EQ(hits[0].collider, sensorShape);
    query.layerMask = 0xFFFFFFFFu;
    query.includeSensors = false;
    query.ignoredBodies = {first};
    CHECK_TRUE(world->raycastAllSync(query, hits) == PhysResult::Ok);
    CHECK_INT_EQ(hits.size(), 1u);
    CHECK_INT_EQ(hits[0].body, second);
    query.ignoredBodies.clear();
    query.maxHits = 1u;
    CHECK_TRUE(world->raycastAllSync(query, hits) == PhysResult::OutOfRange);
    CHECK_TRUE(hits.empty());
    query.maxHits = 64u;
    query.origin.y = 5.0f;
    CHECK_TRUE(world->raycastAllSync(query, hits) == PhysResult::Ok);
    CHECK_TRUE(hits.empty());
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
    RaycastAll2D query;
    std::vector<RaycastHit> hits(1u);
    query.displacement = {};
    CHECK_TRUE(world->raycastAllSync(query, hits) == PhysResult::InvalidParam);
    CHECK_TRUE(hits.empty());
    query.displacement = {1.0f, 0.0f, 0.0f};
    query.origin.x = (std::numeric_limits<float>::infinity)();
    CHECK_TRUE(world->raycastAllSync(query, hits) == PhysResult::InvalidParam);
    query.origin = {};
    query.maxHits = 0u;
    CHECK_TRUE(world->raycastAllSync(query, hits) == PhysResult::InvalidParam);
    query.maxHits = 1025u;
    CHECK_TRUE(world->raycastAllSync(query, hits) == PhysResult::InvalidParam);
    query.maxHits = 64u;
    query.ignoredBodies.resize(65u);
    CHECK_TRUE(world->raycastAllSync(query, hits) == PhysResult::InvalidParam);
    query.ignoredBodies.clear();
    CHECK_TRUE(world->raycastAllSync(query, hits) == PhysResult::Unsupported);
    CHECK_TRUE(hits.empty());
    manager->shutdown();
    hits.resize(1u);
    CHECK_TRUE(world->raycastAllSync(query, hits) == PhysResult::InvalidState);
    CHECK_TRUE(hits.empty());
}

TEST_SUITE_END
