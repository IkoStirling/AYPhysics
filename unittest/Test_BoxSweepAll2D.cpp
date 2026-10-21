#include <AYPhysics/PhysicsManager.h>
#include <AYPhysics/PhysicsWorld2D.h>
#include <AYTest.h>

#include "Test_Box2DBackend2D_Helpers.h"

#include <limits>

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

TEST_SUITE(BoxSweepAll2DTests)

#if defined(AYPHYSICS_HAS_BOX2D)
TEST_CASE(CollectsSortedExactCollidersAndHonorsFilters) {
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

    BoxSweep2D sweep;
    sweep.origin = {0.0f, 7.0f, 9.0f};
    sweep.displacement = {0.0f, -10.0f, 3.0f};
    sweep.halfExtents = {0.75f, 0.5f, 0.0f};
    PhysicsDebugDrawOptions options;
    options.enabled = true;
    CHECK_TRUE(manager->debugDraw().configure(options) == PhysResult::Ok);
    std::vector<RaycastHit> hits;
    CHECK_TRUE(world->boxSweepAllSync(sweep, hits) == PhysResult::Ok);
    CHECK_INT_EQ(hits.size(), 2u);
    CHECK_INT_EQ(hits[0].body, top);
    CHECK_INT_EQ(hits[0].collider, topCollider);
    CHECK_INT_EQ(hits[1].body, bottom);
    CHECK_INT_EQ(hits[1].collider, bottomCollider);
    CHECK_TRUE(hits[0].distance < hits[1].distance);
    CHECK_TRUE(hits[0].normal.y > 0.5f);
    CHECK_FLOAT_EQ(hits[0].point.z, 0.0f, 1.0e-6f);
    const auto records = manager->debugDraw().consumeRecords();
    CHECK_INT_EQ(records.size(), 1u);
    CHECK_TRUE(records[0].label == "Box sweep all");
    CHECK_INT_EQ(records[0].lines.size(), 12u);

    sweep.includeSensors = true;
    CHECK_TRUE(world->boxSweepAllSync(sweep, hits) == PhysResult::Ok);
    CHECK_INT_EQ(hits.size(), 3u);
    CHECK_INT_EQ(hits[1].body, sensor);
    CHECK_INT_EQ(hits[1].collider, sensorCollider);
    sweep.layerMask = 1u << 3u;
    CHECK_TRUE(world->boxSweepAllSync(sweep, hits) == PhysResult::Ok);
    CHECK_INT_EQ(hits.size(), 2u);
    sweep.ignoredBodies = {top};
    CHECK_TRUE(world->boxSweepAllSync(sweep, hits) == PhysResult::Ok);
    CHECK_INT_EQ(hits.size(), 1u);
    CHECK_INT_EQ(hits[0].body, bottom);

    CHECK_TRUE(world->boxSweepAllSync(sweep, hits, 1u) == PhysResult::Ok);
    CHECK_INT_EQ(hits.size(), 1u);
    sweep.ignoredBodies.clear();
    CHECK_TRUE(world->boxSweepAllSync(sweep, hits, 1u)
        == PhysResult::OutOfRange);
    CHECK_TRUE(hits.empty());
    manager->shutdown();
}

TEST_CASE(OneBodyCanReturnMultipleExactColliders) {
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
    shape.halfExtents = {0.5f, 0.2f, 0.0f};
    shape.offset = {0.0f, 1.0f, 0.0f};
    ColliderHandle upper = 0u, lower = 0u;
    CHECK_TRUE(world->createCollider(shape, upper) == PhysResult::Ok);
    shape.offset.y = -1.0f;
    CHECK_TRUE(world->createCollider(shape, lower) == PhysResult::Ok);
    CHECK_TRUE(manager->stepAndWait(1.0f / 60.0f) == PhysResult::Ok);

    BoxSweep2D sweep;
    sweep.origin = {0.0f, 4.0f, 0.0f};
    sweep.displacement = {0.0f, -8.0f, 0.0f};
    std::vector<RaycastHit> hits;
    CHECK_TRUE(world->boxSweepAllSync(sweep, hits) == PhysResult::Ok);
    CHECK_INT_EQ(hits.size(), 2u);
    CHECK_INT_EQ(hits[0].body, body);
    CHECK_INT_EQ(hits[1].body, body);
    CHECK_INT_EQ(hits[0].collider, upper);
    CHECK_INT_EQ(hits[1].collider, lower);
    CHECK_TRUE(hits[0].distance < hits[1].distance);
    manager->shutdown();
}
#endif

TEST_CASE(InvalidInputAndNullBackendClearOutput) {
    PhysicsBackendDescriptor descriptor;
    descriptor.kind3D = BackendKind::Null;
    descriptor.kind2D = BackendKind::Null;
    auto manager = PhysicsManager::create(descriptor);
    CHECK_TRUE(manager != nullptr);
    PhysicsWorld2D* world = manager->world2D();
    BoxSweep2D sweep;
    sweep.displacement = {0.0f, -5.0f, 0.0f};
    std::vector<RaycastHit> hits(1u);
    CHECK_TRUE(world->boxSweepAllSync(sweep, hits, 0u)
        == PhysResult::InvalidParam);
    CHECK_TRUE(hits.empty());
    CHECK_TRUE(world->boxSweepAllSync(sweep, hits, 1025u)
        == PhysResult::InvalidParam);
    sweep.halfExtents.x = 0.0f;
    CHECK_TRUE(world->boxSweepAllSync(sweep, hits) == PhysResult::InvalidParam);
    sweep.halfExtents.x = 0.5f;
    sweep.displacement = {0.0f, 0.0f, 1.0f};
    CHECK_TRUE(world->boxSweepAllSync(sweep, hits) == PhysResult::InvalidParam);
    sweep.displacement = {0.0f, -5.0f, 0.0f};
    sweep.origin.x = (std::numeric_limits<float>::quiet_NaN)();
    CHECK_TRUE(world->boxSweepAllSync(sweep, hits) == PhysResult::InvalidParam);
    sweep.origin = {};
    sweep.ignoredBodies.resize(65u);
    CHECK_TRUE(world->boxSweepAllSync(sweep, hits) == PhysResult::InvalidParam);
    sweep.ignoredBodies.clear();
    CHECK_TRUE(world->boxSweepAllSync(sweep, hits) == PhysResult::Unsupported);
    CHECK_TRUE(hits.empty());
    manager->shutdown();
    CHECK_TRUE(world->boxSweepAllSync(sweep, hits) == PhysResult::InvalidState);
    CHECK_TRUE(hits.empty());
}

TEST_SUITE_END
