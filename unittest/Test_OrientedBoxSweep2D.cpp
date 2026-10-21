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

TEST_SUITE(OrientedBoxSweep2DTests)

#if defined(AYPHYSICS_HAS_BOX2D)
TEST_CASE(RotationChangesContactAndFiltersApply) {
    auto manager = ayt::physics::test_helpers::makeBox2DMgr();
    CHECK_TRUE(manager != nullptr);
    PhysicsWorld2D* world = manager->world2D();
    BodyHandle sensor = 0u, wall = 0u;
    ColliderHandle sensorCollider = 0u, wallCollider = 0u;
    CHECK_TRUE(addStaticBox(*world, 3.0f, 2u, true,
        sensor, sensorCollider));
    CHECK_TRUE(addStaticBox(*world, 0.0f, 3u, false,
        wall, wallCollider));
    CHECK_TRUE(manager->stepAndWait(1.0f / 60.0f) == PhysResult::Ok);

    OrientedBoxSweep2D sweep;
    sweep.origin = {0.0f, 5.0f, 7.0f};
    sweep.displacement = {0.0f, -10.0f, 4.0f};
    sweep.halfExtents = {1.0f, 0.2f, 0.0f};
    sweep.angleRadians = 1.57079632679f;
    PhysicsDebugDrawOptions options;
    options.enabled = true;
    CHECK_TRUE(manager->debugDraw().configure(options) == PhysResult::Ok);

    RaycastHit hit;
    CHECK_TRUE(world->orientedBoxSweepSync(sweep, hit) == PhysResult::Ok);
    CHECK_TRUE(hit.hit);
    CHECK_INT_EQ(hit.body, wall);
    CHECK_INT_EQ(hit.collider, wallCollider);
    CHECK_TRUE(hit.distance > 3.7f && hit.distance < 3.9f);
    CHECK_TRUE(hit.normal.y > 0.5f);
    CHECK_FLOAT_EQ(hit.point.z, 0.0f, 1.0e-6f);
    const auto records = manager->debugDraw().consumeRecords();
    CHECK_INT_EQ(records.size(), 1u);
    CHECK_TRUE(records[0].label == "Rotated box sweep");
    CHECK_INT_EQ(records[0].lines.size(), 12u);

    sweep.angleRadians = 0.0f;
    CHECK_TRUE(world->orientedBoxSweepSync(sweep, hit) == PhysResult::Ok);
    CHECK_TRUE(hit.distance > 4.5f && hit.distance < 4.7f);
    sweep.angleRadians = 1.57079632679f;
    sweep.includeSensors = true;
    CHECK_TRUE(world->orientedBoxSweepSync(sweep, hit) == PhysResult::Ok);
    CHECK_INT_EQ(hit.body, sensor);
    CHECK_INT_EQ(hit.collider, sensorCollider);
    CHECK_TRUE(hit.distance > 0.7f && hit.distance < 0.9f);
    sweep.layerMask = 1u << 3u;
    CHECK_TRUE(world->orientedBoxSweepSync(sweep, hit) == PhysResult::Ok);
    CHECK_INT_EQ(hit.body, wall);
    sweep.includeSensors = false;
    sweep.ignoredBodies.push_back(wall);
    CHECK_TRUE(world->orientedBoxSweepSync(sweep, hit) == PhysResult::Ok);
    CHECK_FALSE(hit.hit);

    sweep.ignoredBodies.clear();
    sweep.origin = {0.0f, 0.0f, 0.0f};
    CHECK_TRUE(world->orientedBoxSweepSync(sweep, hit) == PhysResult::Ok);
    CHECK_TRUE(hit.hit);
    CHECK_FLOAT_EQ(hit.distance, 0.0f, 1.0e-6f);
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
    OrientedBoxSweep2D sweep;
    sweep.displacement = {0.0f, -5.0f, 0.0f};
    RaycastHit hit;
    hit.hit = true;
    sweep.angleRadians = (std::numeric_limits<float>::quiet_NaN)();
    CHECK_TRUE(world->orientedBoxSweepSync(sweep, hit) == PhysResult::InvalidParam);
    CHECK_FALSE(hit.hit);
    sweep.angleRadians = 0.0f;
    sweep.halfExtents.x = 0.0f;
    CHECK_TRUE(world->orientedBoxSweepSync(sweep, hit) == PhysResult::InvalidParam);
    sweep.halfExtents.x = 0.5f;
    sweep.displacement = {0.0f, 0.0f, 1.0f};
    CHECK_TRUE(world->orientedBoxSweepSync(sweep, hit) == PhysResult::InvalidParam);
    sweep.displacement = {10001.0f, 0.0f, 0.0f};
    CHECK_TRUE(world->orientedBoxSweepSync(sweep, hit) == PhysResult::InvalidParam);
    sweep.displacement = {0.0f, -5.0f, 0.0f};
    sweep.ignoredBodies.resize(65u);
    CHECK_TRUE(world->orientedBoxSweepSync(sweep, hit) == PhysResult::InvalidParam);
    sweep.ignoredBodies.clear();
    PhysicsDebugDrawOptions options;
    options.enabled = true;
    CHECK_TRUE(manager->debugDraw().configure(options) == PhysResult::Ok);
    hit.hit = true;
    CHECK_TRUE(world->orientedBoxSweepSync(sweep, hit) == PhysResult::Unsupported);
    CHECK_FALSE(hit.hit);
    const auto records = manager->debugDraw().consumeRecords();
    CHECK_INT_EQ(records.size(), 1u);
    CHECK_TRUE(records[0].status == PhysResult::Unsupported);
    CHECK_INT_EQ(records[0].lines.size(), 9u);
    manager->shutdown();
    hit.hit = true;
    CHECK_TRUE(world->orientedBoxSweepSync(sweep, hit) == PhysResult::InvalidState);
    CHECK_FALSE(hit.hit);
}

TEST_SUITE_END
