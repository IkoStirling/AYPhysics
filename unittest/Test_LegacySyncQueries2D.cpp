#include <AYPhysics/PhysicsManager.h>
#include <AYPhysics/PhysicsWorld2D.h>
#include <AYTest.h>

#include <limits>
#include <vector>

using namespace ayt::physics;

TEST_SUITE(LegacySyncQueries2DTests)

TEST_CASE(RejectsInvalidGeometryAndClearsStaleResults) {
    PhysicsBackendDescriptor descriptor;
    descriptor.kind3D = BackendKind::Null;
    descriptor.kind2D = BackendKind::Null;
    auto manager = PhysicsManager::create(descriptor);
    CHECK_TRUE(manager != nullptr);
    auto* world = manager->world2D();
    const float infinity = (std::numeric_limits<float>::infinity)();

    ayt::math::Ray ray;
    ray.dir = {1.0f, 0.0f, 0.0f};
    ray.origin.x = infinity;
    RaycastHit hit;
    hit.hit = true;
    hit.body = makeHandle(1u, 1u);
    CHECK_TRUE(world->raycastSync(ray, hit) == PhysResult::InvalidParam);
    CHECK_FALSE(hit.hit);
    CHECK_INT_EQ(hit.body, InvalidBodyHandle);
    ray.origin = {};
    ray.dir.y = infinity;
    hit.hit = true;
    CHECK_TRUE(world->raycastSync(ray, hit) == PhysResult::InvalidParam);
    CHECK_FALSE(hit.hit);

    std::vector<BodyHandle> bodies{makeHandle(1u, 1u)};
    CHECK_TRUE(world->overlapSphereSync({}, 0.0f, bodies)
        == PhysResult::InvalidParam);
    CHECK_TRUE(bodies.empty());
    bodies.push_back(makeHandle(1u, 1u));
    CHECK_TRUE(world->overlapSphereSync({infinity, 0.0f, 0.0f}, 1.0f, bodies)
        == PhysResult::InvalidParam);
    CHECK_TRUE(bodies.empty());
    bodies.push_back(makeHandle(1u, 1u));
    CHECK_TRUE(world->overlapBoxSync({}, {1.0f, -1.0f, 0.0f}, bodies)
        == PhysResult::InvalidParam);
    CHECK_TRUE(bodies.empty());
    bodies.push_back(makeHandle(1u, 1u));
    CHECK_TRUE(world->overlapBoxSync({}, {1.0f, 1.0f, infinity}, bodies)
        == PhysResult::InvalidParam);
    CHECK_TRUE(bodies.empty());
    manager->shutdown();
}

TEST_CASE(NullQueriesPreserveLegacyEmptyResultAndShutdownReturnsImmediately) {
    PhysicsBackendDescriptor descriptor;
    descriptor.kind3D = BackendKind::Null;
    descriptor.kind2D = BackendKind::Null;
    auto manager = PhysicsManager::create(descriptor);
    CHECK_TRUE(manager != nullptr);
    auto* world = manager->world2D();
    ayt::math::Ray ray;
    ray.dir = {1.0f, 0.0f, 0.0f};
    RaycastHit hit;
    hit.hit = true;
    CHECK_TRUE(world->raycastSync(ray, hit) == PhysResult::Ok);
    CHECK_FALSE(hit.hit);
    std::vector<BodyHandle> bodies{makeHandle(1u, 1u)};
    CHECK_TRUE(world->overlapSphereSync({}, 1.0f, bodies) == PhysResult::Ok);
    CHECK_TRUE(bodies.empty());
    bodies.push_back(makeHandle(1u, 1u));
    CHECK_TRUE(world->overlapBoxSync({}, {1.0f, 1.0f, 0.0f}, bodies)
        == PhysResult::Ok);
    CHECK_TRUE(bodies.empty());

    manager->shutdown();
    hit.hit = true;
    CHECK_TRUE(world->raycastSync(ray, hit) == PhysResult::InvalidState);
    CHECK_FALSE(hit.hit);
    bodies.push_back(makeHandle(1u, 1u));
    CHECK_TRUE(world->overlapSphereSync({}, 1.0f, bodies)
        == PhysResult::InvalidState);
    CHECK_TRUE(bodies.empty());
    bodies.push_back(makeHandle(1u, 1u));
    CHECK_TRUE(world->overlapBoxSync({}, {1.0f, 1.0f, 0.0f}, bodies)
        == PhysResult::InvalidState);
    CHECK_TRUE(bodies.empty());
}

TEST_CASE(AsyncQueriesRejectInvalidGeometryBeforeQueueing) {
    PhysicsBackendDescriptor descriptor;
    descriptor.kind3D = BackendKind::Null;
    descriptor.kind2D = BackendKind::Null;
    auto manager = PhysicsManager::create(descriptor);
    CHECK_TRUE(manager != nullptr);
    auto* world = manager->world2D();
    const float infinity = (std::numeric_limits<float>::infinity)();
    const float nan = (std::numeric_limits<float>::quiet_NaN)();

    ayt::math::Ray ray;
    ray.dir = {1.0f, 0.0f, 0.0f};
    ray.origin.z = infinity;
    CHECK_INT_EQ(world->raycastAsync(ray), 0u);
    ray.origin = {};
    ray.dir.x = nan;
    CHECK_INT_EQ(world->raycastAsync(ray), 0u);
    ray.dir = {1.0f, 0.0f, 0.0f};
    CHECK_TRUE(world->raycastAsync(ray) != 0u);

    CHECK_INT_EQ(world->overlapSphereAsync({}, 0.0f), 0u);
    CHECK_INT_EQ(world->overlapSphereAsync({infinity, 0.0f, 0.0f}, 1.0f), 0u);
    CHECK_INT_EQ(world->overlapSphereAsync({}, nan), 0u);
    CHECK_TRUE(world->overlapSphereAsync({}, 1.0f) != 0u);

    CHECK_INT_EQ(world->overlapBoxAsync({}, {0.0f, 1.0f, 0.0f}), 0u);
    CHECK_INT_EQ(world->overlapBoxAsync({}, {1.0f, 1.0f, infinity}), 0u);
    CHECK_INT_EQ(world->overlapBoxAsync({nan, 0.0f, 0.0f}, {1.0f, 1.0f, 0.0f}), 0u);
    CHECK_TRUE(world->overlapBoxAsync({}, {1.0f, 1.0f, 0.0f}) != 0u);

    manager->shutdown();
    CHECK_INT_EQ(world->raycastAsync(ray), 0u);
    CHECK_INT_EQ(world->overlapSphereAsync({}, 1.0f), 0u);
    CHECK_INT_EQ(world->overlapBoxAsync({}, {1.0f, 1.0f, 0.0f}), 0u);
}

TEST_SUITE_END
