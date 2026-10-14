#include <AYTest.h>
#include "Physics3DTestScene.h"
#include <algorithm>
#include <limits>
#include <AYPhysics/PhysicsQueryAdapter.h>
using namespace ayt::physics;
using namespace ayt::physics::controller_tests;
TEST_SUITE(JoltColliderHitTests)
#if defined(AYPHYSICS_HAS_JOLT)
TEST_CASE(DebugDrawingTracksRealHitsMissesAndLayerFilters) {
    Scene scene;
    scene.box({0,0,3},{0.5f,0.5f,0.5f});
    scene.tick();
    PhysicsDebugDrawOptions options; options.enabled=true;
    accepted(scene.manager->debugDraw().configure(options));
    ayt::math::Ray ray; ray.dir={0,0,2};
    RaycastHit hit;
    accepted(scene.world->raycastSync(ray,hit)); CHECK(hit.hit);
    auto lines=scene.manager->debugDraw().consumeLines();
    CHECK(lines.size()==5); CHECK(lines.front().color==options.hitColor);
    CHECK(std::abs(lines[1].from.z-hit.point.z)<0.001f);
    accepted(scene.world->raycastSync(ray,hit,0)); CHECK(!hit.hit);
    lines=scene.manager->debugDraw().consumeLines(); CHECK(lines.size()==1);
    CHECK(lines.front().color==options.missColor);
    std::vector<BodyHandle> bodies;
    accepted(scene.world->overlapSphereSync({0,0,3},1,bodies,UINT32_MAX)); CHECK(!bodies.empty());
    lines=scene.manager->debugDraw().consumeLines(); CHECK(lines.size()==96);
    CHECK(lines.front().color==options.hitColor);
}
TEST_CASE(CompoundRaysReturnDistinctCollidersAndRealNormals) {
    Scene scene;
    RigidbodyDesc desc;
    desc.type = BodyType::Static;
    BodyHandle body;
    accepted(scene.world->createRigidbody(desc,body));
    ColliderDesc shape;
    shape.body = body;
    shape.halfExtents = {0.5f,0.5f,0.5f};
    shape.offset = {-2,0,0};
    ColliderHandle left, right;
    accepted(scene.world->createCollider(shape,left));
    shape.offset = {2,0,0};
    accepted(scene.world->createCollider(shape,right));
    scene.tick();
    for (auto [x,collider] : {std::pair{-2.0f,left},std::pair{2.0f,right}}) {
        ayt::math::Ray ray{}; ray.origin = {x,0,-3}; ray.dir = {0,0,2};
        RaycastHit hit;
        accepted(scene.world->raycastSync(ray,hit));
        CHECK_TRUE(hit.hit);
        CHECK_INT_EQ(hit.body,body);
        CHECK_INT_EQ(hit.collider,collider);
        CHECK_TRUE(std::abs(hit.distance-2.5f) < 0.01f);
        CHECK_TRUE(std::abs(hit.point.z+0.5f) < 0.01f);
        CHECK_TRUE(hit.normal.z < -0.99f);
    }
    ayt::math::Ray miss{}; miss.origin = {0,0,-3}; miss.dir = {0,0,1};
    RaycastHit output;
    accepted(scene.world->raycastSync(miss,output));
    CHECK_FALSE(output.hit);
    accepted(scene.world->destroyCollider(left));
    scene.tick();
    miss.origin.x = -2;
    accepted(scene.world->raycastSync(miss,output));
    CHECK_FALSE(output.hit);
    miss.origin.x = 2;
    accepted(scene.world->raycastSync(miss,output));
    CHECK_INT_EQ(output.collider,right);
    accepted(scene.world->destroyCollider(right));
    scene.tick();
    accepted(scene.world->raycastSync(miss,output));
    CHECK_FALSE(output.hit); // No placeholder geometry or stale collider remains.
}
TEST_CASE(AsyncIdentityLayersAndBodyDeletion) {
    Scene scene;
    ColliderHandle nearShape, farShape;
    const auto nearBody = scene.box({0,0,1},{0.2f,0.2f,0.2f},0,5,false,&nearShape);
    scene.box({0,0,3},{0.2f,0.2f,0.2f},0,0,false,&farShape);
    scene.tick();
    ayt::math::Ray ray{}; ray.dir={0,0,1};
    RaycastHit hit;
    PhysicsQueryAdapter facade;
    facade.bind(scene.manager.get());
    accepted(facade.raycastSync(ray,hit,1u));
    CHECK_INT_EQ(hit.collider,farShape);
    accepted(scene.world->raycastSync(ray,hit,0u));
    CHECK_FALSE(hit.hit);
    const auto query = scene.world->raycastAsync(ray,1u<<5);
    CHECK_TRUE(query != 0);
    scene.tick();
    const auto& results = scene.manager->fetchResults().queryResults;
    const auto found = std::find_if(results.begin(),results.end(),[query](const auto& r){return r.queryId==query;});
    CHECK_TRUE(found != results.end());
    if (found != results.end()) {
        CHECK_INT_EQ(found->hitCount,1u);
        CHECK_INT_EQ(found->hits.front().collider,nearShape);
        CHECK_INT_EQ(found->hits.front().body,nearBody);
    }
    accepted(scene.world->destroyRigidbody(nearBody));
    scene.tick();
    accepted(scene.world->raycastSync(ray,hit));
    CHECK_INT_EQ(hit.collider,farShape);
    ray.dir = {};
    CHECK_TRUE(scene.world->raycastSync(ray,hit) == PhysResult::InvalidParam);
    CHECK_FALSE(hit.hit);
}
TEST_CASE(MeshSubShapesResolveToTheirOwningCollider) {
    Scene scene;
    RigidbodyDesc desc;desc.type=BodyType::Static;
    BodyHandle body;
    accepted(scene.world->createRigidbody(desc,body));
    auto data=std::make_shared<ColliderShapeData>();
    data->meshVertices={{-1,0,-1},{1,0,-1},{1,0,1},{-1,0,1}};
    data->meshIndices={0,2,1,0,3,2};
    ColliderDesc collider;
    collider.body=body;collider.shape=ColliderShape::TriangleMesh;collider.shapeData=data;
    ColliderHandle mesh,box;
    accepted(scene.world->createCollider(collider,mesh));
    collider.shape=ColliderShape::Box;collider.shapeData.reset();collider.offset={3,0,0};
    accepted(scene.world->createCollider(collider,box));
    scene.tick();
    ayt::math::Ray ray{};ray.origin={0.3f,3,0};ray.dir={0,-1,0};
    RaycastHit hit;
    accepted(scene.world->raycastSync(ray,hit));
    CHECK_TRUE(hit.hit);
    CHECK_INT_EQ(hit.collider,mesh);
    CHECK_TRUE(hit.normal.y > 0.99f);
    accepted(scene.world->destroyCollider(box));scene.tick();
    accepted(scene.world->raycastSync(ray,hit));
    CHECK_INT_EQ(hit.collider,mesh);
}
TEST_CASE(MixedSensorRejectionAndGroundContactRebuildAreSafe) {
    Scene scene;
    ColliderHandle floorShape;
    const auto floor=scene.floor(&floorShape);
    ColliderDesc extra;
    extra.body=floor;extra.offset={3,2,0};
    ColliderHandle solid;
    accepted(scene.world->createCollider(extra,solid));
    extra.isTrigger=true;extra.offset={-3,2,0};
    ColliderHandle rejected;
    accepted(scene.world->createCollider(extra,rejected)); // Queue acknowledgement, native rejection.
    const auto character=scene.character();
    scene.tick(3);
    CHECK_TRUE(scene.state(character).isGrounded());
    CHECK_INT_EQ(scene.state(character).groundCollider,floorShape);
    ayt::math::Ray ray{};ray.origin={-3,2,-3};ray.dir={0,0,1};
    RaycastHit hit;
    accepted(scene.world->raycastSync(ray,hit));
    CHECK_FALSE(hit.hit);
    accepted(scene.world->destroyCollider(solid)); // Compound-to-single changes subshape paths.
    scene.tick();
    CHECK_TRUE(scene.state(character).isGrounded());
    CHECK_INT_EQ(scene.state(character).groundCollider,floorShape);
}
TEST_CASE(RotatedSurfaceNormalAndStaleColliderCommand) {
    Scene scene;
    ColliderHandle collider;
    scene.box({0,0,0},{3,0.2f,2},30,0,false,&collider);
    scene.tick();
    ayt::math::Ray ray{}; ray.origin={0,5,0}; ray.dir={0,-1,0};
    RaycastHit hit;
    accepted(scene.world->raycastSync(ray,hit));
    CHECK_INT_EQ(hit.collider,collider);
    CHECK_TRUE(hit.normal.x < -0.49f && hit.normal.y > 0.86f);
    const auto stale=makeHandle(handleIndex(collider),bumpGeneration(handleGeneration(collider)));
    accepted(scene.world->destroyCollider(stale));
    scene.tick();
    accepted(scene.world->raycastSync(ray,hit));
    CHECK_INT_EQ(hit.collider,collider);
}
#endif
TEST_SUITE_END
