#include <AYTest.h>
#include <AYPhysics/PhysicsManager.h>
#include <AYPhysics/PhysicsWorld2D.h>
#include <AYPhysics/PhysicsWorld3D.h>
#include <algorithm>
#include <cmath>
#include "Physics3DTestScene.h"
using namespace ayt::physics;
TEST_SUITE(PhysicsDebugSnapshotTests)
#if defined(AYPHYSICS_HAS_JOLT)
TEST_CASE(native_static_hull_mesh_heightfield_and_controller_are_owned) {
    controller_tests::Scene scene;scene.manager->setDebugSnapshotEnabled(true);
    scene.floor();scene.character();
    auto add=[&](ColliderShape type,std::shared_ptr<ColliderShapeData> data,float x) {
        RigidbodyDesc desc;desc.type=BodyType::Static;desc.position={x,0,0};BodyHandle body;
        CHECK(scene.world->createRigidbody(desc,body)==PhysResult::Ok);
        ColliderDesc shape;shape.body=body;shape.shape=type;shape.shapeData=data;ColliderHandle collider;
        CHECK(scene.world->createCollider(shape,collider)==PhysResult::Ok);
    };
    auto hull=std::make_shared<ColliderShapeData>();hull->hullPoints={{-1,0,-1},{1,0,-1},{0,0,1},{0,2,0}};add(ColliderShape::ConvexHull,hull,5);
    auto mesh=std::make_shared<ColliderShapeData>();mesh->meshVertices={{0,0,0},{0,0,2},{2,0,0}};mesh->meshIndices={0,1,2};add(ColliderShape::TriangleMesh,mesh,10);
    auto terrain=std::make_shared<ColliderShapeData>();terrain->heightGridN=4;terrain->heightSamples.assign(16,0);terrain->heightSamples[5]=1;add(ColliderShape::Heightfield,terrain,15);
    scene.tick();const auto snapshot=scene.manager->fetchDebugSnapshot();
    CHECK(std::count_if(snapshot.records.begin(),snapshot.records.end(),[](auto& r){return r.category==PhysicsDebugCategory::Collider && !r.lines.empty();})==4);
    CHECK(std::any_of(snapshot.records.begin(),snapshot.records.end(),[](auto& r){return r.category==PhysicsDebugCategory::Character && r.lines.size()>30;}));
    scene.tick();CHECK(snapshot.records.size()>4); // value copy survives another publication
    scene.manager->setDebugSnapshotEnabled(false);CHECK(scene.manager->fetchDebugSnapshot().records.empty());
}
TEST_CASE(async_oriented_box_is_real_filtered_and_captured_once) {
    controller_tests::Scene scene;auto body=scene.box({2,0,0},{.2f,.2f,.2f},0,3);scene.tick();
    PhysicsDebugDrawOptions options;options.enabled=true;scene.manager->debugDraw().configure(options);
    const auto rotation=ayt::math::FQuaternion::fromAxisAngle({0,0,1},1.57079632679f);
    const auto miss=scene.world->overlapBoxAsync({0,0,0},{.2f,3,.2f},ayt::math::FQuaternion::identity(),1u<<3);
    const auto hit=scene.world->overlapBoxAsync({0,0,0},{.2f,3,.2f},rotation,1u<<3);
    const auto filtered=scene.world->overlapBoxAsync({0,0,0},{3,3,3},rotation,1u<<2);
    CHECK(miss && hit && filtered);scene.tick();
    const auto& results=scene.manager->fetchResults();
    auto find=[&](auto id)->const QueryResult& {return *std::find_if(results.queryResults.begin(),results.queryResults.end(),[&](auto& r){return r.queryId==id;});};
    CHECK(find(miss).hitCount==0);CHECK(find(hit).hitCount==1);CHECK(find(hit).hits[0].body==body);CHECK(find(filtered).hitCount==0);
    auto records=scene.manager->debugDraw().consumeRecords();CHECK(records.size()==3);
    CHECK(records[1].queryId==hit && records[1].hit && records[1].layerMask==(1u<<3));
    CHECK(records[1].hitBodies.size()==1 && records[1].hitBodies[0]==body);
    scene.manager->fetchResults();CHECK(scene.manager->debugDraw().consumeRecords().empty());
}
TEST_CASE(joints_export_native_anchor_frames_and_distance_limits) {
    controller_tests::Scene scene;scene.manager->setDebugSnapshotEnabled(true,1u<<5);
    const auto fixed=scene.floor();RigidbodyDesc desc;desc.type=BodyType::Dynamic;desc.position={0,3,0};BodyHandle moving;
    CHECK(scene.world->createRigidbody(desc,moving)==PhysResult::Ok);
    ColliderDesc shape;shape.body=moving;ColliderHandle collider;CHECK(scene.world->createCollider(shape,collider)==PhysResult::Ok);
    JointDesc joint;joint.type=JointType::Distance;joint.bodyA=fixed;joint.bodyB=moving;joint.anchorA={0,0,0};joint.anchorB={0,3,0};joint.minDistance=1;joint.maxDistance=3;JointHandle handle;
    CHECK(scene.world->createJoint(joint,handle)==PhysResult::Ok);scene.tick();
    const auto snapshot=scene.manager->fetchDebugSnapshot();CHECK(snapshot.records.size()==1);
    if(snapshot.records.size()!=1)return;const auto& record=snapshot.records[0];
    CHECK(record.category==PhysicsDebugCategory::Joint);CHECK(record.body==fixed && record.otherBody==moving);CHECK(record.lines.size()>=8);
    CHECK(std::abs(record.lines.back().to.y-record.lines.back().from.y-2)<.01f);
}
TEST_CASE(sphere_overlap_has_no_broadphase_false_positive_and_honors_mask) {
    controller_tests::Scene scene;scene.box({.9f,.9f,0},{.05f,.05f,.05f},0,2);scene.tick();
    std::vector<BodyHandle> bodies;
    CHECK(scene.world->overlapSphereSync({},1,bodies,1u<<2)==PhysResult::Ok);CHECK(bodies.empty());
    CHECK(scene.world->overlapSphereSync({},2,bodies,1u<<3)==PhysResult::Ok);CHECK(bodies.empty());
}
#endif
#if defined(AYPHYSICS_HAS_BOX2D)
TEST_CASE(two_d_native_geometry_and_solved_impulse_are_diagnostic_only) {
    PhysicsBackendDescriptor d;d.kind3D=BackendKind::Null;d.kind2D=BackendKind::DefaultBox2D;d.maxBodies=128;
    auto manager=PhysicsManager::create(d);CHECK(manager!=nullptr);manager->setDebugSnapshotEnabled(true);auto* world=manager->world2D();
    auto create=[&](BodyType type,float y,ayt::math::FVector3 half) {
        RigidbodyDesc desc;desc.type=type;desc.position={0,y,0};BodyHandle body;world->createRigidbody(desc,body);
        ColliderDesc shape;shape.body=body;shape.halfExtents=half;ColliderHandle collider;world->createCollider(shape,collider);return body;
    };
    create(BodyType::Static,-.5f,{10,.5f,.5f});create(BodyType::Dynamic,1,{.5f,.5f,.5f});
    bool impulse=false;
    for(int i=0;i<90;++i) {
        CHECK(manager->stepAndWait(1.f/60)==PhysResult::Ok);
        const auto snapshot=manager->fetchDebugSnapshot();
        for(const auto& r:snapshot.records) if(r.category==PhysicsDebugCategory::Contact && r.impulseAvailable && r.impulse>0) impulse=true;
    }
    CHECK(impulse);const auto snapshot=manager->fetchDebugSnapshot();
    CHECK(std::count_if(snapshot.records.begin(),snapshot.records.end(),[](auto& r){return r.category==PhysicsDebugCategory::Collider;})==2);
}
#endif
TEST_SUITE_END
