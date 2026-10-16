#include <AYTest.h>
#include "Physics3DTestScene.h"
#include <AYPhysics/PhysicsQueryAdapter.h>
#include <limits>
using namespace ayt::physics;
using namespace ayt::physics::controller_tests;
TEST_SUITE(SphereSweepTests)
#if defined(AYPHYSICS_HAS_JOLT)
TEST_CASE(FiniteSweepHitsSphereEdgeFiltersSensorsLayersAndGenerationHandles) {
    Scene scene; ColliderHandle wallShape;
    const auto self = scene.box({0,0,0},{.3f,.3f,.3f});
    scene.box({0,0,-1},{1,1,.1f},0,0,true);
    const auto wall = scene.box({.4f,0,-3},{.3f,1,.2f},0,5,false,&wallShape);
    scene.tick(); PhysicsQueryAdapter facade; facade.bind(scene.manager.get());
    SphereSweep sweep; sweep.origin={0,0,0}; sweep.displacement={0,0,-10}; sweep.radius=.2f;
    sweep.ignoredBodies={self}; RaycastHit hit; accepted(facade.sphereSweepSync(sweep,hit));
    CHECK(hit.hit); CHECK_INT_EQ(hit.body,wall); CHECK_INT_EQ(hit.collider,wallShape);
    CHECK(hit.distance>2.5f && hit.distance<2.9f); CHECK(hit.normal.z>0.5f);
    sweep.layerMask=1; accepted(facade.sphereSweepSync(sweep,hit)); CHECK_FALSE(hit.hit);
    sweep.layerMask=~0u; sweep.includeSensors=true; accepted(facade.sphereSweepSync(sweep,hit));
    CHECK(hit.hit); CHECK(hit.distance<1); sweep.includeSensors=false;
    sweep.ignoredBodies.push_back(wall); accepted(facade.sphereSweepSync(sweep,hit)); CHECK_FALSE(hit.hit);
    accepted(scene.world->destroyRigidbody(wall)); scene.tick();
    const auto replacement=scene.box({0,0,-3},{1,1,.2f}); scene.tick();
    accepted(facade.sphereSweepSync(sweep,hit)); CHECK(hit.hit); CHECK_INT_EQ(hit.body,replacement);
    sweep.origin={0,0,-3}; accepted(facade.sphereSweepSync(sweep,hit)); CHECK(hit.hit); CHECK_FLOAT_EQ(hit.distance,0,1e-5f);
}
TEST_CASE(FailedQueriesResetHitAndRejectNonfiniteOversizedOrZeroTravel) {
    Scene scene; SphereSweep sweep; sweep.displacement={0,0,-5}; RaycastHit hit;
    sweep.radius=0; hit.hit=true; CHECK(scene.world->sphereSweepSync(sweep,hit)==PhysResult::InvalidParam); CHECK_FALSE(hit.hit);
    sweep.radius=.2f; sweep.displacement={}; CHECK(scene.world->sphereSweepSync(sweep,hit)==PhysResult::InvalidParam);
    sweep.displacement={0,0,10001}; CHECK(scene.world->sphereSweepSync(sweep,hit)==PhysResult::InvalidParam);
    sweep.displacement={0,0,5}; sweep.origin.x=std::numeric_limits<float>::quiet_NaN();
    CHECK(scene.world->sphereSweepSync(sweep,hit)==PhysResult::InvalidParam);
    sweep.origin={}; sweep.ignoredBodies.resize(65); CHECK(scene.world->sphereSweepSync(sweep,hit)==PhysResult::InvalidParam);
    scene.manager->shutdown(); sweep.ignoredBodies.clear(); CHECK(scene.world->sphereSweepSync(sweep,hit)==PhysResult::InvalidState);
}
#endif
TEST_CASE(NullAndMockReportUnsupportedRatherThanUnobstructedSpace) {
    for(auto kind:{BackendKind::Null,BackendKind::Mock}) {
        PhysicsBackendDescriptor descriptor; descriptor.kind3D=kind; descriptor.kind2D=BackendKind::Null;
        auto manager=PhysicsManager::create(descriptor); CHECK(manager!=nullptr);
        SphereSweep sweep; sweep.displacement={0,0,5}; RaycastHit hit; hit.hit=true;
        CHECK(manager->world3D()->sphereSweepSync(sweep,hit)==PhysResult::Unsupported); CHECK_FALSE(hit.hit);
    }
}
TEST_SUITE_END
