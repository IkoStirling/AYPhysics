#include <AYTest.h>
#include <AYPhysics/GravityField3D.h>
#include <AYPhysics/FlightControl3D.h>
#include "Physics3DTestScene.h"
#include <cmath>
#include <limits>

using namespace ayt::physics;
using ayt::math::FVector3;
TEST_SUITE(GravityField3DTests)

TEST_CASE(PriorityOverlapAndVacuum) {
    GravityField3D field;
    GravitySource3D down; down.id=1; down.acceleration=4;
    GravitySource3D side; side.id=2; side.shape=GravityShape3D::Box;
    side.priority=1; side.direction={1,0,0}; side.acceleration=6;
    CHECK_TRUE(field.replace({down,side}));
    const auto inside=field.sample({0,0,0});
    CHECK_INT_EQ(inside.primarySource,2u);
    CHECK_FLOAT_EQ(inside.acceleration.x,6,1e-4f);
    CHECK_FLOAT_EQ(inside.acceleration.y,0,1e-4f);
    const auto outside=field.sample({5,0,0});
    CHECK_FLOAT_EQ(outside.acceleration.y,-4,1e-4f);
    CHECK_TRUE(field.replace({side}));
    CHECK_FALSE(field.sample({5,0,0}).active);
}
TEST_CASE(RadialFalloffAndAtomicValidation) {
    GravityField3D field;
    GravitySource3D radial; radial.id=10; radial.shape=GravityShape3D::Radial;
    radial.radius=10; radial.acceleration=20; radial.linearFalloff=true;
    CHECK_TRUE(field.replace({radial}));
    CHECK_FLOAT_EQ(field.sample({5,0,0}).acceleration.x,-10,1e-4f);
    CHECK_FALSE(field.sample({11,0,0}).active);
    CHECK_FALSE(field.sample({0,0,0}).active);
    radial.radius=0;
    CHECK_FALSE(field.replace({radial}));
    CHECK_INT_EQ(field.size(),1u);
    CHECK_FLOAT_EQ(field.sample({5,0,0}).acceleration.x,-10,1e-4f);
}
TEST_CASE(SourceChannelsFilterBeforePrioritySelection) {
    GravityField3D field;
    GravitySource3D player; player.id=1; player.priority=10;
    player.channelMask=1; player.direction={1,0,0};
    GravitySource3D props; props.id=2; props.priority=1;
    props.channelMask=2; props.direction={0,-1,0};
    CHECK_TRUE(field.replace({player,props}));
    const auto a=field.sample({0,0,0},1);
    const auto b=field.sample({0,0,0},2);
    CHECK_FLOAT_EQ(a.acceleration.x,9.81f,1e-4f);
    CHECK_FLOAT_EQ(a.acceleration.y,0,1e-4f);
    CHECK_FLOAT_EQ(b.acceleration.x,0,1e-4f);
    CHECK_FLOAT_EQ(b.acceleration.y,-9.81f,1e-4f);
    CHECK_FALSE(field.sample({0,0,0},4).active);
}
TEST_CASE(FlightWrenchRotatesAllSixAxesAndRejectsInvalidInput) {
    FlightWrench3D result;
    FlightInput3D input; input.translation={1,0,0}; input.rotation={0,0,1};
    const auto orientation=ayt::math::FQuaternion::fromAxisAngle({0,1,0},1.5707963268f);
    CHECK_TRUE(flightWrench3D(input,orientation,{10,4},result));
    CHECK_FLOAT_EQ(result.force.z,-10,1e-3f);
    CHECK_FLOAT_EQ(result.torque.x,4,1e-3f);
    input.translation.x=std::numeric_limits<float>::quiet_NaN();
    CHECK_FALSE(flightWrench3D(input,orientation,{10,4},result));
    CHECK_FLOAT_EQ(result.force.z,-10,1e-3f);
}

#if defined(AYPHYSICS_HAS_JOLT)
TEST_CASE(SpatialGravityAcceleratesDynamicBodyWithoutGlobalGravity) {
    PhysicsBackendDescriptor desc;
    desc.kind3D=BackendKind::DefaultJolt; desc.kind2D=BackendKind::Null;
    desc.gravity3D={0,0,0}; desc.jobWorkerCount=1;
    desc.maxBodies=256; desc.maxBodyPairs=1024; desc.maxContactConstraints=1024;
    auto manager=PhysicsManager::create(desc);
    CHECK_NOT_NULL(manager.get());
    auto* world=manager->world3D();
    RigidbodyDesc body; body.type=BodyType::Dynamic; body.position={0,0,0};
    body.alwaysSync=true; body.linearDamping=0;
    BodyHandle handle{}; ayt::physics::controller_tests::accepted(world->createRigidbody(body,handle));
    ColliderDesc collider; collider.body=handle;
    ColliderHandle ch{}; ayt::physics::controller_tests::accepted(world->createCollider(collider,ch));
    GravityField3D field;
    GravitySource3D source; source.id=1; source.direction={1,0,0}; source.acceleration=10;
    CHECK_TRUE(field.replace({source}));
    FVector3 position{};
    for (int i=0;i<60;++i) {
        const GravityBodySample3D sample{handle,position};
        const auto dispatch=submitGravity3D(*world,field,std::span(&sample,1));
        CHECK_TRUE(dispatch.result==PhysResult::Ok);
        ayt::physics::controller_tests::accepted(manager->stepAndWait(1.f/60));
        for (const auto& transform:manager->fetchResults().transforms)
            if (transform.body==handle) position=transform.position;
    }
    CHECK_TRUE(position.x>3);
    CHECK_TRUE(std::abs(position.y)<.1f);
}
#endif
TEST_SUITE_END
