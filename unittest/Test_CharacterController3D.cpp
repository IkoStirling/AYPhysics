#include <AYTest.h>
#include "Physics3DTestScene.h"
#include <limits>
#include <algorithm>
#include "JoltBackend3D.h"

using namespace ayt::physics;
using namespace ayt::physics::controller_tests;
TEST_SUITE(CharacterController3DTests)

TEST_CASE(UnsupportedBackendAndInvalidDescriptorsFailExplicitly) {
    PhysicsBackendDescriptor backend;
    backend.kind3D = BackendKind::Null;
    auto manager = PhysicsManager::create(backend);
    CHECK_NOT_NULL(manager.get());
    CharacterHandle handle = 123;
    CharacterController3DDesc desc;
    CHECK_TRUE(manager->world3D()->createCharacter(desc,handle) == PhysResult::Unsupported);
    CHECK_INT_EQ(handle,0u);
    desc.radius = -1;
    CHECK_TRUE(manager->world3D()->createCharacter(desc,handle) == PhysResult::InvalidParam);
    desc = {}; desc.maxSlopeDegrees = 90;
    CHECK_TRUE(manager->world3D()->createCharacter(desc,handle) == PhysResult::InvalidParam);
    desc = {}; desc.height = desc.radius;
    CHECK_TRUE(manager->world3D()->createCharacter(desc,handle) == PhysResult::InvalidParam);
    desc = {}; desc.position.x = std::numeric_limits<float>::quiet_NaN();
    CHECK_TRUE(manager->world3D()->createCharacter(desc,handle) == PhysResult::InvalidParam);
    desc = {}; desc.groundSnapDistance = std::numeric_limits<float>::infinity();
    CHECK_TRUE(manager->world3D()->createCharacter(desc,handle) == PhysResult::InvalidParam);
}
#if !defined(AYPHYSICS_HAS_JOLT)
TEST_CASE(DefaultJoltFallbackCannotPretendToCreateController) {
    auto manager=PhysicsManager::create(PhysicsBackendDescriptor{});
    CHECK_NOT_NULL(manager.get());
    CharacterHandle character=123;
    CHECK_TRUE(manager->world3D()->createCharacter({},character) == PhysResult::Unsupported);
    CHECK_INT_EQ(character,0u);
}
#endif

#if defined(AYPHYSICS_HAS_JOLT)
TEST_CASE(WalkStopAndExactGroundIdentity) {
    Scene scene;
    ColliderHandle groundCollider;
    const auto ground = scene.floor(&groundCollider);
    const auto character = scene.character();
    scene.tick(3);
    auto state = scene.state(character);
    CHECK_TRUE(state.isGrounded());
    CHECK_TRUE(std::abs(state.position.y) < 0.06f);
    CHECK_INT_EQ(state.groundBody, ground);
    CHECK_INT_EQ(state.groundCollider, groundCollider);
    scene.move(character,0,3);
    scene.tick(60);
    state = scene.state(character);
    CHECK_TRUE(std::abs(state.position.z-3) < 0.15f);
    CHECK_TRUE(state.isGrounded());
    scene.move(character,0,0);
    scene.tick(20);
    CHECK_TRUE(std::abs(scene.state(character).position.z-state.position.z) < 0.03f);
}
TEST_CASE(WallCollisionAndSliding) {
    Scene scene;
    scene.floor();
    scene.box({0,2,2},{10,2,0.15f});
    const auto character = scene.character();
    scene.tick(3);
    scene.move(character,1,3);
    scene.tick(90);
    const auto state = scene.state(character);
    CHECK_TRUE(state.position.z < 1.6f);
    CHECK_TRUE(state.position.x > 1.0f);
    CHECK_TRUE(state.isGrounded());
}
TEST_CASE(WalkableSlopeClimbsAndRemainsSupported) {
    Scene scene;
    scene.box({0,0,0},{5,0.1f,3},30);
    CharacterController3DDesc desc;
    desc.position = {-2,-1.0f,0};
    const auto character = scene.character(desc);
    scene.tick(20);
    const auto before = scene.state(character);
    CHECK_TRUE(before.isGrounded());
    CHECK_TRUE(before.groundNormal.y > 0.8f);
    scene.move(character,2,0);
    scene.tick(60);
    const auto after = scene.state(character);
    CHECK_TRUE(after.position.x > before.position.x + 1.2f);
    CHECK_TRUE(after.position.y > before.position.y + 0.6f);
    CHECK_TRUE(after.isGrounded());
}
TEST_CASE(SteepSlopeDoesNotBecomeWalkable) {
    Scene scene;
    scene.box({0,0,0},{5,0.1f,3},60);
    CharacterController3DDesc desc;
    desc.position = {-1,-1.2f,0};
    const auto character = scene.character(desc);
    scene.tick(5);
    const auto before = scene.state(character);
    scene.move(character,4,0);
    scene.tick(25);
    const auto after = scene.state(character);
    CHECK_FALSE(after.isGrounded());
    CHECK_TRUE(after.position.x < before.position.x + 0.3f);
}
TEST_CASE(LowStepClimbsButHighStepBlocks) {
    {
        Scene scene;
        scene.floor();
        scene.box({0,0.1f,3},{2,0.1f,2});
        const auto character = scene.character();
        scene.tick(3);
        scene.move(character,0,2);
        scene.tick(100);
        const auto state = scene.state(character);
        CHECK_TRUE(state.position.z > 3.0f);
        CHECK_TRUE(state.position.y > 0.18f && state.position.y < 0.26f);
        CHECK_TRUE(state.isGrounded());
    }
    {
        Scene scene;
        scene.floor();
        scene.box({0,0.4f,3},{2,0.4f,2});
        const auto character = scene.character();
        scene.tick(3);
        scene.move(character,0,2);
        scene.tick(100);
        const auto state = scene.state(character);
        CHECK_TRUE(state.position.z < 0.8f);
        CHECK_TRUE(state.position.y < 0.08f);
    }
}
TEST_CASE(DescendingStepKeepsContactAndLargeDropFalls) {
    Scene scene;
    scene.floor();
    scene.box({0,0.125f,0},{2,0.125f,1});
    CharacterController3DDesc desc;
    desc.position = {0,0.25f,0};
    const auto character = scene.character(desc);
    scene.tick(3);
    scene.move(character,0,2);
    bool lostContact = false;
    for (int frame=0;frame<60;++frame) {
        scene.tick();
        const auto current = scene.state(character);
        // The rounded capsule can briefly classify the step's edge as steep.
        // It must maintain support/contact and reach the walkable lower floor.
        lostContact |= current.groundState == CharacterGroundState3D::InAir ||
                       current.groundState == CharacterGroundState3D::NotSupported;
    }
    CHECK_FALSE(lostContact);
    CHECK_TRUE(scene.state(character).isGrounded());
    CHECK_TRUE(scene.state(character).position.y < 0.06f);
    accepted(scene.world->teleportCharacter(character,{0,3,0}));
    scene.tick();
    CHECK_FALSE(scene.state(character).isGrounded());
    scene.tick(90);
    CHECK_TRUE(scene.state(character).isGrounded());
    CHECK_TRUE(scene.state(character).position.y < 0.31f);
}
TEST_CASE(JumpIsOneShotAndAirborneRequestIsNotBuffered) {
    Scene scene;
    scene.floor();
    const auto character = scene.character();
    scene.tick(3);
    scene.move(character,0,0,5);
    scene.tick(10);
    CHECK_TRUE(scene.state(character).position.y > 0.5f);
    CHECK_FALSE(scene.state(character).isGrounded());
    scene.move(character,0,0,20); // Consumed while airborne; no double jump.
    scene.tick(8);
    CHECK_TRUE(scene.state(character).linearVelocity.y < 5);
    scene.tick(120);
    const auto state = scene.state(character);
    CHECK_TRUE(state.isGrounded());
    CHECK_TRUE(state.position.y < 0.06f);
}
TEST_CASE(LayersAndSensorsDoNotBlockMovement) {
    Scene scene;
    scene.floor();
    scene.box({0,2,1},{2,2,0.1f},0,3,true);
    scene.box({0,2,2},{2,2,0.1f},0,5,false);
    CharacterController3DDesc desc;
    desc.collideMask = 0xFFFFFFFFu & ~(1u<<5); // Sensor layer is included, solid wall excluded.
    const auto character = scene.character(desc);
    scene.tick(3);
    scene.move(character,0,3);
    scene.tick(60);
    CHECK_TRUE(scene.state(character).position.z > 2.8f);
    CHECK_TRUE(scene.state(character).isGrounded());
}
TEST_CASE(DestroyTeleportAndStaleHandlesRespectLifecycle) {
    Scene scene;
    const auto ground = scene.floor();
    const auto character = scene.character();
    scene.tick(3);
    scene.move(character,0,3);
    accepted(scene.world->teleportCharacter(character,{4,0,0}));
    scene.tick(5);
    auto state = scene.state(character);
    CHECK_TRUE(std::abs(state.position.x-4) < 0.02f);
    CHECK_TRUE(std::abs(state.position.z) < 0.02f);
    CHECK_TRUE(state.isGrounded());
    const auto stale = makeHandle(handleIndex(character),bumpGeneration(handleGeneration(character)));
    CHECK_TRUE(scene.world->destroyCharacter(stale) == PhysResult::NotFound);
    scene.tick();
    CHECK_TRUE(scene.state(character).isGrounded());
    accepted(scene.world->destroyRigidbody(ground));
    scene.tick();
    CHECK_FALSE(scene.state(character).isGrounded());
    CHECK_INT_EQ(scene.state(character).groundBody,0u);
    accepted(scene.world->destroyCharacter(character));
    scene.tick();
    CHECK_TRUE(scene.world->getCharacterState(character,state) == PhysResult::NotFound);
    CHECK_TRUE(scene.manager->fetchResults().characters.empty());
    scene.tick(); // Ensure alternating snapshots cannot resurrect removed state.
    CHECK_TRUE(scene.manager->fetchResults().characters.empty());
}
TEST_CASE(InvalidInputNeverReachesSolverAndCapacityIsBounded) {
    Scene scene;
    scene.floor();
    const auto character = scene.character();
    scene.tick(3);
    CHECK_TRUE(scene.world->setCharacterInput(character,{{1,2,0},0}) == PhysResult::InvalidParam);
    CHECK_TRUE(scene.world->setCharacterInput(character,{{0,0,0},-1}) == PhysResult::InvalidParam);
    CHECK_TRUE(scene.world->teleportCharacter(character,{0,std::numeric_limits<float>::infinity(),0}) == PhysResult::InvalidParam);
    CHECK_TRUE(scene.world->destroyCharacter(1) == PhysResult::InvalidParam);
    CHECK_TRUE(scene.state(character).isGrounded());
    scene.manager->shutdown();
    CharacterHandle output=123;
    CHECK_TRUE(scene.world->createCharacter({},output) == PhysResult::InvalidState);
    CHECK_INT_EQ(output,0u);
    scene.manager.reset();scene.world=nullptr;
    {
        Scene small(2);
        const auto old=small.character();
        CHECK_TRUE(small.world->createCharacter({},output) == PhysResult::OutOfRange);
        CHECK_INT_EQ(output,0u);
        accepted(small.world->destroyCharacter(old));
        const auto reused=small.character(); // FIFO destroy/create can safely share a slot.
        CHECK_INT_EQ(handleIndex(old),handleIndex(reused));
        CHECK_TRUE(old != reused);
        CHECK_TRUE(small.world->destroyCharacter(old) == PhysResult::NotFound);
        small.tick();
        CHECK_INT_EQ(small.state(reused).character,reused);
    }
}
TEST_CASE(TwoCharactersCannotWalkThroughEachOther) {
    Scene scene;
    scene.floor();
    const auto first = scene.character();
    CharacterController3DDesc desc;
    desc.position = {0,0,2};
    const auto second = scene.character(desc);
    scene.tick(3);
    scene.move(first,0,2);
    scene.tick(60);
    CHECK_TRUE(scene.state(first).position.z < scene.state(second).position.z-0.5f);
}
TEST_CASE(DisabledStepAndSnapSettingsAreHonored) {
    {
        Scene scene;
        scene.floor();
        scene.box({0,0.15f,3},{2,0.15f,2});
        CharacterController3DDesc desc;
        desc.maxStepHeight=0;
        const auto character=scene.character(desc);
        scene.tick(3);scene.move(character,0,2);scene.tick(100);
        CHECK_TRUE(scene.state(character).position.z < 0.8f);
    }
    {
        Scene scene;
        scene.floor();
        scene.box({0,0.125f,0},{2,0.125f,1});
        CharacterController3DDesc desc;
        desc.position={0,0.25f,0};
        desc.groundSnapDistance=0;
        const auto character=scene.character(desc);
        scene.tick(3);scene.move(character,0,2);
        bool inAir=false;
        for (int i=0;i<60;++i) {
            scene.tick();
            inAir |= scene.state(character).groundState == CharacterGroundState3D::InAir;
        }
        CHECK_TRUE(inAir);
        CHECK_TRUE(scene.state(character).isGrounded());
    }
}
TEST_CASE(MovingPlatformCarriesIdleController) {
    Scene scene;
    RigidbodyDesc desc;
    desc.type=BodyType::Kinematic;
    desc.linearVelocity={1,0,0};
    desc.position={0,-0.25f,0};
    BodyHandle platform;
    accepted(scene.world->createRigidbody(desc,platform));
    ColliderDesc collider;
    collider.body=platform;collider.halfExtents={4,0.25f,4};
    ColliderHandle shape;
    accepted(scene.world->createCollider(collider,shape));
    const auto character=scene.character();
    scene.tick(60);
    const auto state=scene.state(character);
    CHECK_TRUE(state.isGrounded());
    CHECK_INT_EQ(state.groundBody,platform);
    CHECK_TRUE(state.position.x > 0.85f);
    CHECK_TRUE(std::abs(state.groundVelocity.x-1) < 0.05f);
}
TEST_CASE(CeilingBlocksJumpAndBidirectionalMaskExcludesWall) {
    Scene scene;
    scene.floor();
    scene.box({0,2.3f,0},{4,0.1f,4});
    RigidbodyDesc body;
    body.type=BodyType::Static;
    body.position={0,2,2};
    body.collideMask=1u; // Excludes the Character layer although the character wants it.
    BodyHandle wall;
    accepted(scene.world->createRigidbody(body,wall));
    ColliderDesc shape;
    shape.body=wall;shape.halfExtents={3,2,0.1f};
    ColliderHandle collider;
    accepted(scene.world->createCollider(shape,collider));
    const auto character=scene.character();
    scene.tick(3);scene.move(character,0,0,8);
    float peak=0;
    for (int i=0;i<90;++i) {
        scene.tick();peak=std::max(peak,scene.state(character).position.y);
    }
    CHECK_TRUE(peak < 0.45f);
    CHECK_TRUE(scene.state(character).isGrounded());
    scene.move(character,0,3);scene.tick(60);
    CHECK_TRUE(scene.state(character).position.z > 2.8f);
}
#endif
TEST_SUITE_END
