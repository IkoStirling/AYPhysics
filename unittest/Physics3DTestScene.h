#pragma once
#include <AYPhysics/PhysicsManager.h>
#include <AYPhysics/PhysicsWorld3D.h>
#include <stdexcept>
#include <cmath>

namespace ayt::physics::controller_tests {
inline void accepted(PhysResult result) {
    if (result != PhysResult::Ok) throw std::runtime_error(toString(result));
}
struct Scene {
    std::unique_ptr<PhysicsManager> manager;
    PhysicsWorld3D* world = nullptr;
    explicit Scene(uint32_t capacity = 256) {
        PhysicsBackendDescriptor desc;
        desc.kind3D = BackendKind::DefaultJolt;
        desc.kind2D = BackendKind::Null;
        desc.maxBodies = capacity;
        desc.maxBodyPairs = 1024;
        desc.maxContactConstraints = 1024;
        desc.jobWorkerCount = 1;
        manager = PhysicsManager::create(desc);
        if (!manager) throw std::runtime_error("Jolt manager creation failed");
        world = manager->world3D();
    }
    void tick(int frames = 1) {
        for (int i = 0; i < frames; ++i) accepted(manager->stepAndWait(1.0f/60));
    }
    BodyHandle box(ayt::math::FVector3 center, ayt::math::FVector3 half,
                   float angleZDegrees = 0, PhysLayer layer = 0, bool sensor = false,
                   ColliderHandle* outCollider = nullptr) {
        RigidbodyDesc body;
        body.type = BodyType::Static;
        body.position = center;
        body.layer = layer;
        const float halfAngle = angleZDegrees * 0.00872664626f;
        body.rotation = {0, 0, std::sin(halfAngle), std::cos(halfAngle)};
        BodyHandle handle;
        accepted(world->createRigidbody(body, handle));
        ColliderDesc shape;
        shape.body = handle;
        shape.halfExtents = half;
        shape.isTrigger = sensor;
        ColliderHandle collider;
        accepted(world->createCollider(shape, collider));
        if (outCollider) *outCollider = collider;
        return handle;
    }
    BodyHandle floor(ColliderHandle* collider = nullptr) {
        return box({0,-0.25f,0}, {20,0.25f,20}, 0, 0, false, collider);
    }
    CharacterHandle character(CharacterController3DDesc desc = {}) {
        CharacterHandle handle;
        accepted(world->createCharacter(desc, handle));
        return handle;
    }
    CharacterController3DState state(CharacterHandle handle) {
        CharacterController3DState output;
        accepted(world->getCharacterState(handle, output));
        return output;
    }
    void move(CharacterHandle handle, float x, float z, float jump = 0) {
        accepted(world->setCharacterInput(handle, {{x,0,z},jump}));
    }
};
} // namespace ayt::physics::controller_tests
