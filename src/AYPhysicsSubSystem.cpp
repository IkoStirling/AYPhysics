#include "AYPhysicsSubSystem.h"

#include "AYPhysicsManager.h"

namespace ayt::physics {

PhysicsSubSystem::PhysicsSubSystem()  = default;
PhysicsSubSystem::~PhysicsSubSystem() { shutdown(); }

bool PhysicsSubSystem::initialize(const PhysicsBackendDescriptor& desc) {
    if (_initialized) return true;
    _manager = PhysicsManager::create(desc);
    if (!_manager) return false;
    _descriptor = desc;
    _initialized = true;
    return true;
}

void PhysicsSubSystem::shutdown() {
    if (_manager) _manager->shutdown();
    _manager.reset();
    _initialized = false;
}

void PhysicsSubSystem::update(float deltaTime) {
    if (!_manager) return;
    (void)_manager->step(deltaTime);
    (void)_manager->fetchResults();
}

void PhysicsSubSystem::fixedUpdate(float /*fixedDeltaTime*/) {
    // R1.5 Jolt will own fixed-step; for now no-op.
}

} // namespace ayt::physics