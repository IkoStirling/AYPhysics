#include "AYPhysicsSubSystem.h"

#include "AYPhysicsManager.h"

#include <AYGameLoop.h>

namespace ayt::physics {

PhysicsSubSystem* PhysicsSubSystem::s_instance = nullptr;

PhysicsSubSystem::PhysicsSubSystem()
{
    s_instance = this;
    // design §11: priority 700; Real time so pause/timeScale does not stall physics.
    _loopDescriptor.name = "Physics";
    _loopDescriptor.basePriority = 700;
    _loopDescriptor.timeType = ayt::game::SubSystemDescriptor::TimeType::Real;
}

PhysicsSubSystem::~PhysicsSubSystem()
{
    shutdown();
    if (s_instance == this) {
        s_instance = nullptr;
    }
}

const ayt::game::SubSystemDescriptor& PhysicsSubSystem::getDescriptor() const
{
    return _loopDescriptor;
}

void PhysicsSubSystem::setDescriptor(const PhysicsBackendDescriptor& desc)
{
    if (_initialized) {
        return;
    }
    _descriptor = desc;
}

void PhysicsSubSystem::registerSubSystem(const PhysicsBackendDescriptor& desc)
{
    auto* sub = new PhysicsSubSystem();
    sub->setDescriptor(desc);
    ayt::game::GameLoop::instance().registerSubSystem(sub);
}

bool PhysicsSubSystem::initialize()
{
    if (_initialized) {
        return true;
    }
    _manager = PhysicsManager::create(_descriptor);
    if (!_manager) {
        return false;
    }
    _query.bind(_manager.get());
    _initialized = true;
    return true;
}

void PhysicsSubSystem::shutdown()
{
    _query.bind(nullptr);
    if (_manager) {
        _manager->shutdown();
    }
    _manager.reset();
    _initialized = false;
}

void PhysicsSubSystem::update(float /*deltaTime*/)
{
    // Present/variable tick: physics advances on fixedUpdate only.
}

void PhysicsSubSystem::fixedUpdate(float fixedDeltaTime)
{
    if (!_manager) {
        return;
    }
    (void)_manager->step(fixedDeltaTime);
    (void)_manager->fetchResults();
}

} // namespace ayt::physics
