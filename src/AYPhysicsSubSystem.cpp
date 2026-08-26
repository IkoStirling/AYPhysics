#include "AYPhysics/PhysicsSubSystem.h"

#include "AYPhysics/PhysicsEventBridge.h"
#include "AYPhysics/PhysicsManager.h"

#include <AYGameLoop.h>
#include <AYLog.h>

namespace ayt::physics {

PhysicsSubSystem* PhysicsSubSystem::s_instance = nullptr;

PhysicsSubSystem::PhysicsSubSystem()
{
    s_instance = this;
    // Physics owns the fixed simulation barrier. timeScale changes how many
    // fixed steps are due, never the constant dt passed to a physics step.
    _loopDescriptor.name = "Physics";
    _loopDescriptor.basePriority = 700;
    _loopDescriptor.timeType = ayt::game::SubSystemDescriptor::TimeType::Scaled;
    _loopDescriptor.phases = ayt::game::phaseBit(ayt::game::FramePhase::FixedPhysics);
    _loopDescriptor.clock = ayt::game::ClockDomain::Game;
    _loopDescriptor.phasePriority = 0;
    _loopDescriptor.reads = {"Physics.Commands"};
    _loopDescriptor.writes = {"Physics.Snapshot"};
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
    (void)runFixedStep(fixedDeltaTime);
}

bool PhysicsSubSystem::tickChecked(ayt::game::FramePhase phase,
                                   const ayt::game::FrameContext& context)
{
    if (phase == ayt::game::FramePhase::FixedPhysics) {
        return runFixedStep(context.fixedDeltaTime);
    }
    tick(phase, context);
    return true;
}

bool PhysicsSubSystem::runFixedStep(float fixedDeltaTime)
{
    if (!_manager) return false;

    const PhysResult result = _manager->stepAndWait(
        fixedDeltaTime,
        _descriptor.fixedStepTimeout);
    if (result != PhysResult::Ok) {
        // F-P2: clean shutdown wakes the cv with !_running before the in-flight
        // step's completionSequence lands; stepAndWait then returns InvalidState.
        // That is expected — don't pollute the log with a spurious "failed"
        // line on every shutdown.
        if (_manager->isRunning()) {
            ayt::log::error("[Physics] Fixed step completion failed: %s",
                            toString(result));
            return false;
        }
        return false;
    }

    // E-3: fan out snapshot collisions on the game thread (not from
    // ContactListener / physics-thread publishSnapshot).
    const PhysFrameSnapshot& snap = _manager->fetchResults();
    (void)publishCollisionEventsToBus(snap);
    return true;
}

} // namespace ayt::physics
