#pragma once
// AYPhysics/PhysicsSubSystem.h - GameLoop integration (§11 / E-1)
//
// PhysicsSubSystem : ISubSystem. Owns PhysicsManager for the subsystem
// lifetime. GameLoop calls fixedUpdate() → step + fetchResults + E-3
// PhysicsEventBridge (collision → EventBus). update() is a no-op.
//
// Registration: ayt::app::registerPhysicsModule() (AYApplication) or
// PhysicsSubSystem::registerSubSystem(desc) directly.

#include "AYPhysics/PhysicsTypes.h"
#include "AYPhysics/PhysicsQueryAdapter.h"

#include <AYGameLoop.h>

#include <memory>

namespace ayt::physics {

class PhysicsManager;
class IPhysicsQuery;

class PhysicsSubSystem : public ayt::game::ISubSystem {
public:
    PhysicsSubSystem();
    ~PhysicsSubSystem() override;

    /// Most recent live instance (GameLoop typically owns one).
    static PhysicsSubSystem* findRegistered() { return s_instance; }

    /// Register into GameLoop. Optional descriptor applied before initialize().
    static void registerSubSystem(const PhysicsBackendDescriptor& desc = {});

    const char* getName() const override { return "Physics"; }
    const ayt::game::SubSystemDescriptor& getDescriptor() const override;

    /// Install backend descriptor before initialize(). No-op after init.
    void setDescriptor(const PhysicsBackendDescriptor& desc);

    bool initialize() override;
    void update(float deltaTime) override;
    void fixedUpdate(float fixedDeltaTime) override;
    bool tickChecked(ayt::game::FramePhase phase,
                     const ayt::game::FrameContext& context) override;
    void shutdown() override;

    PhysicsManager* manager() { return _manager.get(); }
    const PhysicsManager* manager() const { return _manager.get(); }

    /// Narrow query facade (bound after successful initialize).
    IPhysicsQuery* query() { return &_query; }

private:
    bool runFixedStep(float fixedDeltaTime);

    std::unique_ptr<PhysicsManager> _manager;
    PhysicsQueryAdapter _query;
    PhysicsBackendDescriptor _descriptor{};
    ayt::game::SubSystemDescriptor _loopDescriptor{};
    bool _initialized = false;

    static PhysicsSubSystem* s_instance;
};

} // namespace ayt::physics
