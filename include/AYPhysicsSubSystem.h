#pragma once
// AYPhysicsSubSystem.h - GameLoop integration (§11)
//
// R1 stub: declares the SubSystem interface; concrete impl lives in
// src/AYPhysicsSubSystem.cpp. GameLoop wiring is R3 (E-1).

#include "AYPhysicsTypes.h"

#include <memory>

namespace ayt::physics {

class PhysicsManager;

// Lightweight wrapper for GameLoop registration. Owns the manager for the
// subsystem's lifetime; on update() calls step() and fetchResults().
class PhysicsSubSystem {
public:
    PhysicsSubSystem();
    ~PhysicsSubSystem();

    bool initialize(const PhysicsBackendDescriptor& desc);
    void shutdown();

    void update(float deltaTime);   // step + fetchResults
    void fixedUpdate(float fixedDeltaTime);  // no-op for now; Jolt takes over in R1.5

    PhysicsManager* manager() { return _manager.get(); }

private:
    std::unique_ptr<PhysicsManager> _manager;
    PhysicsBackendDescriptor _descriptor{};
    bool _initialized = false;
};

} // namespace ayt::physics