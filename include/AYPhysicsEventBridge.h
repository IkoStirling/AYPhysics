#pragma once
// AYPhysicsEventBridge.h — E-3: snapshot collisions → EventBus
//
// Maps PhysFrameSnapshot::collisionEvents onto ayt::event::PhysicsCollisionEvent
// on the game/main thread (after PhysicsManager::fetchResults). Does not post
// from ContactListener / physics-thread publishSnapshot.

#include "AYPhysicsTypes.h"

#include <ayevent/EventBus.h>

#include <cstddef>
#include <span>

namespace ayt::physics {

/// Map + post collision PODs. Returns number of events posted.
/// kind 0/1/2 pass through; Box2D sensor 3→0 / 4→2; other kinds skipped.
/// Events with InvalidBodyHandle (0) on either body are skipped.
std::size_t publishCollisionEventsToBus(
    std::span<const CollisionEvent> events,
    ayt::event::EventBus& bus);

/// Convenience: drain `snap.collisionEvents` onto `EventBus::instance()`.
std::size_t publishCollisionEventsToBus(const PhysFrameSnapshot& snap);

} // namespace ayt::physics
