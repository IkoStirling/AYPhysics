#include "AYPhysics/PhysicsEventBridge.h"

#include <AYEventSystem/Events/PhysicsEvents.h>

namespace ayt::physics {
namespace {

bool mapKind(uint8_t in, uint8_t& out)
{
    // 0=enter, 1=stay, 2=exit — EventBus POD contract.
    if (in <= 2) {
        out = in;
        return true;
    }
    // Box2D sensors: 3=enter, 4=exit → bus enter/exit.
    if (in == 3) {
        out = 0;
        return true;
    }
    if (in == 4) {
        out = 2;
        return true;
    }
    return false;
}

} // namespace

std::size_t publishCollisionEventsToBus(std::span<const CollisionEvent> events,
                                        ayt::event::EventBus& bus)
{
    std::size_t posted = 0;
    for (const CollisionEvent& ce : events) {
        if (ce.bodyA == InvalidBodyHandle || ce.bodyB == InvalidBodyHandle) {
            continue;
        }
        uint8_t kind = 0;
        if (!mapKind(ce.kind, kind)) {
            continue;
        }
        ayt::event::PhysicsCollisionEvent ev{};
        ev.bodyA = ce.bodyA;
        ev.bodyB = ce.bodyB;
        ev.kind = kind;
        bus.post(ev);
        ++posted;
    }
    return posted;
}

std::size_t publishCollisionEventsToBus(const PhysFrameSnapshot& snap)
{
    return publishCollisionEventsToBus(
        std::span<const CollisionEvent>(snap.collisionEvents.data(),
                                        snap.collisionEvents.size()),
        ayt::event::EventBus::instance());
}

} // namespace ayt::physics
