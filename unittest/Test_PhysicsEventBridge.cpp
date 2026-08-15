// Test_PhysicsEventBridge.cpp — E-3: collision snapshot → EventBus

#include "AYPhysics/PhysicsEventBridge.h"
#include "AYPhysics/PhysicsHandles.h"
#include "AYTest.h"

#include <AYEventSystem/EventBus.h>
#include <AYEventSystem/Events/PhysicsEvents.h>

#include <vector>

using ayt::physics::BodyHandle;
using ayt::physics::CollisionEvent;
using ayt::physics::InvalidBodyHandle;
using ayt::physics::makeHandle;
using ayt::physics::publishCollisionEventsToBus;

namespace {

void resetBus()
{
    ayt::event::EventBus::instance().unsubscribeAll();
    ayt::event::EventBus::instance().resetCounters();
}

} // namespace

TEST_SUITE(PhysicsEventBridge)

TEST_CASE(E3_PostsEnterStayExit)
{
    resetBus();
    auto& bus = ayt::event::EventBus::instance();

    std::vector<ayt::event::PhysicsCollisionEvent> got;
    bus.subscribe<ayt::event::PhysicsCollisionEvent>(
        [&](const ayt::event::PhysicsCollisionEvent& e) { got.push_back(e); });

    const BodyHandle a = makeHandle(1, 1);
    const BodyHandle b = makeHandle(2, 1);
    CollisionEvent events[] = {
        {a, b, {}, {}, {}, 0.0f, 0},
        {a, b, {}, {}, {}, 0.0f, 1},
        {a, b, {}, {}, {}, 0.0f, 2},
    };

    CHECK_INT_EQ(publishCollisionEventsToBus(events, bus), 3u);
    bus.pump();

    CHECK_INT_EQ(static_cast<int>(got.size()), 3);
    CHECK_INT_EQ(got[0].bodyA, a);
    CHECK_INT_EQ(got[0].bodyB, b);
    CHECK_INT_EQ(static_cast<int>(got[0].kind), 0);
    CHECK_INT_EQ(static_cast<int>(got[1].kind), 1);
    CHECK_INT_EQ(static_cast<int>(got[2].kind), 2);

    resetBus();
}

TEST_CASE(E3_MapsSensorKindsAndSkipsInvalid)
{
    resetBus();
    auto& bus = ayt::event::EventBus::instance();

    std::vector<uint8_t> kinds;
    bus.subscribe<ayt::event::PhysicsCollisionEvent>(
        [&](const ayt::event::PhysicsCollisionEvent& e) {
            kinds.push_back(e.kind);
        });

    const BodyHandle a = makeHandle(3, 1);
    const BodyHandle b = makeHandle(4, 1);
    CollisionEvent events[] = {
        {a, b, {}, {}, {}, 0.0f, 3},                 // sensor enter → 0
        {a, b, {}, {}, {}, 0.0f, 4},                 // sensor exit  → 2
        {InvalidBodyHandle, b, {}, {}, {}, 0.0f, 0}, // skip
        {a, InvalidBodyHandle, {}, {}, {}, 0.0f, 0}, // skip
        {a, b, {}, {}, {}, 0.0f, 9},                 // unknown kind skip
    };

    CHECK_INT_EQ(publishCollisionEventsToBus(events, bus), 2u);
    bus.pump();

    CHECK_INT_EQ(static_cast<int>(kinds.size()), 2);
    CHECK_INT_EQ(static_cast<int>(kinds[0]), 0);
    CHECK_INT_EQ(static_cast<int>(kinds[1]), 2);

    resetBus();
}

TEST_SUITE_END
