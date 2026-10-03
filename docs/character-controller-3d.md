# 3D character controller and exact collider hits

**Status:** Implemented, beta · **Date:** 2026-10-03

## Runtime entry

Use `PhysicsManager::world3D()` for controller ownership. In a standard engine
host, resolve `PhysicsSubSystem::findRegistered()`, check `manager()` and
`isRunning()`, then borrow `world3D()`. An embedded/headless host may own a
`PhysicsManager` directly. All facade calls run on the game thread. Only the
physics thread accesses Jolt.

`CharacterController3DDesc` describes an upright capsule; `position` is its feet,
`height` includes the spherical caps, and `maxSlopeDegrees` uses degrees. The
controller supports planar movement, wall sliding, walkable/steep slope
classification, configurable stairs and ground snapping, gravity, one-shot
grounded jumps, ceiling collision, moving-platform velocity and mutual layer
filtering. Sensors do not block it. Controllers also collide with one another.

```cpp
#include <AYPhysics/PhysicsManager.h>
#include <AYPhysics/PhysicsWorld3D.h>

using namespace ayt::physics;
// manager is a live, Jolt-enabled PhysicsManager owned by the host.
auto* physics = manager.world3D();
CharacterController3DDesc desc;
desc.position = {0, 0, 0};
desc.maxSlopeDegrees = 45;
desc.maxStepHeight = 0.3f;
desc.groundSnapDistance = 0.4f;
CharacterHandle character = InvalidCharacterHandle;
auto result = physics->createCharacter(desc, character);
if (result != PhysResult::Ok) { /* Handle unsupported backend/invalid config/capacity. */ }

// FixedPrePhysics: sample gameplay input. This is world-space velocity, not displacement.
result = physics->setCharacterInput(character, {{0, 0, 3}, 0});
// The standard GameLoop owns the step. Embedded hosts call it themselves:
result = manager.stepAndWait(1.0f / 60.0f);
// FixedPostPhysics: copy the feet position to the entity/presentation transform.
CharacterController3DState state;
result = physics->getCharacterState(character, state);
if (result == PhysResult::Ok) { /* Consume state.position/state.isGrounded(). */ }

// World/entity teardown, while the manager is still alive:
result = physics->destroyCharacter(character);
```

GameLoop hosts must not add a second physics step to this example. Submit input
before `FixedPhysics`, read after its completion barrier. Horizontal velocity
persists until replaced; the last input command before a step wins. Set zero
velocity to stop. A positive `jumpSpeed` is consumed once at the next step; it
only launches from walkable ground while not already moving upward. Airborne
jumps are discarded, not queued for landing. Upward jumps are never snapped
back to the floor. Set step/snap distances to zero to disable those behaviors.

`OnGround` permits jumping; `OnSteepGround` is a contact on an unwalkable slope,
`NotSupported` is contact without support, and `InAir` is no support/contact.
At a sharp descending step, a round capsule can briefly report steep edge
contact before reaching the walkable lower floor. A gap beyond the snap distance
falls under gravity. A teleport clears input/velocity and refreshes contacts;
it is not a swept movement and callers must choose a sensible spawn location.

## Lifecycle, failures and scope

- `Ok` for create acknowledges enqueueing. Native creation is confirmed by a
  successful `getCharacterState` after a completed step. Before that it may
  return `NotFound`. Capacity is `maxBodies - 1` controller slots, independently
  of body/collider slots. Destroyed slots are reused with a bumped generation.
- Destroy enqueue success immediately invalidates the gameplay handle. The
  next completed snapshot removes its native state. Stale input/destroy/read
  returns `NotFound`; old handles cannot affect a reused slot. Queue/pool
  exhaustion leaves no live handle and reports `QueueFull`/`NoMemory`.
- Invalid/nonfinite dimensions, slope outside `[0,90)`, negative step/snap/jump,
  nonzero input Y, or layer >= 32 report `InvalidParam`. Null/Mock/Jolt-off
  creation reports `Unsupported`; a stopped manager reports `InvalidState`.
- Handles are manager-local. Do not transfer them between managers or keep
  borrowed facade pointers after destruction. Destroy controllers on World
  removal; a manager may outlive several Worlds. Read snapshots after the fixed
  barrier; asynchronous overlapping simulation/read is not this contract.
- This first version uses Y-up upright capsules and the world's Y gravity.
  It is a virtual controller, not a Rigidbody: ray/overlap queries do not return
  it as a BodyHandle, and it does not produce ordinary rigidbody contact events.
  Entity/Scene auto-authoring, crouch, arbitrary-up gravity and root motion are
  outside this interface. Gameplay may copy state to Transform explicitly.
- Character-to-character detection is a simple serial list; very large crowds
  require a spatial index. Jolt is not a deterministic lockstep solver.

## Exact 3D ray hits and compounds

`createCollider` appends to a body's shape set. Multiple shapes become a tagged
Jolt compound, preserving each local offset. `destroyCollider` removes its
geometry; destroying the body invalidates all its colliders. Empty bodies use
Jolt `EmptyShape`, with no phantom sphere and no backend-wide shape cache.

Sync and async rays return the same `RaycastHit`: exact `body` and `collider`,
world-space intersection point, surface normal and distance. Layer masks filter
the targets. Jolt casts at most 10000 world units and normalizes a finite,
nonzero direction; a zero/nonfinite sync direction reports `InvalidParam`.
Misses clear the hit. Internal subshape paths are never exposed or retained as
gameplay identity. Compound children store the full ColliderHandle, resolved
against both collider and owner-body generations. A mesh's triangle path still
resolves to its owning collider. Character ground snapshots use the same mapping.

Jolt friction/restitution and sensor state remain **body-wide**. All colliders
on one body must agree on sensor status; mixed solid/sensor creates reject
without replacing existing shapes. Use a separate body for trigger geometry.
`createCollider` still acknowledges the queue rather than native acceptance;
the backend rejection counter and subsequent query/state expose rejection.
Overlap APIs continue to return body identities, not exact hit parts.

## Evidence

`AYPhysics_Tests` suites `CharacterController3DTests` and `JoltColliderHitTests`
exercise real Jolt through the manager queue, step barrier and query facade.
They cover movement/stop, wall slide, walkable/steep slopes, low/high stairs,
descending contact/snap, disabled step/snap, jump/ceiling, moving platforms,
sensors/layers, two characters, teleport, deletion, stale handles, reuse,
capacity, invalid input, compounds, async hits and mesh subshapes. The backend
fallback test also runs without Jolt. See the engine's
[`physics-gameplay-3d.md`](../../../AYDocs/physics-gameplay-3d.md) for Ability
hit-part integration and recorded verification.
