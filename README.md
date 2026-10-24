# AYPhysics

AY Engine physics subsystem: rigidbody dynamics, collision detection, constraint joints, with **2D and 3D as fully separated worlds**. Backend-pluggable (Jolt for 3D, Box2D for 2D).

**Current architecture (2026-10-22):** `PhysicsManager` owns separate 2D and
3D command queues and synchronous query mailboxes, serviced by one physics
worker and a shared step/snapshot publication barrier. Box2D 3.x and Jolt are
both integrated. `AYPhysics2DBridge` is a separate adapter for tile terrain,
triggers and the 2D character controller; it is not the whole 2D physics API.
For connected 3D regions, `PhysicsWorld3D::transferRigidbody` admits one
pose/linear/angular velocity and layer/mask transition command. The Jolt worker
applies it before the next fixed step; rejected queue admission changes nothing.
The Phase/R2.5 table below records the original delivery plan and is not a
current availability matrix. Use the [2D SDK contract](../../AYDocs/sdk/contracts/2d.md#2dphysicsraycast)
and [architecture audit](../../AYDocs/AYPHYSICS-ARCHITECTURE-REVIEW-20261022.md)
for the current query surface and remaining work.

The 3D capsule controller also accepts one collision-swept world-space
displacement per fixed step through `PhysicsWorld3D::setCharacterStepDisplacement`.
It can move along the current up direction, is consumed once without lasting
velocity, and publishes the actual traveled feet position after the step.
See [controller behavior and fixed-step usage](docs/character-controller-3d.md).

Box2D and Jolt raycasts return the body and exact generation-checked collider.
For 3D sight queries, `IPhysicsQuery::raycastSyncIgnoringBody` excludes one
generation-checked body and returns the next eligible Jolt hit. Backends without
the operation return `Unsupported`; the ordinary `raycastSync` keeps its behavior.
The legacy 3D ray and sphere-overlap queries now reject invalid/non-finite
geometry before mailbox submission. Sync failures clear their hit/body output;
the Host query facade also clears output when it is unbound or stopped, and
`isReady()` reports false after shutdown. The three legacy 3D async ray,
sphere-overlap and oriented-box-overlap entries return query ID zero on invalid
geometry or a stopped manager; accepted results still arrive in a later frame.
The original 2D `raycastSync`, `overlapSphereSync` and `overlapBoxSync` clear
caller output on errors or misses. They reject non-finite geometry and
non-positive overlap size before submitting to the physics worker, and return
`InvalidState` promptly after shutdown. Valid legacy overlap queries keep
their historical `Ok`/empty result with Null 2D.
The corresponding legacy async queries reject the same invalid geometry and
return query ID zero if the manager is stopped or the command cannot enqueue.
Valid async results remain available through a later `PhysFrameSnapshot`.
The shared synchronous query mailbox retains a slot while a timed-out request
is still executing on the physics worker. Its late response is discarded and
the slot returns to the pool only after backend execution finishes, so a new
query cannot receive a prior request's result.
2D and 3D body, collider and joint creation now require a running manager.
`NoMemory` and `QueueFull` leave the output handle invalid and preserve the
next handle index and generation. Repeated rejected requests therefore cannot
exhaust the handle range before a creation is accepted.
`PhysicsWorld2D::raycastAllSync` casts a finite XY segment and returns every
eligible exact collider hit in distance order. It supports layer masks,
ignored bodies and optional sensors; its explicit hit limit rejects overflow
without exposing a partial list. Null 2D reports `Unsupported`.
The 2D world also offers a finite XY circle sweep through the existing
`SphereSweep` request: `PhysicsWorld2D::sphereSweepSync` returns the closest
body/collider, contact point and normal, and circle-center travel distance.
It supports layer masks, ignored bodies, optional sensors and initial overlap;
the Null 2D backend reports `Unsupported`.
`PhysicsWorld2D::boxSweepSync` casts an axis-aligned XY rectangle with
independent half-extents through the same worker mailbox. It reports the
closest exact collider and center travel, with the same filters and clear-on-
failure behavior as circle sweep; Null 2D reports `Unsupported`.
`PhysicsWorld2D::boxSweepAllSync` returns every eligible exact collider along
that rectangle's path, sorted by center travel. A 1..1024 hit limit reports
`OutOfRange` without exposing a partial list; Null 2D is `Unsupported`.
The corresponding `sphereSweepAllSync`, `capsuleSweepAllSync` and
`orientedBoxSweepAllSync` queries provide the same sorted, bounded multi-hit
results for circles, upright capsules and fixed-angle rotated rectangles.
`PhysicsWorld2D::capsuleSweepSync` casts an upright XY capsule with total height
including its round ends. It reports the closest collider and center travel,
supports the same filters and failure behavior, and accepts a circular capsule
when height equals diameter. Null 2D reports `Unsupported`.
`PhysicsWorld2D::overlapCapsuleSync` checks an upright capsule at one center
and returns sorted unique body handles. It supports layer masks, ignored bodies
and optional sensors; a valid empty result differs from Null `Unsupported`.
`PhysicsWorld2D::overlapOrientedBoxSync` checks a rotated XY rectangle with the
same filters and sorted unique body results. Its angle is counterclockwise in
radians, and it leaves the simulation unchanged; Null 2D returns `Unsupported`.
`overlapCapsuleCollidersSync` and `overlapOrientedBoxCollidersSync` return exact
body/collider pairs instead of only unique bodies. Equal capsule height and
diameter queries a circle; zero box angle queries an axis-aligned rectangle.
Both use sorted, bounded results and return `OutOfRange` without partial data.
`PhysicsWorld2D::orientedBoxSweepSync` translates a rotated rectangle while
holding its angle fixed. It returns the first exact collider, contact geometry
and center travel distance, with the same filters as the other 2D sweeps;
Null 2D returns `Unsupported`.
Jolt now preserves multiple colliders as a compound, removes their geometry on
destruction, honors ray layer masks and returns real surface normals. Body-only
overlaps do not identify an individual shape.

**2026-10-23 spatial gravity:** `GravityField3D` samples prioritized uniform,
box and radial sources; `submitGravity3D` queues mass-independent acceleration
for selected dynamic bodies. `FlightControl3D` supplies local six-axis
thrust/torque and a runnable zero-gravity flight example. The Jolt capsule
controller accepts per-character gravity and world-space tangent movement,
then publishes its current up direction. See the
[spatial gravity guide](docs/spatial-gravity-3d.md).

**2026-10-03 current addition:** the formal `PhysicsWorld3D` controller API owns
upright capsule movement, slopes, stairs, ground snapping, one-shot jumps and
moving-platform support on the physics thread. See the
[controller guide](docs/character-controller-3d.md) for acquisition, examples,
lifecycle, failure behavior and limits. The older phase table below is historical.

Connected-region adapters can also call `PhysicsWorld3D::setRigidbodyRoute`
for an existing body's layer/mask and `transferCharacter` for one queued
feet/velocity/up/filter transition. The latter preserves mapped momentum for
the first destination step; see the [controller guide](docs/character-controller-3d.md).

- **Authoritative design:** [`design.md`](design.md) (v0.3, 2026-07-30) — see §17 for perf / Jolt gate
- **AI / dev rules:** [`CLAUDE.md`](CLAUDE.md)
- **3D backend:** Jolt (locked, real impl via vcpkg `jolt-physics` 5.5.0); see design §4.1
- **2D backend:** Box2D 3.x flat-API (locked, R2.5 parallel); see design §4.2 (legacy Box2D 2.4 OO style forbidden)

扭矩回归通过 `stepAndWait` 逐步完成固定步，核对每步角速度增长、快照包含刚体，
以及转过整圈前的明确转角；不以多圈旋转后的包裹朝向判断是否发生旋转。

---

## Status

| Phase | Scope | Status |
|-------|-------|--------|
| R0 | design.md + CLAUDE.md + README.md + .gitignore | ✅ |
| R0.1 | Compact command, generation handles, sparse snapshot, §17 perf gate | ✅ |
| R1 | Null/Mock + compact SPSC + create pool + Manager + handle/snapshot tests (111/111) | ✅ commit `8cabf84` |
| R1.5a | `JoltBackend3D` stub + vcpkg three-tier fallback (126/126) | ✅ commit `54c2f7c` |
| R1.5b | Real `JoltBackend3D` (Box/Sphere/Capsule + Hinge/Fixed/Distance + ContactListener + layer filters + JobSystem) — 172/172 | ✅ commit `8de0529` |
| R1.5c | `Bench_PhysicsStep` (P1/P2/P3 + sleeping%) + §17.8 gate close | ⏳ next |
| R2 | 3D ConvexHull/Mesh/Heightfield + ConeTwist/Point joints + BodyActivationListener (ragdoll unlock) | ⏳ |
| R2.5 | `Box2DBackend2D` real impl + tilemap↔physics bridge (parallel to R2) | ⏳ |
| R3 | `PhysicsSubSystem` + GameLoop integration | ⏳ |
| R4 | `RigidbodyComponent` / ECS bridge | ⏳ |
| R5 | `.physscene` JSON + `AYResource` bridge | ⏳ |
| R6 | Determinism gate (DET-07 stub) | ⏳ |
| R7 | Effects (Cloth / Fluid / Particle) | ⏳ |
| R8 | Editor hooks | 🅿 deferred |

✅ shipped · ⏳ planned · 🅿 deferred

**Current capability (R1.5b):** Static/Dynamic/Kinematic bodies, Box/Sphere/Capsule colliders, Hinge/Fixed/Distance joints, ContactListener enter/stay/exit, sync+async raycast + sphere overlap, gravity + impulse + apply-force. **R2 unlocks:** ragdoll (ConvexHull + ConeTwist), terrain (Heightfield), Mesh triangles.

---

## Quick start (R1, Null backend)

```cpp
#include "AYPhysics.h"

using namespace ayt::physics;

PhysicsBackendDescriptor desc;
desc.kind3D = BackendKind::Null;           // no Jolt needed for this run
desc.kind2D = BackendKind::Null;
desc.fixedDeltaTime = 1.0f / 60.0f;

auto manager = PhysicsManager::create(desc);

PhysicsWorld3D* world = manager->world3D();

RigidbodyDesc rb;
rb.type = BodyType::Dynamic;
rb.position = FVector3(0, 10, 0);
BodyHandle body = InvalidBodyHandle;
world->createRigidbody(rb, body);

// Fixed-phase path: returns only after both backends advanced and the new
// snapshot was published. step() remains available for explicit async use.
manager->stepAndWait(1.0f / 60.0f);

const PhysFrameSnapshot& snap = manager->fetchResults();
// iterate snap.transforms for matching handle (sparse list; Null mode: unchanged)

manager->shutdown();
```

---

## Tests

```bash
cmake --build build --target AYPhysics_Tests -j
./build/AYRuntime/AYPhysics/unittest/AYPhysics_Tests.exe
```

Expected (R1): types / handles / command queue / snapshot / manager / null / mock suites green.
R1.5 adds Jolt tests + `Bench_PhysicsStep` (§17.2).

If `vcpkg jolt-physics` is installed, Jolt-backed test variant is built automatically. Otherwise the Null-mode tests run only.

---

## Engine integration

| Module | Relationship |
|--------|--------------|
| **AYCore** | Required: smart pointers, threading primitives |
| **AYMath** | Required: `FVector3`, `FQuaternion`, `FMatrix4x4` |
| **AYGameLoop** | Required: `ISubSystem`, frame deltaTime |
| **AYEventSystem** | E-3: collision events via typed `EventBus` (`PhysicsEventBridge`) |
| **AYEntity** | R4+: `RigidbodyComponent`, `ColliderComponent` |
| **AYResource** | R5+: `.physscene` asset loading |
| **AYRenderer** | Optional: debug-draw line/shape submission |
| **AYAnimation** | R7+: cloth / ragdoll / IK drives physics state |
| **Jolt** | Optional: `vcpkg jolt-physics`; absent → Null mode |

`PhysicsSubSystem` runs in `FixedPhysics` and uses `stepAndWait()`. The game
thread therefore enters `FixedPostPhysics` only after the matching snapshot is
published; a queue error, timeout, or shutdown fails the phase and the pending
simulation tick is not committed.

Default Client / Server / Editor hosts install this subsystem through
`PhysicsRuntimeModule` (`AYPhysics.Runtime`) and pass the backend descriptor to
the module constructor. `PhysicsSubSystem::registerSubSystem()` remains the
compatibility path for standalone demos and focused tests.

---

## Directory (target)

See [`design.md` §15](design.md#15-directory-layout-target) for the full target tree.

Current public layout: `interface/AYPhysics/`, `include/AYPhysics/`, `backend/{Null,Mock,Jolt}Backend3D.{h,cpp}`, `src/`, `unittest/`.

---

## Related engine docs

- [`ENGINE-DETERMINISM-ARCHITECTURE.md`](../../ENGINE-DETERMINISM-ARCHITECTURE.md) — Physics-A vs Physics-B dual paths
- [`AYEntity/design.md §14`](../AYEntity/design.md) — `SystemLane::Sim`
- [`AYGameLoop/design.md`](../AYGameLoop/design.md) — SubSystem priority
- [`AYAudio/design.md`](../AYAudio/design.md) — SPSC command pattern + backend abstraction reference
- [`AYAudio/CMakeLists.txt`](../AYAudio/CMakeLists.txt) — vcpkg find_package pattern reference

---

## Changelog

| Date | Change |
|------|--------|
| 2026-07-20 | **R0** — industrial-grade design rewrite: 16-chapter structure (Goals/Non-Goals/Anti-Goals, decisions, threading, backend abstraction, handles, determinism, effects-as-contracts); companion CLAUDE.md + README.md |
| 2026-07-09 | Original dual-path summary (now design.md §10) |
| Earlier | Initial rigidbody / cloth / fluid / particle sketch |

## Debug visualization (2026-10-15)

Configure the manager-owned PhysicsDebugDraw on the game thread. Synchronous
and asynchronous 2D/3D queries retain their real result, dimension, request ID,
mask, hit bodies, point, normal and distance. Green is miss, red hit, yellow
failure. Async completions are consumed by fetchResults or fetchDebugSnapshot;
use one game-thread owner. Timed lifetime starts at backend completion.

Frame, Timed and Persistent history share a 1024-record / 16384-line budget;
new records are rejected instead of evicting retained history. Freeze suspends
capture and expiry; unfreeze shifts timed deadlines. consumeRecords and
consumeLines are alternative single-consumer APIs. recordGeometry accepts
explicit adapter observations; diagnostic failure never changes simulation.

setDebugSnapshotEnabled opts into physics-thread native inspection, with a
category mask. fetchDebugSnapshot returns an owned, mutex-protected copy;
ordinary sparse simulation snapshots keep their existing cadence. Native Jolt
shape triangles include convex hulls, meshes, heightfields, offsets, sleeping
and static non-ECS bodies. Box2D uses its native circle/capsule/polygon/segment
shapes. Unsupported or over-budget geometry is counted rather than replaced
with an AABB. Character capsules, ground state/normal, snap/step ranges,
requested/actual velocity, joint anchors/axes and available limits are included.
Contacts retain enter/stay/exit markers; Box2D exposes solved normal impulse.
Jolt solved contact impulse is unavailable through the current integration and
is explicitly flagged unavailable. No fabricated force value is displayed.

3D oriented box async queries now transfer a full request through the existing
out-of-band pool (command remains <=64 bytes). Jolt sphere/box and Box2D
circle/box overlap use narrowphase geometry and the requested layer mask.
Deterministic physics instrumentation remains outside this native diagnostic path.
See PhysicsDebugDrawTests, PhysicsDebugSnapshotTests, JoltColliderHitTests and
[SDK contract](../../AYDocs/sdk/contracts/common.md#commonphysicsdebug-draw).

Verification: full audited inventory 146/146 passed, including real Jolt shape/
controller/joint snapshots, oriented async boxes, narrowphase filtering and
Box2D solved impulses. Final report:
`out/debug-visualization-20261015/physics-final/a6118164d5314dbb8d3190c1a1137268/cases.json`.

## Debug content extension (2026-10-15)

PhysicsDebugCategory appends Animation/Combat/Lighting/Audio/Particle/Brain at bits9..14 without renumbering existing categories. Owned PhysicsDebugRecord details support numeric/state inspection. Generic history/snapshot budgets and clocks stay shared; native physics inspection still runs on its worker.

Module-local regression sources were updated. Runtime tests and interactive editor acceptance are deferred by the current task request; previous validation reports cover the earlier baseline. These changes remain uncommitted until focused runtime checks are requested and pass.

## Extended diagnostic observations (2026-10-15)

Diagnostic category IDs now extend through Resource (24), with kPhysicsDebugCategoryCount and kPhysicsDebugAllCategories. PhysicsDebugRecord can own logical screenLines and independent transientEvent identity metadata; PhysicsDebugDraw and PhysicsDebugSnapshot count world+screen segments together and reject invalid category IDs/nonfinite geometry. consumeRecords preserves both coordinate spaces; consumeLines is the alternative world-only consumer. Engine service inspection remains in AYEditor, with no renderer/service dependency added to the physics worker.

Regression sources are included. Runtime tests and interactive editor acceptance are deferred for this content task; untested changes remain uncommitted.

## Camera / spatial hierarchy integration (2026-10-14)

Camera controllers bind persistent Entity identity with a World-scoped runtime handle; explicit missing targets cannot rebind by name. Presentation control samples world-space ancestor poses and converts once back to local position/orientation through the same alpha, preserving local scale and refusing singular/sheared parents.

Entity.PhysicsIntegration installs the default Jolt sphere-sweep adapter. Host custom queries override it; collisionEnabled explicitly disables per controller. The finite-volume query ignores sensors/self/target bodies, checks generation handles and reports initial penetration at zero center travel. Null/Mock/2D return Unsupported. Orbit distance plus collision radius/padding implement the camera arm and damped recovery. Explicit manager adapters must be detached before manager destruction.

Editor has stable target selection, collision parameters, and undoable main-window hierarchy dragging. CameraRigProject is a portable visual/authoring fixture; its Wall is visual-only, while AYEntity_CameraRigIntegrationTests exercises actual Jolt wall avoidance, lens effects/blending, self-ignore and recovery. See ../../AYDocs/CAMERA-SYSTEM.md and the integration evidence in ../../AYDocs/ENTITY-SPATIAL-HIERARCHY.md. Native Computer Use acceptance is deferred by user request.

## Spatial gravity source channels

Spatial gravity sources now support channel masks. `GravityField3D::sample`
filters matching receiver channels before source priority selection; default
masks preserve existing callers. Scene-authored Spatial gravity is integrated
by AYEntity; see `docs/spatial-gravity-3d.md`.
