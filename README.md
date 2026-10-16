# AYPhysics

AY Engine physics subsystem: rigidbody dynamics, collision detection, constraint joints, with **2D and 3D as fully separated worlds**. Backend-pluggable (Jolt for 3D, Box2D for 2D).

Box2D and Jolt raycasts return the body and exact generation-checked collider.
Jolt now preserves multiple colliders as a compound, removes their geometry on
destruction, honors ray layer masks and returns real surface normals. Body-only
overlaps do not identify an individual shape.

**2026-10-03 current addition:** the formal `PhysicsWorld3D` controller API owns
upright capsule movement, slopes, stairs, ground snapping, one-shot jumps and
moving-platform support on the physics thread. See the
[controller guide](docs/character-controller-3d.md) for acquisition, examples,
lifecycle, failure behavior and limits. The older phase table below is historical.

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
