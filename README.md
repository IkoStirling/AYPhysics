# AYPhysics

AY Engine physics subsystem: rigidbody dynamics, collision detection, constraint joints, with **2D and 3D as fully separated worlds**. Backend-pluggable (Jolt for 3D, Box2D for 2D).

- **Authoritative design:** [`design.md`](design.md) (v0.3, 2026-07-30) — see §17 for perf / Jolt gate
- **AI / dev rules:** [`CLAUDE.md`](CLAUDE.md)
- **3D backend:** Jolt (locked, real impl via vcpkg `jolt-physics` 5.5.0); see design §4.1
- **2D backend:** Box2D 3.x flat-API (locked, R2.5 parallel); see design §4.2 (legacy Box2D 2.4 OO style forbidden)

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

manager->step(1.0f / 60.0f);

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

## Engine integration (planned)

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

---

## Directory (target)

See [`design.md` §15](design.md#15-directory-layout-target) for the full target tree.

R1 ships with: `interface/`, `include/`, `backend/{Null,Mock,Jolt}Backend3D.{h,cpp}`, `src/`, `unittest/`.

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