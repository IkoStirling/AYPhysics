# AYPhysics 项目 AI 工作注意事项

> **注意**：AYPhysics 是独立子模块，遵循本文件定义的规则。AYTest 是独立测试框架库，位于 `AYTest/CLAUDE.md`。
> **权威设计**：[`design.md`](design.md)（v0.3, 2026-07-30）。代码与 design.md 不一致时，design.md 优先。

## 重要规则

1. **生成代码时不要使用 GBK 中文注释**
   - 代码中不要包含 GBK 编码的中文注释（会导致编译错误）
   - 尽量不要有中文，仅在用户明确要求添加中文注释时才添加
   - 如需中文注释，确保是 UTF-8 编码

2. **使用 AYTest 框架进行单元测试**
   - AYPhysics 使用 AYTest 框架（位于 `D:/Projects/AYTest/`）
   - 测试文件 `#include "AYTest.h"`，使用 `TEST_SUITE` / `TEST_CASE` / `CHECK_*` 等宏
   - 测试文件放在 `AYPhysics/unittest/` 目录下
   - 每个 `Test_*.cpp` 是一个独立 TU（避免 MSVC include-order 问题，参见 AYAudio 范本）

3. **可主动构建 + 跑测试**
   - 写完代码后可主动运行 `cmake` / `msbuild` / `ninja` 编译并跑测试
   - 不必等用户同意

4. **缩短思考时间，及时结束会话**
   - 完成代码修改并确认逻辑后立即结束
   - 在回复结尾明确提醒用户自行运行验证

5. **新增任何 backend / shape / effect / 模块 → 同步新增 unittest**
   - 所有新增模块都需对应 unittest
   - 新增 backend 必须在 `unittest/Test_<BackendName>.cpp` 加命令捕获 / 状态断言测试

## 架构决策（v1）

### Backend 选型
- **3D 后端 = Jolt**（locked，见 design.md §4.1），通过 vcpkg `jolt-physics` 集成（vcpkg 端口名上游叫 `jolt-physics`，但 CMake config 导出名为 **`Jolt`** / target **`Jolt::Jolt`**——不要写 `find_package(joltphysics ...)` 或 `joltphysics::joltphysics`，那些名字不存在）；若 vcpkg 未装，自动降级为 Null 模式（`AYPHYSICS_NO_JOLT=1`）
- **2D 后端 = Box2D**（locked 2026-07-30，见 design.md §4.2），R2.5 与 R2 并行开发
- **Box2D API 版本 = 3.x flat-API**（vcpkg `box2d:x64-windows` 当前装的版本）。**禁止**使用老版 Box2D 2.4 OO 风格：`b2World*` / `w->CreateBody()` / `b2Vec2(x,y)` 工厂函数 / `new`/`delete` 都是禁止的。R2.5 实现必须用 3.x：`b2CreateWorld` + `b2BodyId` / `b2ShapeId` / `b2JointId` 句柄（无 `*`）+ `b2Vec2{x,y}` aggregate init（POD）+ `b2World_Step(w, dt, subStepCount)` + `b2DestroyBody`/`b2DestroyWorld` 按 id 释放。详细对照表见 design.md §4.2。
- **2D / 3D 完全分离**：`IPhysicsBackend3D` / `IPhysicsBackend2D` 各自接口、各自 handle 空间（见 design.md §1.1 / §6）

### 命令通道 / 性能硬规则（design §5 / §17）
- **Game → Physics**：compact `PhysicsCommandQueue`（SPSC, pow2）。**禁止**在每个 ring slot 嵌入完整 `RigidbodyDesc`/`ColliderDesc`/`JointDesc`
- **`sizeof(PhysicsCommand) <= 64`**（`static_assert`）；创建描述走 `PhysicsCreatePool` + `CreateSlotId`
- **Physics → Game**：双缓冲 **稀疏** `PhysFrameSnapshot`（仅 Awake ∪ AlwaysSync；禁止按 handle 稠密下标）
- **查询双路径**：`*Async`（下帧 snapshot）+ `*Sync`（mailbox 同帧阻塞）；禁止 game thread 碰 `JPH::*`
- **Drain / 反压**：`maxDrainPerTick` 默认 4096；满队列返回 `QueueFull`，创建路径禁止静默丢弃
- Jolt 仅 `backend/JoltBackend3D.cpp`；**公开头禁止** `#include <Jolt/Jolt.h>`
- **R1.5 合并门禁**：design.md §17.8 checklist 必须在 PR 勾选；未达标不得合入真 Jolt 后端

### 命名空间 / 类型
- 命名空间：`ayt::physics`
- 公开类名**不带** `AY` 前缀（`class PhysicsManager`、`class Rigidbody`、`class Cloth`）
- 文件名带 `AY` 前缀（`AYPhysicsManager.h`、`AYPhysicsCloth.h`）
- 接口文件用 `I` 前缀（`IPhysicsBackend.h`）
- 私有成员：`_` 前缀 camelCase（`_system`、`_bodies`）
- 公共成员：camelCase（`gravity`、`mass`）
- 句柄：`using BodyHandle = uint32_t`，**packed index(20)+generation(12)**，`Invalid*Handle = 0`（见 design §7.1）
- 错误返回：所有公开 mutator 返回 `PhysResult`（`enum class`），**禁止** `void` 写失败路径

### 时间单位 / AYTime v1.2
- **所有时长 / 时间点都走 `ayt::time::Duration` / `ayt::time::TimePoint`**（v1.2 value-type，0 alloc，header-only constexpr）。**禁止**写 `IDuration*` / `ITimePoint*` / `IClock::*`（v1.2 已全删）。
- `std::condition_variable::wait_for` / `Thread::sleep` / `Clock::performanceNowUs` 等需要 chrono 边界的接口，通过 `Duration::fromUs(...).toChronoMicroseconds()` 转换；**禁止**直接写 `std::chrono::microseconds(us)`（那是 v1.0 时代的兼容写法，AYTime v1.2 已 ship 后没必要）。
- 引擎时间语义（game time / paused）走 `ayt::time::Clock::*`（`gameNow` / `setGameTime` / `addGameDuration` / `pauseGame` / `resumeGame` / `isGamePaused`），全部 0 alloc。

### 确定性硬规则
- `PhysicsManager::step()` 入口检测 lockstep；若激活，**禁止**调用 Jolt，返回 `PhysResult::Unsupported`
- R1 stub：`bool ayt::physics::isLockstepActive() noexcept` 返回 `false`
- R3 联调 `ayt::game::LockstepSession::isActive()`

### 资源 / 序列化
- R1：手写 `PhysSceneDesc` 结构体 + 程序内构造；**不**做 JSON / 序列化
- R2：`.physscene` JSON（nlohmann/json，仿 AYLayoutLoader）
- R3：cooked binary via `AYSerializer`

### 特效（Cloth / Fluid / Particle）
- **R1 不实现**。仅在 `include/` 留 `.h` 桩（`AYPhysicsCloth.h` 等），**不**创建空 `.cpp`
- 完整实装在 F-1/F-2/F-3（R3+）

## 文件命名规范

| 类型 | 命名 | 示例 |
|---|---|---|
| 公开头 | `AY` 前缀 | `AYPhysicsManager.h`、`AYPhysicsCommandQueue.h` |
| 接口头 | `I` 前缀 | `IPhysicsBackend.h`、`IPhysicsBackend3D.h` |
| Backend 头 | `AY` 前缀 + 后缀 | `NullBackend3D.h`、`JoltBackend3D.h` |
| 公开类 | **不带**前缀 | `class PhysicsManager`、`class Rigidbody` |
| 公开接口类 | `I` 前缀 | `class IPhysicsBackend` |
| 命名空间 | `ayt::physics` | |
| 私有成员 | `_` 前缀 camelCase | `_world3D`、`_queue` |
| 私有函数 | `_` 前缀 camelCase | `_stepInternal()` |

## 目录结构（R1 实际）

```
AYPhysics/
├── design.md
├── CLAUDE.md
├── README.md
├── .gitignore
├── CMakeLists.txt
├── AYPhysics.h                          # umbrella include
├── interface/
│   ├── IPhysicsBackend.h
│   ├── IPhysicsBackend3D.h
│   ├── IPhysicsBackend2D.h              # R1 占位
│   └── PhysicsScene.h
├── include/
│   ├── AYPhysicsTypes.h                 # handles / PhysResult / descriptors
│   ├── AYPhysicsManager.h
│   ├── AYPhysicsWorld3D.h
│   ├── AYPhysicsWorld2D.h               # R1 占位
│   ├── AYPhysicsCommandQueue.h
│   ├── AYPhysicsSubSystem.h
│   ├── AYPhysicsBackendTestAccess.h     # 测试专用
│   ├── AYPhysicsCloth.h                 # R3+ 占位
│   ├── AYPhysicsFluid.h                 # R3+ 占位
│   └── AYPhysicsParticle.h              # R3+ 占位
├── backend/
│   ├── NullBackend3D.{h,cpp}            # 永远存在
│   ├── MockBackend3D.{h,cpp}            # 永远存在（测试）
│   └── JoltBackend3D.{h,cpp}            # 条件编译（vcpkg Jolt 可选）
├── src/
│   ├── AYPhysicsManager.cpp
│   ├── AYPhysicsWorld3D.cpp
│   ├── AYPhysicsWorld2D.cpp
│   ├── AYPhysicsCommandQueue.cpp
│   ├── AYPhysicsSubSystem.cpp
│   └── Physics/
│       ├── Rigidbody.cpp
│       ├── Collider.cpp
│       ├── Joint.cpp
│       └── PhysicsScene.cpp
└── unittest/
    ├── CMakeLists.txt
    ├── main.cpp
    ├── Test_PhysicsTypes.cpp
    ├── Test_PhysicsCommandQueue.cpp
    ├── Test_PhysicsManager.cpp
    ├── Test_NullBackend3D.cpp
    └── Test_MockBackend3D.cpp
```

## 验证方式

```bash
# 配置（root build）
cmake -B build -DAVPHYSICS_BUILD=ON

# 编译 AYPhysics 模块（Null 模式，不依赖 Jolt）
cmake --build build --target AYPhysics -j

# 编译并跑测试
cmake --build build --target AYPhysics_Tests -j
./build/AYRuntime/AYPhysics/unittest/AYPhysics_Tests.exe

# 若 vcpkg 已装 jolt-physics：
cmake -B build -DAVPHYSICS_BUILD=ON -DCMAKE_TOOLCHAIN_FILE=$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake
```

预期：
- Null 模式下编译 0 错 0 警
- 5 个 TEST_SUITE 全部通过
- Jolt 缺失时 `JoltBackend3D.cpp` 自动排除编译

## 新增模块要求

### 新增 backend
1. 在 `backend/` 创建 `XxxBackend3D.h` / `.cpp`（或 `XxxBackend2D`）
2. 实现 `IPhysicsBackend3D`（或 `IPhysicsBackend2D`）接口
3. 在 `CMakeLists.txt` `BACKEND_FILES` / `BACKEND_HEADERS` 中加入（条件编译用 `#ifdef` 包裹）
4. 在 `unittest/` 加 `Test_XxxBackend3D.cpp`
5. 在 `CONTROL_CAPABILITY_PLAN.md` 不适用——直接在 `design.md` §3 Phase roadmap 加 step

### 新增 collider shape
1. 在 `include/AYPhysicsTypes.h` 的 `enum class ColliderShape` 加值
2. 在 `RigidbodyDesc` / `ColliderDesc` 加对应字段
3. 在 `backend/<Xxx>Backend3D.cpp` 处理新 shape
4. 在 `unittest/` 加测试

### 新增 joint type
1. 在 `include/AYPhysicsTypes.h` 的 `enum class JointType` 加值
2. 在 `JointDesc` 加类型特定参数
3. 在 backend 实现
4. 在 `unittest/` 加测试

### 新增 effect（Cloth / Fluid / Particle）
- 仅当进入对应 Phase（F-1/F-2/F-3）才动 `include/AYPhysicsCloth.h` 等占位头
- 占位期保持 .h 纯声明、**不**创建 .cpp

## 反模式（禁止）

| 反模式 | 替代 |
|---|---|
| `Rigidbody* createRigidbody(...)` 返回裸指针 | `PhysResult createRigidbody(..., BodyHandle& out)` |
| `void createRigidbody(...)` 失败不告知 | 失败返回 `QueueFull` / `NoMemory`，`out = Invalid` |
| 每个 `PhysicsCommand` 塞满三个 Desc | compact ≤64 B + `PhysicsCreatePool` |
| snapshot 按 handle 稠密数组下标 | 稀疏 Awake∪alwaysSync 列表 |
| 公开头 `#include <Jolt/Jolt.h>` | 仅 `backend/JoltBackend3D.cpp` 单 TU include |
| 后端类型在公开 API 中具名（`JoltPhysicsWorld`） | 仅 `IPhysicsBackend*` 抽象 |
| 在多线程共享状态不通过 SPSC | 一切走 `PhysicsCommandQueue` |
| Lockstep 激活时仍调 Jolt | step() 检测 `isLockstepActive()` → `PhysResult::Unsupported` |
| `enum class` 默认从 1 开始 | 0 = `Ok`，错误码递增 |

## 参考文档

- [design.md](design.md) — **当前权威设计**（v0.2, 2026-07-20；§17 性能门禁）
- [ENGINE-DETERMINISM-ARCHITECTURE.md](../../ENGINE-DETERMINISM-ARCHITECTURE.md) — Physics-A/B 双路径
- [AYEntity/design.md §14](../AYEntity/design.md) — SystemLane
- [AYAudio/CMakeLists.txt](../AYAudio/CMakeLists.txt) — vcpkg 范本
- [AYAudio/design.md §4.4](../AYAudio/design.md) — SPSC 命令通道范本
- [AYAudio/interface/IAudioBackend.h](../AYAudio/interface/IAudioBackend.h) — 后端抽象范本