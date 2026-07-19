# AYPhysics Design

> **变更记录（2026-07-09）**：Jolt 与确定性 Sim 物理双路径 — 见 [§8.3](#83-确定性与双物理路径)；总览见 [`ENGINE-DETERMINISM-ARCHITECTURE.md`](../../ENGINE-DETERMINISM-ARCHITECTURE.md)。

## 1. 概述

AYPhysics 是 AY Engine 的**物理子系统**，负责：
- 刚体动力学与碰撞检测
- 关节约束系统
- 2D/3D 物理混合场景
- 特效系统（布料、毛发、流体、粒子）

### 1.1 设计目标

- **Jolt 后端**：现代 C++ 物理引擎，轻量高性能
- **2D/3D 混合**：2D 角色在 3D 世界中
- **特效自研**：布料/毛发/流体/粒子不依赖物理引擎
- **独立模块**：物理模块独立于 ECS
- **双路径**：Jolt 负责丰富动力学（**非 lockstep**）；确定性碰撞子集（DET-07）供 Sim 轨 — 见 [§8.3](#83-确定性与双物理路径)

### 1.2 在引擎中的位置

```
┌─────────────────────────────────────────────────────────────────┐
│                        Game Engine                               │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌─────────────┐    ┌──────────────────────────────────────┐  │
│  │  AYGame    │───▶│           AYPhysics                 │  │
│  │  Logic     │    │                                      │  │
│  └─────────────┘    │  ┌────────────────────────────────┐   │  │
│                      │  │      PhysicsManager           │   │  │
│                      │  └──────────────┬───────────────┘   │  │
│                      │                 │                     │  │
│                      │  ┌──────────────▼───────────────┐   │  │
│                      │  │     PhysicsWorld3D (Jolt)   │   │  │
│                      │  │  刚体/碰撞体/关节/检测      │   │  │
│                      │  └──────────────┬───────────────┘   │  │
│                      │                 │                     │  │
│                      │  ┌──────────────▼───────────────┐   │  │
│                      │  │     PhysicsWorld2D         │   │  │
│                      │  │  2D 物理 + 坐标转换       │   │  │
│                      │  └─────────────────────────────┘   │  │
│                      │                                    │  │
│                      │  ┌─────────────────────────────┐   │  │
│                      │  │     特效系统 (自研)        │   │  │
│                      │  │  AYCloth / AYFluid / AYParticle │ │  │
│                      │  └─────────────────────────────┘   │  │
│                      └──────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. Jolt 后端

### 2.1 选择理由

| 特性 | Jolt | PhysX |
|-------|------|-------|
| 库大小 | ~5MB | ~50MB |
| API 风格 | 现代 C++ | C++/C |
| 多线程 | 原生优化 | 复杂调度器 |
| 功能覆盖 | 刚体/碰撞/关节/角色/载具 | 全部 + 布料/流体 |
| 许可证 | Zlib (宽松) | NVIDIA EULA |

### 2.2 支持的功能

```
✅ 刚体动力学
✅ 休眠系统 (Sleeping)
✅ 约束/关节
   ├── Hinge (铰链)
   ├── Fixed (固定)
   ├── Distance (距离)
   ├── Spring (弹簧)
   ├── Slider (滑动)
   ├── Point (点)
   └── Cone (锥形)
✅ 碰撞检测
   ├── Broadphase (SAP/AABB)
   └── Narrowphase (GJK/EPA)
✅ 碰撞形状
   ├── Box
   ├── Sphere
   ├── Capsule
   ├── Convex Hull
   ├── Triangle Mesh
   └── Heightfield
✅ Character Controller
✅ Ragdoll
✅ Vehicle
✅ Ray/Shape Cast
✅ Volume Queries (Overlap)
❌ 布料/毛发（自研 AYCloth）
❌ 流体（自研 AYFluid）
❌ 粒子（自研 AYParticle）
```

---

## 3. 核心架构

### 3.1 架构设计

```
┌─────────────────────────────────────────────────────────────┐
│                    AYPhysics                               │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  PhysicsManager (管理器)                                    │
│      │                                                        │
│      ├── PhysicsWorld3D (Jolt)                             │
│      │   ├── 刚体管理                                       │
│      │   ├── 碰撞体管理                                     │
│      │   ├── 关节管理                                       │
│      │   └── 碰撞检测/响应                                  │
│      │                                                        │
│      └── PhysicsWorld2D                                     │
│          ├── 刚体管理                                       │
│          ├── 碰撞体管理                                     │
│          ├── 关节管理                                       │
│          └── 2D/3D 坐标转换                               │
│                                                              │
│  特效系统                                                  │
│      ├── AYCloth (布料/毛发 - Verlet 积分)                 │
│      ├── AYFluid (流体 - SPH)                             │
│      └── AYParticle (GPU 粒子)                             │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

### 3.2 2D/3D 混合场景

```
典型用例：
├── 3D 地图/环境
├── 2D 俯视角角色 (ARPG、塔防)
└── 2D UI/特效在屏幕空间

坐标转换：
2D World ←→ 3D World
pos2D → pos3D (x, y, z=0)
pos3D → pos2D (x, y)
```

---

## 4. 核心接口

### 4.1 物理管理器

```cpp
class PhysicsManager {
public:
    static PhysicsManager& instance();

    // 世界获取
    PhysicsWorld3D* getWorld3D() { return m_world3D.get(); }
    PhysicsWorld2D* getWorld2D() { return m_world2D.get(); }

    // 2D/3D 坐标转换
    FVector3 world2DTo3D(const FVector2& pos2D) const {
        return FVector3(pos2D.x, pos2D.y, 0.0f);
    }

    FVector2 world3DTo2D(const FVector3& pos3D) const {
        return FVector2(pos3D.x, pos3D.y);
    }

    // 时间步
    void step(float deltaTime);

private:
    std::unique_ptr<PhysicsWorld3D> m_world3D;
    std::unique_ptr<PhysicsWorld2D> m_world2D;
};
```

### 4.2 物理世界基类

```cpp
class IPhysicsWorld {
public:
    virtual ~IPhysicsWorld() = default;

    // 时间步
    virtual void step(float deltaTime) = 0;

    // 碰撞检测
    virtual bool raycast(const Ray& ray, RaycastHit& hit) = 0;
    virtual void overlapSphere(const FVector3& center, float radius,
                               std::vector<Collider*>& results) = 0;
    virtual void overlapBox(const FVector3& center, const FVector3& halfExtents,
                           std::vector<Collider*>& results) = 0;

    // 调试绘制
    virtual void debugDraw(DebugRenderer* renderer) = 0;

    // 休眠管理
    virtual void wakeAll() = 0;
    virtual void sleepAll() = 0;
};
```

### 4.3 3D 物理世界

```cpp
class PhysicsWorld3D : public IPhysicsWorld {
public:
    PhysicsWorld3D();
    ~PhysicsWorld3D();

    // ============== 世界设置 ==============
    void setGravity(const FVector3& gravity);
    void setSubSteps(int numSubSteps);  // 物理子步数
    void setLinearDamping(float damping);
    void setAngularDamping(float damping);

    // ============== 刚体 ==============
    Rigidbody3D* createRigidbody(const RigidbodyDesc& desc);
    void destroyRigidbody(Rigidbody3D* body);

    // 批量操作
    void addRigidbody(Rigidbody3D* body);
    void removeRigidbody(Rigidbody3D* body);

    // ============== 碰撞体 ==============
    Collider3D* createCollider(const ColliderDesc& desc);
    void destroyCollider(Collider3D* collider);

    // ============== 关节 ==============
    Joint3D* createJoint(const JointDesc& desc);
    void destroyJoint(Joint3D* joint);

    // ============== 物理查询 ==============
    void step(float deltaTime) override;
    bool raycast(const Ray& ray, RaycastHit& hit) override;
    void overlapSphere(const FVector3& center, float radius,
                       std::vector<Collider*>& results) override;
    void overlapBox(const FVector3& center, const FVector3& halfExtents,
                    std::vector<Collider*>& results) override;

    // ============== 调试 ==============
    void debugDraw(DebugRenderer* renderer) override;
    void wakeAll() override;
    void sleepAll() override;

private:
    JPH::PhysicsSystem m_system;  // Jolt 物理系统
    JPH::BodyManager m_bodyManager;
    JPH::ConstraintManager m_constraintManager;
};
```

### 4.4 2D 物理世界

```cpp
class PhysicsWorld2D : public IPhysicsWorld {
public:
    PhysicsWorld2D();
    ~PhysicsWorld2D();

    // ============== 世界设置 ==============
    void setGravity(const FVector2& gravity);

    // ============== 刚体 ==============
    Rigidbody2D* createRigidbody(const RigidbodyDesc& desc);
    void destroyRigidbody(Rigidbody2D* body);

    // ============== 碰撞体 ==============
    Collider2D* createCollider(const ColliderDesc& desc);
    void destroyCollider(Collider2D* collider);

    // ============== 关节 ==============
    Joint2D* createJoint(const JointDesc& desc);
    void destroyJoint(Joint2D* joint);

    // ============== 物理查询 ==============
    void step(float deltaTime) override;
    bool raycast(const Ray& ray, RaycastHit& hit) override;
    void overlapSphere(const FVector3& center, float radius,
                       std::vector<Collider*>& results) override;
    void overlapBox(const FVector3& center, const FVector3& halfExtents,
                    std::vector<Collider*>& results) override;

    // ============== 调试 ==============
    void debugDraw(DebugRenderer* renderer) override;
    void wakeAll() override;
    void sleepAll() override;

private:
    // 可以用 Jolt 2D 模式或 Box2D
    // 这里用 Jolt 的 2D 兼容模式
};
```

---

## 5. 刚体与碰撞体

### 5.1 刚体描述

```cpp
enum class BodyType { Static, Dynamic, Kinematic };

enum class MotionMode { Translatable, Rotatable, Full };

struct RigidbodyDesc {
    BodyType type = BodyType::Dynamic;
    MotionMode motionMode = MotionMode::Full;

    FVector3 position;
    FQuaternion rotation;

    FVector3 linearVelocity;
    FVector3 angularVelocity;

    float mass = 1.0f;
    float friction = 0.5f;
    float restitution = 0.0f;
    float linearDamping = 0.01f;
    float angularDamping = 0.01f;

    uint32_t collisionGroup = 0;
    uint32_t collisionMask = 0xFFFFFFFF;
};
```

### 5.2 碰撞体描述

```cpp
// 3D 碰撞体类型
enum class ColliderType3D {
    Box,
    Sphere,
    Capsule,
    ConvexHull,
    TriangleMesh,
    Heightfield
};

// 碰撞体基类
class Collider3D {
public:
    virtual ~Collider3D() = default;

    ColliderType3D getType() const { return m_type; }

    // 变换
    void setPosition(const FVector3& pos);
    void setRotation(const FQuaternion& rot);

    // 材质
    void setFriction(float friction);
    void setRestitution(float restitution);

protected:
    ColliderType3D m_type;
    Collider3D(ColliderType3D type) : m_type(type) {}
};

// Box 碰撞体
class BoxCollider3D : public Collider3D {
public:
    BoxCollider3D(const FVector3& halfExtents);

    FVector3 getHalfExtents() const { return m_halfExtents; }

private:
    FVector3 m_halfExtents;
};

// Sphere 碰撞体
class SphereCollider3D : public Collider3D {
public:
    SphereCollider3D(float radius);

    float getRadius() const { return m_radius; }

private:
    float m_radius;
};

// Capsule 碰撞体
class CapsuleCollider3D : public Collider3D {
public:
    CapsuleCollider3D(float height, float radius);

    float getHeight() const { return m_height; }
    float getRadius() const { return m_radius; }

private:
    float m_height;
    float m_radius;
};
```

---

## 6. 关节

### 6.1 关节类型

```cpp
enum class JointType {
    Hinge,    // 铰链
    Fixed,     // 固定
    Distance,  // 距离
    Spring,    // 弹簧
    Slider,    // 滑动
    Point,     // 点约束
    Cone       // 锥形
};

struct JointDesc {
    JointType type;

    Rigidbody3D* bodyA;
    Rigidbody3D* bodyB;

    FVector3 anchorA;  // 本地锚点
    FVector3 anchorB;

    // 类型特定参数
    float minDistance = 0.0f;    // Distance
    float maxDistance = 0.0f;    // Distance
    float stiffness = 0.0f;      // Spring
    float damping = 0.0f;        // Spring
    FVector3 axisA;               // Hinge/Slider
    FVector3 axisB;
};
```

---

## 7. 特效系统（自研）

### 7.1 AYCloth - 布料/毛发

**使用 Verlet 积分**，适合布料和毛发模拟。

```cpp
class AYCloth {
public:
    struct Particle {
        FVector3 position;        // 当前位置
        FVector3 prevPosition;    // 上一步位置 (Verlet)
        FVector3 acceleration;     // 加速度
        float invMass;            // 质量倒数

        void integrate(float deltaTime) {
            FVector3 velocity = position - prevPosition;
            prevPosition = position;
            position = position + velocity + acceleration * deltaTime * deltaTime;
            acceleration = FVector3::zero();
        }
    };

    struct DistanceConstraint {
        int p1, p2;
        float restLength;
        float stiffness = 1.0f;

        void satisfy(std::vector<Particle>& particles) {
            FVector3 diff = particles[p2].position - particles[p1].position;
            float dist = diff.length();
            if (dist < 0.0001f) return;

            FVector3 correction = diff * ((dist - restLength) / dist) * 0.5f * stiffness;
            particles[p1].position += correction;
            particles[p2].position -= correction;
        }
    };

    // ============== 模拟 ==============
    void simulate(float deltaTime);

    void addForce(const FVector3& force) {
        for (auto& p : m_particles) {
            p.acceleration += force * p.invMass;
        }
    }

    void setWind(const FVector3& wind) { m_wind = wind; }

    // ============== 形状 ==============
    void setAsGrid(int rows, int cols, float spacing);
    void addConstraint(int p1, int p2, float stiffness = 1.0f);

    // ============== 绑定 ==============
    void pinParticle(int index, const FVector3& position);

private:
    std::vector<Particle> m_particles;
    std::vector<DistanceConstraint> m_constraints;
    FVector3 m_wind;
    int m_pinnedCount = 0;
};
```

### 7.2 AYFluid - 流体 (SPH)

**使用 SPH (Smoothed Particle Hydrodynamics)**。

```cpp
class AYFluid {
public:
    struct FluidParticle {
        FVector3 position;
        FVector3 velocity;
        FVector3 force;
        float density;
        float pressure;
        float mass;
    };

    // ============== 参数 ==============
    void setParticleRadius(float radius) { m_particleRadius = radius; }
    void setRestDensity(float density) { m_restDensity = density; }
    void setViscosity(float viscosity) { m_viscosity = viscosity; }
    void setGasStiffness(float stiffness) { m_gasStiffness = stiffness; }

    // ============== 模拟 ==============
    void simulate(float deltaTime);

    // 添加粒子
    void emit(const FVector3& position, int count);

    // 空间哈希加速邻居查询
    void buildSpatialHash();

private:
    // SPH 参数
    float m_particleRadius = 0.1f;
    float m_restDensity = 1000.0f;
    float m_viscosity = 0.01f;
    float m_gasStiffness = 2000.0f;
    float m_kernelRadius = 0.2f;

    std::vector<FluidParticle> m_particles;

    // 空间哈希 (加速邻居查询)
    SpatialHash m_spatialHash;
};
```

### 7.3 AYParticle - GPU 粒子

```cpp
class AYParticle {
public:
    struct GPUParticle {
        FVector4 position;   // xyz + lifetime
        FVector4 velocity;   // xyz + scale
        FVector4 color;     // rgba
    };

    struct Emitter {
        std::string name;
        FVector3 position;
        FVector3 emitRate;      // 每秒发射数量
        float emitProbability;   // 发射概率

        std::vector<FVector4> initialVelocity;  // 初始速度范围
        std::vector<FVector2> lifetimeRange;   // 生命周期范围
        std::vector<FVector2> scaleRange;       // 缩放范围
    };

    // ============== 发射器 ==============
    void addEmitter(Emitter emitter);
    void removeEmitter(const std::string& name);

    // ============== 更新 ==============
    void emit(float deltaTime);
    void update(float deltaTime);
    void render(RenderContext* ctx);

    // ============== GPU 缓冲区 ==============
    GPUBuffer<GPUParticle>* getBuffer() { return &m_buffer; }

private:
    std::vector<Emitter> m_emitters;
    GPUBuffer<GPUParticle> m_buffer;
    int m_maxParticles = 100000;
    int m_emitAccumulator = 0;
};
```

---

## 8. 与其他模块的接口

### 8.1 与 AYAnimation

```
AYAnimation 输出骨骼变换
        ↓
AYPhysics 接收骨骼位置
        ↓
设置刚体位置/动画物理混合
```

### 8.2 与 AYRenderer

```
AYPhysics 调试绘制
        ↓
DebugRenderer 输出线条/形状
        ↓
AYRenderer 渲染
```

### 8.3 确定性与双物理路径

> **权威文档**：[`ENGINE-DETERMINISM-ARCHITECTURE.md`](../../ENGINE-DETERMINISM-ARCHITECTURE.md) §6  
> **ECS 分轨**：[`AYEntity/design.md`](../AYEntity/design.md) §14 (`SystemLane`)

本模块默认后端为 **Jolt**。Jolt 适合服务器权威、视觉物理、复杂刚体与关节；**不能**作为跨平台 lockstep / 输入回放的仿真后端（求解器顺序、浮点、多线程与架构差异会导致分歧）。

#### 两条路径

| 路径 | 后端 | `SystemLane` | 典型用途 |
|------|------|--------------|----------|
| **Physics-A（默认）** | Jolt `PhysicsWorld3D` | **Present** 或**仅服务器**权威 | 开放世界、布娃娃、载具、布料/毛发（视觉）、MMD 裙摆等 |
| **Physics-B（Sim）** | 自研确定性子集（DET-07） | **Sim** | 帧同步、rollback、bit-exact 回放 |

```
┌─────────────────────────────────────────────────────────┐
│  Present / Server-authoritative                         │
│  PhysicsWorld3D (Jolt) → float Transform / 网络复制      │
└─────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────┐
│  Sim (lockstep active)                                  │
│  DetBroadphase + Fixed AABB/sweep — 不调用 Jolt          │
│  → SimTransformComponent → Bridge → float Transform      │
└─────────────────────────────────────────────────────────┘
```

#### 硬性规则

1. **`LockstepSession` 激活时**，Sim 轨 **禁止** 调用 `PhysicsWorld3D` / Jolt API。
2. Jolt 结果若需参与多人玩法，走 **服务器权威 + 状态复制**（`AYNetwork`），而非各端本地 Jolt lockstep。
3. `RigidBodyComponent` 与 Jolt 刚体同步属于 **Physics-A**；玩法位移/碰撞判定在 lockstep 产品中应优先使用 **Physics-B** 或 Sim 逻辑，避免「视觉刚体」与「仿真状态」混用同一 float 变换。
4. 与 `AYAnimation` 的混合（§8.1）仅限 **表现**（动画驱动刚体位置）；lockstep 命中判定使用 [`AYEntity` §14.5](../AYEntity/design.md#145-组件命名约定未来) 的 Sim 代理，不读蒙皮矩阵。

#### 实现优先级（确定性）

| ID | 内容 | 依赖 |
|----|------|------|
| DET-07 | `DetBroadphase` + Fixed AABB 解算 MVP | DET-01 (`AYMath::Fixed`) |
| — | Jolt 集成 | 不依赖 DET；与 lockstep 正交 |

DET-07 为**产品触发**（见 [`ENGINE-DETERMINISM-ARCHITECTURE.md`](../../ENGINE-DETERMINISM-ARCHITECTURE.md) §9），不阻塞 Jolt Phase 1 与 `ENGINE-FOUNDATION-PLAN` Phase 0–2。

---

## 9. 目录结构

```
AYPhysics/
├── design.md
├── CMakeLists.txt
├── include/
│   └── AYPhysics/
│       ├── AYPhysics.h              # 主入口
│       │
│       ├── Core/
│       │   ├── PhysicsManager.h       # 管理器
│       │   ├── PhysicsWorld3D.h      # 3D 物理世界
│       │   ├── PhysicsWorld2D.h      # 2D 物理世界
│       │   ├── Rigidbody.h          # 刚体
│       │   └── Collider.h           # 碰撞体
│       │
│       ├── Joints/
│       │   ├── Joint.h
│       │   ├── HingeJoint.h
│       │   ├── FixedJoint.h
│       │   ├── DistanceJoint.h
│       │   └── SpringJoint.h
│       │
│       ├── Colliders/
│       │   ├── BoxCollider.h
│       │   ├── SphereCollider.h
│       │   ├── CapsuleCollider.h
│       │   ├── ConvexCollider.h
│       │   └── MeshCollider.h
│       │
│       └── Effects/
│           ├── AYCloth.h           # 布料/毛发
│           ├── AYFluid.h           # 流体
│           └── AYParticle.h         # 粒子
│
└── src/
    ├── AYPhysics.cpp
    ├── PhysicsManager.cpp
    ├── PhysicsWorld3D.cpp
    ├── PhysicsWorld2D.cpp
    ├── Rigidbody.cpp
    ├── Collider.cpp
    ├── Joint.cpp
    ├── Effects/
    │   ├── AYCloth.cpp
    │   ├── AYFluid.cpp
    │   └── AYParticle.cpp
    └── Backend/
        └── JoltBackend.h   # Jolt 后端封装
```

---

## 10. 实现优先级

### Phase 1: 核心
- [ ] PhysicsManager
- [ ] PhysicsWorld3D (Jolt)
- [ ] Rigidbody (Box/Sphere/Capsule)
- [ ] 基础碰撞检测

### Phase 2: 完整碰撞
- [ ] Convex/Concave Mesh
- [ ] Heightfield
- [ ] Raycast/Overlap 查询

### Phase 3: 关节
- [ ] Hinge Joint
- [ ] Fixed Joint
- [ ] Distance/Spring Joint
- [ ] Character Controller

### Phase 4: 2D + 特效
- [ ] PhysicsWorld2D
- [ ] 2D/3D 坐标转换
- [ ] AYCloth
- [ ] AYFluid
- [ ] AYParticle

### Phase 5: 高级
- [ ] Ragdoll
- [ ] Vehicle
- [ ] 优化/调试工具

### Phase 6: 确定性碰撞（按需，DET-07）
- [ ] `DetBroadphase` + Fixed AABB / sweep（**不**使用 Jolt）
- [ ] 与 `AYEntity` `SystemLane::Sim` 集成

> 与 Jolt Phase 1–5 **并行可选**；仅 lockstep / 回放产品需要时启动。

---

## 11. 参考

- [Engine determinism architecture](../../ENGINE-DETERMINISM-ARCHITECTURE.md) — Physics-A vs Physics-B, DET-07
- [AYEntity SystemLane](../AYEntity/design.md#14-simulation-vs-presentation-systemlane)
- [Jolt Physics](https://jrouwe.github.io/JoltPhysics/)
- [SPH Fluid Simulation](http://mmacklin.com/sphfluid.pdf)
- [Verlet Integration](https://en.wikipedia.org/wiki/Verlet_integration)
- NVIDIA Flex / HairWorks / TressFX