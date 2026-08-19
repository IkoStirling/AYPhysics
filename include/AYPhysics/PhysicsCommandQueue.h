#pragma once
// AYPhysics/PhysicsCommandQueue.h - compact SPSC command queue + create pool (§5.2, §17.1)
//
// Hard contracts:
//   sizeof(PhysicsCommand) <= 64 (static_asserted).
//   Create payloads live in PhysicsCreatePool addressed by CreateSlotId;
//     ring only carries slot id + allocated handle — never full descriptors.
//   Power-of-2 capacity; min 2; max 65536.
//   Cross-thread: game thread = tryPush; physics thread = tryPop.
//
// Test-only inspection lives in AYPhysics/PhysicsBackendTestAccess.h.

#include "AYPhysics/PhysicsTypes.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

namespace ayt::physics {

class IPhysicsBackend;  // forward decl for SyncQueryMailbox::serviceAll

namespace _syncquery_internal {
struct ServiceScratch;
}

enum class PhysicsCommandType : uint8_t {
    Step                  = 0,
    CreateRigidbody       = 1,
    DestroyRigidbody      = 2,
    SetRigidbodyTransform = 3,
    CreateCollider        = 4,
    DestroyCollider       = 5,
    SetColliderShape      = 6,
    CreateJoint           = 7,
    DestroyJoint          = 8,
    ApplyForce            = 9,
    ApplyImpulse          = 10,
    RaycastAsync          = 11,
    OverlapSphereAsync    = 12,
    OverlapBoxAsync       = 13,
    WakeAll               = 14,
    SleepAll              = 15,
    SetRigidbodyCollideMask = 16,
    SetRigidbodyVelocity  = 17,  // R2: runtime linear-velocity set (b2Body_SetLinearVelocity)
    SetGravityScale       = 18,  // R6: per-body gravity multiplier (b2Body_SetGravityScale)
    ApplyTorque           = 19,  // R6: Z-axis torque (b2Body_ApplyTorque)
    ApplyAngularImpulse   = 20,  // R6: Z-axis angular impulse (b2Body_ApplyAngularImpulse)
    SetMass               = 21,  // R10: runtime mass override (vec4.x = mass)
    SetMaterial           = 22,  // R10: runtime friction/restitution override
                                  //      (vec4.x = friction, vec4.y = restitution; collider-scoped)
};

using CreateSlotId = uint32_t;
constexpr CreateSlotId InvalidCreateSlotId = 0;

// Out-of-band create payload (§5.2). Game allocates, physics consumes + frees slot.
struct PhysicsCreatePayload {
    enum class Kind : uint8_t { Rigidbody = 0, Collider = 1, Joint = 2 } kind = Kind::Rigidbody;
    RigidbodyDesc rigidDesc{};
    ColliderDesc  colliderDesc{};
    JointDesc     jointDesc{};
};

// Compact PhysicsCommand (§5.2). No descriptor copies; create uses CreateSlotId.
struct PhysicsCommand {
    PhysicsCommandType type       = PhysicsCommandType::Step;
    BodyHandle         body       = InvalidBodyHandle;
    ColliderHandle     collider   = InvalidColliderHandle;
    JointHandle        joint      = InvalidJointHandle;
    CreateSlotId       createSlot = InvalidCreateSlotId;
    uint32_t           queryId    = 0;
    uint32_t           layerMask  = 0xFFFFFFFFu;

    union {
        struct {
            float deltaTime;
            uint64_t completionSequence;
        } step;
        struct { float x, y, z, w; } vec4;            // force / impulse / halfExtents / etc.
        struct { float px, py, pz, qx, qy, qz, qw; } xform;
        struct { float ox, oy, oz, dx, dy, dz; } ray;
        struct { float cx, cy, cz, radius; } sphere;
        struct { float cx, cy, cz, hx, hy, hz; } box;  // R1: overlap box center + half-extents (AABB, no rotation)
    } u{};
};

static_assert(sizeof(PhysicsCommand) <= 64,
              "PhysicsCommand must stay cache-friendly (§17.1)");

// Power-of-2 SPSC ring.
class PhysicsCommandQueue {
public:
    PhysicsCommandQueue() = default;
    ~PhysicsCommandQueue() { shutdown(); }

    bool initialize(uint32_t capacity);   // rounds up to power of two; min 2
    void shutdown();

    // Game thread enqueue. Returns false on full (caller decides backpressure).
    bool tryPush(const PhysicsCommand& cmd);

    // Physics thread dequeue. Returns false on empty.
    bool tryPop(PhysicsCommand& outCmd);

    uint32_t approximateDepth() const noexcept;
    uint32_t capacity() const noexcept { return _capacity; }
    bool isInitialized() const noexcept { return _capacity != 0; }

private:
    std::vector<PhysicsCommand> _buffer;
    uint32_t _capacity = 0;
    uint32_t _mask     = 0;
    std::atomic<uint64_t> _writeIndex{0};
    std::atomic<uint64_t> _readIndex{0};
};

// Out-of-band create pool. Game allocates a slot, takes a CreateSlotId back,
// pushes the slot id through the ring; physics thread takes + executes + frees.
// Capacity is the upper bound on in-flight creates (mirrors descriptor setting).
//
// allocate() (game) and take() (physics) both mutate _freeList — they are NOT
// single-owner. A mutex serializes freelist + slot busy/payload handoff; the
// previous unsynchronized vector pop/push raced and could double-issue a slot
// so the second take() saw busy=false and dropped the create payload.
class PhysicsCreatePool {
public:
    PhysicsCreatePool() = default;
    ~PhysicsCreatePool() { shutdown(); }

    bool initialize(uint32_t capacity);
    void shutdown();

    // Game thread: copy payload into free slot; returns slot id (>= 1) or 0 on full.
    CreateSlotId allocate(const PhysicsCreatePayload& payload);

    // Physics thread: take ownership; slot returned to free list after execute.
    bool take(CreateSlotId slotId, PhysicsCreatePayload& out);

    uint32_t capacity() const noexcept { return _capacity; }
    uint32_t inFlightCount() const noexcept;

private:
    struct Slot {
        bool                 busy = false;
        PhysicsCreatePayload payload{};
        Slot() = default;
        Slot(const Slot&) = delete;
        Slot& operator=(const Slot&) = delete;
        Slot(Slot&&) = delete;
        Slot& operator=(Slot&&) = delete;
    };

    mutable std::mutex _mutex;
    std::unique_ptr<Slot[]> _slots;
    std::vector<uint32_t> _freeList;     // indices; guarded by _mutex
    uint32_t _capacity = 0;
    uint32_t _inFlight = 0;              // guarded by _mutex
};

// =============================================================================
// Sync-query mailbox (§5.4)
// =============================================================================
//
// SPSC queue: game thread submits request + waits on its own cv; physics thread
// services before/after step. Capacity is small (default 64 per frame).

class SyncQueryMailbox {
public:
    SyncQueryMailbox() = default;
    ~SyncQueryMailbox() { shutdown(); }

    bool initialize(uint32_t capacity);
    void shutdown();

    // Game thread: blocks until response or shutdown.
    // timeoutMicroseconds < 0 = wait forever. 0 = non-blocking.
    bool submitAndWait(const SyncQueryRequest& request,
                       SyncQueryResponse& outResponse,
                       int64_t timeoutMicroseconds);

    // Physics thread: drain + execute + return responses.
    // Implementation-defined: backend's executeSync may use its own scratch.
    void serviceAll(IPhysicsBackend* backend);

private:
    struct PendingSlot {
        std::atomic<bool> ready{false};
        SyncQueryRequest request{};
        SyncQueryResponse response{};
        PendingSlot() = default;
        PendingSlot(const PendingSlot&) = delete;
        PendingSlot& operator=(const PendingSlot&) = delete;
        PendingSlot(PendingSlot&&) = delete;
        PendingSlot& operator=(PendingSlot&&) = delete;
    };

    std::unique_ptr<PendingSlot[]> _slots;
    std::vector<uint32_t>    _freeList;     // pool indices
    // F-H/I — bit i = slot i is currently in flight (game thread claimed it
    // from _freeList, hasn't received its response yet). Lets serviceAll skip
    // the per-slot linear _freeList scan and run in O(popcount) instead of
    // O(N^2). Capacity is capped at 64 (kSyncQueryMailboxCapacity), so a
    // single uint64_t covers every slot.
    std::atomic<uint64_t>    _pendingMask{0};
    uint32_t _capacity = 0;
    uint32_t _mask     = 0;
    // cv/mutex per slot via a single condvar + wake count.
    void* _cv = nullptr;                   // std::condition_variable* (opaque)
    void* _mutex = nullptr;                // std::mutex* (opaque)
};

} // namespace ayt::physics
