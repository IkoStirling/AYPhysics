#include "AYPhysics/PhysicsCommandQueue.h"
#include "AYPhysics/IPhysicsBackend.h"

#include "AYTime/Duration.h"

#include <bit>                // std::countr_zero (C++20) for F-H/I bit iteration
#include <cassert>
#include <chrono>             // microsecond timeouts on std::condition_variable
                               // (interops via ayt::time::Duration::toChrono)
#include <condition_variable>
#include <cstring>
#include <mutex>

namespace ayt::physics {

// =============================================================================
// PhysicsCommandQueue
// =============================================================================

namespace {
uint32_t nextPow2(uint32_t v) {
    if (v < 2) return 2;
    v--;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    return v + 1;
}
}  // namespace

bool PhysicsCommandQueue::initialize(uint32_t capacity) {
    if (capacity == 0) return false;
    _capacity = nextPow2(capacity);
    _mask     = _capacity - 1;
    _buffer.assign(_capacity, PhysicsCommand{});
    _writeIndex.store(0, std::memory_order_relaxed);
    _readIndex.store(0,  std::memory_order_relaxed);
    return true;
}

void PhysicsCommandQueue::shutdown() {
    _buffer.clear();
    _capacity = 0;
    _mask     = 0;
    _writeIndex.store(0, std::memory_order_relaxed);
    _readIndex.store(0,  std::memory_order_relaxed);
}

bool PhysicsCommandQueue::tryPush(const PhysicsCommand& cmd) {
    if (_capacity == 0) return false;
    const uint64_t write = _writeIndex.load(std::memory_order_relaxed);
    const uint64_t read  = _readIndex.load(std::memory_order_acquire);
    if (write - read >= _capacity) return false;
    _buffer[write & _mask] = cmd;
    _writeIndex.store(write + 1, std::memory_order_release);
    return true;
}

bool PhysicsCommandQueue::tryPop(PhysicsCommand& outCmd) {
    if (_capacity == 0) return false;
    const uint64_t read  = _readIndex.load(std::memory_order_relaxed);
    const uint64_t write = _writeIndex.load(std::memory_order_acquire);
    if (read == write) return false;
    outCmd = _buffer[read & _mask];
    _readIndex.store(read + 1, std::memory_order_release);
    return true;
}

uint32_t PhysicsCommandQueue::approximateDepth() const noexcept {
    const uint64_t write = _writeIndex.load(std::memory_order_relaxed);
    const uint64_t read  = _readIndex.load(std::memory_order_relaxed);
    return static_cast<uint32_t>(write - read);
}

// =============================================================================
// PhysicsCreatePool
// =============================================================================

bool PhysicsCreatePool::initialize(uint32_t capacity) {
    if (capacity == 0) return false;
    std::lock_guard<std::mutex> lock(_mutex);
    _capacity = capacity;
    _slots = std::unique_ptr<Slot[]>(new Slot[capacity]);
    _freeList.clear();
    _freeList.reserve(capacity);
    for (uint32_t i = 0; i < capacity; ++i) {
        _freeList.push_back(i);
    }
    _inFlight = 0;
    return true;
}

void PhysicsCreatePool::shutdown() {
    std::lock_guard<std::mutex> lock(_mutex);
    _slots.reset();
    _freeList.clear();
    _capacity = 0;
    _inFlight = 0;
}

CreateSlotId PhysicsCreatePool::allocate(const PhysicsCreatePayload& payload) {
    std::lock_guard<std::mutex> lock(_mutex);
    if (_freeList.empty()) return InvalidCreateSlotId;
    const uint32_t idx = _freeList.back();
    _freeList.pop_back();
    Slot& slot = _slots[idx];
    slot.payload = payload;
    slot.busy = true;
    ++_inFlight;
    // Slot id is (idx + 1) so 0 stays reserved for invalid.
    return idx + 1;
}

bool PhysicsCreatePool::take(CreateSlotId slotId, PhysicsCreatePayload& out) {
    if (slotId == InvalidCreateSlotId || slotId > _capacity) return false;
    const uint32_t idx = slotId - 1;
    std::lock_guard<std::mutex> lock(_mutex);
    if (idx >= _capacity) return false;
    Slot& slot = _slots[idx];
    if (!slot.busy) return false;
    out = slot.payload;
    slot.busy = false;
    _freeList.push_back(idx);
    --_inFlight;
    return true;
}

uint32_t PhysicsCreatePool::inFlightCount() const noexcept {
    std::lock_guard<std::mutex> lock(_mutex);
    return _inFlight;
}

// =============================================================================
// SyncQueryMailbox
// =============================================================================
//
// Single-producer (game) / single-consumer (physics) mailbox. Each slot has its
// own ready-flag; the game thread busy-waits or condvar-waits on the ready flag.
// Capacity is small (default 64) so spin is acceptable; we use condvar to be
// friendly to scheduler.

struct SyncQueryMailboxInternal {
    std::mutex              mu;
    std::condition_variable cv;
    bool                    wake = false;
};

bool SyncQueryMailbox::initialize(uint32_t capacity) {
    if (capacity == 0) return false;
    _capacity = nextPow2(capacity);
    _mask     = _capacity - 1;
    // F-H/I — pending-mask requires capacity <= 64 (one bit per slot).
    // Callers (PhysicsManager) only pass kSyncQueryMailboxCapacity = 64; clamp
    // anything larger to 64 rather than silently widening the mask to __int128.
    if (_capacity > 64u) _capacity = 64u;
    _mask     = _capacity - 1;
    _slots    = std::unique_ptr<PendingSlot[]>(new PendingSlot[_capacity]);
    _freeList.clear();
    _freeList.reserve(_capacity);
    for (uint32_t i = 0; i < _capacity; ++i) _freeList.push_back(i);
    _pendingMask.store(0, std::memory_order_relaxed);
    if (!_cv) {
        auto* internal = new SyncQueryMailboxInternal{};
        _cv = internal;
    }
    return true;
}

void SyncQueryMailbox::shutdown() {
    if (_cv) {
        delete static_cast<SyncQueryMailboxInternal*>(_cv);
        _cv = nullptr;
    }
    _mutex = nullptr;
    _slots.reset();
    _freeList.clear();
    _capacity = 0;
    _mask = 0;
    _pendingMask.store(0, std::memory_order_relaxed);
}

bool SyncQueryMailbox::submitAndWait(const SyncQueryRequest& request,
                                     SyncQueryResponse& outResponse,
                                     int64_t timeoutMicroseconds) {
    if (_capacity == 0) return false;
    auto* internal = static_cast<SyncQueryMailboxInternal*>(_cv);

    // Allocate a slot AND mark it pending atomically. The mask bit is the
    // single source of truth for "physics thread should service this slot";
    // the free list is only consulted by the game thread for slot allocation.
    uint32_t slotIdx;
    {
        std::lock_guard<std::mutex> lk(internal->mu);
        if (_freeList.empty()) return false;
        slotIdx = _freeList.back();
        _freeList.pop_back();
    }
    PendingSlot& slot = _slots[slotIdx];
    slot.request = request;
    slot.response = {};
    slot.ready.store(false, std::memory_order_release);
    _pendingMask.fetch_or(uint64_t(1) << slotIdx, std::memory_order_release);

    // Wait for slot.ready to become true.
    //
    // We pass the timeout through ayt::time::Duration (AYTime v1.2 value-type,
    // 0 alloc, constexpr) and convert at the cv.wait_for boundary via
    // toChronoMicroseconds(). This keeps the wait duration expressed in
    // engine units while std::condition_variable still receives the chrono
    // duration it requires. Mirrors AYGameLoop's queue wait.
    auto waitUntil = [&]() {
        std::unique_lock<std::mutex> lk(internal->mu);
        const auto timeout = ayt::time::Duration::fromUs(timeoutMicroseconds)
                                 .toChronoMicroseconds();
        internal->cv.wait_for(lk, timeout,
                              [&]() { return slot.ready.load(std::memory_order_acquire); });
    };
    auto releaseSlot = [&]() {
        // F-H/I — timed-out / error path. Clear the pending bit so serviceAll
        // doesn't service a stale request once the response finally arrives,
        // and push the slot back to the free list.
        _pendingMask.fetch_and(~(uint64_t(1) << slotIdx), std::memory_order_release);
        std::lock_guard<std::mutex> lk(internal->mu);
        _freeList.push_back(slotIdx);
    };
    if (timeoutMicroseconds < 0) {
        std::unique_lock<std::mutex> lk(internal->mu);
        internal->cv.wait(lk, [&]() { return slot.ready.load(std::memory_order_acquire); });
    } else if (timeoutMicroseconds == 0) {
        if (!slot.ready.load(std::memory_order_acquire)) {
            releaseSlot();
            return false;
        }
    } else {
        waitUntil();
        if (!slot.ready.load(std::memory_order_acquire)) {
            releaseSlot();
            return false;
        }
    }

    outResponse = slot.response;
    // Release the slot back to the pool + clear the pending bit.
    _pendingMask.fetch_and(~(uint64_t(1) << slotIdx), std::memory_order_release);
    {
        std::lock_guard<std::mutex> lk(internal->mu);
        _freeList.push_back(slotIdx);
    }
    return true;
}

void SyncQueryMailbox::serviceAll(IPhysicsBackend* backend) {
    if (!backend || _capacity == 0) return;
    // F-H/I — iterate ONLY slots whose pending-mask bit is set. Old code did
    // a linear scan over the freeList for every slot (O(N^2) for cap=64);
    // popcount of the mask is O(popcount) and bit iteration is O(active).
    uint64_t pending = _pendingMask.load(std::memory_order_acquire);
    while (pending != 0) {
        // C++20 std::countr_zero is portable across MSVC + GCC/Clang; non-zero
        // invariant (we checked pending != 0 above) keeps the precondition safe.
        const uint32_t slotIdx = static_cast<uint32_t>(std::countr_zero(pending));
        pending &= pending - 1;  // clear lowest set bit

        PendingSlot& slot = _slots[slotIdx];
        // Defensive: if the slot already became ready (game thread reaped
        // the response while we were running) the pending bit should have
        // been cleared already. Skip — but DON'T clear the bit again.
        if (slot.ready.load(std::memory_order_acquire)) continue;

        SyncQueryResponse resp;
        resp.requestId = slot.request.requestId;
        backend->executeSync(slot.request, resp);
        slot.response = resp;
        slot.ready.store(true, std::memory_order_release);

        auto* internal = static_cast<SyncQueryMailboxInternal*>(_cv);
        {
            std::lock_guard<std::mutex> lk(internal->mu);
            internal->cv.notify_all();
        }
    }
}

} // namespace ayt::physics