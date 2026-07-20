#include "AYPhysicsCommandQueue.h"
#include "IPhysicsBackend.h"

#include "aytime/Duration.h"

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
    _capacity = capacity;
    _slots = std::unique_ptr<Slot[]>(new Slot[capacity]);
    _freeList.clear();
    _freeList.reserve(capacity);
    for (uint32_t i = 0; i < capacity; ++i) {
        _freeList.push_back(i);
    }
    _inFlight.store(0, std::memory_order_relaxed);
    return true;
}

void PhysicsCreatePool::shutdown() {
    _slots.reset();
    _freeList.clear();
    _capacity = 0;
    _inFlight.store(0, std::memory_order_relaxed);
}

CreateSlotId PhysicsCreatePool::allocate(const PhysicsCreatePayload& payload) {
    if (_freeList.empty()) return InvalidCreateSlotId;
    // Single-producer (game thread) use; no CAS needed.
    const uint32_t idx = _freeList.back();
    _freeList.pop_back();
    Slot& slot = _slots[idx];
    slot.payload = payload;
    slot.busy.store(true, std::memory_order_release);
    _inFlight.fetch_add(1, std::memory_order_relaxed);
    // Slot id is (idx + 1) so 0 stays reserved for invalid.
    return idx + 1;
}

bool PhysicsCreatePool::take(CreateSlotId slotId, PhysicsCreatePayload& out) {
    if (slotId == InvalidCreateSlotId || slotId > _capacity) return false;
    const uint32_t idx = slotId - 1;
    Slot& slot = _slots[idx];
    if (!slot.busy.load(std::memory_order_acquire)) return false;
    out = slot.payload;
    slot.busy.store(false, std::memory_order_release);
    _freeList.push_back(idx);
    _inFlight.fetch_sub(1, std::memory_order_relaxed);
    return true;
}

uint32_t PhysicsCreatePool::inFlightCount() const noexcept {
    return _inFlight.load(std::memory_order_relaxed);
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
    _slots    = std::unique_ptr<PendingSlot[]>(new PendingSlot[_capacity]);
    _freeList.clear();
    _freeList.reserve(_capacity);
    for (uint32_t i = 0; i < _capacity; ++i) _freeList.push_back(i);
    _writeIdx.store(0, std::memory_order_relaxed);
    _readIdx.store(0,  std::memory_order_relaxed);
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
    _writeIdx.store(0, std::memory_order_relaxed);
    _readIdx.store(0,  std::memory_order_relaxed);
}

bool SyncQueryMailbox::submitAndWait(const SyncQueryRequest& request,
                                     SyncQueryResponse& outResponse,
                                     int64_t timeoutMicroseconds) {
    if (_capacity == 0) return false;
    auto* internal = static_cast<SyncQueryMailboxInternal*>(_cv);

    // Allocate a slot.
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

    // Publish slot index to physics thread.
    const uint64_t write = _writeIdx.fetch_add(1, std::memory_order_acq_rel);
    // Store slot idx in mask bits of write index is not feasible — use a side-band
    // queue? For R1, single-slot-in-flight is supported by serializing at the
    // game side (PhysicsWorld3D::raycastSync acquires a per-world mutex). For the
    // simple mailbox, we just bump the index and let physics iterate all slots
    // checking ready==true. But we still need to know which slot index to wait
    // on. We store the slotIdx in a private map keyed by an in-flight counter.

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
    if (timeoutMicroseconds < 0) {
        std::unique_lock<std::mutex> lk(internal->mu);
        internal->cv.wait(lk, [&]() { return slot.ready.load(std::memory_order_acquire); });
    } else if (timeoutMicroseconds == 0) {
        if (!slot.ready.load(std::memory_order_acquire)) return false;
    } else {
        waitUntil();
    }

    if (!slot.ready.load(std::memory_order_acquire)) return false;
    outResponse = slot.response;
    {
        std::lock_guard<std::mutex> lk(internal->mu);
        _freeList.push_back(slotIdx);
    }
    (void)write;
    return true;
}

void SyncQueryMailbox::serviceAll(IPhysicsBackend* backend) {
    if (!backend || _capacity == 0) return;
    // Linear scan over slots; mark-ready ones get executed and response written.
    // We rely on slot.request.requestId being set by game thread before submission.
    for (uint32_t i = 0; i < _capacity; ++i) {
        PendingSlot& slot = _slots[i];
        // "Pending" = slot has been claimed (freeList doesn't include it) but not yet ready.
        // The free-list membership is the simplest marker.
        bool inFree = false;
        for (uint32_t f : _freeList) { if (f == i) { inFree = true; break; } }
        if (inFree) continue;
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
    (void)_readIdx;
}

} // namespace ayt::physics