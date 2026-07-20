#include "PhysicsScene.h"

namespace ayt::physics {

bool HandleAllocator::initialize(uint32_t capacity) {
    if (capacity == 0 || capacity > kPhysHandleMaxIndex) {
        return false;
    }
    _slots.assign(capacity, Slot{});
    _freeIndices.clear();
    _freeIndices.reserve(capacity);
    // Index 0 is reserved for "invalid"; start from 1.
    for (uint32_t i = 1; i < capacity; ++i) {
        _freeIndices.push_back(i);
    }
    _capacity = capacity;
    _liveCount = 0;
    return true;
}

void HandleAllocator::shutdown() {
    _slots.clear();
    _freeIndices.clear();
    _capacity = 0;
    _liveCount = 0;
}

BodyHandle HandleAllocator::allocate() {
    if (_freeIndices.empty()) {
        return InvalidBodyHandle;
    }
    const uint32_t idx = _freeIndices.back();
    _freeIndices.pop_back();
    Slot& slot = _slots[idx];
    // Slot starts at generation=0; bump to 1 (first allocation = generation 1).
    if (slot.generation == 0) {
        slot.generation = 1;
    } else {
        slot.generation = bumpGeneration(slot.generation);
    }
    slot.inUse = true;
    ++_liveCount;
    return makeHandle(idx, slot.generation);
}

void HandleAllocator::free(BodyHandle h) {
    if (h == InvalidBodyHandle) return;
    const uint32_t idx = handleIndex(h);
    const uint32_t gen = handleGeneration(h);
    if (idx == 0 || idx >= _capacity) return;
    Slot& slot = _slots[idx];
    if (!slot.inUse || slot.generation != gen) return;
    slot.inUse = false;
    _freeIndices.push_back(idx);
    --_liveCount;
}

bool HandleAllocator::isLive(BodyHandle h) const {
    if (h == InvalidBodyHandle) return false;
    const uint32_t idx = handleIndex(h);
    const uint32_t gen = handleGeneration(h);
    if (idx == 0 || idx >= _capacity) return false;
    const Slot& slot = _slots[idx];
    return slot.inUse && slot.generation == gen;
}

uint32_t HandleAllocator::liveCount() const noexcept {
    return _liveCount;
}

} // namespace ayt::physics