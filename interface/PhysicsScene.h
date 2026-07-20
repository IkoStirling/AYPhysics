#pragma once
// PhysicsScene.h - per-world handle allocator (§7.1, §17.4)
//
// Each PhysicsWorld{2D,3D} owns a HandleAllocator. Generation wraps 1..4095
// (0 reserved for invalid). Index reuses from a free list; on reuse,
// generation = bumpGeneration(prevGen).
//
// This header is forward-friendly: backend impls need only the public API.

#include "AYPhysicsHandles.h"
#include "AYPhysicsTypes.h"

#include <vector>

namespace ayt::physics {

class HandleAllocator {
public:
    bool initialize(uint32_t capacity);
    void shutdown();

    // Allocates a fresh handle. Returns InvalidBodyHandle on exhaustion.
    BodyHandle allocate();
    void free(BodyHandle h);

    // Validation: generation must match slot.
    bool isLive(BodyHandle h) const;

    uint32_t capacity() const noexcept { return _capacity; }
    uint32_t liveCount() const noexcept;

private:
    struct Slot {
        uint32_t generation = 0;  // 0 = empty; generation 1..4095 = live
        bool     inUse      = false;
    };

    std::vector<Slot> _slots;
    std::vector<uint32_t> _freeIndices;
    uint32_t _capacity = 0;
    uint32_t _liveCount = 0;
};

// 2D world uses the same allocator (generic over handle type at the call site).
using ColliderHandleAllocator = HandleAllocator;
using JointHandleAllocator    = HandleAllocator;

} // namespace ayt::physics