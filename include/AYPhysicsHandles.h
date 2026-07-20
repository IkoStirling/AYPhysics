#pragma once
// AYPhysicsHandles.h - packed index+generation handle helpers (§7.1)
//
// Layout (locked):
//   bits [0..19]  = index      (1 .. 2^20-1); 0 reserved
//   bits [20..31] = generation (1 .. 2^12-1); 0 reserved for invalid
// Invalid value = 0 (both index and generation reserved).
//
// All live bodies / colliders / joints use these helpers. Backend mutators
// MUST verify (slot generation == handle generation) and return PhysResult::NotFound
// on mismatch; see design §17.4.

#include <cstdint>

namespace ayt::physics {

using BodyHandle     = uint32_t;
using ColliderHandle = uint32_t;
using JointHandle    = uint32_t;

constexpr BodyHandle     InvalidBodyHandle     = 0;
constexpr ColliderHandle InvalidColliderHandle = 0;
constexpr JointHandle    InvalidJointHandle    = 0;

constexpr uint32_t kPhysHandleIndexBits = 20;
constexpr uint32_t kPhysHandleGenBits   = 12;
constexpr uint32_t kPhysHandleIndexMask = (1u << kPhysHandleIndexBits) - 1u;
constexpr uint32_t kPhysHandleGenMask   = (1u << kPhysHandleGenBits) - 1u;
constexpr uint32_t kPhysHandleMaxIndex  = kPhysHandleIndexMask;  // 1,048,575
constexpr uint32_t kPhysHandleMaxGen    = kPhysHandleGenMask;    // 4095

inline uint32_t handleIndex(BodyHandle h) noexcept {
    return h & kPhysHandleIndexMask;
}

inline uint32_t handleGeneration(BodyHandle h) noexcept {
    return (h >> kPhysHandleIndexBits) & kPhysHandleGenMask;
}

inline BodyHandle makeHandle(uint32_t index, uint32_t generation) noexcept {
    return ((generation & kPhysHandleGenMask) << kPhysHandleIndexBits)
         | (index & kPhysHandleIndexMask);
}

// Single templated check; BodyHandle / ColliderHandle / JointHandle all alias
// to uint32_t and check "non-zero" suffices for the invalid = 0 convention.
template <typename Handle>
inline bool isValidHandle(Handle h) noexcept {
    return h != 0;
}

// Bump generation on reuse: wraps 4095 -> 1 (0 reserved for invalid).
inline uint32_t bumpGeneration(uint32_t g) noexcept {
    return (g % kPhysHandleMaxGen) + 1u;
}

} // namespace ayt::physics