#pragma once
#include "AYPhysics/PhysicsHandles.h"
#include <AYMath/MathTypes.h>
#include <cstdint>

namespace ayt::physics {

/** Configuration for a Y-up capsule controller. Position is the capsule's feet.
 * Create through PhysicsManager::world3D()->createCharacter(), never a backend.
 * Dimensions are world units, slope is degrees. The physics thread owns the
 * solver; this descriptor contains no Jolt types or borrowed resources. */
struct CharacterController3DDesc {
    ayt::math::FVector3 position{};
    float radius = 0.3f;
    float height = 1.8f; // Total height including both spherical caps; > 2*radius.
    float maxSlopeDegrees = 45.0f;
    float maxStepHeight = 0.3f;
    float groundSnapDistance = 0.4f;
    float gravityScale = 1.0f;
    float mass = 70.0f;
    float maxPushStrength = 100.0f;
    uint16_t layer = 2; // PhysDefaultLayer::Character
    uint32_t collideMask = 0xFFFFFFFFu;
};

/** World-space XZ velocity, in units/second (Y must be zero).
 * Velocity persists until replaced. jumpSpeed is a one-shot upward speed,
 * consumed at the next step and accepted only on walkable ground. */
struct CharacterController3DInput {
    ayt::math::FVector3 horizontalVelocity{};
    float jumpSpeed = 0.0f;
};

enum class CharacterGroundState3D : uint8_t {
    OnGround, OnSteepGround, NotSupported, InAir
};

/** Value snapshot published after a completed physics step. No backend pointers.
 * Ground handles are borrowed identities; destroy/rebuild invalidates them.
 * Only OnGround is walkable and permits jumping. */
struct CharacterController3DState {
    CharacterHandle character = InvalidCharacterHandle;
    ayt::math::FVector3 position{}; // Feet, not center of mass.
    ayt::math::FVector3 linearVelocity{};
    CharacterGroundState3D groundState = CharacterGroundState3D::InAir;
    ayt::math::FVector3 groundNormal{};
    ayt::math::FVector3 groundVelocity{};
    BodyHandle groundBody = InvalidBodyHandle;
    ColliderHandle groundCollider = InvalidColliderHandle;
    bool isGrounded() const noexcept { return groundState == CharacterGroundState3D::OnGround; }
};

} // namespace ayt::physics
