#pragma once
#include <AYPhysics/CharacterController3D.h>
#include <cmath>

namespace ayt::physics::detail {
inline bool finiteVector(const ayt::math::FVector3& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
inline bool validCharacterDesc(const CharacterController3DDesc& d) {
    return finiteVector(d.position) && std::isfinite(d.radius) && d.radius > 0 &&
        std::isfinite(d.height) && d.height > 2*d.radius &&
        std::isfinite(d.maxSlopeDegrees) && d.maxSlopeDegrees >= 0 && d.maxSlopeDegrees < 90 &&
        std::isfinite(d.maxStepHeight) && d.maxStepHeight >= 0 &&
        std::isfinite(d.groundSnapDistance) && d.groundSnapDistance >= 0 &&
        std::isfinite(d.gravityScale) && d.gravityScale >= 0 &&
        std::isfinite(d.mass) && d.mass > 0 &&
        std::isfinite(d.maxPushStrength) && d.maxPushStrength >= 0 && d.layer < 32;
}
inline bool validCharacterHandle(CharacterHandle h) {
    return handleIndex(h) != 0 && handleGeneration(h) != 0;
}
inline bool validCharacterInput(const CharacterController3DInput& input) {
    return finiteVector(input.horizontalVelocity) && input.horizontalVelocity.y == 0 &&
        std::isfinite(input.jumpSpeed) && input.jumpSpeed >= 0;
}
} // namespace ayt::physics::detail
