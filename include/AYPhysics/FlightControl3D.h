#pragma once

#include <AYMath/MathTypes.h>

namespace ayt::physics {

/** Local-axis six-degree-of-freedom input. Translation is right/up/forward;
 * rotation is pitch/yaw/roll about the ship's local X/Y/Z axes. */
struct FlightInput3D {
    ayt::math::FVector3 translation{};
    ayt::math::FVector3 rotation{};
};

struct FlightTuning3D {
    float thrust = 20.0f;
    float torque = 8.0f;
};

struct FlightWrench3D {
    ayt::math::FVector3 force{};
    ayt::math::FVector3 torque{};
};

/** Convert local ship input to world-space force and torque. Returns false
 * for invalid input without modifying output. Use a zero-gravity world and
 * submit the values before the fixed physics step. */
[[nodiscard]] bool flightWrench3D(const FlightInput3D& input,
                                  const ayt::math::FQuaternion& orientation,
                                  const FlightTuning3D& tuning,
                                  FlightWrench3D& out) noexcept;

} // namespace ayt::physics
