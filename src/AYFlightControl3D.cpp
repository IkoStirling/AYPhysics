#include <AYPhysics/FlightControl3D.h>
#include <algorithm>
#include <cmath>

namespace ayt::physics {
namespace {
bool finite(const ayt::math::FVector3& v) noexcept {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
ayt::math::FVector3 clampAxes(const ayt::math::FVector3& v) noexcept {
    return {std::clamp(v.x,-1.0f,1.0f), std::clamp(v.y,-1.0f,1.0f),
            std::clamp(v.z,-1.0f,1.0f)};
}
}
bool flightWrench3D(const FlightInput3D& input,
                    const ayt::math::FQuaternion& orientation,
                    const FlightTuning3D& tuning,
                    FlightWrench3D& out) noexcept {
    const float norm = orientation.dot(orientation);
    if (!finite(input.translation) || !finite(input.rotation)
        || !std::isfinite(norm) || norm < 1.0e-8f
        || !std::isfinite(tuning.thrust) || tuning.thrust < 0
        || !std::isfinite(tuning.torque) || tuning.torque < 0)
        return false;
    const auto rotation = orientation.normalize();
    const FlightWrench3D candidate{
        rotation * (clampAxes(input.translation) * tuning.thrust),
        rotation * (clampAxes(input.rotation) * tuning.torque)};
    if (!finite(candidate.force) || !finite(candidate.torque)) return false;
    out = candidate;
    return true;
}
} // namespace ayt::physics
