#include <AYPhysics/GravityField3D.h>
#include <AYPhysics/PhysicsWorld3D.h>
#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace ayt::physics {
namespace {
bool finite(const ayt::math::FVector3& v) noexcept {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
double length(const ayt::math::FVector3& v) noexcept {
    return std::sqrt(double(v.x)*v.x + double(v.y)*v.y + double(v.z)*v.z);
}
}

bool GravityField3D::replace(std::vector<GravitySource3D> sources,
                             std::string* error) {
    const auto fail = [&](const char* reason) {
        if (error) *error = reason;
        return false;
    };
    if (sources.size() > 64) return fail("gravity source limit is 64");
    std::unordered_set<uint64_t> ids;
    for (const auto& source : sources) {
        if (source.id == 0 || !ids.insert(source.id).second
            || !finite(source.direction) || !finite(source.center)
            || !finite(source.halfExtent)
            || !std::isfinite(source.radius)
            || !std::isfinite(source.acceleration)
            || source.acceleration < 0 || source.acceleration > 10000)
            return fail("invalid gravity source");
        if (source.shape == GravityShape3D::Uniform
            || source.shape == GravityShape3D::Box) {
            if (length(source.direction) < 1.0e-6 || length(source.direction) > 1.0e6)
                return fail("zero gravity direction");
        } else if (source.shape == GravityShape3D::Radial) {
            if (source.radius <= 0 || source.radius > 1.0e6f)
                return fail("invalid radial radius");
        } else return fail("unknown gravity shape");
        if (source.shape == GravityShape3D::Box
            && (source.halfExtent.x <= 0 || source.halfExtent.y <= 0
                || source.halfExtent.z <= 0 || source.halfExtent.x > 1.0e6f
                || source.halfExtent.y > 1.0e6f || source.halfExtent.z > 1.0e6f))
            return fail("invalid box half extent");
    }
    std::sort(sources.begin(), sources.end(), [](const auto& a, const auto& b) {
        if (a.priority != b.priority) return a.priority > b.priority;
        return a.id < b.id;
    });
    sources_ = std::move(sources);
    if (error) error->clear();
    return true;
}

GravitySample3D GravityField3D::sample(
    const ayt::math::FVector3& position, uint32_t receiverChannels) const noexcept {
    GravitySample3D result;
    if (!finite(position)) return result;
    int32_t priority = 0;
    for (const auto& source : sources_) {
        if ((source.channelMask & receiverChannels) == 0) continue;
        if (result.active && source.priority < priority) break;
        ayt::math::FVector3 acceleration{};
        if (source.shape == GravityShape3D::Uniform) {
            acceleration = source.direction * float(source.acceleration / length(source.direction));
        } else if (source.shape == GravityShape3D::Box) {
            const auto d = position - source.center;
            if (std::abs(d.x) > source.halfExtent.x
                || std::abs(d.y) > source.halfExtent.y
                || std::abs(d.z) > source.halfExtent.z) continue;
            acceleration = source.direction * float(source.acceleration / length(source.direction));
        } else {
            const auto d = source.center - position;
            const double distance = length(d);
            if (distance < 1.0e-6f || distance > source.radius) continue;
            float magnitude = source.acceleration;
            if (source.linearFalloff) magnitude *= float(1.0 - distance/source.radius);
            acceleration = d * float(magnitude/distance);
        }
        if (!result.active) {
            result.active = true;
            result.primarySource = source.id;
            priority = source.priority;
        }
        result.acceleration += acceleration;
    }
    return result;
}

GravityDispatch3D submitGravity3D(
    PhysicsWorld3D& world, const GravityField3D& field,
    std::span<const GravityBodySample3D> bodies) {
    GravityDispatch3D dispatch;
    for (const auto& body : bodies) {
        if (!body.body || !finite(body.position)) {
            dispatch.result = PhysResult::InvalidParam;
            break;
        }
        const auto gravity = field.sample(body.position, body.receiverChannels).acceleration;
        if (gravity.x == 0 && gravity.y == 0 && gravity.z == 0) continue;
        dispatch.result = world.applyAcceleration(body.body, gravity);
        if (dispatch.result != PhysResult::Ok) break;
        ++dispatch.submitted;
    }
    return dispatch;
}

} // namespace ayt::physics
