#pragma once

#include <AYPhysics/PhysicsTypes.h>
#include <span>
#include <string>
#include <vector>

namespace ayt::physics {

class PhysicsWorld3D;

enum class GravityShape3D : uint8_t { Uniform, Box, Radial };

/** A gravity source in world coordinates. Uniform and Box use normalized
 * direction * acceleration; Radial points toward center. Box is axis aligned.
 * radius limits Radial influence; linearFalloff reaches zero at its edge. */
struct GravitySource3D {
    uint64_t id = 0;
    GravityShape3D shape = GravityShape3D::Uniform;
    int32_t priority = 0;
    ayt::math::FVector3 direction{0.0f, -1.0f, 0.0f};
    ayt::math::FVector3 center{};
    ayt::math::FVector3 halfExtent{1.0f, 1.0f, 1.0f};
    float radius = 1.0f;
    float acceleration = 9.81f;
    bool linearFalloff = false;
    uint32_t channelMask = 0xffffffffu;
};

struct GravitySample3D {
    ayt::math::FVector3 acceleration{};
    uint64_t primarySource = 0;
    bool active = false;
};

/** Game-thread gravity catalog. replace() is atomic and validates every
 * source. Highest-priority overlapping sources contribute; equal priorities
 * sum in stable source-ID order. Empty coverage means zero gravity. */
class GravityField3D {
public:
    [[nodiscard]] bool replace(std::vector<GravitySource3D> sources,
                               std::string* error = nullptr);
    /// Only sources sharing a bit with receiverChannels are considered.
    [[nodiscard]] GravitySample3D sample(const ayt::math::FVector3& position,
                                         uint32_t receiverChannels = 0xffffffffu) const noexcept;
    [[nodiscard]] size_t size() const noexcept { return sources_.size(); }
private:
    std::vector<GravitySource3D> sources_;
};

struct GravityBodySample3D {
    BodyHandle body = InvalidBodyHandle;
    ayt::math::FVector3 position{}; // previous completed physics snapshot
    uint32_t receiverChannels = 0xffffffffu;
};

struct GravityDispatch3D {
    PhysResult result = PhysResult::Ok;
    size_t submitted = 0;
};

/** Enqueue one mass-independent acceleration for each registered dynamic body
 * before stepAndWait(). Set world gravity to zero for heterogeneous fields.
 * QueueFull may leave a submitted prefix; never retry that prefix this tick. */
[[nodiscard]] GravityDispatch3D submitGravity3D(
    PhysicsWorld3D& world, const GravityField3D& field,
    std::span<const GravityBodySample3D> bodies);

} // namespace ayt::physics
