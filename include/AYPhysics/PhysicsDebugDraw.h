#pragma once
#include <AYPhysics/PhysicsTypes.h>
#include <AYTime/Clock.h>
#include <deque>

namespace ayt::physics {
enum class DebugDrawLifetime : uint8_t { Frame, Timed, Persistent };
struct PhysicsDebugDrawOptions {
    bool enabled = false;
    DebugDrawLifetime lifetime = DebugDrawLifetime::Frame;
    ayt::time::Duration duration = ayt::time::Duration::fromSeconds(3);
    uint32_t missColor = 0xff00ff00u; // ABGR
    uint32_t hitColor = 0xff0000ffu;
    uint32_t errorColor = 0xff00ffffu;
};
struct PhysicsDebugLine {
    ayt::math::FVector3 from{}, to{};
    uint32_t color = 0xff00ff00u;
};
struct PhysicsDebugQuery {
    SyncQueryType type = SyncQueryType::Raycast;
    PhysicsDimension dimension = PhysicsDimension::ThreeD;
    ayt::math::Ray ray{};
    ayt::math::FVector3 center{}, halfExtents{};
    float radius = 0;
};

/// Game-thread diagnostic history, independent of simulation and Renderer.
/// Timed entries use a monotonic real clock, including while gameplay is paused.
/// consumeLines copies visible geometry and retires Frame entries only; persistent
/// entries live until clear. Limits reject whole records (no eviction of history).
class PhysicsDebugDraw {
public:
    using TimeSource = uint64_t (*)();
    static constexpr size_t kMaxRecords = 1024, kMaxLines = 16384;
    explicit PhysicsDebugDraw(TimeSource clock = ayt::time::Clock::performanceNowUs)
        : _clock(clock ? clock : ayt::time::Clock::performanceNowUs) {}
    /// New settings apply to future completed queries. Disable clears history.
    PhysResult configure(const PhysicsDebugDrawOptions& options);
    const PhysicsDebugDrawOptions& options() const noexcept { return _options; }
    /// Capture a completed real query; errors use a distinct color, never green.
    /// Returns QueueFull at the diagnostic budget; physics results are unaffected.
    PhysResult record(const PhysicsDebugQuery& query, PhysResult result,
                      bool hit, const RaycastHit* detail = nullptr) noexcept;
    std::vector<PhysicsDebugLine> consumeLines();
    void clear() noexcept { _records.clear(); _lineCount = 0; _dropped = 0; }
    size_t droppedRecords() const noexcept { return _dropped; }
    /// Wire geometry shared with editor authoring. Pose has no implicit scale;
    /// the Entity physics bridge likewise ignores Transform scale.
    static std::vector<PhysicsDebugLine> colliderLines(const ColliderDesc& shape,
        const ayt::math::FVector3& position, const ayt::math::FQuaternion& rotation,
        PhysicsDimension dimension, uint32_t color);
private:
    struct Record { std::vector<PhysicsDebugLine> lines; DebugDrawLifetime lifetime; uint64_t expires; };
    void expire(uint64_t now);
    TimeSource _clock;
    PhysicsDebugDrawOptions _options;
    std::deque<Record> _records;
    size_t _lineCount = 0, _dropped = 0;
};
} // namespace ayt::physics
