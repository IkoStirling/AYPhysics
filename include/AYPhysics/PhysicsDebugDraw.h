#pragma once
#include <AYPhysics/PhysicsTypes.h>
#include <AYTime/Clock.h>
#include <deque>
#include <string>
#include <map>

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
enum class PhysicsDebugCategory : uint8_t { Collider, Query, Contact, Character, Motion, Joint, Camera, Skeleton, Ability, Animation, Combat, Lighting, Audio, Particle, Brain, DeterministicPhysics, PathInteraction, Respawn, Stats, CameraBehavior, UI, Input, Network, Sequence, Resource };
inline constexpr uint32_t kPhysicsDebugCategoryCount=25;
inline constexpr uint32_t kPhysicsDebugAllCategories=(1u<<kPhysicsDebugCategoryCount)-1;
struct PhysicsDebugQuery {
    SyncQueryType type = SyncQueryType::Raycast;
    PhysicsDimension dimension = PhysicsDimension::ThreeD;
    ayt::math::Ray ray{};
    ayt::math::FVector3 center{}, halfExtents{};
    float radius = 0;
    ayt::math::FQuaternion rotation = ayt::math::FQuaternion::identity();
    uint32_t queryId = 0;
    PhysLayerMask layerMask = 0xffffffffu;
    uint64_t completedAtUs = 0;
};

/// Owned diagnostic values; identities include the independent physics dimension.
struct PhysicsDebugRecord {
    PhysicsDebugCategory category = PhysicsDebugCategory::Query;
    PhysicsDimension dimension = PhysicsDimension::ThreeD;
    BodyHandle body = 0, otherBody = 0;
    ColliderHandle collider = 0;
    uint32_t queryId = 0, entityId = 0, otherEntityId = 0;
    PhysLayer layer = 0;
    PhysLayerMask layerMask = 0xffffffffu;
    PhysResult status = PhysResult::Ok;
    bool hit = false, impulseAvailable = false;
    float distance = 0, impulse = 0;
    uint64_t completedAtUs = 0;
    std::vector<BodyHandle> hitBodies;
    std::vector<uint32_t> relatedEntities;
    ayt::math::FVector3 anchor{}, velocity{}, desiredVelocity{}, hitPoint{}, normal{};
    std::string label;
    std::vector<PhysicsDebugLine> lines;
    /// Owned human-readable state; diagnostic consumers must not execute or parse it as gameplay input.
    std::string details;
    /// UI lines use logical screen coordinates, never the world camera.
    std::vector<PhysicsDebugLine> screenLines;
    /// Independent event source identity: capture once per record key, without replaying gameplay.
    bool transientEvent=false;
};
struct PhysicsDebugSnapshot {
    uint32_t categories = kPhysicsDebugAllCategories;
    uint64_t frameIndex = 0;
    float stepSeconds = 0;
    size_t droppedRecords = 0, lineCount = 0;
    std::vector<PhysicsDebugRecord> records;
    std::vector<QueryResult> queryResults;
    /// Reject whole records at the diagnostic budget; never changes simulation.
    bool append(PhysicsDebugRecord record);
};

/// Game-thread diagnostic history, independent of simulation and Renderer.
/// Timed entries use a monotonic real clock, including while gameplay is paused.
/// consumeLines copies visible world geometry (screenLines require consumeRecords) and retires Frame entries only; persistent
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
                      bool hit, const RaycastHit* detail = nullptr,
                      const std::vector<BodyHandle>* overlaps = nullptr) noexcept;
    std::vector<PhysicsDebugLine> consumeLines();
    /// Same lifetime semantics as consumeLines; use exactly one consumer.
    std::vector<PhysicsDebugRecord> consumeRecords();
    /// Track accepted asynchronous work; capture only completed results.
    PhysResult track(const PhysicsDebugQuery& query) noexcept;
    void complete(const QueryResult& result);
    /// Explicit owned diagnostics from gameplay adapters; respects the same lifetime/budget.
    PhysResult recordGeometry(PhysicsDebugRecord record) noexcept;
    void setFrozen(bool frozen) noexcept;
    void clearCategory(PhysicsDebugCategory category) noexcept;
    size_t recordCount() const noexcept { return _records.size(); }
    void clear() noexcept { _records.clear(); _lineCount = 0; _dropped = 0; _pending.clear(); }
    size_t droppedRecords() const noexcept { return _dropped; }
    /// Wire geometry shared with editor authoring. Pose has no implicit scale;
    /// the Entity physics bridge likewise ignores Transform scale.
    static std::vector<PhysicsDebugLine> colliderLines(const ColliderDesc& shape,
        const ayt::math::FVector3& position, const ayt::math::FQuaternion& rotation,
        PhysicsDimension dimension, uint32_t color);
private:
    struct Record { PhysicsDebugRecord value; DebugDrawLifetime lifetime; uint64_t expires; };
    void expire(uint64_t now);
    bool _frozen = false;
    uint64_t _freezeTime = 0;
    std::map<uint64_t, PhysicsDebugQuery> _pending;
    TimeSource _clock;
    PhysicsDebugDrawOptions _options;
    std::deque<Record> _records;
    size_t _lineCount = 0, _dropped = 0;
};
} // namespace ayt::physics
