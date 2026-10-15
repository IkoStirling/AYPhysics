#include "AYTest.h"
#include <AYPhysics/PhysicsDebugDraw.h>
#include <AYPhysics/PhysicsManager.h>
#include <AYPhysics/PhysicsWorld3D.h>
#include <AYPhysics/PhysicsWorld2D.h>
#include <cmath>
#include <algorithm>
using namespace ayt::physics;
namespace {
uint64_t nowUs=0;
uint64_t clockUs() { return nowUs; }
PhysicsDebugQuery rayQuery() { PhysicsDebugQuery q; q.ray.dir={1,0,0}; return q; }
PhysicsDebugDrawOptions options(DebugDrawLifetime life) { PhysicsDebugDrawOptions o; o.enabled=true; o.lifetime=life; return o; }
}
TEST_SUITE(PhysicsDebugDrawTests)
TEST_CASE(extreme_ray_directions_stay_finite_and_two_d_ignores_z) {
    PhysicsDebugDraw d(clockUs); CHECK(d.configure(options(DebugDrawLifetime::Frame))==PhysResult::Ok);
    for(float x : {1e-40f,3e38f}) {
        auto q=rayQuery(); q.ray.dir={x,0,0};
        CHECK(d.record(q,PhysResult::Ok,false)==PhysResult::Ok);
        auto lines=d.consumeLines(); CHECK(lines.size()==1); CHECK(lines[0].to.x==10000);
    }
    auto q=rayQuery(); q.dimension=PhysicsDimension::TwoD; q.ray.origin={0,0,9}; q.ray.dir={1,0,7};
    CHECK(d.record(q,PhysResult::Ok,false)==PhysResult::Ok);
    const auto lines=d.consumeLines(); CHECK(lines[0].from.z==0); CHECK(lines[0].to.z==0);
}
TEST_CASE(frame_hit_miss_error_and_hit_marker_are_owned_once) {
    PhysicsDebugDraw d(clockUs); CHECK(d.configure(options(DebugDrawLifetime::Frame))==PhysResult::Ok);
    CHECK(d.record(rayQuery(),PhysResult::Ok,false)==PhysResult::Ok);
    auto lines=d.consumeLines(); CHECK(lines.size()==1); CHECK(lines[0].color==0xff00ff00u); CHECK(lines[0].to.x==10000);
    CHECK(d.consumeLines().empty());
    RaycastHit h; h.hit=true; h.point={2,0,0}; h.normal={-1,0,0};
    CHECK(d.record(rayQuery(),PhysResult::Ok,true,&h)==PhysResult::Ok);
    lines=d.consumeLines(); CHECK(lines.size()==5); CHECK(lines[0].color==0xff0000ffu); CHECK(lines.back().to.x==1.5f);
    CHECK(d.record(rayQuery(),PhysResult::BackendError,false)==PhysResult::Ok);
    CHECK(d.consumeLines()[0].color==0xff00ffffu);
}
TEST_CASE(timed_persistent_and_mixed_settings_obey_monotonic_expiry) {
    nowUs=100; PhysicsDebugDraw d(clockUs);
    auto o=options(DebugDrawLifetime::Timed); o.duration=ayt::time::Duration::fromUs(2500000);
    CHECK(d.configure(o)==PhysResult::Ok); CHECK(d.record(rayQuery(),PhysResult::Ok,false)==PhysResult::Ok);
    CHECK(d.configure(options(DebugDrawLifetime::Persistent))==PhysResult::Ok);
    CHECK(d.record(rayQuery(),PhysResult::Ok,true)==PhysResult::Ok);
    nowUs=2500099; CHECK(d.consumeLines().size()==2);
    nowUs=2500100; CHECK(d.consumeLines().size()==1);
    nowUs=90000000; CHECK(d.consumeLines().size()==1);
    d.clear(); CHECK(d.consumeLines().empty());
    o.duration=ayt::time::Duration::fromUs(0); CHECK(d.configure(o)==PhysResult::InvalidParam);
    CHECK(d.options().lifetime==DebugDrawLifetime::Persistent);
}
TEST_CASE(history_budget_rejects_new_records_without_evicting_old) {
    PhysicsDebugDraw d(clockUs); CHECK(d.configure(options(DebugDrawLifetime::Persistent))==PhysResult::Ok);
    for(size_t i=0;i<PhysicsDebugDraw::kMaxRecords;++i) CHECK(d.record(rayQuery(),PhysResult::Ok,false)==PhysResult::Ok);
    CHECK(d.record(rayQuery(),PhysResult::Ok,true)==PhysResult::QueueFull);
    CHECK(d.droppedRecords()==1); CHECK(d.consumeLines().size()==PhysicsDebugDraw::kMaxRecords);
    PhysicsDebugDrawOptions off; CHECK(d.configure(off)==PhysResult::Ok); CHECK(d.consumeLines().empty());
}
TEST_CASE(box_circle_capsule_offsets_and_xy_geometry_match_descriptors) {
    ColliderDesc s; s.offset={2,0,3};
    auto a=PhysicsDebugDraw::colliderLines(s,{1,0,0},ayt::math::FQuaternion::identity(),PhysicsDimension::ThreeD,1);
    CHECK(a.size()==12); CHECK(a[0].from.x==2.5f); CHECK(a[0].from.z==2.5f);
    a=PhysicsDebugDraw::colliderLines(s,{0,0,9},ayt::math::FQuaternion::identity(),PhysicsDimension::TwoD,1);
    CHECK(a.size()==4); CHECK(a[0].from.z==0);
    s.shape=ColliderShape::Sphere; CHECK(PhysicsDebugDraw::colliderLines(s,{},ayt::math::FQuaternion::identity(),PhysicsDimension::TwoD,1).size()==32);
    s.shape=ColliderShape::Capsule; s.height=4; s.radius=1; s.offset={};
    a=PhysicsDebugDraw::colliderLines(s,{},ayt::math::FQuaternion::identity(),PhysicsDimension::TwoD,1);
    CHECK(a.size()==34); float maxY=-100; for(auto& l:a) maxY=std::max(maxY,l.to.y); CHECK(std::abs(maxY-2)<1e-5f);
    s.radius=-1; CHECK(PhysicsDebugDraw::colliderLines(s,{},ayt::math::FQuaternion::identity(),PhysicsDimension::TwoD,1).empty());
}
TEST_CASE(real_world_facades_capture_completed_sync_queries_only_when_enabled) {
    PhysicsBackendDescriptor desc; desc.kind3D=BackendKind::Null; desc.kind2D=BackendKind::Null;
    auto manager=PhysicsManager::create(desc); CHECK(manager!=nullptr); if(!manager) return;
    RaycastHit hit;
    manager->world3D()->raycastSync(rayQuery().ray,hit);
    CHECK(manager->debugDraw().consumeLines().empty());
    CHECK(manager->debugDraw().configure(options(DebugDrawLifetime::Frame))==PhysResult::Ok);
    manager->world3D()->raycastSync(rayQuery().ray,hit);
    CHECK(manager->debugDraw().consumeLines().size()==1);
    std::vector<BodyHandle> bodies;
    manager->world2D()->overlapBoxSync({},{1,2,0},bodies);
    CHECK(manager->debugDraw().consumeLines().size()==4);
}
TEST_CASE(freeze_preserves_timed_records_and_category_clear_is_independent) {
    nowUs=100;PhysicsDebugDraw d(clockUs);auto o=options(DebugDrawLifetime::Timed);o.duration=ayt::time::Duration::fromUs(100);d.configure(o);
    d.record(rayQuery(),PhysResult::Ok,false);d.setFrozen(true);nowUs=1000;
    CHECK(d.consumeRecords().size()==1);CHECK(d.record(rayQuery(),PhysResult::Ok,true)==PhysResult::Ok);CHECK(d.recordCount()==1);
    d.setFrozen(false);nowUs=1099;CHECK(d.consumeRecords().size()==1);nowUs=1100;CHECK(d.consumeRecords().empty());
    d.configure(options(DebugDrawLifetime::Persistent));PhysicsDebugRecord ability;ability.category=PhysicsDebugCategory::Ability;ability.lines={{{},{1,0,0},0xffffffff}};
    d.recordGeometry(ability);d.record(rayQuery(),PhysResult::Ok,false);d.clearCategory(PhysicsDebugCategory::Query);
    CHECK(d.consumeRecords().size()==1);CHECK(d.consumeRecords()[0].category==PhysicsDebugCategory::Ability);
}
TEST_CASE(async_dimensions_do_not_alias_and_expiry_starts_at_completion) {
    nowUs=1000;PhysicsDebugDraw d(clockUs);auto o=options(DebugDrawLifetime::Timed);o.duration=ayt::time::Duration::fromUs(100);d.configure(o);
    auto q=rayQuery();q.queryId=7;q.dimension=PhysicsDimension::TwoD;d.track(q);q.dimension=PhysicsDimension::ThreeD;d.track(q);
    QueryResult result;result.queryId=7;result.dimension=PhysicsDimension::TwoD;result.completedAtUs=1000;result.status=PhysResult::BackendError;
    d.complete(result);result.dimension=PhysicsDimension::ThreeD;result.status=PhysResult::Ok;result.hitCount=1;d.complete(result);
    const auto records=d.consumeRecords();CHECK(records.size()==2);CHECK(records[0].lines[0].color==o.errorColor);CHECK(records[1].hit);
    nowUs=1100;CHECK(d.consumeRecords().empty());d.complete(result);CHECK(d.consumeRecords().empty());
}
TEST_SUITE_END
