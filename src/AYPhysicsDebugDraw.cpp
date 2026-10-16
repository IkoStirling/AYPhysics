#include <AYPhysics/PhysicsDebugDraw.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace ayt::physics {
namespace {
using V = ayt::math::FVector3;
bool finite(const V& v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }
V rotate(const V& p, const ayt::math::FQuaternion& q) {
    const V u(q.x,q.y,q.z);
    return p + u.cross(p) * (2*q.w) + u.cross(u.cross(p))*2;
}
void line(std::vector<PhysicsDebugLine>& out, V a, V b, uint32_t color) { out.push_back({a,b,color}); }
void circle(std::vector<PhysicsDebugLine>& out, V center, V u, V v, float r, uint32_t c) {
    constexpr int segments=32;
    V previous=center+u*r;
    for(int i=1;i<=segments;++i) {
        const float a=6.28318530718f*float(i)/segments;
        V next=center+(u*std::cos(a)+v*std::sin(a))*r;
        line(out,previous,next,c); previous=next;
    }
}
}
PhysResult PhysicsDebugDraw::configure(const PhysicsDebugDrawOptions& o) {
    if (o.lifetime != DebugDrawLifetime::Frame && o.lifetime != DebugDrawLifetime::Timed
        && o.lifetime != DebugDrawLifetime::Persistent) return PhysResult::InvalidParam;
    if (o.lifetime == DebugDrawLifetime::Timed && o.duration.toUs() <= 0)
        return PhysResult::InvalidParam;
    _options=o;
    if(!o.enabled) clear();
    return PhysResult::Ok;
}
void PhysicsDebugDraw::expire(uint64_t now) {
    for(auto i=_records.begin();i!=_records.end();) {
        if(i->lifetime==DebugDrawLifetime::Timed && now>=i->expires) {
            _lineCount-=(i->value.lines.size()+i->value.screenLines.size()); i=_records.erase(i);
        } else ++i;
    }
}
std::vector<PhysicsDebugLine> PhysicsDebugDraw::colliderLines(const ColliderDesc& s,
    const V& position, const ayt::math::FQuaternion& rotation, PhysicsDimension dimension, uint32_t color) {
    std::vector<PhysicsDebugLine> out;
    if(!finite(position)||!finite(s.offset)||!std::isfinite(rotation.x)||!std::isfinite(rotation.y)
        ||!std::isfinite(rotation.z)||!std::isfinite(rotation.w)) return out;
    const bool twoD=dimension==PhysicsDimension::TwoD;
    auto pose=rotation;
    V bodyPosition=position;
    if(twoD) {
        pose=ayt::math::FQuaternion::fromEulerAngles({0,0,rotation.toEulerAngles().z});
        bodyPosition.z=0;
    }
    V offset=s.offset; if(twoD) offset.z=0;
    auto world=[&](V p) { return bodyPosition+rotate(p+offset,pose); };
    if(s.shape==ColliderShape::Box) {
        const V h=s.halfExtents;
        if(!finite(h)||h.x<=0||h.y<=0||(!twoD&&h.z<=0)) return out;
        V p[8];
        for(int i=0;i<8;++i) p[i]=world({(i&1)?h.x:-h.x,(i&2)?h.y:-h.y,twoD?0.0f:((i&4)?h.z:-h.z)});
        const int count=twoD?4:8;
        for(int i=0;i<count;++i) for(int bit=1;bit<count;bit*=2)
            if(!(i&bit)) line(out,p[i],p[i|bit],color);
    } else if(s.shape==ColliderShape::Sphere) {
        if(!std::isfinite(s.radius)||s.radius<=0) return out;
        const V c=world({});
        circle(out,c,rotate({1,0,0},pose),rotate({0,1,0},pose),s.radius,color);
        if(!twoD) {
            circle(out,c,rotate({1,0,0},rotation),rotate({0,0,1},rotation),s.radius,color);
            circle(out,c,rotate({0,1,0},rotation),rotate({0,0,1},rotation),s.radius,color);
        }
    } else if(s.shape==ColliderShape::Capsule) {
        if(!std::isfinite(s.radius)||s.radius<=0||!std::isfinite(s.height)||s.height<0) return out;
        const float h=std::max(0.0f,s.height*0.5f-s.radius),r=s.radius;
        // Backend capsule height includes both hemispheres.
        for(int plane=0;plane<(twoD?1:2);++plane) {
            V u=plane==0?V{1,0,0}:V{0,0,1};
            line(out,world(u*r+V{0,-h,0}),world(u*r+V{0,h,0}),color);
            line(out,world(u*(-r)+V{0,-h,0}),world(u*(-r)+V{0,h,0}),color);
            for(int end : {-1,1}) {
                V previous=world(u*r+V{0,float(end)*h,0});
                for(int i=1;i<=16;++i) {
                    const float a=3.14159265359f*float(i)/16;
                    V next=world(u*(r*std::cos(a))+V{0,float(end)*(h+r*std::sin(a)),0});
                    line(out,previous,next,color); previous=next;
                }
            }
        }
        if(!twoD) for(int end : {-1,1}) circle(out,world({0,float(end)*h,0}),
            rotate({1,0,0},rotation),rotate({0,0,1},rotation),r,color);
    }
    if(std::any_of(out.begin(),out.end(),[](const auto& l){return !finite(l.from)||!finite(l.to);})) out.clear();
    return out;
}
PhysResult PhysicsDebugDraw::record(const PhysicsDebugQuery& q, PhysResult result, bool hit, const RaycastHit* detail, const std::vector<BodyHandle>* overlaps) noexcept try {
    if(!_options.enabled || _frozen) return PhysResult::Ok;
    const uint64_t now=_clock(); expire(now);
    const uint32_t c=result!=PhysResult::Ok?_options.errorColor:(hit?_options.hitColor:_options.missColor);
    std::vector<PhysicsDebugLine> lines;
    if(q.type==SyncQueryType::Raycast) {
        V direction=q.ray.dir;
        V origin=q.ray.origin;
        if(q.dimension==PhysicsDimension::TwoD) { direction.z=0; origin.z=0; }
        const double length=std::sqrt(double(direction.x)*direction.x+double(direction.y)*direction.y+double(direction.z)*direction.z);
        if(!finite(q.ray.origin)||!finite(direction)||!(length>0)) return PhysResult::InvalidParam;
        V end=origin+V{float(double(direction.x)/length*10000.0),
                      float(double(direction.y)/length*10000.0),
                      float(double(direction.z)/length*10000.0)};
        line(lines,origin,end,c);
        if(result==PhysResult::Ok&&hit&&detail&&finite(detail->point)&&finite(detail->normal)) {
            const V p=detail->point;
            for(V axis : {V{0.08f,0,0},V{0,0.08f,0},V{0,0,0.08f}})
                line(lines,p-axis,p+axis,c);
            line(lines,p,p+detail->normal*0.5f,c);
        }
    } else {
        ColliderDesc s;
        if(q.type==SyncQueryType::OverlapSphere) { s.shape=ColliderShape::Sphere; s.radius=q.radius; }
        else if(q.type==SyncQueryType::OverlapBox) { s.shape=ColliderShape::Box; s.halfExtents=q.halfExtents; }
        else return PhysResult::InvalidParam;
        lines=colliderLines(s,q.center,q.rotation,q.dimension,c);
        if(lines.empty()) return PhysResult::InvalidParam;
    }
    if(std::any_of(lines.begin(),lines.end(),[](const auto& l){return !finite(l.from)||!finite(l.to);})) return PhysResult::InvalidParam;
    PhysicsDebugRecord value;
    value.category=PhysicsDebugCategory::Query; value.dimension=q.dimension;
    value.queryId=q.queryId; value.layerMask=q.layerMask; value.status=result; value.hit=hit;
    value.anchor=q.type==SyncQueryType::Raycast?q.ray.origin:q.center;
    value.label=q.type==SyncQueryType::Raycast?"Raycast":q.type==SyncQueryType::OverlapSphere?"Overlap sphere":"Overlap box";
    if(detail) { value.body=detail->body; value.collider=detail->collider; value.distance=detail->distance;value.hitPoint=detail->point;value.normal=detail->normal; }
    if(overlaps) {value.hitBodies=*overlaps;if(!value.body && !overlaps->empty())value.body=overlaps->front();}
    else if(detail && detail->body) value.hitBodies.push_back(detail->body);
    value.completedAtUs=q.completedAtUs;
    value.lines=std::move(lines);
    return recordGeometry(std::move(value));
} catch (...) {
    ++_dropped;
    return PhysResult::NoMemory;
}
std::vector<PhysicsDebugLine> PhysicsDebugDraw::consumeLines() {
    std::vector<PhysicsDebugLine> out;
    for(auto& value:consumeRecords()) out.insert(out.end(),value.lines.begin(),value.lines.end());
    return out;
}
std::vector<PhysicsDebugRecord> PhysicsDebugDraw::consumeRecords() {
    if(!_frozen) expire(_clock());
    std::vector<PhysicsDebugRecord> out; out.reserve(_records.size());
    for(auto i=_records.begin();i!=_records.end();) {
        out.push_back(i->value);
        if(!_frozen && i->lifetime==DebugDrawLifetime::Frame) { _lineCount-=(i->value.lines.size()+i->value.screenLines.size()); i=_records.erase(i); }
        else ++i;
    }
    return out;
}
PhysResult PhysicsDebugDraw::recordGeometry(PhysicsDebugRecord value) noexcept try {
    if(!_options.enabled || _frozen) return PhysResult::Ok;
    if(uint32_t(value.category)>=kPhysicsDebugCategoryCount)return PhysResult::InvalidParam;
    const auto now=_clock();expire(now);
    if(std::any_of(value.lines.begin(),value.lines.end(),[](const auto& l){return !finite(l.from)||!finite(l.to);}) || std::any_of(value.screenLines.begin(),value.screenLines.end(),[](const auto& l){return !finite(l.from)||!finite(l.to);})) return PhysResult::InvalidParam;
    if(_records.size()>=kMaxRecords || value.lines.size()+value.screenLines.size()>kMaxLines-_lineCount) {++_dropped;return PhysResult::QueueFull;}
    const auto duration=static_cast<uint64_t>(std::max<int64_t>(0,_options.duration.toUs()));
    const auto completed=value.completedAtUs?value.completedAtUs:now;
    value.completedAtUs=completed;
    const auto expires=duration>UINT64_MAX-completed?UINT64_MAX:completed+duration;
    const auto count=value.lines.size()+value.screenLines.size();_records.push_back({std::move(value),_options.lifetime,expires});_lineCount+=count;
    return PhysResult::Ok;
} catch(...) {++_dropped;return PhysResult::NoMemory;}
void PhysicsDebugDraw::setFrozen(bool frozen) noexcept {
    if(frozen==_frozen) return;
    const auto now=_clock();
    if(frozen) _freezeTime=now;
    else for(auto& r:_records) if(r.lifetime==DebugDrawLifetime::Timed) {
        const auto delta=now-_freezeTime; r.expires=delta>UINT64_MAX-r.expires?UINT64_MAX:r.expires+delta;
    }
    _frozen=frozen;
}
void PhysicsDebugDraw::clearCategory(PhysicsDebugCategory category) noexcept {
    for(auto i=_records.begin();i!=_records.end();) if(i->value.category==category) {
        _lineCount-=(i->value.lines.size()+i->value.screenLines.size()); i=_records.erase(i);
    } else ++i;
    if(category==PhysicsDebugCategory::Query) _pending.clear();
}
PhysResult PhysicsDebugDraw::track(const PhysicsDebugQuery& q) noexcept try {
    if(!_options.enabled || _frozen) return PhysResult::Ok;
    if(_pending.size()>=kMaxRecords) {++_dropped; return PhysResult::QueueFull;}
    _pending[(uint64_t(q.dimension)<<32)|q.queryId]=q; return PhysResult::Ok;
} catch(...) {++_dropped; return PhysResult::NoMemory;}
void PhysicsDebugDraw::complete(const QueryResult& result) {
    const auto key=(uint64_t(result.dimension)<<32)|result.queryId;
    const auto found=_pending.find(key); if(found==_pending.end()) return;
    auto query=found->second; _pending.erase(found);query.completedAtUs=result.completedAtUs;
    std::vector<BodyHandle> bodies;for(const auto& hit:result.hits) if(hit.body) bodies.push_back(hit.body);
    (void)record(query,result.status,result.hitCount!=0,result.hits.empty()?nullptr:&result.hits.front(),&bodies);
}
bool PhysicsDebugSnapshot::append(PhysicsDebugRecord record) {
    if(uint32_t(record.category)>=kPhysicsDebugCategoryCount){++droppedRecords;return false;}
    if(!(categories&(1u<<uint32_t(record.category)))) return true;
    if(records.size()>=PhysicsDebugDraw::kMaxRecords || record.lines.size()+record.screenLines.size()>PhysicsDebugDraw::kMaxLines-lineCount) {
        ++droppedRecords; return false;
    }
    lineCount+=record.lines.size()+record.screenLines.size(); records.push_back(std::move(record)); return true;
}
} // namespace ayt::physics
