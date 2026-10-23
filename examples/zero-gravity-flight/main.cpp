#include <AYPhysics/FlightControl3D.h>
#include <AYPhysics/PhysicsManager.h>
#include <AYPhysics/PhysicsWorld3D.h>
#include <cmath>
#include <iostream>

int main() {
    using namespace ayt::physics;
    PhysicsBackendDescriptor config;
    config.kind3D=BackendKind::DefaultJolt;
    config.kind2D=BackendKind::Null;
    config.gravity3D={0,0,0};
    config.jobWorkerCount=1;
    config.maxBodies=256;
    config.maxBodyPairs=1024;
    config.maxContactConstraints=1024;
    auto manager=PhysicsManager::create(config);
    if (!manager) { std::cerr << "manager creation failed\n"; return 1; }
    auto* world=manager->world3D();
    RigidbodyDesc ship;
    ship.type=BodyType::Dynamic;
    ship.alwaysSync=true;
    ship.linearDamping=0;
    ship.angularDamping=.5f;
    BodyHandle body{};
    if (world->createRigidbody(ship,body)!=PhysResult::Ok) { std::cerr << "body admission failed\n"; return 2; }
    ColliderDesc shape;
    shape.body=body;
    shape.halfExtents={.5f,.2f,1.f};
    ColliderHandle collider{};
    if (world->createCollider(shape,collider)!=PhysResult::Ok) { std::cerr << "collider admission failed\n"; return 3; }
    if (manager->stepAndWait(1.f/60)!=PhysResult::Ok) { std::cerr << "initial step failed\n"; return 4; }
    ayt::math::FQuaternion orientation{0,0,0,1};
    ayt::math::FVector3 position{};
    for (int frame=0;frame<120;++frame) {
        FlightInput3D input;
        input.translation={0,0,1};
        input.rotation={0,frame<60?.5f:0,0};
        FlightWrench3D wrench;
        if (!flightWrench3D(input,orientation,{20,8},wrench)
            || world->applyForce(body,wrench.force)!=PhysResult::Ok
            || world->applyTorque(body,wrench.torque)!=PhysResult::Ok
            || manager->stepAndWait(1.f/60)!=PhysResult::Ok) { std::cerr << "flight step failed at " << frame << '\n'; return 4; }
        for (const auto& transform:manager->fetchResults().transforms) {
            if (transform.body==body) {
                position=transform.position;
                orientation=transform.rotation;
            }
        }
    }
    std::cout << "6DOF ship position: " << position.x << ", " << position.y
              << ", " << position.z << '\n';
    return std::isfinite(position.x) && position.lengthSq()>1 ? 0 : 5;
}
