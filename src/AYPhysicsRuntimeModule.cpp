#include <AYPhysics/PhysicsRuntimeModule.h>

#include <AYPhysics/PhysicsSubSystem.h>

#include <memory>
#include <string>
#include <utility>

namespace ayt::physics
{

PhysicsRuntimeModule::PhysicsRuntimeModule(
    PhysicsBackendDescriptor descriptor)
    : SubSystemModule(
          ayt::module::ModuleDescriptor{
              .id = std::string(kPhysicsRuntimeModuleId),
              .displayName = "AYPhysics Runtime",
              .version = "0.4.0",
              .dependencies = {}},
          "Physics",
          [descriptor = std::move(descriptor)]() {
              auto system = std::make_unique<PhysicsSubSystem>();
              system->setDescriptor(descriptor);
              return system;
          })
{
}

} // namespace ayt::physics
