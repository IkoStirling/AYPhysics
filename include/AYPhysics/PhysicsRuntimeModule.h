#pragma once

#include <AYGameLoop/SubSystemModule.h>
#include <AYPhysics/PhysicsTypes.h>

#include <string_view>

namespace ayt::physics
{

inline constexpr std::string_view kPhysicsRuntimeModuleId =
    "AYPhysics.Runtime";

class PhysicsRuntimeModule final : public ayt::game::SubSystemModule
{
public:
    explicit PhysicsRuntimeModule(
        PhysicsBackendDescriptor descriptor = {});
};

} // namespace ayt::physics
