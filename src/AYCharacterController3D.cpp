#include <AYPhysics/PhysicsWorld3D.h>
#include <AYPhysics/PhysicsManager.h>
#include <AYPhysics/IPhysicsBackend3D.h>
#include "CharacterController3DValidation.h"
#include <algorithm>
#include <cmath>

namespace ayt::physics {
namespace {
PhysResult controllerReady(PhysicsManager* manager) {
    if (!manager || !manager->isRunning()) return PhysResult::InvalidState;
    if (!manager->backend3D() || !manager->backend3D()->supportsCharacterControllers())
        return PhysResult::Unsupported;
    return PhysResult::Ok;
}
}

bool PhysicsWorld3D::ownsCharacter(CharacterHandle h) const noexcept {
    const auto index = handleIndex(h);
    return index < _characterLive.size() && _characterLive[index] &&
           _characterGenerations[index] == handleGeneration(h);
}

PhysResult PhysicsWorld3D::createCharacter(const CharacterController3DDesc& desc,
                                           CharacterHandle& outHandle) {
    outHandle = InvalidCharacterHandle;
    if (!detail::validCharacterDesc(desc)) return PhysResult::InvalidParam;
    const auto ready = controllerReady(_manager);
    if (ready != PhysResult::Ok) return ready;
    const auto capacity = _manager->descriptor().maxBodies;
    if (_characterGenerations.empty()) {
        _characterGenerations.resize(capacity,0);
        _characterLive.resize(capacity,false);
    }
    uint32_t index;
    if (!_freeCharacterIndices.empty()) {
        index = _freeCharacterIndices.back();
        _freeCharacterIndices.pop_back();
    } else {
        if (_nextCharacterIndex + 1 >= capacity) return PhysResult::OutOfRange;
        index = ++_nextCharacterIndex;
    }
    _characterGenerations[index] = static_cast<uint16_t>(bumpGeneration(_characterGenerations[index]));
    const auto handle = makeHandle(index,_characterGenerations[index]);
    PhysicsCreatePayload payload{};
    payload.kind = PhysicsCreatePayload::Kind::Character;
    payload.characterDesc = desc;
    const auto slot = _manager->createPool()->allocate(payload);
    if (!slot) { _freeCharacterIndices.push_back(index); return PhysResult::NoMemory; }
    PhysicsCommand command{};
    command.type = PhysicsCommandType::CreateCharacter;
    command.character = handle;
    command.createSlot = slot;
    if (!_manager->commandQueue()->tryPush(command)) {
        (void)_manager->createPool()->take(slot, payload);
        _freeCharacterIndices.push_back(index);
        return PhysResult::QueueFull;
    }
    outHandle = handle;
    _characterLive[index] = true;
    return PhysResult::Ok;
}

PhysResult PhysicsWorld3D::destroyCharacter(CharacterHandle h) {
    if (!detail::validCharacterHandle(h)) return PhysResult::InvalidParam;
    const auto ready = controllerReady(_manager);
    if (ready != PhysResult::Ok) return ready;
    if (!ownsCharacter(h)) return PhysResult::NotFound;
    PhysicsCommand command{};
    command.type = PhysicsCommandType::DestroyCharacter;
    command.character = h;
    if (!_manager->commandQueue()->tryPush(command)) return PhysResult::QueueFull;
    _characterLive[handleIndex(h)] = false;
    _freeCharacterIndices.push_back(handleIndex(h));
    return PhysResult::Ok;
}

PhysResult PhysicsWorld3D::setCharacterInput(CharacterHandle h,
                                            const CharacterController3DInput& input) {
    if (!detail::validCharacterHandle(h) || !detail::validCharacterInput(input))
        return PhysResult::InvalidParam;
    const auto ready = controllerReady(_manager);
    if (ready != PhysResult::Ok) return ready;
    if (!ownsCharacter(h)) return PhysResult::NotFound;
    PhysicsCommand command{};
    command.type = PhysicsCommandType::SetCharacterInput;
    command.character = h;
    command.u.characterInput = {input.horizontalVelocity.x,
        input.horizontalVelocity.y, input.horizontalVelocity.z, input.jumpSpeed,
        input.gravity.x, input.gravity.y, input.gravity.z, 0.0f};
    command.layerMask = input.useGravityOverride ? 1u : 0u;
    return _manager->commandQueue()->tryPush(command) ? PhysResult::Ok : PhysResult::QueueFull;
}

PhysResult PhysicsWorld3D::teleportCharacter(CharacterHandle h, const ayt::math::FVector3& feet) {
    if (!detail::validCharacterHandle(h) || !detail::finiteVector(feet)) return PhysResult::InvalidParam;
    const auto ready = controllerReady(_manager);
    if (ready != PhysResult::Ok) return ready;
    if (!ownsCharacter(h)) return PhysResult::NotFound;
    PhysicsCommand command{};
    command.type = PhysicsCommandType::TeleportCharacter;
    command.character = h;
    command.u.vec4 = {feet.x, feet.y, feet.z, 0};
    return _manager->commandQueue()->tryPush(command) ? PhysResult::Ok : PhysResult::QueueFull;
}

PhysResult PhysicsWorld3D::transferCharacter(CharacterHandle h,
    const ayt::math::FVector3& feet,
    const ayt::math::FVector3& linearVelocity,
    const ayt::math::FVector3& up,
    PhysLayer layer, PhysLayerMask collideMask) {
    const float upLengthSq = up.lengthSq();
    if (!detail::validCharacterHandle(h) || !detail::finiteVector(feet)
        || !detail::finiteVector(linearVelocity) || !detail::finiteVector(up)
        || !std::isfinite(upLengthSq) || std::fabs(upLengthSq - 1.0f) > 1.0e-3f
        || layer >= 32) return PhysResult::InvalidParam;
    const auto ready = controllerReady(_manager);
    if (ready != PhysResult::Ok) return ready;
    if (!ownsCharacter(h)) return PhysResult::NotFound;
    PhysicsCreatePayload payload{};
    payload.kind = PhysicsCreatePayload::Kind::CharacterTransfer3D;
    payload.characterTransfer3D = {feet, linearVelocity, up, layer, collideMask};
    const auto slot = _manager->createPool()->allocate(payload);
    if (!slot) return PhysResult::NoMemory;
    PhysicsCommand command{};
    command.type = PhysicsCommandType::TransferCharacter3D;
    command.character = h;
    command.createSlot = slot;
    if (!_manager->commandQueue()->tryPush(command)) {
        (void)_manager->createPool()->take(slot, payload);
        return PhysResult::QueueFull;
    }
    return PhysResult::Ok;
}

PhysResult PhysicsWorld3D::getCharacterState(CharacterHandle h, CharacterController3DState& out) {
    out = {};
    if (!detail::validCharacterHandle(h)) return PhysResult::InvalidParam;
    const auto ready = controllerReady(_manager);
    if (ready != PhysResult::Ok) return ready;
    if (!ownsCharacter(h)) return PhysResult::NotFound;
    const auto& states = _manager->fetchResults().characters;
    const auto found = std::find_if(states.begin(), states.end(),
        [h](const auto& state) { return state.character == h; });
    if (found == states.end()) return PhysResult::NotFound;
    out = *found;
    return PhysResult::Ok;
}
} // namespace ayt::physics
