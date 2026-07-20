#include "AYPhysicsTypes.h"

namespace ayt::physics {

const char* toString(PhysResult r) noexcept {
    switch (r) {
        case PhysResult::Ok:            return "Ok";
        case PhysResult::InvalidParam:  return "InvalidParam";
        case PhysResult::NoMemory:      return "NoMemory";
        case PhysResult::AlreadyExists: return "AlreadyExists";
        case PhysResult::NotFound:      return "NotFound";
        case PhysResult::InvalidState:  return "InvalidState";
        case PhysResult::BackendError:  return "BackendError";
        case PhysResult::OutOfRange:    return "OutOfRange";
        case PhysResult::Unsupported:   return "Unsupported";
        case PhysResult::QueueFull:     return "QueueFull";
    }
    return "Unknown";
}

namespace _detail {
    // Inline-and-not-exported helper visible only inside this TU; used by tests.
    static volatile bool g_lockstepForTests = false;
}
bool isLockstepActive() noexcept {
    // R1 stub: always false. R3 wires to ayt::game::LockstepSession::isActive().
    return false;
}

} // namespace ayt::physics