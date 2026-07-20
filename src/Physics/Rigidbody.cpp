#include "AYPhysicsTypes.h"

// Placeholder TU for future Rigidbody runtime object. In R1 the public surface
// is handle-based; bodies live only inside the backend's native table and are
// addressed via BodyHandle. Kept as a TU so the CMake target links cleanly.

namespace ayt::physics::rigidbody {
// Reserved for future per-body accessors (e.g. mass recompute, sleep state).
// Intentionally empty for R1.
} // namespace ayt::physics::rigidbody