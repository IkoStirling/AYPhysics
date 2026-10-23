# Spatial gravity and six-axis movement

This is a game-thread gravity catalog and fixed-step force bridge. Create a
Jolt `PhysicsManager` with `gravity3D={0,0,0}` when bodies in one world need
different gravity. Nonzero world gravity remains additive to dispatched field
acceleration. `GravityField3D::replace` validates the complete source list and
keeps the old catalog on failure. Sources are uniform, axis-aligned box, or
radial within a finite radius. The highest overlapping priority wins; sources
at that priority add in ID order. Outside coverage acceleration is zero.
Sources carry a 32-bit channel mask; `sample(position, receiverChannels)`
ignores sources without a shared bit before priority selection. The default
mask keeps existing all-source behavior.

```cpp
GravityField3D field;
GravitySource3D planet;
planet.id=1;
planet.shape=GravityShape3D::Radial;
planet.center={0,0,0};
planet.radius=100;
planet.acceleration=9.81f;
field.replace({planet});

// FixedPrePhysics, using a pose copied from the last completed snapshot:
GravityBodySample3D body{shipBody, shipPosition};
const auto delivery=submitGravity3D(*manager.world3D(),field,std::span(&body,1));
// QueueFull may leave an already-submitted prefix; do not retry it this tick.
// FixedPhysics: standard Host steps here; embedded hosts call stepAndWait.
```

Register dynamic bodies explicitly. `applyAcceleration` converts the sample
into force using native body mass at command execution; static and kinematic
bodies are skipped. Sampling is game-thread work, not automatic ECS discovery.
Use the last completed pose and a fixed body order, then read new poses after
the completion barrier. The field does not change global gravity or gravityScale.

For an arbitrary-up character, sample at the feet, set
`CharacterController3DInput::useGravityOverride=true`, and put the sample in
`gravity`. Nonzero gravity sets `up=-normalize(gravity)`; zero retains the
previous up. Input velocity is projected onto the tangent plane. Jump, stairs
and ground snap follow current up. `CharacterController3DDesc::up` supplies
the initial orientation. The solver rotates the capsule immediately on a
gravity change; game code can blend the camera horizon for comfort.

Copy completed character state's `up` into
`CameraControllerComponent::upVector`. Free and orbit modes derive yaw/pitch
from that up, and `rollRadians` adds bank. Manual camera mode can directly
follow a ship's rigidbody orientation for unrestricted 6DOF rolls.

The runnable `AYPhysics_ZeroGravityFlightExample` demonstrates a zero-gravity
Jolt body, local-axis thrust/torque input, and fixed-step pose feedback.
`flightWrench3D` maps local right/up/forward and pitch/yaw/roll axes to world
force and torque. It rejects invalid input. Braking, auto-level, camera chase
and network prediction are game policies built on this primitive.

CPU 3D particle simulations can share the catalog through
`particle::GravityQuery3D`: implement `acceleration(worldPosition)` by calling
`field.sample({x,y,z}).acceleration`, then set the query on
`ParticleSimulationSystem` for scene playback or pass it to headless
`ParticleInstance::update`. The query is borrowed and must stay alive for each
update. Spatial sampling is CPU only; scene integration reports CPU fallback
while the query is set. Particle simulation remains visual presentation time.

For Scene-authored 3D rigid bodies, Spatial Entity has intrinsic gravity
settings. Entity.PhysicsIntegration samples enabled Spatial sources and Field
receivers before each fixed step. The direct `GravityField3D` path above
remains available for bodies managed outside AYEntity. See AYEntity's design
for the built-in authoring modes and legacy rigidbody scale interaction.
