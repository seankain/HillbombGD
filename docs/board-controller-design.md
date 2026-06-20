# Board Controller Design

## Overview

The `BoardController` (`assets/Scenes/BoardController.cs`) is the core player controller for HillbombGD. It drives a `CharacterBody3D` down procedurally-cycled hill chunks using a lean-to-steer model inspired by real skateboard truck geometry, but tuned for arcade responsiveness.

## Design Goals

- Responsive, intuitive steering at all speeds
- Natural terrain following — the board hugs slopes and launches off ramps
- Increasing difficulty at high speed via speed wobble
- Foundation for future mechanics (powerslide, grinds, wallrides) without requiring a rewrite

## Architecture

### Why CharacterBody3D + MoveAndSlide

The controller uses `CharacterBody3D` with `MoveAndSlide()` rather than `RigidBody3D`. This gives direct control over velocity composition while still getting Godot's built-in floor detection, slope snapping, and collision sliding. A `RigidBody3D` approach was prototyped (`vehicle_board.cs`) but abandoned because it made terrain-following and deterministic speed control difficult.

### Internal Speed State

Speed (`_speed`) is tracked as a scalar along the board's heading direction rather than derived from `Velocity`. This is a deliberate design choice: `MoveAndSlide()` projects velocity against collision surfaces, which would bleed speed every frame on slopes (by `cos(slopeAngle)`). Keeping `_speed` as a pure physics integration variable prevents this lossy feedback loop. Vertical velocity (`_airVelY`) is synced back from `Velocity.Y` only when airborne, to let MoveAndSlide handle landing collisions naturally.

### Yaw + Lean State Model

The board's direction is tracked as a scalar yaw angle (`_yaw`) rather than a quaternion or direction vector. Combined with the scalar lean angle (`_leanAngle`), this keeps the physics update simple and avoids accumulated floating-point drift in orientation. The world-space forward vector is reconstructed each frame as `(sin(ψ), 0, cos(ψ))`.

## Steering

### Lean-to-Steer Coupling

The yaw rate formula comes from real skateboard truck geometry — the rolling-without-slip constraint at both truck contact patches:

```
ψ̇ = v · (tan λ_f + tan λ_r) · tan φ / L
```

Where `λ_f`, `λ_r` are front/rear truck pivot angles (kingpin inclination), `φ` is lean angle, and `L` is wheelbase. This produces the correct qualitative behavior: steeper truck angles = more responsive turning, wider wheelbase = more stable.

### Why Not the Full Academic Model

The original implementation used the full Lagrangian lean equation of motion from Rosatello et al. ("The Skateboard Speed Wobble", HAL-01369978, ASME IDETC 2015):

```
I·φ̈ = m·g·h·sin φ − m·v·ψ̇·h·cos φ − c·φ̇ + τ_input
```

The centripetal torque term (`−m·v·ψ̇·h·cos φ`) grows proportional to v² (since ψ̇ itself is proportional to v). At high speeds this overwhelms the input torque and acts as a powerful restoring force, making the board nearly impossible to steer. This is physically accurate — it models why real skateboards self-stabilize at speed — but it's the opposite of what the game needs. The academic model was designed to analyze resonance conditions that produce speed wobble, not to be a fun game controller.

### Current Lean Model

Lean is driven by a first-order spring that tracks the player's input target:

```
φ̇ = k · (φ_target − φ)
```

The `LeanSpringRate` (default 25) controls responsiveness. This is independent of speed, so steering feels consistent whether crawling or bombing. The effective speed used in the yaw-rate formula is clamped to `[MinSteerSpeed, MaxSteerSpeed]` to prevent both dead-zone at low speed and over-rotation at terminal velocity.

### Yaw Rate Sign

The yaw rate is negated relative to the geometric formula. This compensates for `UpdateOrientation()` constructing its coordinate frame with `fwdH.Cross(normal)`, which produces a leftward vector (not rightward) due to Godot's right-handed coordinate system. The cross product order mirrors the visual frame, so the physics yaw must be negated to match. The visual lean quaternion uses the un-negated angle because the mirrored frame already inverts it correctly.

## Speed Wobble

Speed wobble is implemented as a sinusoidal perturbation injected into the lean angle when `_speed` exceeds `WobbleSpeedThreshold`. The wobble intensity ramps linearly from 0 to 1 over 15 m/s above threshold. Because wobble modifies `_leanAngle` directly, it feeds back into the yaw-rate calculation next frame, producing genuine steering instability rather than just a visual shake.

Key parameters:
- `WobbleSpeedThreshold` (18 m/s): onset speed, roughly 60-70% of terminal velocity on a steep grade
- `WobbleMaxAmplitude` (0.08 rad / ~4.6 deg): enough to be felt, not enough to instantly crash
- `WobbleFrequencyRange` (8 Hz): base oscillation frequency

## Jump / Ollie

Jump applies a vertical impulse (`_airVelY = JumpSpeed`) and uses a `jumping` flag to force the airborne velocity path on the same frame. Without this flag, the grounded velocity branch (which has no Y component) would swallow the impulse, and `MoveAndSlide()` would snap the character back to the floor before the next frame's `IsOnFloor()` check could return false.

When airborne:
- Lean and lean rate decay exponentially toward zero (self-righting in air)
- Gravity is applied to `_airVelY` each frame
- Forward speed experiences only air drag (no rolling resistance or slope gravity)
- `_airVelY` is synced back from `Velocity.Y` after `MoveAndSlide()` to let the engine handle landing

## Grind

Grinding latches the board onto a rail and rides it, replacing the normal free-movement update while active (`UpdateGrind` returns early from `_PhysicsProcess`, mirroring the crash bail).

Rails are `Path3D` curves on geometry tagged with the `Grindable` group. After `MoveAndSlide`, slide collisions are scanned (`TryEnterGrind`): a grindable contact becomes a grind — taking priority over crash detection — when the board has at least `GrindMinEntrySpeed` and is travelling within `GrindMaxAlignmentAngleDeg` of the rail tangent. The approach-alignment gate is what makes a rail feel grindable when ridden along but solid (crashable) when hit across.

While grinding, the board is pinned to the curve rather than moved by `MoveAndSlide`. `Curve3D.GetClosestOffset` finds where on the rail the board entered; each frame `_grindOffset` advances by `_speed · _grindDir · dt` and the board is snapped to `SampleBaked(offset)` plus `GrindHeightOffset`. Reusing the scalar `_speed` and `_yaw` state (yaw is reconstructed from the tangent each frame) means exit flows seamlessly back into the free-movement model — `ExitGrind` just converts the rail tangent into `_yaw` and `_airVelY`.

A balance mini-game runs in parallel: `_grindBalance` is an unstable tilt that diverges proportionally to itself (`GrindBalanceInstabilityDeg`) with added noise, and the player counters with lean input (`GrindBalanceControl`). This reuses the lean coupling for the visual tilt (`_leanAngle = _grindBalance`). Exceeding `GrindBalanceLimitDeg` calls `BailGrind`, which drops into the existing crash bail. The grind ends on jump (adds `JumpSpeed`), reaching either rail end, or a bail; `GrindReentryCooldown` blocks immediate re-latching.

## Surface Detection

Two raycasts (`FrontTruckRay`, `RearTruckRay`) positioned at the front and rear truck locations sample the terrain normal. When both hit, the normals are averaged to produce a smooth surface orientation. This drives:
- **Slope angle calculation**: determines gravitational acceleration along the heading
- **Velocity projection**: the forward vector is projected onto the slope plane so the board follows terrain contours
- **Board orientation**: the visual basis aligns to the surface normal with slerp smoothing

## Forward Speed Model

On the ground, speed integrates three forces:
- **Gravity along slope**: `g · sin(α)` — the driving force downhill
- **Aerodynamic drag**: `½ρCdA · v|v| / m` — quadratic, dominant at high speed
- **Rolling resistance**: `μr · g · cos(α)` — small constant drain

Terminal velocity on a given slope is where drag + rolling resistance equals gravity component. The drag model uses `v|v|` (not `v²`) so it correctly decelerates the board if it ever travels uphill.

## Orientation

`UpdateOrientation()` constructs a surface-aligned basis each frame and slerps the board's transform toward it. The lean quaternion rotates the up and right vectors around the forward axis by `_leanAngle`, producing the visual tilt. The slerp rate (12/s) is fast enough to track terrain changes without popping.

## Scene Structure

```
BoardController (CharacterBody3D)
├── skateboard_deck (GLB model)
│   ├── FrontTruckRayCast (offset along board nose)
│   └── RearTruckRaycast (offset along board tail)
├── CollisionShape3D (capsule at COM height)
├── CameraController (third-person follow)
└── AnimationPlayer (TurnLeft / TurnRight lean animations)
```

## Tuning Guide

| Parameter | Default | Effect |
|-----------|---------|--------|
| `MaxLeanAngleDeg` | 22 | Max turn sharpness. Higher = tighter turns possible |
| `LeanSpringRate` | 25 | Input responsiveness. Higher = snappier, lower = floatier |
| `MinSteerSpeed` | 3 | Allows turning at near-standstill. Set to 0 for dead zone at rest |
| `MaxSteerSpeed` | 25 | Caps steering sensitivity. Lower = less twitchy at terminal velocity |
| `FrontPivotAngleDeg` | 50 | Steeper = more responsive front truck. Real boards: 45-55 |
| `RearPivotAngleDeg` | 50 | Steeper = more responsive rear truck. Often lower than front for stability |
| `Wheelbase` | 0.35 | Longer = more stable, wider turn radius |
| `WobbleSpeedThreshold` | 18 | Lower = wobble kicks in sooner. Should be ~60% of expected terminal velocity |
| `WobbleMaxAmplitude` | 0.08 | Higher = scarier wobble. Beyond 0.15 becomes hard to recover |
| `JumpSpeed` | 5 | Vertical impulse in m/s. 5 gives roughly 1.25m peak height |
| `DragCoefficient` | 1.0 | Higher = lower terminal velocity. Tucking could reduce this |
| `RollingResistance` | 0.01 | Higher = more speed bleed on flat/uphill. Surface-type dependent |
