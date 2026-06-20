# Future Features

## Powerslide / Speed Scrub

**Priority: High** — Core to the game's speed management loop.

The player holds a powerslide input to enter a sideways slide that bleeds speed through increased friction. During a powerslide the board yaws 60-90 degrees from the heading while continuing to travel roughly forward.

Implementation approach:
- New state: `_isSliding` bool, `_slideYawOffset` float
- While held: apply a large yaw offset to the board's visual orientation and increase rolling resistance by 5-10x
- Speed decays faster but steering is reduced (committed to the slide direction)
- Release snaps the board back to heading direction
- Visual: wheel dust particles, slide sound effect
- The `_speed` variable already isolates forward velocity, so the slide is purely a friction multiplier + visual rotation

Key design question: should powerslide be a binary toggle (hold button) or analog (lean angle during slide affects braking force)?

## Grind System

**Status: Implemented.** Remaining: authoring grindable rails into hill chunks, grind sound, and score integration.

The player can grind on rails, ledges, and certain edges by riding onto them along their length. Grinding locks the player to the grind path and accelerates slightly downhill while a balance mini-game runs.

Implemented in `BoardController` (`TryEnterGrind` / `EnterGrind` / `UpdateGrind` / `ExitGrind` / `BailGrind`):
- Grindable geometry is tagged with the `Grindable` group. The rail line is a `Path3D`, resolved from the grindable node via a `grind_path` NodePath metadata, by the grindable being a `Path3D` itself, or as a descendant `Path3D`.
- Entry: after `MoveAndSlide`, slide collisions are scanned for a grindable. The board latches on if it has at least `GrindMinEntrySpeed` and is travelling within `GrindMaxAlignmentAngleDeg` of the rail tangent (so you must approach along the rail, not across it). Grind entry takes priority over crash detection.
- Grind state: the board is pinned to the curve (`Curve3D.GetClosestOffset` / `SampleBaked`) at `GrindHeightOffset` above the rail, `_speed` is driven along the tangent (gravity downhill minus `GrindFriction`, floored at `GrindMinSpeed`), and `_yaw` is set from the tangent so exit momentum is correct.
- Balance: `_grindBalance` tilts with an instability term (`GrindBalanceInstabilityDeg`) plus random noise (`GrindBalanceNoiseDeg`); the player counters with lean (`GrindBalanceControl`). Exceeding `GrindBalanceLimitDeg` bails into a crash.
- Exit: jump off (`ExitGrind(true)` adds `JumpSpeed`), reach either end of the rail, or lose balance (`BailGrind` → `EnterCrash`). A `GrindReentryCooldown` prevents instantly re-latching.

To author a grindable surface:
1. Add a body with collision (e.g. `StaticBody3D`) for the rail geometry and put it in the `Grindable` group.
2. Add a `Path3D` describing the rail line (as a child of the grindable, or referenced via a `grind_path` metadata NodePath).

Surfaces to grind:
- Stair rails (straight, downhill)
- Curb edges (low, short)
- Planter ledges (medium height, longer)
- Parked car bumpers (short, angled)

## Wallride

**Priority: Medium** — Natural extension of the board's terrain-following.

The player can transition onto steep wall surfaces by approaching at an angle with sufficient speed. The board rides along the wall briefly before gravity pulls it back down.

Implementation approach:
- The existing surface normal system already handles arbitrary slope angles
- Wallride entry: detect when the surface normal is nearly horizontal (wall) and the player has lateral velocity toward it
- During wallride: `_speed` decays faster (fighting gravity laterally), surface normal drives orientation
- Exit: speed drops below threshold, player jumps off, or wall ends
- The `ComputeSlopeAngle()` function already handles vertical-ish surfaces correctly (returns ~90 degrees), so gravity will naturally slow the rider

The main challenge is entry detection — the transition from ground to wall needs to feel smooth, not like hitting a wall. May need a brief slerp of the movement direction onto the wall plane.

## Crash / Collision Response

**Status: Implemented (crash bail).** Remaining: tuning, bump response, animation/ragdoll, score display.

Differentiate between minor bumps and crashes based on impact force.

The crash bail is implemented in `BoardController` (`CheckForCrash` / `EnterCrash` / `UpdateCrash`): hard horizontal impacts above `CrashImpactThreshold` disable board and camera control for `CrashBailDuration` before respawning. Still open: a sub-threshold "bump" response (speed reduction / yaw perturbation) and a crash animation / score display.

Implementation approach:
- Override `_PhysicsProcess` to check collision info from `MoveAndSlide()` via `GetSlideCollisionCount()` / `GetSlideCollision()`
- Compute impact force: `collision.GetTravel().Length() * _speed` or use the normal component of relative velocity
- Below threshold: nudge the player (brief speed reduction, slight yaw perturbation)
- Above threshold: trigger crash state
- Crash state: disable input, play crash animation/ragdoll, display score, respawn after delay
- The `IRespawnablePlayer` interface and `PlayerRespawned` event already support the respawn flow

Tuning: the crash threshold needs to feel fair. Players should be able to brush past obstacles at glancing angles without crashing, but head-on hits at speed should always crash.

## Tuck / Draft

**Priority: Low** — Nice-to-have for speed management variety.

The player can tuck (crouch) to reduce frontal area and drag coefficient, increasing terminal velocity. This is the counterpoint to powerslide — powerslide slows you down, tuck speeds you up.

Implementation approach:
- While held: reduce `FrontalArea` and `DragCoefficient` by ~40%
- Trade-off: reduced steering responsiveness while tucked (lower `MaxLeanAngleDeg` or `LeanSpringRate`)
- Visual: rider crouches on the board, lower camera angle

## Obstacle Spawning / Traffic System

**Priority: High** — Required for the game to have meaningful gameplay.

Spawn cars, pedestrians, and props within hill chunks as the player progresses.

Implementation approach:
- Each `HillChunk` defines spawn points / lanes for different obstacle types
- `ChunkCycler` populates obstacles when recycling a chunk to the front
- Obstacle density increases with distance (difficulty curve)
- Cars follow lane paths at set speeds using `PathFollow3D`
- Pedestrians walk on sidewalks with simple patrol behavior
- Static props are placed at authored positions within chunks

## Score / UI

**Priority: High** — Core feedback loop.

- Distance-based score displayed in HUD (existing `DebugHud` can be extended)
- High score persistence (save to file)
- Speed display (speedometer or just a number)
- Score multiplier for sustained high speed (ties into risk/reward of wobble)

## Audio

**Priority: Medium** — Significant feel improvement.

- Wheel roll sound (pitch scales with speed)
- Wind/rush sound at high speed
- Grind sound (metal scrape)
- Powerslide sound (wheel screech)
- Impact sounds (bump vs crash)
- Wobble sound (rhythmic rattle at wobble frequency)

## Camera Improvements

**Priority: Low** — Current camera works but could feel better.

- FOV increase with speed (sense of velocity)
- Camera pull-back at high speed (wider view for obstacle reading)
- Slight camera shake during wobble
- Impact shake on bumps
- Smooth transition between ground and air camera behavior
