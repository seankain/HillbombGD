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

**Priority: Medium** — Adds depth to obstacle interaction.

The player can grind on rails, ledges, and certain edges by jumping onto them at an appropriate angle. Grinding locks the player to the grind path and maintains speed (or accelerates slightly downhill).

Implementation approach:
- Tag grindable surfaces with a `Grindable` group or metadata
- On collision/overlap with a grindable while airborne or at a shallow angle, enter grind state
- Grind state: project `_speed` along the grind path's tangent, lock lateral position to the path
- Balance mechanic: optional lean-to-balance during grinds (lean too far = bail)
- Exit: jump off, reach end of rail, or lose balance
- Grind paths can be defined as `Path3D` nodes attached to rail/ledge geometry

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

**Priority: High** — Required for the score-reset loop.

Differentiate between minor bumps and crashes based on impact force.

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
