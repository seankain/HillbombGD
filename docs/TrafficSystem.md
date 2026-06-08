# Traffic System Design

## Overview

The traffic system spawns NPC cars from an object pool, drives them along Path3D curves within terrain chunks, and returns them to the pool when they reach the end of their path. Cars serve as the primary mobile obstacle the player must dodge while skateboarding downhill.

## Architecture

```
level.tscn
├── HillChunks (ChunkCycler)
│   ├── HillChunk (HillChunk)
│   │   ├── OnComingTrafficThruPath      (Path3D, X=3)
│   │   ├── OutgoigTrafficThruPath       (Path3D, X=-3)
│   │   ├── OncomingIntersectionRightPath (Path3D)
│   │   ├── IntersectionLeftPath         (Path3D)
│   │   └── TrafficSpawner              (TrafficSpawner)
│   ├── HillChunk2 ...
│   └── HillChunk3 ...
├── TrafficPool (TrafficPool)
│   └── NpcCar x20 (pre-instantiated, disabled)
├── BoardController (groups: ["Player"])
└── ...
```

The `TrafficPool` lives at the level root as a sibling to `HillChunks`. Each `HillChunk` contains a `TrafficSpawner` with exported references to that chunk's four Path3D nodes. `ChunkCycler` holds an `[Export]` reference to the `TrafficPool` and passes it to each chunk's spawner during `_Ready()` via `HillChunk.InitializeTraffic()`.

## Components

### NpcCar (`assets/Scripts/NpcCar.cs`)

**Type:** `AnimatableBody3D` — kinematic body that reports velocity to colliding bodies, giving natural impact when the player's `CharacterBody3D.MoveAndSlide()` encounters it.

**Scene:** `assets/Scenes/NpcCar.tscn` — uses `shitbox.glb` model with two `BoxShape3D` collision shapes (body + cabin).

**Key mechanics:**
- Creates a `PathFollow3D` node at `_Ready()` and reuses it across activations
- `Activate()` parents the PathFollow3D under the target Path3D, sets speed and initial progress
- `_PhysicsProcess()` advances `PathFollow.Progress += _speed * delta` and copies `GlobalTransform` from the PathFollow3D
- When `ProgressRatio >= 1.0`, fires `CarReachedSink` event (pool returns the car)
- `Disable()` sets `ProcessMode = Disabled`, hides visuals, disables all collision shapes
- `Enable()` reverses Disable
- `Reset()` removes PathFollow3D from its parent path, clears state

**Chunk repositioning:** The PathFollow3D is a child of the Path3D, which is a child of the HillChunk. When ChunkCycler moves a chunk, the PathFollow3D moves with it automatically. The NpcCar reads its transform from the PathFollow3D each physics frame, so no manual position adjustment is needed.

### TrafficPool (`assets/Scripts/TrafficPool.cs`)

**Type:** `Node3D`, placed at the level root.

**Exports:**
- `PackedScene NpcCarScene` — the NpcCar.tscn scene
- `int PoolSize = 20` — number of pre-instantiated cars

**Lifecycle:**
- `_Ready()`: instantiates `PoolSize` cars as children, disables each, adds to `"MovableObstacles"` group, subscribes to `CarReachedSink` for automatic return
- `Checkout()`: pops from `Stack<NpcCar> _available`, adds to `HashSet<NpcCar> _active`, calls `Enable()`. Returns null if pool is empty
- `Return(NpcCar)`: calls `Reset()` + `Disable()`, moves from active to available
- `ReturnAll()`: returns all active cars (called on player respawn)

### TrafficSpawner (`assets/Scripts/TrafficSpawner.cs`)

**Type:** `Node3D`, one per HillChunk.

**Exports:**
- `Path3D[] TrafficPaths` — wired to the chunk's four Path3D nodes
- `float MinSpawnInterval = 2.0f`, `MaxSpawnInterval = 5.0f`
- `float MinCarSpeed = 8.0f`, `MaxCarSpeed = 15.0f`

**Behavior:**
- `Initialize(TrafficPool pool)`: stores pool reference, seeds per-path spawn timers with random intervals
- `SpawnTick(double delta)`: called from `HillChunk._Process()` only when `!Passed`. Decrements per-path timers; on expiry, checks out a car and activates it at path start (progress = 0)
- `ReturnAllCars()`: returns all cars spawned by this spawner. Called by `HillChunk.CycleObstacles()` before chunk repositioning

The spawner does not run its own `_Process` — HillChunk controls when it ticks.

## Integration with Existing Systems

### ChunkCycler (`assets/Scripts/ChunkCycler.cs`)

- `[Export] TrafficPool TrafficPool` field wired in `level.tscn`
- `_Ready()`: iterates `ChunkPool` and calls `chunk.InitializeTraffic(TrafficPool)` on each
- `ChunkCycler_PlayerRespawned()`: calls `TrafficPool.ReturnAll()` before `ResetChunks()` to clear all traffic

### HillChunk (`assets/Scripts/HillChunk.cs`)

- `[Export] TrafficSpawner Spawner` field wired in `HillChunk.tscn`
- `InitializeTraffic(TrafficPool pool)`: passes pool to spawner
- `CycleObstacles()`: calls `Spawner.ReturnAllCars()` — invoked by ChunkCycler before repositioning a chunk
- `_Process(double delta)`: calls `Spawner.SpawnTick(delta)` when `!Passed`

### ChunkTrigger (`assets/Scripts/ChunkTrigger.cs`)

- `BodyEntered`/`BodyExited` handlers filter by `body.IsInGroup("Player")` so NPC cars do not trigger chunk cycling
- `BoardController` is in the `"Player"` group (set in `BoardController.tscn`)

## Traffic Lane Behavior

| Path | Direction | Gameplay Effect |
|------|-----------|-----------------|
| `OnComingTrafficThruPath` (X=3) | Toward player | High closing speed (~25-40 m/s relative). Dodge-or-die |
| `OutgoigTrafficThruPath` (X=-3) | Same as player | Player approaches from behind, must weave around |
| `OncomingIntersectionRightPath` | Cross-traffic | Timed hazard windows at intersections |
| `IntersectionLeftPath` | Cross-traffic | Timed hazard windows at intersections |

## Collision Setup

- NPC cars use `collision_layer = 4` (layer 3) in the scene file. To enable player-car collisions, they should share a layer with the player or have their mask configured so the player's `MoveAndSlide()` detects them
- Cars do NOT collide with each other (path-driven; spacing handled by spawn timers)
- Cars do NOT collide with terrain (they follow paths)
- `AnimatableBody3D` automatically computes and reports velocity to colliding `CharacterBody3D` nodes

## Object Lifecycle

```
Pool._Ready()
  └─ Instantiate N cars → Disable() → push to _available

SpawnTick() timer expires
  └─ Pool.Checkout() → pop _available → Enable() → add to _active
      └─ car.Activate(path, chunk, speed)
          └─ PathFollow3D added as child of Path3D
          └─ _PhysicsProcess advances progress each frame

Car reaches end of path (ProgressRatio >= 1.0)
  └─ CarReachedSink event → Pool.Return(car)
      └─ car.Reset() → remove PathFollow3D from path
      └─ car.Disable() → push back to _available

Chunk recycled (MoveChunk / MoveChunkToPosition)
  └─ chunk.CycleObstacles() → Spawner.ReturnAllCars()
      └─ Pool.Return() for each active car on that chunk

Player respawn
  └─ TrafficPool.ReturnAll() → return every active car
  └─ ResetChunks()
```

## Tuning Parameters

| Parameter | Default | Location | Effect |
|-----------|---------|----------|--------|
| `PoolSize` | 20 | TrafficPool | Max concurrent NPC cars. 3 chunks × 4 paths × ~2 cars = 24 max demand; 20 means some lanes occasionally go empty |
| `MinSpawnInterval` | 2.0s | TrafficSpawner | Minimum time between spawns per path |
| `MaxSpawnInterval` | 5.0s | TrafficSpawner | Maximum time between spawns per path |
| `MinCarSpeed` | 8.0 m/s | TrafficSpawner | Slowest NPC car speed |
| `MaxCarSpeed` | 15.0 m/s | TrafficSpawner | Fastest NPC car speed |

## File Index

| File | Purpose |
|------|---------|
| `assets/Scripts/NpcCar.cs` | Individual car: path following, enable/disable, collision |
| `assets/Scripts/TrafficPool.cs` | Object pool: checkout, return, lifecycle management |
| `assets/Scripts/TrafficSpawner.cs` | Per-chunk spawner: timer-based spawning on Path3D lanes |
| `assets/Scenes/NpcCar.tscn` | Car scene: AnimatableBody3D + shitbox.glb + 2 box colliders |
| `assets/Scripts/ChunkCycler.cs` | Modified: TrafficPool export, initialization, respawn cleanup |
| `assets/Scripts/HillChunk.cs` | Modified: Spawner export, CycleObstacles, _Process tick |
| `assets/Scripts/ChunkTrigger.cs` | Modified: Player group filter on body enter/exit |
| `assets/Scenes/BoardController.tscn` | Modified: added to "Player" group |
| `assets/Scenes/HillChunk.tscn` | Modified: TrafficSpawner child node with Path3D exports |
| `assets/Scenes/level.tscn` | Modified: TrafficPool node, ChunkCycler export wiring |
