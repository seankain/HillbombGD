# Traffic System Design

## Overview

The traffic system spawns NPC cars from an object pool, drives them along Path3D curves within terrain chunks, and returns them to the pool when they reach the end of their path. Cars serve as the primary mobile obstacle the player must dodge while skateboarding downhill.

A central `TrafficSimulator` prevents cars from clipping through each other by tracking per-path vehicle spacing. Cars transition seamlessly between paths at intersections and between chunks on straight lanes. Traffic lights at each intersection cycle between straight and cross-traffic phases, controlling both spawn timing and vehicle stopping behavior.

## Architecture

```
level.tscn
├── HillChunks (ChunkCycler)
│   │   └── TrafficSimulator (created at runtime)
│   ├── HillChunk (HillChunk)
│   │   ├── OnComingTrafficThruPath      (Path3D, X=3)
│   │   ├── OutgoigTrafficThruPath       (Path3D, X=-3)
│   │   ├── OncomingIntersectionRightPath (Path3D)
│   │   ├── OutgoingInersectionLeftPath  (Path3D)
│   │   ├── IntersectionLeftPath         (Path3D)
│   │   ├── IntersectionRightPath        (Path3D)
│   │   ├── Intersection
│   │   │   ├── OutgoingStraightTrafficSignal  (TrafficLight)
│   │   │   ├── IncomingStraightTrafficSignal  (TrafficLight)
│   │   │   ├── IntersectionRightTrafficSignal (TrafficLight)
│   │   │   ├── IntersectionLeftTrafficSignal  (TrafficLight)
│   │   │   └── TrafficLightController
│   │   └── TrafficSpawner              (TrafficSpawner)
│   ├── HillChunk2 ...
│   └── HillChunk3 ...
├── TrafficPool (TrafficPool)
│   └── NpcCar x20 (pre-instantiated, disabled)
├── BoardController (groups: ["Player"])
└── ...
```

The `TrafficPool` lives at the level root as a sibling to `HillChunks`. Each `HillChunk` contains a `TrafficSpawner` with exported references to that chunk's Path3D nodes, and a `TrafficLightController` wired to the four traffic signals in the intersection. `ChunkCycler` creates a `TrafficSimulator` at runtime and passes it (along with the `TrafficPool`) to each chunk's spawner during initialization.

## Components

### TrafficSimulator (`assets/Scripts/TrafficSimulator.cs`)

**Type:** `Node` — central collision-prevention manager, created by ChunkCycler at runtime.

**Key mechanics:**
- Maintains a `Dictionary<Path3D, List<NpcCar>>` tracking which cars are on which paths
- `RegisterCar(car, path)` / `UnregisterCar(car, path)` called by NpcCar on activation, path transfer, and reset
- `CanSpawnOnPath(path, progress)` — checked by TrafficSpawner before spawning; returns false if any car is within `MinFollowDistance` of the spawn point
- `IsBlockedByTraffic(car, path, progress)` — checked by NpcCar each physics frame; returns true if another car ahead is within `MinFollowDistance`, causing the car to hold position

**Export:** `MinFollowDistance = 6.0f` — minimum meters between cars on the same path.

### NpcCar (`assets/Scripts/NpcCar.cs`)

**Type:** `AnimatableBody3D` — kinematic body that reports velocity to colliding bodies.

**Scene:** `assets/Scenes/NpcCar.tscn` — uses `shitbox.glb` model with two `BoxShape3D` collision shapes.

**Key mechanics:**
- Creates a `PathFollow3D` node at `_Ready()` and reuses it across activations
- `Activate()` parents the PathFollow3D under the target Path3D, registers with simulator
- `_PhysicsProcess()`:
  1. Checks `ShouldStopAtLight()` — if the light is red/yellow and car is in the first 30% of the path, car stops
  2. Checks `IsBlockedByTraffic()` via the simulator — if a car ahead is too close, car holds position
  3. Otherwise advances `PathFollow.Progress += _speed * delta` and copies `GlobalTransform`
- When `ProgressRatio >= 1.0`, attempts path transition before falling back to sink
- `Reset()` unregisters from simulator, removes PathFollow3D from path, clears state

**Path transitions:**
- `TryTransitionToNextPath()` determines whether the current path is a straight thru lane or an intersection path
- Straight thru lanes → `TryTransitionToNextChunk()` — finds the same-named path on the neighboring chunk via `ChunkCycler.TryGetNeighborChunk()`
- Intersection paths → `TryTransitionToThruPath()` — finds the nearest ThruPath endpoint within 8m and transfers the car to it
- `TransferToPath()` handles the actual transfer: unregisters from old path, reparents PathFollow3D, registers on new path

### TrafficPool (`assets/Scripts/TrafficPool.cs`)

**Type:** `Node3D`, placed at the level root.

**Exports:**
- `PackedScene NpcCarScene` — the NpcCar.tscn scene
- `int PoolSize = 20` — number of pre-instantiated cars

**Lifecycle:**
- `_Ready()`: instantiates `PoolSize` cars as children, disables each, adds to `"MovableObstacles"` group, subscribes to `CarReachedSink` for automatic return
- `SetSimulator(TrafficSimulator)`: stores reference, passed to each car on checkout
- `Checkout()`: pops from `Stack<NpcCar> _available`, adds to `HashSet<NpcCar> _active`, calls `Enable()`, sets simulator. Returns null if pool is empty
- `Return(NpcCar)`: calls `Reset()` + `Disable()`, moves from active to available
- `ReturnAll()`: returns all active cars (called on player respawn)

### TrafficSpawner (`assets/Scripts/TrafficSpawner.cs`)

**Type:** `Node3D`, one per HillChunk.

**Exports:**
- `Path3D[] TrafficPaths` — wired to the chunk's Path3D nodes
- `float MinSpawnInterval = 2.0f`, `MaxSpawnInterval = 5.0f`
- `float MinCarSpeed = 8.0f`, `MaxCarSpeed = 15.0f`

**Behavior:**
- `Initialize(TrafficPool pool, TrafficSimulator simulator)`: stores pool and simulator references, seeds per-path spawn timers
- `SetLightController(TrafficLightController)`: stores light controller reference
- `SpawnTick(double delta)`: called from `HillChunk._Process()` only when `!Passed`. On timer expiry:
  1. Checks `simulator.CanSpawnOnPath()` — skips if another car is too close to spawn point
  2. Checks `lightController.GetStateForPath()` — skips if light is red
  3. Checks out a car, activates it, and passes the light controller to it

### TrafficLight (`assets/Scripts/TrafficLight.cs`)

**Type:** `Node3D` — attached to each traffic signal placeholder in the intersection.

**Scene:** `assets/Scenes/traffic_signal.tscn` — BoxMesh with TrafficLight script.

**Key mechanics:**
- Maintains a `TrafficLightState` enum (Green, Yellow, Red)
- `SetState(state)` updates the state and applies an emissive material color to the mesh
- Green = (0.1, 0.8, 0.1), Yellow = (0.9, 0.8, 0.1), Red = (0.9, 0.1, 0.1)

### TrafficLightController (`assets/Scripts/TrafficLightController.cs`)

**Type:** `Node3D` — one per intersection, child of the Intersection node.

**Exports:**
- `TrafficLight StraightOutgoing, StraightIncoming` — the two straight-direction signals
- `TrafficLight CrossRight, CrossLeft` — the two cross-traffic signals
- `float GreenDuration = 8.0f`, `YellowDuration = 2.0f`, `AllRedDuration = 1.0f`

**Phase cycle:** StraightGreen → StraightYellow → AllRed → CrossGreen → CrossYellow → AllRed → (repeat)

**Key method:** `GetStateForPath(Path3D)` — returns the current light state for a given path based on its name (ThruPath → straight signals, Intersection → cross signals). Used by both TrafficSpawner (spawn gating) and NpcCar (stop behavior).

## Integration with Existing Systems

### ChunkCycler (`assets/Scripts/ChunkCycler.cs`)

- `[Export] TrafficPool TrafficPool` field wired in `level.tscn`
- `_Ready()`: creates a `TrafficSimulator` as a child node, calls `TrafficPool.SetSimulator()`, iterates `ChunkPool` and calls `chunk.InitializeTraffic(TrafficPool, simulator)` on each
- `ChunkCycler_PlayerRespawned()`: calls `TrafficPool.ReturnAll()` before `ResetChunks()` to clear all traffic
- `TryGetNeighborChunk()`: used by NpcCar for cross-chunk path transitions on straight lanes

### HillChunk (`assets/Scripts/HillChunk.cs`)

- `[Export] TrafficSpawner Spawner` field wired in `HillChunk.tscn`
- `[Export] TrafficLightController LightController` field wired to `Intersection/TrafficLightController`
- `InitializeTraffic(TrafficPool pool, TrafficSimulator simulator)`: passes pool and simulator to spawner, sets light controller on spawner
- `CycleObstacles()`: calls `Spawner.ReturnAllCars()` — invoked by ChunkCycler before repositioning a chunk
- `_Process(double delta)`: calls `Spawner.SpawnTick(delta)` when `!Passed`

### ChunkTrigger (`assets/Scripts/ChunkTrigger.cs`)

- `BodyEntered`/`BodyExited` handlers filter by `body.IsInGroup("Player")` so NPC cars do not trigger chunk cycling

## Traffic Lane Behavior

| Path | Direction | Gameplay Effect |
|------|-----------|-----------------|
| `OnComingTrafficThruPath` (X=3) | Toward player | High closing speed (~25-40 m/s relative). Dodge-or-die |
| `OutgoigTrafficThruPath` (X=-3) | Same as player | Player approaches from behind, must weave around |
| `OncomingIntersectionRightPath` | Cross-traffic from right | Timed hazard windows at intersections, gated by traffic lights |
| `IntersectionLeftPath` | Cross-traffic from left | Timed hazard windows at intersections, gated by traffic lights |
| `OutgoingInersectionLeftPath` | Exits intersection left | Continuation path for cross-traffic |
| `IntersectionRightPath` | Exits intersection right | Continuation path for cross-traffic |

## Path Transition Behavior

### Intersection transitions (cross-traffic → thru lanes)
When a car on an intersection path (e.g. `OncomingIntersectionRightPath`) reaches `ProgressRatio >= 1.0`, it searches for the nearest ThruPath endpoint within 8 meters. If found, the car transfers to that ThruPath and continues driving, rather than being returned to the pool.

### Chunk transitions (straight lanes)
When a car on a ThruPath reaches the end, it looks for a neighboring chunk (via `ChunkCycler.TryGetNeighborChunk()`) in the appropriate direction (inbound paths look forward in Z, outbound paths look backward). If a neighbor exists and has a matching path, the car transfers to it and continues. Cars only sink to the pool when no transition is possible (e.g., at the world boundary).

## Traffic Light Behavior

Each intersection has a `TrafficLightController` that cycles through six phases:

| Phase | Duration | Straight Signals | Cross Signals |
|-------|----------|-----------------|---------------|
| StraightGreen | 8s | Green | Red |
| StraightYellow | 2s | Yellow | Red |
| AllRedBeforeCross | 1s | Red | Red |
| CrossGreen | 8s | Red | Green |
| CrossYellow | 2s | Red | Yellow |
| AllRedBeforeStraight | 1s | Red | Red |

**Spawn gating:** The spawner won't spawn cars on paths whose light is red.

**Vehicle stopping:** Cars in the first 30% of their path will stop if the light is red or yellow. Cars already past the 30% threshold continue through the intersection.

**Visual feedback:** Signal meshes change color via emissive materials, visible to the player as they approach the intersection.

## Collision Prevention

- **Spawn spacing:** `TrafficSimulator.CanSpawnOnPath()` prevents spawning a car within `MinFollowDistance` (6m) of any existing car on the same path
- **Follow distance:** `TrafficSimulator.IsBlockedByTraffic()` prevents a car from advancing if another car ahead on the same path is within `MinFollowDistance`
- **Traffic lights:** Red lights prevent spawning and stop early-path vehicles, creating natural gaps between cross-traffic and straight traffic
- NPC cars use `collision_layer = 4` (layer 3). Player-car collisions happen via `MoveAndSlide()`
- Cars do NOT physically collide with each other (spacing is managed logically by the simulator)

## Object Lifecycle

```
Pool._Ready()
  └─ Instantiate N cars → Disable() → push to _available

ChunkCycler._Ready()
  └─ Create TrafficSimulator → Pool.SetSimulator()
  └─ chunk.InitializeTraffic(pool, simulator) for each chunk
      └─ spawner.Initialize(pool, simulator)
      └─ spawner.SetLightController(lightController)

SpawnTick() timer expires
  └─ Simulator.CanSpawnOnPath() check
  └─ LightController.GetStateForPath() check
  └─ Pool.Checkout() → Enable() → SetSimulator() → add to _active
      └─ car.Activate(path, chunk, speed)
          └─ Simulator.RegisterCar(car, path)
          └─ PathFollow3D added as child of Path3D

Car._PhysicsProcess()
  └─ ShouldStopAtLight() → hold position if red/yellow and early in path
  └─ Simulator.IsBlockedByTraffic() → hold position if car ahead too close
  └─ Advance progress, copy transform

Car reaches end of path (ProgressRatio >= 1.0)
  ├─ TryTransitionToThruPath() → if intersection path, find nearby ThruPath
  ├─ TryTransitionToNextChunk() → if thru path, find same path on neighbor chunk
  └─ CarReachedSink → Pool.Return(car)
      └─ Simulator.UnregisterCar(car, path)
      └─ car.Reset() → remove PathFollow3D from path
      └─ car.Disable() → push back to _available

Chunk recycled (MoveChunk)
  └─ chunk.CycleObstacles() → Spawner.ReturnAllCars()

Player respawn
  └─ TrafficPool.ReturnAll() → return every active car
  └─ ResetChunks()
```

## Tuning Parameters

| Parameter | Default | Location | Effect |
|-----------|---------|----------|--------|
| `PoolSize` | 20 | TrafficPool | Max concurrent NPC cars |
| `MinSpawnInterval` | 2.0s | TrafficSpawner | Minimum time between spawns per path |
| `MaxSpawnInterval` | 5.0s | TrafficSpawner | Maximum time between spawns per path |
| `MinCarSpeed` | 8.0 m/s | TrafficSpawner | Slowest NPC car speed |
| `MaxCarSpeed` | 15.0 m/s | TrafficSpawner | Fastest NPC car speed |
| `MinFollowDistance` | 6.0 m | TrafficSimulator | Minimum gap between cars on same path |
| `GreenDuration` | 8.0s | TrafficLightController | Green phase length |
| `YellowDuration` | 2.0s | TrafficLightController | Yellow phase length |
| `AllRedDuration` | 1.0s | TrafficLightController | All-red clearance interval |

## File Index

| File | Purpose |
|------|---------|
| `assets/Scripts/NpcCar.cs` | Individual car: path following, collision avoidance, path transitions, light stops |
| `assets/Scripts/TrafficPool.cs` | Object pool: checkout, return, simulator wiring |
| `assets/Scripts/TrafficSpawner.cs` | Per-chunk spawner: timer-based spawning with simulator and light gating |
| `assets/Scripts/TrafficSimulator.cs` | Central collision prevention: per-path car tracking, spacing enforcement |
| `assets/Scripts/TrafficLight.cs` | Individual signal: state management, emissive color display |
| `assets/Scripts/TrafficLightController.cs` | Per-intersection light cycle: phase timing, state queries |
| `assets/Scenes/NpcCar.tscn` | Car scene: AnimatableBody3D + shitbox.glb + 2 box colliders |
| `assets/Scenes/traffic_signal.tscn` | Signal scene: Node3D + BoxMesh + TrafficLight script |
| `assets/Scripts/ChunkCycler.cs` | Chunk recycler: creates TrafficSimulator, initializes traffic system |
| `assets/Scripts/HillChunk.cs` | Chunk container: spawner + light controller exports, traffic initialization |
| `assets/Scripts/ChunkTrigger.cs` | Chunk boundary: player group filter |
| `assets/Scenes/HillChunk.tscn` | Chunk scene: paths, intersection, traffic signals, light controller |
| `assets/Scenes/level.tscn` | Level root: TrafficPool node, ChunkCycler export wiring |
