# Traffic System Design

## Overview

The traffic system spawns NPC cars from an object pool, drives them along a directed waypoint graph within terrain chunks, and returns them to the pool when they reach a terminal waypoint. Cars serve as the primary mobile obstacle the player must dodge while skateboarding downhill.

A central `TrafficSimulator` prevents cars from clipping through each other using world-space proximity checks. Cars navigate between `TrafficWaypoint` nodes that form a directed graph with branching support — intersection waypoints can route cars straight through or onto turning lanes. Traffic lights at each intersection cycle between straight and cross-traffic phases, and cars stop at designated stop-line waypoints when the light is red.

## Architecture

```
level.tscn
├── HillChunks (ChunkCycler)
│   │   └── TrafficSimulator (created at runtime)
│   ├── HillChunk (HillChunk)
│   │   ├── OncomingEntry       (TrafficWaypoint, spawn, LaneId)
│   │   ├── OncomingStop        (TrafficWaypoint, stop line)
│   │   ├── OncomingMid         (TrafficWaypoint)
│   │   ├── OncomingExit        (TrafficWaypoint, LaneId)
│   │   ├── OutgoingEntry       (TrafficWaypoint, spawn, LaneId)
│   │   ├── OutgoingMid         (TrafficWaypoint)
│   │   ├── OutgoingStop        (TrafficWaypoint, stop line)
│   │   ├── OutgoingExit        (TrafficWaypoint, LaneId)
│   │   ├── CrossRightEntry     (TrafficWaypoint, spawn)
│   │   ├── CrossRightStop      (TrafficWaypoint, stop line)
│   │   ├── CrossRightCenter    (TrafficWaypoint, branches)
│   │   ├── CrossRightExit      (TrafficWaypoint, sink)
│   │   ├── CrossLeftEntry      (TrafficWaypoint, spawn)
│   │   ├── CrossLeftStop       (TrafficWaypoint, stop line)
│   │   ├── CrossLeftCenter     (TrafficWaypoint, branches)
│   │   ├── CrossLeftExit       (TrafficWaypoint, sink)
│   │   ├── Intersection
│   │   │   ├── OutgoingStraightTrafficSignal  (TrafficLight)
│   │   │   ├── IncomingStraightTrafficSignal  (TrafficLight)
│   │   │   ├── IntersectionRightTrafficSignal (TrafficLight)
│   │   │   ├── IntersectionLeftTrafficSignal  (TrafficLight)
│   │   │   └── TrafficLightController
│   │   └── TrafficSpawner
│   ├── HillChunk2 ...
│   └── HillChunk3 ...
├── TrafficPool (TrafficPool)
│   └── NpcCar x20 (pre-instantiated, disabled)
├── BoardController (groups: ["Player"])
└── ...
```

## Components

### TrafficWaypoint (`assets/Scripts/TrafficWaypoint.cs`)

**Type:** `Node3D` — a node in a directed graph placed at road-surface positions.

**Exports:**
- `TrafficWaypoint[] NextWaypoints` — directed edges to next waypoints. Multiple entries = branching (e.g., intersection center can route straight or turn).
- `bool IsSpawnPoint` — cars can be spawned here by the TrafficSpawner.
- `bool IsStopLine` — cars arriving here check the associated traffic light and stop if red/yellow.
- `TrafficLight StopLight` — which traffic light this stop line obeys.
- `string LaneId` — cross-chunk matching key (e.g., "OncomingThru", "OutgoingThru"). Set on boundary waypoints only.

### NpcCar (`assets/Scripts/NpcCar.cs`)

**Type:** `AnimatableBody3D` — kinematic body that reports velocity to colliding bodies.

**Key mechanics:**
- Navigates between waypoints using linear interpolation: `GlobalPosition = CurrentWaypoint.Lerp(TargetWaypoint, segmentProgress)`
- Rotation smoothed via quaternion Slerp between segment orientations
- On arrival at a waypoint:
  1. If `IsStopLine` and light is red/yellow → stop at the exact waypoint position
  2. If `NextWaypoints` has entries → pick one (random for branches) and advance
  3. If no next waypoints but `LaneId` is set → try cross-chunk transition
  4. Otherwise → `CarReachedSink` (return to pool)
- Checks `TrafficSimulator.IsBlocked()` each physics frame to maintain follow distance

### TrafficSimulator (`assets/Scripts/TrafficSimulator.cs`)

**Type:** `Node` — central collision-prevention manager.

**Key mechanics:**
- Maintains a `HashSet<NpcCar>` of all active cars
- `IsBlocked(car)` — returns true if any car is within `MinFollowDistance` ahead (world-space dot product check with car's forward direction)
- `CanSpawnAt(position)` — returns true if no active car is within `MinFollowDistance` of the position

### TrafficPool (`assets/Scripts/TrafficPool.cs`)

**Type:** `Node3D` — object pool of pre-instantiated cars.

- `Checkout()` → pop from available stack, enable, set simulator
- `Return(car)` → reset, disable, push back to available
- `ReturnAll()` → return all active cars (player respawn)

### TrafficSpawner (`assets/Scripts/TrafficSpawner.cs`)

**Type:** `Node3D` — per-chunk spawner.

**Exports:**
- `TrafficWaypoint[] SpawnPoints` — waypoints where cars can spawn
- `float MinSpawnInterval/MaxSpawnInterval` — timer range per spawn point
- `float MinCarSpeed/MaxCarSpeed` — speed range

**Behavior:** On timer expiry, checks `simulator.CanSpawnAt()` and stop-line light state, then spawns a car at the waypoint.

### TrafficLight (`assets/Scripts/TrafficLight.cs`)

**Type:** `Node3D` — individual signal with state and emissive color display.

### TrafficLightController (`assets/Scripts/TrafficLightController.cs`)

**Type:** `Node3D` — per-intersection light cycle.

**Phase cycle:** StraightGreen(8s) → StraightYellow(2s) → AllRed(1s) → CrossGreen(8s) → CrossYellow(2s) → AllRed(1s) → repeat.

Cars check lights via the `TrafficWaypoint.StopLight` reference — the controller just cycles the phases.

## Waypoint Graph Layout Per Chunk

### Oncoming lane (X=3, cars travel high Z → low Z)
```
OncomingEntry (3, -28, 122)  [spawn, LaneId="OncomingThru"]
  → OncomingStop (3, -29, 115)  [stop line, IncomingStraightTrafficSignal]
    → OncomingMid (3, -14, 50)
      → OncomingExit (3, 1, 0)  [LaneId="OncomingThru", cross-chunk exit]
```

### Outgoing lane (X=-3, cars travel low Z → high Z)
```
OutgoingEntry (-3, 1, 0)  [spawn, LaneId="OutgoingThru"]
  → OutgoingMid (-3, -14, 50)
    → OutgoingStop (-3, -27, 95)  [stop line, OutgoingStraightTrafficSignal]
      → OutgoingExit (-3, -28, 122)  [LaneId="OutgoingThru", cross-chunk exit]
```

### Cross-traffic from right (traveling -X, Z≈107)
```
CrossRightEntry (50, -29, 107)  [spawn]
  → CrossRightStop (8, -29, 107)  [stop line, IntersectionRightTrafficSignal]
    → CrossRightCenter (0, -29, 107)  [branches:]
      → CrossRightExit (-50, -29, 107)  [sink]
      → OncomingMid (3, -14, 50)  [turn onto oncoming lane]
```

### Cross-traffic from left (traveling +X, Z≈113)
```
CrossLeftEntry (-50, -29, 113)  [spawn]
  → CrossLeftStop (-8, -29, 113)  [stop line, IntersectionLeftTrafficSignal]
    → CrossLeftCenter (0, -29, 113)  [branches:]
      → CrossLeftExit (50, -29, 113)  [sink]
      → OutgoingStop (-3, -27, 95)  [turn onto outgoing lane]
```

## Cross-Chunk Transitions

When a car reaches a boundary waypoint (has `LaneId`, no `NextWaypoints`):
1. NpcCar asks `ChunkCycler.TryGetNeighborChunk()` for the adjacent chunk
2. `ChunkCycler.FindEntryWaypoint()` searches the neighbor for a waypoint with matching `LaneId` and `IsSpawnPoint`
3. Car transfers to that waypoint and continues driving

Direction is determined by the `LaneId`: names containing "Oncoming" → Inbound, others → Outbound.

## Stop Line Behavior

- Cars check the light only on arrival at a stop-line waypoint (`_segmentProgress >= 1.0`)
- If the light is red or yellow, the car stops at the exact waypoint position
- Cars already past the stop line continue through the intersection
- When the light turns green, stopped cars resume

## Collision Prevention

- `TrafficSimulator.IsBlocked()` — world-space forward dot-product check prevents advancing into a car ahead within `MinFollowDistance` (6m)
- `TrafficSimulator.CanSpawnAt()` — prevents spawning within `MinFollowDistance` of any active car
- Traffic lights create natural gaps between conflicting traffic streams
- NPC cars use `collision_layer = 4`. Player-car collisions happen via `MoveAndSlide()`

## Object Lifecycle

```
Pool._Ready()
  └─ Instantiate N cars → Disable() → push to _available

ChunkCycler._Ready()
  └─ Create TrafficSimulator → Pool.SetSimulator()
  └─ chunk.InitializeTraffic(pool, simulator) for each chunk

SpawnTick() timer expires
  └─ Simulator.CanSpawnAt(wp.GlobalPosition) check
  └─ Stop-line light check (skip if red)
  └─ Pool.Checkout() → Enable() → SetSimulator()
      └─ car.Activate(waypoint, chunk, speed)
          └─ Simulator.Register(car)
          └─ AdvanceToNextWaypoint()

Car._PhysicsProcess()
  └─ Stop-line check → hold at stop line if red/yellow
  └─ Simulator.IsBlocked() → hold if car ahead too close
  └─ Advance segmentProgress, lerp position, slerp rotation
  └─ On arrival: check stop line, pick next waypoint or cross-chunk

Car reaches terminal waypoint (no next, no cross-chunk)
  └─ CarReachedSink → Pool.Return(car)
      └─ Simulator.Unregister(car)
      └─ car.Reset() + Disable()

Chunk recycled → chunk.CycleObstacles() → Spawner.ReturnAllCars()
Player respawn → TrafficPool.ReturnAll()
```

## Tuning Parameters

| Parameter | Default | Location | Effect |
|-----------|---------|----------|--------|
| `PoolSize` | 20 | TrafficPool | Max concurrent NPC cars |
| `MinSpawnInterval` | 2.0s | TrafficSpawner | Minimum time between spawns per point |
| `MaxSpawnInterval` | 5.0s | TrafficSpawner | Maximum time between spawns per point |
| `MinCarSpeed` | 8.0 m/s | TrafficSpawner | Slowest NPC car speed |
| `MaxCarSpeed` | 15.0 m/s | TrafficSpawner | Fastest NPC car speed |
| `MinFollowDistance` | 6.0 m | TrafficSimulator | Minimum gap between cars |
| `GreenDuration` | 8.0s | TrafficLightController | Green phase length |
| `YellowDuration` | 2.0s | TrafficLightController | Yellow phase length |
| `AllRedDuration` | 1.0s | TrafficLightController | All-red clearance interval |

## File Index

| File | Purpose |
|------|---------|
| `assets/Scripts/TrafficWaypoint.cs` | Waypoint node: directed graph edges, spawn/stop-line/lane metadata |
| `assets/Scripts/NpcCar.cs` | Vehicle: waypoint-to-waypoint interpolation, stop-line stops, cross-chunk transitions |
| `assets/Scripts/TrafficSimulator.cs` | Collision prevention: world-space proximity + direction check |
| `assets/Scripts/TrafficPool.cs` | Object pool: checkout, return, simulator wiring |
| `assets/Scripts/TrafficSpawner.cs` | Per-chunk spawner: timer-based spawning at waypoints |
| `assets/Scripts/TrafficLight.cs` | Individual signal: state + emissive color |
| `assets/Scripts/TrafficLightController.cs` | Per-intersection light cycle: phase timing |
| `assets/Scenes/NpcCar.tscn` | Car scene: AnimatableBody3D + model + colliders |
| `assets/Scenes/traffic_signal.tscn` | Signal scene: Node3D + BoxMesh + TrafficLight script |
| `assets/Scripts/ChunkCycler.cs` | Chunk recycler: creates simulator, FindEntryWaypoint() |
| `assets/Scripts/HillChunk.cs` | Chunk container: spawner export, traffic initialization |
| `assets/Scenes/HillChunk.tscn` | Chunk scene: 14 waypoints, intersection, traffic signals |
| `assets/Scenes/level.tscn` | Level root: TrafficPool, ChunkCycler wiring |
