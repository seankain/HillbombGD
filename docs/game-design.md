# HillbombGD Game Design

## Concept

An arcade-style skateboarding game where the player bombs down an endless San Francisco hill, dodging obstacles and racking up distance-based score. Inspired by Top Skater's accessible feel but set in a city street environment with downhill momentum as the core tension.

## Core Loop

1. Player accelerates downhill automatically via gravity
2. Steer left/right to dodge obstacles, jump to clear low obstacles or gaps
3. Score increases with distance traveled
4. Significant collisions crash the player and reset the score
5. Speed wobble at high velocity creates risk/reward — faster = more points per second but harder to control

## Controls

| Input | Action |
|-------|--------|
| A / D | Lean left / right (steering) |
| Space | Jump / ollie |
| R | Reset / respawn |

Future inputs (not yet bound):
- Powerslide (scrub speed) - Can be performed up to a certain maximum speed threshold, any faster and the player has to find another way to slow down.
- Grab / tuck (reduce drag) - Push the the center of gravity forward and increase stability at the expense of turning speed.

## Scoring

Score increases proportional to distance traveled down the hill. Resets to zero on crash. No combo system or trick scoring in the base design — the game is about survival and speed management, not trick chains.

## Crash System

A crash occurs when a collision's impact force exceeds a threshold. Light bumps (glancing off a trash can, clipping a curb) should jostle the player but not end the run. Head-on collisions with cars, walls, or pedestrians at speed should crash.

The crash bail is implemented in `BoardController`: after `MoveAndSlide()`, each slide collision's closing speed (relative velocity into the surface) is scaled by mass to an impact force. Near-vertical contacts (floors, landings) are ignored so only roughly head-on impacts crash. Above `CrashImpactThreshold` the rider enters a bail — input and camera tracking are disabled for `CrashBailDuration` while the board settles, then `Respawn()` fires.

Design considerations:
- Impact force = relative velocity dot collision normal, scaled by mass
- Need a tunable threshold that feels fair — the player should always understand why they crashed
- Brief crash animation / ragdoll before respawn (animation not yet added)
- Score display on crash (high score tracking) (not yet added)

## Speed Management

The player has limited tools to control speed:
- **Steering**: turning bleeds some forward momentum into lateral movement
- **Powerslide** (planned): deliberate speed scrub at the cost of control
- **Air drag**: natural terminal velocity cap, but still fast enough to be dangerous
- **Speed wobble**: passive difficulty increase that pressures the player to manage speed

The game should never feel like the player needs to slow down to have fun. Speed wobble and obstacle density are the governors — the player chooses how much risk to accept.

## Obstacle Types

### Mobile / NPC (Planned)
- **Cars**: moving along cross streets or parked. Largest collision threat
- **Pedestrians**: walking on sidewalks, occasionally crossing. Smaller hitbox but still crash-worthy at speed
- **Other skaters / cyclists**: moving in the same direction, slower. Minor collisions = bump, head-on = crash

### Static Props (Planned)
- **Trash cans**: light, can be knocked over. Minor bump unless hit at high speed
- **Traffic cones**: very light, mostly visual. Knocked aside on contact
- **Fire hydrants / bollards**: fixed, solid. Crash on direct hit
- **Parked cars**: fixed, large. Crash or grind (if hitting edge at angle)

### Terrain Features (Partially Implemented)
- **Ramps / funboxes**: launch the player into the air. Currently in scene assets
- **Curbs**: small step that can be ollied or ridden off
- **Half pipes**: transition surfaces for wallrides (planned)
- **Rails / ledges**: grindable surfaces (planned)

## World Structure

The hill uses a chunk-cycling system (`ChunkCycler.cs`) that recycles terrain segments behind the player and repositions them ahead. This creates an endless descent without loading screens or level boundaries. Chunks are authored as individual scenes (`HillChunk.tscn`) with entry/exit triggers that drive the cycling.

Future considerations:
- Multiple chunk variants for visual variety
- Increasing obstacle density / speed with distance (difficulty curve)
- Distinct neighborhoods / visual themes as the player descends
- Periodic flat sections or uphill bumps as natural breathers
