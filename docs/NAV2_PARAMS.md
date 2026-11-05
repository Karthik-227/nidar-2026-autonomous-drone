# Our nav2_params.yaml, Explained

**NIDAR 2026–27 · Air Mouse — this is not a Nav2 tutorial.** `CONCEPTS.md`
covers what a costmap or a controller *is* in general. This document walks
through **our actual file**, section by section, with the real numbers we
fly and the reasons behind each one. If you only read one section, read §2
— it explains the controller swap that just happened and why.

---

## 1. The shape of the problem this file is tuned for

Every number in this file answers to the same arena geometry, so it's worth
fixing in your head before anything else:

- **Corridors are 1.00 m wide**, walls 0.10 m thick, cells on a 1.10 m pitch.
- **The drone's collision radius is 0.25 m** (`robot_radius`, set in four
  places that must all agree — the two costmaps here, `frontier_explorer.py`,
  and the Gazebo collision body). The real airframe sweeps 0.240 m; 0.25
  keeps 10 mm of margin.
- That leaves the drone's **centre a ±0.22–0.25 m band** to work with in any
  corridor. There is no "far from the wall" available — only "centred."
- The drone is a **multirotor, not a car**: it can move sideways without
  turning (holonomic), and turning costs it nothing extra the way it would
  cost a ground robot — but it also means every controller choice here has
  to be told explicitly whether it's allowed to use that sideways freedom.

Almost every "why is this number X and not Y" in this file traces back to
one of these four facts.

---

## 2. `controller_server` — DWB, and why it replaced two other controllers

This is the node that turns "the plan is this line on the map" into
"command this velocity, right now." It's the only part of this file that's
changed controller *technology* more than once this project — so it's worth
being honest about that history rather than pretending DWB was always the
answer.

### What ran before

- **MPPI** (`nav2_mppi_controller`) — samples ~1000–2000 random possible
  futures every control cycle, scores each one, and averages the good ones
  into a command. Powerful, but its own sampling noise showed up as a
  visible wander on a dead-straight corridor, and its cost math doesn't
  translate to any other controller's numbers.
- **RegulatedPurePursuit (RPP)** — steers toward a point a fixed distance
  ahead on the path ("the carrot"). Simple and fast, but it is fundamentally
  a **car-following** algorithm: its whole output is (forward speed, turn
  rate), so it makes the vehicle *turn to face* where it's going. A
  multirotor doesn't need to turn to go sideways, and the turning it does to
  satisfy RPP has real lag a car's steering doesn't — which showed up as
  genuine oscillation around corners.

### What runs now: DWB, nav2's own default

`dwb_core::DWBLocalPlanner`. Simpler than MPPI (it doesn't do random
sampling — it tries a fixed grid of velocity combinations and scores each
one against a handful of named "critics"), and unlike RPP it can be told to
move in any direction **without turning at all**:

```yaml
min_vel_theta: 0.0
max_vel_theta: 0.0
vtheta_samples: 1
```

Yaw is locked to exactly zero — on both axes, with only one value to even
try. This has to be paired with ArduPilot's own `WP_YAW_BEHAVIOR 0`
(in `nidar_tuning.parm`): if the autopilot still tries to yaw to face its
velocity while Nav2 never planned for that, the body rotates underneath a
command that assumed it wouldn't, and that's the exact feedback loop that
caused the earlier corner oscillation.

**Translation limits**, both axes, since the vehicle can strafe:

```yaml
min_vel_x: -0.40   max_vel_x: 0.56
min_vel_y: -0.56   max_vel_y: 0.56
max_speed_xy: 0.56     # the vector cap, not a per-axis one
```

0.56 m/s is 70% of this project's earlier 0.8 m/s cruise speed — a
deliberate, requested slow-down, not a controller limitation.

**How far ahead it looks** (`sim_time: 1.2`) is sized against the arena, not
guessed: at 0.56 m/s a 1.2 s rollout reaches 0.67 m, comfortably inside the
1.10 m cell pitch. Make this too long and a simulated trajectory can end
*inside a wall at the next corner* before the planner even gets to react —
the same geometry trap that made the old MPPI tuning crawl.

**The critics actually used:**

```yaml
critics: ["Oscillation", "BaseObstacle", "PathDist", "GoalDist"]
```

Four, deliberately, out of DWB's stock seven. The three left out —
`RotateToGoal`, `GoalAlign`, `PathAlign` — all score how well the robot's
**heading** matches something (the goal direction, or the path's tangent).
A vehicle that is locked at zero yaw cannot act on a heading score no matter
how it's weighted, so keeping them would only add noise to which trajectory
wins. `Oscillation` is kept on purpose — it's nav2's own anti-wobble
critic, penalising a trajectory that keeps reversing direction, and it's
aimed directly at the thing earlier tuning rounds spent real effort fighting.

`BaseObstacle` outranks `PathDist` (0.03 vs 20.0 — DWB's critic scales
aren't on the same numeric footing as weights, so don't compare them to
MPPI's numbers by eye) for the same reason obstacle cost outranked path
alignment under every earlier controller too: the global path is a route
suggestion computed on a map that's one SLAM update stale; the live
costmap is what the lidar can see *right now*. Where they disagree, the
lidar should win.

**Honest caveat:** this is a fresh swap, done in one pass. The weights are
reasoned starting points, not numbers validated against real telemetry the
way the MPPI tuning was (that one went through several measured rounds —
see the git history on this file for that story). Expect to retune after
the first few real flights.

---

## 3. Costmaps — what "wall" actually means to the software

Two costmaps, and they are **not interchangeable**, even though they build
from similar pieces:

| | `local_costmap` | `global_costmap` |
|---|---|---|
| Frame | `odom` (moves with the drone) | `map` (fixed) |
| Window | 5 × 5 m, rolling | the whole known map |
| Plugins | `static_layer`, `obstacle_layer`, `inflation_layer` | `static_layer`, `inflation_layer` |
| Job | what the controller steers on *right now* | what the planner routes a path through |

### Why the local costmap has a layer the global one doesn't

`obstacle_layer` reads the **live** lidar scan. Frontier exploration's
entire purpose is flying into space nothing has mapped yet — and in that
space, the static map (`/map_filtered`, built from SLAM) has nothing to
say. The live obstacle layer is the only thing standing between the drone
and a wall that exists *right now* but hasn't gone through the
SLAM → filter → static-layer pipeline yet (measured at roughly 400 ms
end to end). Remove it, and the drone flies blind for that whole window on
every frontier approach.

It's correctly **absent** from the global costmap, for the opposite
reason: the global obstacle layer marks straight from the raw scan,
bypassing `map_filter`'s phantom-rejection entirely, and because its own
clearing range is short, anything it marks beyond that range is never
un-marked. A phantom speckle there is permanent. The global costmap trusts
only the filtered, SLAM-confirmed map.

### `robot_radius: 0.25` — the number that draws the "you cannot be here" ring

This single number, repeated in four files, is what Nav2 calls the
**inscribed ring**: a band exactly `robot_radius` wide around every known
obstacle where the drone's *centre point* may not legally be, because if it
were, the body would be overlapping the wall. With
`consider_footprint: false` (both DWB and the earlier MPPI run this way —
checking the robot's full rotated outline on every sampled trajectory is
too expensive to run in real time), this ring **is** the entire collision
check. Shrinking it doesn't make the corridor wider in reality — it makes
the software stop noticing when the drone won't fit.

### `inflation_radius` / `cost_scaling_factor` — the part that is advice, not law

Beyond the inscribed ring, cost falls off smoothly:

```
cost(d) = 252 × exp( −cost_scaling_factor × (d − robot_radius) )
```

This band **blocks nothing** — its only job is to make "close to a wall"
cost more than "middle of the corridor," so the planner and controller
*prefer* the middle. The counter-intuitive part: `cost_scaling_factor` is a
**decay rate**, not a strength. A smaller number looks like it should mean
"stronger repulsion," but it actually *flattens* the gradient — and a flat
gradient is one the controller can't find the middle of. We measured this
directly once: at `cost_scaling_factor: 3`, the gradients from both walls
of a 1.0 m corridor overlapped across the *entire* width, so there was no
zero-cost cell anywhere to aim for. At `cost_scaling_factor: 12`
(the current value) the corridor genuinely has a free middle again.

### `track_unknown_space`

Two different flags with the same name, doing different jobs, both present
on purpose:

- **`static_layer.track_unknown_space`** — `true` on the global costmap
  (unexplored means unexplored — useful for the planner to know), **`false`**
  on the local one. If it were `true` there, every unmapped cell would read
  as a hard collision to the controller's cost critics, and since a
  frontier goal sits *at the edge of unknown space by definition*, every
  single approach to a frontier would score as driving into a wall.
  Exploration would be structurally impossible.
- **The costmap's own top-level `track_unknown_space`** — a separate switch
  on the whole merged grid, set explicitly rather than left to whatever a
  given nav2 build happens to default to.

---

## 4. `planner_server` — the long-range router

`NavfnPlanner`, running Dijkstra/A* over the global costmap, producing the
green line you see in RViz. It only has to be right about the *route*; the
controller (§2) is what actually keeps the drone off walls along the way.
`allow_unknown: true` lets it route through cells nothing has seen yet —
necessary for the same reason as the local costmap's `track_unknown_space`
above: a frontier goal is unknown territory by definition, and a planner
that refuses to touch unknown cells would refuse most frontier goals too.

## 5. `behavior_server` — what happens when the controller gives up

Runs the recovery behaviors (`spin`, `backup`, `drive_on_heading`, `wait`,
`assisted_teleop`) that the navigation behaviour tree falls back to when
the controller reports it's stuck. `simulate_ahead_time: 1.0` matters more
than it looks: this is how far ahead the behavior server checks before
agreeing to run a recovery at all. Set too long (it was measured at 2.0 at
one point), the forward-check sweeps far enough that in a 1.0 m corridor
*something* is always in the way — every recovery refused instantly with
"Collision Ahead," leaving the drone stuck in the exact state the
controller had just failed in, with no path out.

Reads `local_costmap/costmap_raw` rather than the usual display topic — the
full-resolution internal cost values (0–254), not the 0–100 scale used for
RViz. The behavior server needs the real numbers to check a backup or spin
maneuver accurately; the display topic is lossy on purpose, for rendering.

## 6. `velocity_smoother` — the last word before the bridge

Sits between whichever controller is active and `cmd_vel_bridge.py`,
rate-limiting acceleration regardless of what the controller asked for.
Every number here is set with headroom above the controller's own limits —
`max_velocity`/`max_accel` higher than DWB's, matching ArduPilot's own
`WP_ACC`/`WP_SPD`, which are in turn set below the autopilot's hard lean
ceiling (`ATC_ANGLE_MAX`). That's deliberate: a clipped command isn't a
safe no-op, it's a *different* command than the one the controller
computed, and the controller then fights its own clipped output — which is
indistinguishable from the oscillation this whole file has spent its
history trying to remove.

## 7. What's deliberately *not* in this file anymore

- **AMCL** — a particle-filter localizer for a *given, static* map. This
  project localizes continuously via SLAM instead, so AMCL was configured
  but never started by any launch file. Dead weight, removed.
- **`robot_state_publisher`** — normally parses a URDF to publish the fixed
  transforms between a robot's body and its sensors. This project has no
  URDF; ArduPilot's own DDS client publishes `tf`/`tf_static` directly.
  Nothing ever started this node either. Removed.

Both were leftovers from the stock nav2 example this file was originally
built from — harmless to leave, but also nothing lost by taking them out,
and one less thing to wonder about when reading the file.
