# NIDAR tuning parameters — where they live and what binds

Every value below was read off the files in this package, not remembered.
Regenerate the tables the same way if you doubt one:

```bash
grep -n "<key>" config/nav2_params.yaml config/nidar_tuning.parm
grep -n "declare_parameter('<key>'" nidar_sim/*.py
```

**Line numbers are deliberately not listed.** They were, and every one of them
rotted the first time a comment was added. Search by key name instead — the
keys are unique within their file.

**`install/` is what runs, not `src/`.** Config files are *copied* into the
install tree by `setup.py`'s `data_files`, not symlinked, so `--symlink-install`
does not help them. Every edit here needs

```bash
colcon build --packages-select nidar_sim --symlink-install
```

and a restart of the node that reads it. ArduPilot params (`nidar_tuning.parm`)
are read **at SITL boot** — a Nav2 relaunch will not pick them up.

---

## A. Speed and acceleration

### A1 — MPPI, `config/nav2_params.yaml`, `controller_server.FollowPath`

| Param | Now | What it does |
|---|---|---|
| `time_steps` × `model_dt` | 24 × 0.05 = **1.20 s** | Rollout horizon. **This is the real speed limit in a maze** — see "what binds" below |
| `vx_max` / `vy_max` | 1.00 | Speed ceilings. Not binding; the horizon settles cruise at ~0.85 m/s |
| `vx_min` | −0.70 | Reverse ceiling, used when backing out |
| `wz_max` | **0.0** | Yaw pinned off. Must stay 0 while `WP_YAW_BEHAVIOR` is 0 — don't raise it |
| `ax_max` / `ay_max` | 1.50 | Per-axis accel. Diagonal 2.12 m/s² → **12.2° lean**. This is the binding accel |
| `ax_min` | −1.50 | Braking ceiling — raise this alone if stopping feels mushy |
| `az_max` | 1.5 | Vertical; mostly inert, the bridge owns altitude |
| `vx_std` / `vy_std` | 0.14 / 0.08 | Sampling cloud width. Sized in **metres over the horizon** (0.168 / 0.096 m), not in isolation — rescale both if `time_steps` changes |
| `gamma` | 0.03 | Control-effort penalty. Effective weight is `gamma/std²`; at `vx_std` 0.14 that is 1.53 |
| `temperature` | 0.25 | Softmax sharpness. Higher blends more mediocre samples — that blend is what wobbles when two dodge directions score alike |
| `prune_distance` | 1.7 | Path kept ahead. Must exceed rollout length (1.20 m) plus PathFollow's carrot |
| `batch_size` / `iteration_count` | 1000 / 1 | Samples per cycle, re-optimisation passes |
| `controller_frequency` | 20.0 | Replan rate (`controller_server`) |

### A2 — Velocity smoother, `velocity_smoother`

| Param | Now | What it does |
|---|---|---|
| `max_velocity` / `min_velocity` | [1.00, 1.00, 0.3] / [−0.70, −0.70, −0.3] | Matched to MPPI. Not binding |
| `max_accel` / `max_decel` | ±[1.80, 1.80, 1.0] | Slack above MPPI's 1.50 so it never clips |
| `velocity_timeout` | 0.25 | Zeroes output if MPPI goes quiet |
| `deadband_velocity` | [0, 0, 0] | Commands below this become zero |

### A3 — Bridge, `nidar_sim/cmd_vel_bridge.py`

| Param | Now | What it does |
|---|---|---|
| `max_linear_vel` | 1.50 | Final **vector** clamp. Must exceed Nav2's diagonal (1.41) or it rotates the command |
| `target_altitude_m` | 1.17 | Cruise height. Changing it means re-deriving `slam.launch.py`'s `max_hit_height_agl_m` and `max_clear_elev_deg` |
| `altitude_gain` / `max_climb_rate` | 0.8 / 0.4 | P-gain on height hold and its rate cap |
| `command_timeout_sec` | 0.3 | Deadman: stop if `/cmd_vel` goes quiet |
| `out_topic` | `/ap/cmd_vel` | **Not** `/ap/v1/cmd_vel`. Verified in `AP_DDS_Topic_Table.h` |
| `negate_lateral_for_frd` | True | ROS is FLU, `ahrs.body_to_earth()` expects FRD. Measured, not theorised |

### A4 — ArduPilot, `config/nidar_tuning.parm` (loaded at SITL boot)

| Param | Now | What it does |
|---|---|---|
| `WP_SPD` | 2.0 | Speed cap, **vector**. Slack above Nav2's 1.41 diagonal |
| `WP_ACC` | 2.6 | Accel cap, **vector**. Slack above the smoother's 2.55 diagonal |
| `WP_ACC_CNR` | 2.6 | Corner accel. Latent — only the S-curve tracker reads it, which GUIDED velocity control never runs |
| `ATC_ANGLE_MAX` | 18 | Lean ceiling → 3.19 m/s². Firmware floor is 10 (silently clamped below that) |
| `PSC_ANGLE_MAX` | 0.0 | Inherits `ATC_ANGLE_MAX`. One place to set the ceiling |
| `PSC_NE_JERK` | 8.0 | How fast lean comes on. Reaches 2.6 m/s² in 0.33 s |
| `ATC_INPUT_TC` | 0.20 | Attitude smoothing — the damping knob. Raise to 0.25 if flight looks twitchy |
| `WP_YAW_BEHAVIOR` | **0** | Mandatory while `wz_max` is 0. SITL defaults to 2, which yaws to face velocity and feeds the weaving loop |

### What actually binds

| Limit | Value | Owner |
|---|---|---|
| **Cruise speed** | **~0.92 m/s** | corridor ÷ horizon: 1.10 m cell ÷ 1.20 s. *Not* `vx_max` |
| Acceleration | 2.12 m/s² diagonal | MPPI `ax_max` |
| Steady lean | 12.2° | consequence of `ax_max` |
| Transient lean | 18° | `ATC_ANGLE_MAX` |

> The ladder is monotonic by design: MPPI 2.12 → smoother 2.55 → `WP_ACC` 2.6 →
> `ATC_ANGLE_MAX` 3.19. **Exactly one layer decides.** A clip is not a safe
> no-op — ArduPilot clipping a Nav2 command flies a vector Nav2 never asked for,
> and Nav2 then corrects against its own clipped output.
>
> To go faster, shorten `time_steps`. Raising any speed or accel ceiling will
> do nothing, because none of them is the binding constraint.

---

## B. Behaviour near walls

### B1 — Costmap geometry

| Param | Now | What it does |
|---|---|---|
| `robot_radius` (both costmaps) | **0.28** | Radius marked INSCRIBED (253). Must equal `frontier_explorer.robot_radius_m` and the Gazebo `base_collision` radius. True swept radius is 0.240 m, so this carries 40 mm of margin |
| `inflation_radius` (both) | 0.75 | Gradient extent beyond the inscribed ring. In a 1.0 m corridor it is capped by geometry at 0.50; the rest only matters in rooms and junctions |
| `cost_scaling_factor` (both) | 10.0 | **Decay rate, not strength.** Lowering it raises cost everywhere but *shrinks* the centre-vs-off-centre contrast that actually steers. Measured: k=10 gives 3.3× contrast, k=4 gives 1.5× |
| `resolution` | 0.05 | Quantises every radius above to 5 cm steps |
| `width` × `height` (local) | 5 × 5 | Rolling window |
| `update_frequency` | 10.0 local / 1.5 global | Freshness when MPPI scores against it |
| `always_send_full_costmap` | True | Visualisation only — without it a rolling window's RViz display goes blank |

**Corridor budget.** 1.00 m corridor, 0.28 m inscribed ring → the drone's centre
has **±0.22 m** of legal band. There is no "far from walls" in a corridor, only
"centred". Measured cost profile across one (published 0–100 scale, 99 =
inscribed = MPPI collision): `99 68 49 34 23 15 0 0 15 23 34 49 68 99`.

### B2 — What gets marked as a wall

| Param | Now | What it does |
|---|---|---|
| local `plugins` | static + obstacle + inflation | **`static_layer` is load-bearing.** Without it the local costmap knows only what the lidar sees *this instant*, and the obstacle layer clears what it ray-traces through — so an occluded wall ceases to exist for MPPI |
| local `static_layer.track_unknown_space` | **False** | Must stay false. True writes unknown as 255, which CostCritic scores as a collision, and every frontier approach becomes unplannable |
| global `plugins` | static + inflation only | `obstacle_layer` deliberately absent — it bypassed `map_filter` and accumulated phantoms permanently. Don't re-add it |
| `map_topic` (both static layers) | `/map_filtered` | Speckle-filtered, not raw `/map` |
| `obstacle_max_range` | 5.0 | Furthest a hit is marked. Coupled to lean: beyond `lidar_z / tan θ` you mark floor |
| `raytrace_max_range` | 5.5 | Furthest cells are cleared. Must be ≥ obstacle range |
| `inf_is_valid` | True | Infinite returns clear as free space |

### B3 — Perception gates

`scan_tilt_gate.py` is **deprecated** — it dropped whole scans on tilt, which
deadlocked a vehicle that must lean to move. `scan_stabilizer` replaced it: it
re-projects each ray to its true horizontal bearing and NaNs only the individual
returns a given lean makes untrustworthy.

| Param | File | Now | What it does |
|---|---|---|---|
| `max_hit_dz_m` | `launch/slam.launch.py` | 1.05 | Rejects returns too far off the scan plane |
| `min_hit_height_agl_m` | " | 0.25 | Floor-strike rejection |
| `max_hit_height_agl_m` | " | derived = `flight_altitude` + 1.15 = **2.32** | Net rejection (the net is real: rods at z 2.485–2.515) |
| `max_clear_elev_deg` | " | 10.0 | A no-return ray only clears cells if it stayed this close to horizontal |
| `flight_altitude` | " | 1.17 | Must match the bridge's `target_altitude_m` |
| `min_blob_cells` | " | 30 | `map_filter` deletes occupied islands smaller than this |
| `max_laser_range` | `config/mapper_params.yaml` | 5.0 | Matched to `obstacle_max_range` — one range for the whole stack |
| `min_pass_through` / `occupancy_threshold` | " | 2 / 0.22 | SLAM's hits-to-passes gate |
| `map_update_interval` | " | 0.4 | Republish rate. Does not change what is mapped |
| `minimum_travel_distance` | " | 0.10 | Scan spacing in the pose graph |
| `max_safe_tilt_deg` | `frontier_explorer.py` | 30.0 | **Currently inert** — `ATC_ANGLE_MAX` 18 means it can never fire. Drop to ~20 if you want it live |

> **The lean/range coupling.** At the 12.2° steady lean the floor enters a ray at
> `1.245 / tan(12.2°)` = 5.76 m, outside `obstacle_max_range` 5.0. That margin is
> why `ax_max` is 1.50 and not higher. Raise `ax_max` and you must shorten
> `obstacle_max_range`/`raytrace_max_range` to match, or accept that
> `scan_stabilizer` does more of the work.

### B4 — MPPI's obstacle response

| Param | Now | What it does |
|---|---|---|
| `CostCritic.cost_weight` | **14.0** | How hard obstacles repel. **Must outrank `PathAlignCritic`**, or a wall-hugging global path simply outvotes the live lidar |
| `CostCritic.critical_cost` | 600.0 | Charged to a rollout point at the inscribed ring that is not yet a declared collision |
| `CostCritic.collision_cost` | 1 000 000 | Hard veto |
| `CostCritic.consider_footprint` | false | Centre-point check. `true` is 1000 × 24 footprint polygons per cycle at 20 Hz — the deadline is missed, which is worse |
| `CostCritic.near_goal_distance` | 0.15 | Repulsion relaxes inside this radius of the goal. **Must stay below `xy_goal_tolerance`** or there is a shell where repulsion is off and arrival is undeclared |
| `PathAlignCritic.cost_weight` | 10.0 | Lateral deviation penalty — the anti-oscillation knob, now junior to CostCritic |
| `PathAlignCritic.max_path_occupancy_ratio` | 0.5 | Disables itself above this. At 0.05 it was silently off in every corridor |
| `ConstraintCritic.cost_weight` | 4.0 | Enforces the vel/accel limits |

`GoalAngleCritic` and `PathAngleCritic` are **removed from the `critics` list**.
Both score a heading error a `wz_max: 0.0` Omni vehicle cannot correct; their
blocks remain in the file but are dead.

### B5 — Recovery and stall handling

| Param | Now | What it does |
|---|---|---|
| `required_movement_radius` / `movement_time_allowance` | 0.3 / 10.0 | Must move this far within this long, or "stuck". Loosened for the tighter goal tolerance |
| `failure_tolerance` | 0.3 | Seconds of no valid control before failing |
| `simulate_ahead_time` | 1.0 | Behaviour-server lookahead. At 2.0 every recovery aborted instantly with "Collision Ahead" in a 1 m corridor |
| `max_rotational_vel` | 1.0 | Spin recovery speed |
| `stuck_distance_m` / `stuck_timeout_sec` | 0.35 / 25.0 | Explorer's own stuck detector |

> Once the drone's centre is inside the inscribed ring, **MPPI cannot recover** —
> every sample collides and the behaviours refuse with "Collision Ahead". That
> run is over; restart it.

---

## C. Goals

### C1 — Arrival, `nav2_params.yaml`

| Param | Now | What it does |
|---|---|---|
| `xy_goal_tolerance` | **0.2** | Arrival radius. This, not the explorer, is what decides when the next frontier gets picked |
| `yaw_goal_tolerance` | 3.14 | Off, correctly — yaw is pinned |
| `stateful` | True | Won't un-reach a reached goal |
| `GoalCritic.cost_weight` | 5.0 | Pull toward the goal point |
| `GoalCritic.threshold_to_consider` | **0.8** | Hand-off distance from PathFollow. **Must equal `PathFollowCritic.threshold_to_consider`** or there is a band where both or neither is active |
| `PathFollowCritic.cost_weight` | 11.0 | Progress along the path — nav2's own header calls this the accelerator |
| `PathFollowCritic.offset_from_furthest` | 10 | Carrot, in path points (0.05 m each) = 0.50 m. Longer is faster but shortcuts corners |
| `PathFollowCritic.threshold_to_consider` | 0.8 | Disengages inside this |
| `GridBased.tolerance` | 0.25 | Planner may end a path this far short. Keep ≤ `xy_goal_tolerance` + one cell, or the goal checker is never satisfied |
| `allow_unknown` | false | Planner refuses unknown space — this is why some frontier goals return "no path" |
| `use_astar` | true | A* rather than Dijkstra |

**Three numbers that must be ordered:** `near_goal_distance` (0.15) <
`xy_goal_tolerance` (0.2) ≤ `goal_arrival_tolerance_m` (0.25) ≈
`GridBased.tolerance` (0.25). They live in two different files and have drifted
apart before.

### C2 — Goal selection, `nidar_sim/frontier_explorer.py`

| Param | Now | What it does |
|---|---|---|
| `global_costmap_topic` | `/global_costmap/costmap` | Vetoes any goal whose cell reads ≥ `costmap_lethal_threshold`. This is what stops goals landing inside walls — `/map_filtered` alone is too stale and too thin |
| `costmap_lethal_threshold` | 99 | 99 = INSCRIBED, which is what MPPI treats as a collision. 100 would be *less* strict than the controller |
| `goal_arrival_tolerance_m` | 0.25 | Independent confirmation of Nav2's arrival. Logs loudly if it disagrees with `xy_goal_tolerance` |
| `safe_distance_m` | 0.45 | Goal standoff from mapped obstacles |
| `min_safe_distance_m` | 0.32 | Hard floor the relaxation may not go below. Only 0.04 m outside the inscribed ring — hence the costmap veto above |
| `min_live_clearance_m` | 0.08 | **Separate quantity.** How close the *drone* may be to the inscribed ring, which already includes `robot_radius`. Do not fold it into `min_safe_distance_m` |
| `robot_radius_m` | 0.28 | Dilation for reachability. Must equal the costmaps' `robot_radius` |
| `distance_decay_m` | 4.5 | `exp(−dist/decay)` — the "why are goals so far away" term |
| `commitment_radius_m` | 4.0 | Goals inside this get a ×1.5 bonus; finish local work first |
| `direction_weight_m` | 2.5 | Prefers continuing the current heading — anti-ping-pong |
| `visit_penalty_m` / `visit_radius_m` / `visit_memory_cycles` | 6.0 / 1.0 / 40 | Penalises re-visiting, and for how long |
| `min_frontier_cells` | 5 | 0.25 m. Low on purpose: a junk candidate costs one cycle, a discarded doorway costs every room behind it |
| `min_frontier_dist_m` | 0.35 | Won't target frontiers closer than this |
| `occupied_threshold` | 65 | What counts as a wall in `/map_filtered` (0–100 scale, **not** a costmap) |
| `goal_timeout_sec` | 90.0 | Abandon a goal after this |
| `goal_stall_cycles` / `goal_stall_progress_m` | 3 / 0.15 | Blacklist if distance stops dropping |
| `blacklist_radius_m` / `blacklist_expire_cycles` | 0.6 / 20 | Blacklist geometry and retry delay |
| `replan_period_sec` | 3.0 | Re-evaluation rate. The explorer does **not** preempt an active goal |
| `rejection_streak_limit` | 4 | Consecutive rejects before forcing a far goal |

---

## D. Settled facts — do not re-derive

1. ArduPilot subscribes to **`/ap/cmd_vel`** (`AP_DDS_Topic_Table.h`, `topic_name = "cmd_vel"` under the `ap` namespace). Not `/ap/v1/cmd_vel`.
2. `negate_lateral_for_frd` must stay **True**: `AP_DDS_ExternalControl.cpp` passes `linear.y` unchanged into `ahrs.body_to_earth()`, which expects FRD.
3. `ATC_ANGLE_MAX` is **degrees**; `WP_SPD`/`WP_ACC` are **m/s and m/s²** in this build (SI param names; the old `WPNAV_*` centimetre names do not exist here and would be silently ignored).
4. `WP_ACC` and `WP_SPD` cap the **NE vector**; Nav2's limits are **per axis**. Always compare against the ×√2 diagonal.
5. MPPI collision = centre cost ≥ 253 (INSCRIBED) while `consider_footprint: false`. Inflation cost = `252·exp(−k·(d − robot_radius))`.
6. `WP_YAW_BEHAVIOR 0` is mandatory while `wz_max: 0.0` — the two must agree about who owns heading.
7. **One `/clock` publisher only.** Leftover `gz sim` instances each publish one; more than one freezes every ROS timer and nodes go silent with no error while `ros2 node list` looks healthy. First check: `ros2 topic hz /clock` (~1000 Hz) and `ros2 topic info /clock`.
8. A package manifest at `src/` collapses colcon discovery to a single package and silently installs nothing. Check `colcon list` shows all packages, not one.
