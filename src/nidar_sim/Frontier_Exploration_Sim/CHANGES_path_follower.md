# nidar_arena_1 autonomy — what changed and why (31 Oct 2026)

Branch: `follower-controller`. Files touched are all under `Frontier_Exploration_Sim/`.

## Result

Autonomous frontier exploration of `nidar_arena_1` (14.4 m, 1 m corridors) with no manual input:
**94 % of the arena mapped in 14.5 min** (85.5 % free / 8.8 % wall / 5.7 % unknown), walls single-line thin, saved as `arena1_run2_2014.pgm`. One crash near the end, under investigation.

Before this change the best runs were ~12 and ~20 goals with stalls needing manual restarts.

## The pipeline (unchanged shape, one node bypassed)

```
explorer ──NavigateToPose──▶ bt_navigator ──ComputePathToPose──▶ planner_server (NavFn + global costmap) ──▶ /plan
                                   │                                                                           │
                                   └──FollowPath──▶ controller_server (DWB) ──▶ cmd_vel_nav ──▶ velocity_smoother ──▶ /cmd_vel ──▶ (nobody)
                                                                                                                           │
                              path_follower.py ◀── /plan + TF(map→base_link) + /scan                                        │
                                   │                                                                                       │
                                   └──▶ /cmd_vel_follower ──▶ cmd_vel_stamper.py (-p in_topic:=/cmd_vel_follower) ──▶ /ap/v1/cmd_vel ──▶ ArduPilot
```

Nav2 is launched exactly as before. DWB still runs and still computes velocities, but **nothing listens to `/cmd_vel` any more**: the stamper (the only bridge to ArduPilot) now subscribes to `/cmd_vel_follower`, which `path_follower.py` publishes. That is the whole "bypass".

### Why not remove `controller_server` from the launch instead?

Because `bt_navigator`'s behaviour tree calls the `follow_path` action, and `controller_server` is the only thing serving it. Remove it and every goal aborts. Leaving it running costs nothing and gives two useful side effects:

- DWB's **goal checker** (`xy_goal_tolerance 0.35`) reads the drone's *real* pose from TF and tells `bt_navigator` "reached", so the explorer gets its result and sends the next goal. The follower produced the motion; DWB only reported it.
- DWB's **progress checker** (0.3 m / 15 s) does the same for "stuck". If the follower ever fails to move the drone, the BT's recovery (clear costmaps → Wait 5 s → replan) kicks in as before.

Neither `bt_navigator` nor the explorer sees a velocity. Only poses flow upward, and the poses are the truth.

**If your exploration algorithm does not go through `bt_navigator`** (i.e. you call the planner yourself or plan yourself), the follower only needs a `nav_msgs/Path` on `/plan` in the `map` frame, republished at least every 2 s while a goal is active. Publish that and it works unchanged.

## Why DWB had to go

DWB is a sampling controller: 10×/s it generates ~2000 candidate (vx, vy, vθ), forward-simulates each for 1.7 s **assuming the vehicle instantly does what it's told**, discards any whose footprint touches cost ≥ 253 in the local costmap, scores the rest on distance to path/goal. ArduPilot takes ~0.5 s to reach a commanded velocity and coasts after. In a 1 m corridor with `robot_radius 0.30` the legal lane is 0.4 m wide; the lag error is the width of the lane. Consequences seen in the logs, all in `controller_server`:

| Symptom | Mechanism |
|---|---|
| Drone drifts into the inscribed band, then `Failed to create a plan from potential` for every goal (deadlock) | Simulated trajectory ≠ flown trajectory |
| 55 s at a doorway, zero movement, no error | When all big trajectories are illegal, 0.03 m/s ones are always legal → DWB "succeeds" at a creep that fails the progress checker |
| Drone yaws for no reason | vθ is a free variable in the search; curving trajectories score ≈ straight ones |
| Doorway open for the planner, closed for the controller | Local costmap is built from raw scan hits (thicker walls) vs. SLAM's filtered map |

None of these are parameters; they are what a sampling controller is. Every parameter fix (23 Oct → 31 Oct) moved the symptom, not the cause.

## The follower (`path_follower.py`, ~100 lines)

Every 100 ms:

1. Pose = TF `map → base_link` (x, y, yaw).
2. Nearest path point at or after the last index (no jumping back on U-turns).
3. Walk `lookahead` (0.30 m) forward along the path → the **carrot**.
4. `v_map = gain × (carrot − pose)`, capped at `v_max` (0.30 m/s). Rotate into body FLU. **Yaw rate = 0 always.**
5. Lidar guard: any return < `guard_dist` (0.35 m) within ±30° of the commanded direction → publish zero (hover).
6. Publish `geometry_msgs/Twist` on `/cmd_vel_follower`.

Holds (publishes zero) when: no `/plan` for `path_timeout` (2 s — goal finished/cancelled), no `/scan` for 1 s, no TF, or at the path end within `end_tolerance` (0.10 m). Each state change prints once (`TRACKING idx=…`, `HOLD: <reason>`).

Why lag stops hurting: the carrot is on the path, the path is in the lane centre (inflation gradient). If the drone lags or drifts, the error vector points back to the path, so the correction grows — the same mechanism ArduPilot uses to hold a hover. With DWB, lag pushed the drone *out* of the lane; here it pulls it *in*.

Collision handling, honestly: the follower trusts the path. The planner replans every 1 s (BT `RateController`) against the global costmap, which includes the live scan, with `robot_radius 0.30` / `inflation 0.5`. The guard is the hard stop. The follower does not invent detours; in a static maze it doesn't need to.

Parameters (override with `-p name:=value`): `lookahead 0.30`, `gain 1.0`, `v_max 0.30`, `guard_dist 0.35`, `guard_half_angle_deg 30`, `path_timeout 2.0`, `scan_timeout 1.0`, `end_tolerance 0.10`, `cmd_topic /cmd_vel_follower`.
Tuning signs: weaving along the path → `gain` down (0.6); visible corner-cutting → `lookahead` down (0.2).

## All other changes, with the reason for each

### `cmd_vel_stamper.py` — lateral axis was mirrored
ROS `base_link` is FLU (+y = left); ArduPilot's body frame is FRD (+y = right). AP_DDS treats the incoming y as FRD. The stamper now negates `linear.y`. x and yaw were verified correct and are untouched. **Verified open-loop:** `ros2 topic pub` of +x / +y / +yaw moves the drone along the red arrow / green arrow / CCW. Do this test on any new airframe or bridge before trusting a controller.
The stamper takes `-p in_topic:=…` (default `/cmd_vel`); launch it with `/cmd_vel_follower`.

### `nav2_params_iris .yaml` (note the space in the file name)
- `robot_radius 0.30` (both costmaps) — Iris prop radius; 0.22 is the competition airframe, hardware config only.
- `inflation_radius 0.5`, `cost_scaling_factor 5.0` (both) — cost falls all the way to the corridor centre so the planner is pulled off wall tips; must be ≥ robot_radius or the forbidden band disappears.
- `xy_goal_tolerance 0.35`, `yaw_goal_tolerance 6.28` — 0.60 let "arrived" fire at the wall.
- `velocity_smoother max_velocity [0.3, 0.3, 0.0] / min_velocity [-0.3, -0.3, 0.0]` — the shipped file had y = 0.0, which silently zeroed every lateral command for a month. Now irrelevant (DWB bypassed) but left correct.
- DWB: `min_vel_x -0.3`, `max_vel_x/y 0.3`, `vy_samples 5`, critics `["Oscillation","BaseObstacle","PathDist","GoalDist"]` (GoalAlign/PathAlign removed — they reward nose alignment, meaningless for a holonomic quad), `max_vel_theta 0.0`, `vtheta_samples 1`. Irrelevant now, left for anyone who re-enables DWB.
- `bt_navigator.default_nav_to_pose_bt_xml: "<absolute path>/nidar_bt.xml"` — **absolute path, no `~`**; edit to your own checkout path.

### `nidar_bt.xml` — recovery without Spin/BackUp
Nav2's default tree recovers with ClearCostmap → Spin 90° → Wait → BackUp 0.3 m. Spin and BackUp are ground-robot reflexes: on a multirotor a yaw-rate command translates the vehicle tens of centimetres, smears the 2D scan, and BackUp drives in an arbitrary nose direction. Every crash before 26 Oct was preceded by one. This tree is the default with those two removed (Clear → Wait only). Verify after T4 starts: `ros2 param get /bt_navigator default_nav_to_pose_bt_xml`.

### `frontier_params_iris.yaml` (only relevant if you use the open-source explorer)
`occ_threshold 65` (the explorer marks a goal "blocked" by *costmap* value; corridor centres publish ~36), `min_frontier_size_cells 4` (12 and 15 were tried and filtered out real corridor-mouth frontiers, which are 6–13 cells here — do not raise it), `goal_preemption_enabled false`, `goal_skip_on_blocked_goal false`, suppression on (no_progress 25 s, startup grace 30 s).

## Run procedure

| T | Command | Gate before next step |
|---|---|---|
| 1 | `ros2 launch ardupilot_gz_bringup iris_maze.launch.py rviz:=true use_gz_tf:=true lidar_dim:=2` | Scan visible in RViz. This RViz runs on **wall clock**: viewer only, never click 2D Goal Pose in it (goals get rejected with an extrapolation error). |
| 2 | `mavproxy.py --console --master=:14550` → `mode guided`, `arm throttle`, `takeoff 2` | Hover 15 s. Close the Gazebo GUI window (7.5 GB machine; slam_toolbox dies first under memory pressure). |
| 3 | `ros2 launch slam_toolbox online_async_launch.py slam_params_file:=$HOME/ardu_ws/src/nidar_sim/config/mapper_params.yaml use_sim_time:=true` | Vestibule visible in the map. |
| 4 | `ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true params_file:="$HOME/ardu_ws/src/nidar_sim/Frontier_Exploration_Sim/nav2_params_iris .yaml" 2>&1 \| tee ~/run_$(date +%m%d_%H%M)_t4.log` | `Managed nodes are active`; `ros2 param get /bt_navigator default_nav_to_pose_bt_xml` prints your `nidar_bt.xml` path. |
| 5 | `python3 ~/ardu_ws/src/nidar_sim/Frontier_Exploration_Sim/cmd_vel_stamper.py --ros-args -p in_topic:=/cmd_vel_follower` | — |
| 7 | `python3 ~/ardu_ws/src/nidar_sim/Frontier_Exploration_Sim/path_follower.py --ros-args -p use_sim_time:=true` | Prints `HOLD: no fresh path`. |
| 6 | `ros2 launch frontier_exploration_ros2 frontier_explorer.launch.py use_sim_time:=true params_file:=$HOME/ardu_ws/src/nidar_sim/Frontier_Exploration_Sim/frontier_params_iris.yaml 2>&1 \| tee ~/run_$(date +%m%d_%H%M)_t6.log` (or your own explorer) | T7 switches to `TRACKING`, drone moves along the green path. |
| end | `ros2 run nav2_map_server map_saver_cli -f ~/arena1_$(date +%H%M)` | |

Order 5 → 7 → 6 matters: the stamper must be listening to the follower before any goal exists.

## Known failure modes and what to do

| You see | It is | Do |
|---|---|---|
| T4: `Transform data too old … Transform time: N s` frozen, and/or `Reached the goal!` 60 ms after every goal with the drone not moving | `controller_server`'s TF buffer stopped receiving slam_toolbox's `map→odom` (Fast-DDS delivery fault under memory pressure; slam itself is alive, `tf2_echo` in a fresh terminal looks fine) | Ctrl+C T4, relaunch T4, relaunch T6. Nothing else. Fallback if frequent: `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` in every terminal + full restart. |
| T4: `Extrapolation Error … Requested time 179xxxxxxx` | A wall-clock-stamped goal = an RViz click | Don't click. |
| T4: `Failed to get result for follow_path in node halt!` | The explorer cancelling after its 25 s no-progress timeout | Normal. Not a Nav2 fault. |
| T4: `Failed to create a plan from potential when a legal potential was found` for **every** goal incl. return-to-start | Drone's own cell is inside the inscribed band (≥ 253); NavFn can't trace a path out of it | With the follower this should not happen (it tracks the lane centre). If it does: stop T7, read the lidar first (8-sector script below), nudge 1 s at 0.15 m/s **only** toward a sector with ≥ 0.8 m, relaunch T7. Never nudge from RViz arrows or coordinates — that crashed the drone once. |
| Drone crashed | — | Full restart from T1. Gazebo does not recover an Iris. |

Lidar sector script (body frame, run before any nudge):
```bash
python3 - <<'EOF'
import rclpy, math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
rclpy.init(); n = Node('scan_peek')
def cb(m):
    names=['FRONT  +x','front-left','LEFT   +y','back-left','BACK   -x','back-right','RIGHT  -y','front-right']
    best=[float('inf')]*8
    for i,r in enumerate(m.ranges):
        if not (m.range_min < r < m.range_max): continue
        a=math.degrees(m.angle_min+i*m.angle_increment)%360
        best[int(((a+22.5)%360)//45)]=min(best[int(((a+22.5)%360)//45)],r)
    for k,v in zip(names,best): print(f'{k:12s} {v:5.2f} m')
    rclpy.shutdown()
n.create_subscription(LaserScan,'/scan',cb,10); rclpy.spin(n)
EOF
```

## Open items

- The crash at the end of the 14.5-min run: not yet diagnosed (T7/T4 tails pending). Candidates: corner-cutting with lookahead 0.30, or the guard not covering a lateral approach.
- Two centre pockets remain unexplored (5.7 %). Likely the explorer's suppression after the pocket-tip goals; not a controller issue.
- Mission yaw (camera sweep for survivor detection) is **not** part of navigation. It belongs to a mission layer: on arrival at a cell, hold position, rotate slowly, continue. The follower keeps navigation yaw at zero on purpose; the two must stay separate.
- Hardware: the competition airframe is 0.44 m across in the same 1 m corridors → 0.56 m lane. Same geometry, same controller; `robot_radius 0.22` goes in the hardware config.

## Files in this change

```
Frontier_Exploration_Sim/path_follower.py          new
Frontier_Exploration_Sim/nidar_bt.xml              new (copied from ~/nidar_bt.xml)
Frontier_Exploration_Sim/cmd_vel_stamper.py        y negation; in_topic param already existed
Frontier_Exploration_Sim/nav2_params_iris .yaml    values above + default_nav_to_pose_bt_xml
Frontier_Exploration_Sim/frontier_params_iris.yaml values above
Frontier_Exploration_Sim/CHANGES_path_follower.md  this file
```
Not in this repo: `nidar_arena_1.sdf` installed into `ardupilot_gz_gazebo/worlds/` with `<world name>` renamed to `maze`, and `iris_maze.launch.py` pointed at it (both src and install copies).
