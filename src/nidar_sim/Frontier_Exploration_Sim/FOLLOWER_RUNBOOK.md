# Follower runbook — copy, paste, fly

Same shape as `nidar_sim/RUNBOOK.md`. Steps 0–3 are unchanged from that file.
Steps 4 and 6 are new commands; step 5 is new.

Full rationale (why DWB had to go, why `cmd_vel_bridge` beats the pulled
`cmd_vel_stamper.py`, config deltas, tuning table): see the artifact guide,
or `CHANGES_path_follower.md` in this directory for the original author's
run log.

---

### 0 · Always do this first
```bash
~/ardu_ws/scripts/nidar-clean.sh
```
**CHECK** `--- fastdds shm swept: N -> 0 ---` then empty `--- survivors ---`

---

### 1 · Simulation
```bash
ros2 launch nidar_sim arena.launch.py arena:=1 rviz:=false
```
**CHECK** `got segment base_link` · `ros2 topic hz /clock` ≈ 1000 Hz · log line
`[nidar] arena nidar_arena_1 -> .../worlds/nidar_arena_1.sdf`

Not `sim.launch.py` — that wraps upstream `ardupilot_gz_bringup`'s own
`iris_maze.launch.py`, which loads `ardupilot_gz_gazebo/worlds/maze.sdf` (a
generic maze, unrelated to this arena). `arena.launch.py` is what actually
loads `nidar_sim/worlds/nidar_arena_1.sdf` — the 14.4 m / 1.0 m-corridor arena
this whole guide assumes — and already defaults to `arena:=1`.
`arena.launch.py` sets `GZ_IP` itself, so no need to export it by hand.

---

### 2 · Arm + take off
```bash
mavproxy.py --console --map --aircraft test --master=:14550
```
At the `>` prompt:
```
mode 4
arm throttle
takeoff 1.25
```
**CHECK** drone airborne in Gazebo

---

### 3 · SLAM (+ scan stabilizer)
```bash
ros2 launch nidar_sim slam.launch.py
```
**CHECK** `Registering sensor: [Custom Described Lidar]` · `ros2 topic hz /map` ≈ 1 Hz ·
`scan_stabilizer` prints `stabilised N scans | mean tilt ...` every 10 s

---

### 4 · Nav2 — NEW params, BT, and bridge disabled
```bash
ros2 launch nidar_sim nav2.launch.py bridge:=false params_file:="$HOME/ardu_ws/src/nidar_sim/Frontier_Exploration_Sim/nav2_params_iris .yaml"
```
Back to your teammate's own file from the `follower-controller` branch —
not the AirMouse PDF's reconstruction. Two amendments on top of what was
pulled from GitHub, both noted in the file's own header:
1. `default_nav_to_pose_bt_xml` repointed from the original author's
   `/home/bincy/...` path to this checkout.
2. Both costmaps' `obstacle_layer.scan.topic` changed from `/scan` to
   `/scan_safe` — this branch predates `scan_stabilizer.py`. Raw `/scan`
   let hover-attitude noise jitter wall positions between scans, which made
   NavFn flip between near-equal-cost routes almost every 1Hz replan — the
   actual cause of the drone visibly weaving mid-corridor. Verified against
   `nidar_sim/config/nav2_params.yaml` (this team's real flown config),
   which already reads `/scan_safe`.
(`nav2_params_airmouse.yaml` still exists alongside it if you ever want to
compare `inflation_radius` 0.5 here vs. AirMouse's 0.35 — not used anymore.)
**CHECK**
```bash
ros2 param get /bt_navigator default_nav_to_pose_bt_xml
```
prints `.../Frontier_Exploration_Sim/nidar_bt.xml` — **not** `/home/bincy/nidar_bt.xml`.
Also: `Managed nodes are active` in the launch log, RViz2 shows black walls with
magenta/cyan bands (unchanged from before).

`bridge:=false` matters: the bridge this launches by default listens to `/cmd_vel`,
which nothing writes to after step 6. Two `cmd_vel_bridge` instances fighting is
exactly what its own singleton guard refuses to let happen.

---

### 5 · Path follower — NEW node
```bash
python3 ~/ardu_ws/src/nidar_sim/Frontier_Exploration_Sim/path_follower.py --ros-args -p use_sim_time:=true -p gain:=0.6
```
**CHECK** prints `HOLD: no fresh path` (no goal exists yet — expected)

`gain:=0.6` (default is 1.0): measured live on 2 Nov 2026 — at default gain the drone
visibly swerved along straight corridors (this is a pure-P controller with no
damping, chasing a carrot only `lookahead`=0.30m out, fighting ArduPilot's own
~0.5s velocity-actuation lag). At gain 0.6, cross-track deviation from the
active plan measured max 7.8cm / mean 1.8cm over 1003 samples — essentially a
straight line. This was always the documented fix (`CHANGES_path_follower.md`:
"weaving along the path → gain down (0.6)"), just needed to actually be
applied here.

Optional: point its lidar guard at the tilt-gated scan instead of raw `/scan`:
```bash
python3 ~/ardu_ws/src/nidar_sim/Frontier_Exploration_Sim/path_follower.py --ros-args -p use_sim_time:=true -r /scan:=/scan_safe
```

---

### 6 · cmd_vel bridge — your existing node, re-pointed
```bash
ros2 run nidar_sim cmd_vel_bridge --ros-args -p use_sim_time:=true -p in_topic:=/cmd_vel_follower -p target_altitude_m:=1.17 -p max_linear_vel:=0.5
```
**CHECK** log ends `/cmd_vel_follower (Twist) -> /ap/v1/cmd_vel` ·
`ros2 node list | grep cmd_vel_bridge` shows exactly **one**

Do **not** use the pulled `cmd_vel_stamper.py` here — it never sets `linear.z`,
so nothing holds altitude (your own `nav2.launch.py` already documents this
failure mode for any bridge with no altitude term). `cmd_vel_bridge` already
has altitude hold, the FRD lateral-axis fix, and a deadman stop built in.

---

### 7 · Manual test — do this before autonomy
In RViz2, **2D Goal Pose** somewhere open.
**CHECK** drone flies there · step 5's terminal flips from `HOLD: ...` to
`TRACKING idx=.../...`

If not: stop and fix here. Autonomy will only hide the problem.

---

### 8 · Autonomous exploration — your code, unchanged
```bash
ros2 run nidar_sim frontier_explorer --ros-args -p use_sim_time:=true
```
**CHECK** one line per cycle: `Explored N m2 (X% of mapped area) | K frontier(s) | goal -> (x, y)`

**Order matters: 5 and 6 must already be running before 8 sends its first goal.**
`frontier_params_iris.yaml` is for the *upstream* open-source explorer — not
used here, since `frontier_explorer` is your own node.

---

### End of run
```bash
ros2 run nav2_map_server map_saver_cli -f ~/arena1_$(date +%H%M)
```

---

## Pre-flight checklist
- [ ] `ros2 param get /bt_navigator default_nav_to_pose_bt_xml` → your path, not `/home/bincy/...`
- [ ] `ros2 node list | grep cmd_vel_bridge` → exactly one
- [ ] Step 5 prints `HOLD: no fresh path` at idle
- [ ] 2D Goal Pose test moves the drone and flips step 5 to `TRACKING`
- [ ] Step 8 started last

## Tuning path_follower.py (`-p name:=value`)
| Param | Default | Tune when |
|---|---|---|
| `lookahead` | 0.30 m | corners visibly cut → lower to ~0.2 |
| `gain` | 1.0 (step 5 overrides to **0.6**) | 0.6 confirmed live 2 Nov 2026 — fixes swerving on straights. Going back to 1.0 reintroduces it. |
| `v_max` | 0.30 m/s | speed cap |
| `guard_dist` / `guard_half_angle_deg` | 0.35 m / 30° | lidar e-stop cone |
| `path_timeout` / `scan_timeout` | 2.0 s / 1.0 s | stale-data hold |
| `end_tolerance` | 0.10 m | arrival distance |
| `cmd_topic` | `/cmd_vel_follower` | must match step 6's `in_topic` |

## Known failure modes
| Symptom | Fix |
|---|---|
| Frozen `Transform data too old`, goals "reach" in 60 ms, no motion | Fast-DDS lost SLAM's TF. Ctrl+C, relaunch step 4, then 5+6. If frequent: `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` in every terminal. |
| `Failed to create a plan from potential` on every goal | Drone's own cell reads lethal. Stop step 8, check lidar sectors, nudge toward open space by hand — never from RViz. |
| Old DWB doorway-stall symptom reappears | A stray `cmd_vel_bridge` is still on `/cmd_vel`. Check for a duplicate node. |
| `/scan_safe` silent | `scan_stabilizer` not running — use `nidar_sim slam.launch.py`, not raw `slam_toolbox`. |
