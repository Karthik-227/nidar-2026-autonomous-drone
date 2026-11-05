# Follower runbook — short form

### 0 · Clean
```bash
~/ardu_ws/scripts/nidar-clean.sh
```
Check: `fastdds shm swept: N -> 0`, empty survivors.

### 1 · Sim
```bash
ros2 launch nidar_sim arena.launch.py arena:=1 rviz:=false
```
Not `sim.launch.py` — that loads a generic maze, not this arena. Check: `got segment base_link`.

### 2 · Arm + takeoff
```bash
mavproxy.py --console --aircraft test --master=:14550
```
`mode 4` / `arm throttle` / `takeoff 1.25`

### 3 · SLAM
```bash
ros2 launch nidar_sim slam.launch.py
```
Check: `ros2 topic hz /map` ≈ 1 Hz.

### 4 · Nav2 (new params + BT, bridge off)
```bash
ros2 launch nidar_sim nav2.launch.py bridge:=false params_file:="$HOME/ardu_ws/src/nidar_sim/Frontier_Exploration_Sim/nav2_params_iris .yaml"
```
Check: `ros2 param get /bt_navigator default_nav_to_pose_bt_xml` → your path, not `/home/bincy/...`.

### 5 · Path follower
```bash
python3 ~/ardu_ws/src/nidar_sim/Frontier_Exploration_Sim/path_follower.py --ros-args -p use_sim_time:=true
```
Check: `HOLD: no fresh path`.

### 6 · cmd_vel bridge (re-pointed)
```bash
ros2 run nidar_sim cmd_vel_bridge --ros-args -p use_sim_time:=true -p in_topic:=/cmd_vel_follower -p target_altitude_m:=1.17 -p max_linear_vel:=0.5
```
Don't use `cmd_vel_stamper.py` — no altitude hold. Check: exactly one `cmd_vel_bridge` node.

### 7 · Manual test first
2D Goal Pose in RViz. Check: step 5 flips to `TRACKING`. If not, stop — don't mask it with autonomy.

### 8 · Autonomy
```bash
ros2 run nidar_sim frontier_explorer --ros-args -p use_sim_time:=true
```
5 and 6 must already be running.

### End
```bash
ros2 run nav2_map_server map_saver_cli -f ~/arena1_$(date +%H%M)
```

## Tuning path_follower.py (`-p name:=value` on step 5)
| Param | Default | Tune when |
|---|---|---|
| `lookahead` | 0.30 m | cutting corners → lower to ~0.2 |
| `gain` | 1.0 | weaving on straights → lower to ~0.6 |
| `v_max` | 0.30 m/s | speed cap |
| `guard_dist` / `guard_half_angle_deg` | 0.35 m / 30° | known gap: doesn't cover lateral contact — ran at 90° in testing |
| `path_timeout` / `scan_timeout` | 2.0 s / 1.0 s | stale-data hold |
| `end_tolerance` | 0.10 m | arrival distance |
| `cmd_topic` | `/cmd_vel_follower` | must match step 6's `in_topic` |

## Known issues
| Symptom | Status |
|---|---|
| Physical wedging near walls | **Unfixed.** STUCK recovery cycles (costmap clear, blacklist drop) but nothing backs the drone away — Spin/BackUp were deliberately removed from the BT. |
| Frozen `Transform data too old`, goals "reach" in 60 ms | Fast-DDS lost SLAM's TF. Ctrl+C, relaunch step 4, then 5+6. |
| `Failed to create a plan from potential` on every goal | Drone's own cell reads lethal. Stop step 8, check lidar, nudge by hand — never from RViz. |
| Old DWB doorway-stall symptom reappears | A stray `cmd_vel_bridge` is still on `/cmd_vel`. Check for a duplicate node. |
| `/scan_safe` silent | `scan_stabilizer` not running — use `nidar_sim slam.launch.py`, not raw `slam_toolbox`. |
