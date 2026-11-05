# NIDAR sim — run guide

Autonomous frontier exploration of a generated maze arena, with a 2D lidar on
an ArduPilot multirotor. SLAM maps it, Nav2 flies it, and the explorer takes it
home and lands it.

**Flight height is 1.25 m. Take off to 1.25, not 1 or 2.** Three files derive
their filters from that number; see [Why 1.25 m](#why-125-m).

**Take off before starting SLAM**, so every scan in the map is taken from the
height the drone actually flies at.

---

## Quick start

Three terminals, in this order. Wait for each **CHECK** before the next.

### 0 · Clean — every time, including between restarts

```bash
~/ardu_ws/scripts/nidar-clean.sh
```

**CHECK** `survivors (should be empty)` with nothing under it.

### 1 · Simulation

```bash
cd ~/ardu_ws && source install/setup.bash
export GZ_IP=127.0.0.1
ros2 launch nidar_sim arena.launch.py arena:=1
```

**CHECK** at ~25 s it prints `nidar_tuning.parm applied and verified`.

No such line → **stop and fix it**. Everything below is pointless without it:
the vehicle is on baseline lean limits and the map will fill with phantoms.

### 2 · Take off first

```bash
mavproxy.py --console --map --aircraft test --master=:14550
```

At the `>` prompt:

```
mode 4
arm throttle
takeoff 1.17
```
`mode 4` (GUIDED), not `mode guided` - typed before the real heartbeat
lands, MAVProxy's name lookup defaults to Plane's mode table, where
GUIDED is 15 (AUTOTUNE on a Copter). The vehicle silently stays in
STABILIZE and `takeoff` is rejected. The numeric form bypasses that
lookup and always resolves correctly.

`AHRS: waiting for home` on `arm throttle` means EKF/GPS hasn't converged
yet - normal, not an error. The wait varies run to run (measured 24 s to
90+ s on the same machine, since it tracks sim time and RTF isn't
constant). Just retry `arm throttle` every few seconds until it's accepted.

**CHECK** `ros2 topic echo /ap/pose/filtered --once` shows `z: ~1.25`.

> **Take off BEFORE starting SLAM.** A 2D lidar maps one horizontal slice, and
> which slice depends on where the aircraft is. On the pad the lidar sits at
> 0.275 m, where it can see the 0.28 m survivor models; those get written into
> the map as obstacles, and nothing at flight altitude will ever clear them.
> Start SLAM in the air and every scan in the map is taken from the same
> height the drone will fly at.
>
> Home does not depend on this ordering. The explorer takes the takeoff point
> from the `odom` frame's origin, not from whatever pose it first happens to
> see, and re-derives it through `map`→`odom` each cycle so it stays on the pad
> as SLAM corrects itself.

### 3 · SLAM + Nav2 + bridge + explorer

```bash
cd ~/ardu_ws && source install/setup.bash
ros2 launch nidar_sim nidar_bringup.launch.py
```

**CHECK**

```bash
ros2 topic echo /map --once | head -3                      # a header
ros2 service call /bt_navigator/get_state lifecycle_msgs/srv/GetState
```

`bt_navigator` must say `active`. Anything else → §0, start over.

Then do nothing. It explores, returns to the pad, and lands.

---

## How the run ends

No intervention needed. The explorer:

1. runs out of frontiers, clears its own blacklists and sweeps once more
   before believing it (`final_sweep`);
2. **returns to launch** — navigates back to the takeoff point, taken from the
   `odom` frame's origin and re-derived through `map`→`odom` each cycle so it
   tracks SLAM's corrections instead of drifting off the pad;
3. **lands there**, asking the autopilot for LAND over `/ap/mode_switch`.

One line to grep for:

```
RUN COMPLETE | explored 118 m2 (98.4% of the mapped area known) | home error 0.08 m
```

If Nav2 cannot route home in `return_home_max_attempts` tries it hands over to
the autopilot's own RTL, which climbs above the 2.6 m walls, flies the straight
line and lands.

Turn either off:

```bash
ros2 launch nidar_sim exploration.launch.py return_home_on_complete:=false
ros2 launch nidar_sim exploration.launch.py land_on_arrival:=false
```

---

## Arenas

```bash
ros2 launch nidar_sim arena.launch.py arena:=2
```

| arena | grid | corridor | footprint | character |
|---|---|---|---|---|
| 1 | 13×13 | 1.00 m | 14.4 × 14.4 m | dense maze, deep dead ends |
| 2 | 13×13 | 1.00 m | 14.4 × 14.4 m | balanced corridors and junctions |
| 3 | 13×13 | 1.00 m | 14.4 × 14.4 m | open plan, many ring routes |

Each has a sibling `.yaml` (survivor ground truth) and `.txt` (ASCII plan) in
`worlds/`.

---

## Numbers

| | |
|---|---|
| flight height | **1.17 m** (middle of the net/floor window) |
| corridor | 1.00 m, walls 0.10 m |
| wall height | 2.60 m; net at 2.485 m |
| airframe | Lumenier QAV-R 2, 7" — **480 mm** tip-to-tip (blades at 0.24) |
| `robot_radius` | **0.241** — blade tips (0.240) + 1 mm |
| free lane in a corridor | 1.00 − 2 × 0.241 = **0.518 m** |
| cruise | 1.0 m/s (`vx_max`), `WP_SPD` 1.1 |
| accel | 1.7 m/s² per axis (13.8° lean), `WP_ACC` 2.6, `PSC_NE_JERK` 8 |
| speckle filter | components < 12 cells cleared |
| max lean | 14° (`ATC_ANGLE_MAX`); net/floor reachable at 15.4° |
| lidar trusted to | 4.2 m mark / 4.5 m raytrace |

### Why 1.25 m

Two rules, and the altitude has to satisfy **both** at the lean limit:

```
raytrace_max_range  <  lidar_z           / tan(ATC_ANGLE_MAX)   # floor
raytrace_max_range  <  (2.485 - lidar_z)  / tan(ATC_ANGLE_MAX)   # net

This runs BOTH ways. Shortening raytrace_max_range buys lean angle, and lean
angle is acceleration (a = g·tan θ). 4.8 m puts the net out of reach until
13.6°, which is what allows ATC_ANGLE_MAX 12 and 1.4 m/s² per axis.
```

Break the first and the floor lands in the map as obstacles. Break the second
and rays sail over the walls, return nothing, and **ray-trace real walls out of
the map** — which is what 2.0 m did: SLAM slid 6 m off ground truth and the
drone flew through walls it no longer knew about.

At 1.17 m the lidar sits at 1.245 m, so both limits are ≈ 7 m against a 5.5 m
raytrace. The window is `lidar_z ∈ (0.97, 1.51)`.

**Change it in one place only:**

```bash
ros2 launch nidar_sim nidar_bringup.launch.py flight_altitude:=1.4
```

That drives the bridge's hold altitude *and* `scan_stabilizer`'s ceiling
together. Setting it in one file and not the others is what broke the stack.

---

## The speckle filter

`map_filter` republishes `/map` as `/map_filtered`, clearing occupied
components too small to be a wall. Nav2's static layer and the explorer both
read the filtered topic.

A phantom and a wall are identical at cell level, so size is the only thing
that separates them:

| | cells at 0.05 m |
|---|---|
| shortest real wall (1.10 × 0.10 m) | 44 |
| typical junction run | 200–4000 |
| measured phantom island | 1–6 |
| **threshold** (`min_blob_cells`) | **12** |

Deletion is per **component** with 8-connectivity, so a blob touching a wall
belongs to that wall and is kept. A partly-seen wall is delayed a scan or two,
never lost. It only ever turns occupied into free, so it cannot make the
planner braver about space it has not seen.

```bash
ros2 launch nidar_sim nidar_bringup.launch.py min_blob_cells:=20   # stricter
ros2 launch nidar_sim nidar_bringup.launch.py map_filter:=false    # off
```

Watch it work:

```
map_filter: 20 maps | removed 440 speckle island(s), 1080 cells,
            largest 11 cells | kept 160 structure(s)
```

**What it does not fix.** A wall written in the wrong place is a *large*
component and is correctly kept. Persistent wall-shaped blobs mean SLAM pose
error, not speckle — check `slam_err` in the test harness and fix it at the
source (flight altitude, loop closure), not by raising this threshold.

---

## Is it working?

```bash
ros2 topic hz /clock                  # ~1000 Hz
ros2 topic hz /scan                   # ~10 Hz
ros2 topic hz /scan_safe              # ~10 Hz
ros2 topic echo /map --once | head -3 # a header
ros2 topic hz /cmd_vel                # ~20 Hz once a goal is active
ros2 topic info /ap/cmd_vel           # Publisher 1, Subscription 1
```

Run them in order. The first silent one is the fault.

The drone should move within ~15 s of the explorer logging its first goal, and
reach ~0.6 m/s on a straight corridor leg.

---

## Troubleshooting

| symptom | cause / fix |
|---|---|
| No `applied and verified` at §1 | Vehicle is on baseline lean limits. §0, restart |
| `launch configuration '…' does not exist` | Stale install — `colcon build --packages-select nidar_sim --symlink-install` |
| `bt_navigator` not `active` | Stale Fast DDS shared memory. §0 |
| Phantom specks in the map | Check `map_filter` is running and that Nav2/explorer read `/map_filtered` |
| Large wall-shaped blobs offset from real walls | SLAM pose error, not speckle. The filter will not (and should not) remove these |
| Phantom dots in the costmap | §1 check failed, or the flight height and the scan filters disagree |
| Walls appear then vanish | Flying too high — rays are clearing the wall tops. Check `flight_altitude` |
| Explorer: `THE DRONE IS ON THE GROUND` | Check `/odometry` — altitude comes from there, never from `map→base_link`, whose z is ~0 by construction |
| Survivor-shaped blobs in open floor | SLAM was started on the pad. Take off first (§2) |
| Drone sits still, `Failed to make progress` | `controller_server`'s `odom_topic` must be `/odometry`, not `odom` |
| `/cmd_vel` flowing, drone still | `ros2 topic info /ap/cmd_vel` needs 1 publisher **and** 1 subscriber |
| `failed to create plan` | Drone is in blocked space. Nudge it off the wall in MAVProxy |
| Two drones on screen | Two stacks running. §0 |

---

## Testing

`test/nidar_eval/` scores a flight against the arena's own SDF and Gazebo's
ground truth — coverage, phantom cells, SLAM error, true wall clearance.

```bash
cd ~/ardu_ws/test/nidar_eval
./run.sh mytag 2400 1                 # tag, seconds, arena

python3 score_runs.py results/* --world ~/ardu_ws/src/nidar_sim/worlds/nidar_arena_1.sdf
python3 plot_map.py --run results/mytag --world ~/ardu_ws/src/nidar_sim/worlds/nidar_arena_1.sdf
```

| column | good |
|---|---|
| `floor_mapped_pct` | → 100 |
| `wall_recall_pct` | high |
| `phantom_cells` | 0 |
| `slam_err_p95` | < 0.05 m |
| `clr_min` | > 0.24 m (blade tips) |
| `home_err_m` / `final_alt` | < 0.3 / ≈ 0.2 |

Give it a budget that outlasts exploration (2400 s), or the harness cleanup
will kill the run before it returns home.

---

## Never

* Trust a run without `applied and verified`.
* Start SLAM on the pad — take off first.
* Skip §0.
* Take off to anything but `flight_altitude`.
* `pkill -f` — it matches your own shell. Use `nidar-clean.sh`.
* Run two `run.sh` at once, or edit it mid-run — bash reads scripts lazily.
* Run terminal 1 from anywhere but `~/ardu_ws`.

---

## Tuning oscillation

All in `config/nav2_params.yaml` under `controller_server: FollowPath:`, except
the altitude gain. Rebuild and relaunch Nav2 after each change:

```bash
cd ~/ardu_ws && colcon build --packages-select nidar_sim --symlink-install && source install/setup.bash
```

### Why it oscillates

Three causes, in the order they bite:

**1 · Yaw — the big one.** `cmd_vel_bridge` publishes a **body-frame** twist and
ArduPilot rotates it by the airframe's **current** attitude. While the drone is
yawing, the frame the command was computed in has already turned by the time
the command is applied. At 0.4 rad/s that is 23 °/s, so one control period plus
transport lag smears the commanded direction by a degree or two every cycle.
That smear reads as cross-track error, the controller corrects it, the
correction includes more yaw — a self-sustaining weave. It looks like bad
tuning; it is a frame-timing loop.

**2 · Cross-track gain.** `PathAlignCritic.cost_weight` is the gain on distance
from the planned line. High gain against a path that is replanned every second,
with actuation lag supplying the phase shift, is a textbook oscillator.

**3 · A flip-flopping global path.** At a junction two routes can cost almost
the same, so NavFn alternates between them each replan and PathAlign dutifully
yanks the drone toward whichever won. Watch `/plan`'s point count: if it jumps
between two very different values in place, this is your cause.

### Knobs, in the order to try them

| # | parameter | now | try | effect |
|---|---|---|---|---|
| 1 | `wz_max` | **0.0** | 0.0 | Yaw rate. Keep at 0 — a 360° lidar needs no heading |
| 2 | `PathAlignCritic.cost_weight` | **16.0** | 10–20 | Cross-track gain. **Lower this first.** Too low → wall-hugging on corners; too high → weaving |
| 3 | `gamma` | **0.05** | 0.015–0.1 | Control-effort penalty. Higher = smoother, less responsive |
| 4 | `vx_std` / `vy_std` | **0.25** | 0.15–0.3 | Sampling spread. Lower = less jitter, but explores less |
| 5 | `temperature` | **0.3** | 0.25–0.4 | Softmax width. Higher averages more trajectories = smoother |
| 6 | `vx_max` / `vy_max` | **0.6** | 0.4–0.7 | Speed. Lag-driven oscillation always worsens with speed |
| 7 | `CostCritic.cost_weight` | **4.0** | 3–6 | Obstacle aversion. Raise if it clips corners after lowering #2 |
| 8 | `PathFollowCritic.cost_weight` | **8.0** | 5–12 | Pull *along* the path. Raise if it dawdles after lowering #2 |

**Vertical** oscillation is a different loop — `altitude_gain` (0.8) and
`max_climb_rate` (0.4) in `nidar_sim/cmd_vel_bridge.py`. Halve the gain if the
drone porpoises.

### Method

Change **one** at a time and measure, don't eyeball:

```bash
cd ~/ardu_ws/test/nidar_eval && ./run.sh tune1 600 1
python3 score_runs.py results/tune1 --world ~/ardu_ws/src/nidar_sim/worlds/nidar_arena_1.sdf
```

`clr_min` must stay above **0.24** (the airframe's circumscribed radius) — that is the number
oscillation actually threatens. `path_m` for a given coverage is the cheapest
proxy for weaving: a straighter flight covers the same area in less distance.

Watch it live:

```bash
ros2 topic echo /cmd_vel_nav           # sign flips in linear.y = weaving
ros2 topic hz /plan                    # and echo it: a jumping point count = cause 3
```
