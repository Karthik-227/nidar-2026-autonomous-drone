# RUNBOOK — copy, paste, fly

Six terminals. Wait for each **CHECK** before starting the next.
(Long version with explanations: [GUIDE.md](GUIDE.md))

---

### 0 · Always do this first
```bash
~/ardu_ws/scripts/nidar-clean.sh
```
**CHECK** it prints `--- fastdds shm swept: N -> 0 ---` and then
`--- survivors (should be empty) ---` with nothing under it.

That sweep matters as much as the process kill. Fast DDS keeps a shared-memory
port file and semaphore per process under `/dev/shm`; a hard kill leaves them,
and the next run cannot always reopen one:

```
RTPS_TRANSPORT_SHM Error: Failed init_port fastrtps_port7429: open_and_lock_file failed
```

It fails **per port** and it is **silent above the transport** — nodes start,
topics look connected, and some traffic just never arrives. One run lost the
`lifecycle_manager` → `velocity_smoother` configure request exactly this way:
Nav2 stayed **inactive**, `bt_navigator` rejected every goal, the explorer
logged `Goal rejected (4/4 consecutive)` and `STUCK: moved 0.00 m`, and the
drone hovered in place. Every process was alive and no log said why.

nidar_sim's launch files now also point Fast DDS at
`config/fastdds_no_shm.xml`, which turns the shared-memory transport off and
uses localhost UDP — everything here is on one host, so the cost is nil and the
whole failure class goes away. For a node you start **by hand**, or for
`ardupilot_gz`'s own launch, export it yourself:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$(ros2 pkg prefix nidar_sim)/share/nidar_sim/config/fastdds_no_shm.xml
```

Do not skip this, and do not substitute `pkill -f`. One orphaned `ros_gz_bridge`
keeps publishing `/clock` from a Gazebo that no longer exists; sim time then
jumps between two sources, every TF buffer clears itself continuously, and
Nav2's `controller_server` cannot even reach the configured state. Nothing in
any node's log points at the cause. This was the single largest time sink in
bringing the stack up.

`pkill -f <pattern>` is unreliable here for two reasons: it matches the pattern
against the shell running it (so `pkill -f frontier_explorer` from a script
containing that word kills the script), and processes started under `setsid`
outlive Ctrl+C in the terminal that launched them.

---

### 1 · Simulation
```bash
export GZ_IP=127.0.0.1
ros2 launch nidar_sim sim.launch.py rviz:=false
```
**CHECK** log contains `got segment base_link` · `ros2 topic hz /clock` ≈ 1000 Hz
· the `[nidar] param_profile:=...` line names the `.parm` it appended

`sim.launch.py` is a wrapper around `ardupilot_gz_bringup iris_maze.launch.py`
— same world, same SITL, same RViz — that adds `param_profile` (which lean/accel
limits SITL flies with) and `wipe` (clear the eeprom so a profile can apply at
all). It also defaults `lidar_dim` to 2, which this stack always needs. See
[Param profiles](#param-profiles) below; the short version is

```bash
ros2 launch nidar_sim sim.launch.py rviz:=false param_profile:=tuning wipe:=true
```

The upstream command still works and is unchanged:
`ros2 launch ardupilot_gz_bringup iris_maze.launch.py lidar_dim:=2 rviz:=false`
— it just applies no profile.

`rviz:=false` matters. This launch opens its **own** RViz by default, and step 4
opens the one that can actually show the map — so the default gives you two.
The upstream one has no Map or Costmap display, renders the 1081-point scan and
both costmaps a second time, and floods its terminal with

```
Warning: TF_OLD_DATA ignoring data from the past for frame base_link
```

which looks like a TF bug and is not one. It is also ~50% of a core, on a
machine where Gazebo's lockstep loop with SITL is what actually needs the CPU.

`GZ_IP` matters too, for a measured reason: gz-transport sends discovery
multicast out of *every* interface, and a host with `docker0` up-but-no-carrier
(or wifi + ethernet both up) gets `Exception sending a multicast message:
Network is unreachable` on each attempt. One run's Gazebo log held **29,392** of
them. Everything in this sim is on localhost, so pinning discovery there
removes the flood entirely — verified 29,392 → 0.

---

### 2 · Arm + take off
```bash
mavproxy.py --console --map --aircraft test --master=:14550
```
then at the `>` prompt, one at a time:
```
mode 4
arm throttle
takeoff 1.25
```
`mode 4` (GUIDED's number), not `mode guided` - MAVProxy's mode-name lookup
defaults to Plane's table until the real heartbeat lands, and in Plane's
table GUIDED is 15, which is AUTOTUNE on a Copter. Typed fast enough after
connecting, `mode guided` silently arms Autotune instead, the vehicle stays
in STABILIZE, and `takeoff` gets rejected with no obvious reason why. The
numeric form skips that lookup entirely and is correct every time.

If `arm throttle` is refused with `AHRS: waiting for home`, that's EKF/GPS
not converged yet - not a failure to fix, just not ready. How long that
takes varies run to run (measured 24 s once, 90+ s another time on the
same machine) because it tracks *simulated* time, not wall-clock, and the
sim's real-time factor isn't constant. Just retry `arm throttle` every
few seconds; it's harmless to retry and succeeds the moment EKF converges.
**CHECK** drone is airborne in Gazebo.

---

### 3 · SLAM (+ scan stabilizer)
```bash
ros2 launch nidar_sim slam.launch.py
```
**CHECK** `Registering sensor: [Custom Described Lidar]` · `ros2 topic hz /map` ≈ 1 Hz ·
`scan_stabilizer` prints a `stabilised N scans | mean tilt ...` line every 10 s
(if it ever says `NOT STABILISING`, stop — it has no attitude source and
everything downstream is getting raw tilted scans)

---

### 4 · Nav2 + RViz2 + cmd_vel bridge
```bash
ros2 launch nidar_sim nav2.launch.py
```
**CHECK** `Managed nodes are active` · RViz2 shows black walls with magenta/cyan bands

---

### 5 · Manual test — do this before autonomy
In RViz2 click **2D Goal Pose** somewhere open.
**CHECK** drone flies there. If not, stop and fix — autonomy will only hide the problem.

---

### 6 · Autonomous exploration
```bash
ros2 run nidar_sim frontier_explorer --ros-args -p use_sim_time:=true
```
**CHECK** one line per cycle:
```
Explored 212 m2 (34.2% of mapped area) | 5 frontier(s) | goal -> (2.41, -3.07)
```

---

## Shortcut (steps 3–6 in one)
```bash
ros2 launch nidar_sim nidar_bringup.launch.py
```
`explore:=false` → stop before autonomy · `rviz:=false` → no RViz

---

## Drone armed and hovering but never moves?

Check Nav2 actually **activated** — a configured-but-inactive Nav2 rejects every
goal, and nothing in any log says so:

```bash
ros2 service call /bt_navigator/get_state lifecycle_msgs/srv/GetState
```

`active` is correct. `inactive`, or no reply, means the lifecycle bringup
stalled — look at the end of `~/.ros/log/<latest>/` for the last
`Configuring <node>` line with no matching `Activating`, and see step 0 about
stale Fast DDS shared memory.

---

## Stuck? Run these in order. **First silent one is the fault.**
```bash
ros2 topic hz /clock                            # ~1000 Hz
ros2 topic info /clock                          # exactly 1 publisher
ros2 topic hz /scan                             # ~8 Hz
ros2 run tf2_ros tf2_echo base_link base_scan   # resolves
ros2 topic hz /scan_safe                        # ~8 Hz
ros2 run tf2_ros tf2_echo base_link base_scan_level   # stabilizer broadcasting?
ros2 topic hz /map                              # ~1 Hz
ros2 run tf2_ros tf2_echo map base_link         # resolves
ros2 topic hz /cmd_vel                          # moving only
ros2 topic hz /ap/cmd_vel                    # moving only
```

| Silent | Cause |
|---|---|
| `/clock` | >1 Gazebo, or bridge on a dead gz topic → step 0 |
| `/scan` | wrong lidar — needs `lidar_dim:=2` |
| `base_link→base_scan` | `robot_state_publisher` died → reapply `patches/` |
| `/scan_safe` | stabilizer not running → use `nidar_sim slam.launch.py`, not raw slam_toolbox |
| `/map` | SLAM has no scan — check the two above |
| `map→base_link` | SLAM not localising |
| `/cmd_vel` | Nav2 not active, or no goal |
| `/ap/cmd_vel` | bridge not running — but step 4 already starts one, so check `ros2 node list \| grep cmd_vel_bridge` **before** starting another. Two bridges fight: see "Never do" |

---

## Tuning for coverage

| Symptom | Parameter | Try |
|---|---|---|
| Ignores far rooms | `distance_decay_m` | 4.0 → **6.0** |
| Wanders, skips nearby | `distance_decay_m` | 4.0 → **2.5** |
| Still oscillates | `direction_weight_m` | 2.5 → **4.0** |
| Misses narrow doorways | `min_frontier_cells` | 8 → **5** |
| Stops too early | `empty_streak_limit` | 5 → **8** |
| Cancels goals constantly | `max_safe_tilt_deg` | 30 → **40** |

```bash
ros2 run nidar_sim frontier_explorer --ros-args \
  -p use_sim_time:=true -p distance_decay_m:=6.0 -p min_frontier_cells:=5
```

---

## Param profiles

Which lean and acceleration limits ArduPilot flies with. **A multirotor
accelerates by leaning** (`tan θ = a/g`), the 2D lidar leans with it, and a
scan plane tilted far enough puts rays into the floor — which Nav2 marks and
inflates into phantom blobs sitting in open floor. Capping accel caps the lean.

| Profile | `WP_ACC` | `WP_ACC_CNR` | `ATC_ANGLE_MAX` | `WP_SPD` | `PSC_NE_JERK` | steady-state lean |
|---|---|---|---|---|---|---|
| `baseline` (default) | 2.5 | 0 *(= 2×WP_ACC)* | 30 | 10 | 5 | 14.3° (27° at corners) |
| `tuning` | 1.0 | 1.0 | 10 | 1.0 | 2 | 5.8° |
| `none` | — appends nothing, SITL keeps whatever the eeprom holds — |||||

```bash
ros2 launch nidar_sim sim.launch.py param_profile:=tuning wipe:=true
```

Files: `config/nidar_baseline.parm`, `config/nidar_tuning.parm`. Each carries the
reasoning per param. `baseline` is the revert — it pins the values the stack flew
before the tuning existed, all of which are also this build's compiled defaults.

**`wipe:=true` is not optional when you switch.** `--defaults` does not *set*
params, it sets their *default* values, and a default is invisible for any param
the eeprom already stores a value for. `eeprom.bin` sits in the directory you
launch from (`~/ardu_ws`) and survives every restart, so switching
`param_profile` on its own changes **nothing** for a param already in there:
the launch comes up announcing `tuning` and flies `baseline`, with nothing in any
log to say so. The old eeprom is moved to `eeprom.bin.bak-<timestamp>`, not
deleted — a wipe discards *every* stored param, including anything you set by
hand over MAVLink.

Upstream cannot do this for us: `ardupilot_sitl`'s launch does expose a `wipe`
argument, but `ardupilot_gz`'s `robot.launch.py` hardcodes `"wipe": "False"` on
the way through, so it is unreachable. `sim.launch.py` moves the file instead.

**Switching without a wipe or a restart** — this is what to use for A/B runs:

```bash
python3 ~/ardu_ws/src/nidar_sim/scripts/apply_param_profile.py tuning
python3 ~/ardu_ws/src/nidar_sim/scripts/apply_param_profile.py --show   # read only
```

`PARAM_SET` into the live vehicle, read back and verified, six params touched
and nothing else. It writes to the eeprom too, so it then outranks any defaults
file. Do it disarmed or in a steady hover, not mid-manoeuvre, or the ATT log is
unreadable. It does **not** download the param table — a full `param fetch` here
goes through MAVFTP and times out (`module unload ftp` first if you need one in
MAVProxy).

**CHECK** that a profile actually landed — never trust the launch banner:

```bash
python3 ~/ardu_ws/src/nidar_sim/scripts/apply_param_profile.py --show
```

### The Nav2 half

Nav2's own limits mirror these in `config/nav2_params.yaml`, deliberately set
**at or below** ArduPilot's so ArduPilot never has to clip a Nav2 command:

| | `FollowPath` | `velocity_smoother` | ArduPilot |
|---|---|---|---|
| velocity | `max_vel_x/y` 1.0 | `max_velocity` 1.0 | `WP_SPD` 1.0 |
| accel | `acc_lim_x/y` 0.9 | `max_accel` 0.9 | `WP_ACC` 1.0 |
| decel | `decel_lim_x/y` -0.9 | `max_decel` -0.9 | `WP_ACC` 1.0 |

Move one side and move the other. Whichever layer is lower is the real limit, and
it has to be Nav2 — DWB *simulates* trajectories using its own numbers, so a
decel limit ArduPilot cannot deliver makes DWB believe it can stop in a distance
it cannot, and the obstacle critic then clears a trajectory ending inside a wall.

There is one Nav2 file, not two: the old values live in git, in the commit
before the mirror (`b916749`), and in `# was` comments next to each one. To fly
the Nav2 baseline for an A/B run:

```bash
git -C ~/ardu_ws/src/nidar_sim show b916749:config/nav2_params.yaml \
  > /tmp/nav2_baseline.yaml
ros2 launch nidar_sim nidar_bringup.launch.py params_file:=/tmp/nav2_baseline.yaml
```

(`params_file` is declared on `nav2.launch.py` and inherited through
`nidar_bringup.launch.py`.) To revert for good: `git revert` that one commit.

---

## Walls look wrong in RViz2?

The map should show walls as **solid black lines**. Three distinct failures, and
they look different:

| What you see | Cause | Fix |
|---|---|---|
| Black dots with grey gaps between them, worse further from the drone | lidar angular resolution too coarse at range | `samples` in `lidar_2d/model.sdf` (now 1081) |
| Free-space "fans" of white streaks radiating into grey, no black at the end | returns beyond `max_laser_range` are traced as free but never marked | `max_laser_range` in `mapper_params.yaml` (now 8.0) |
| A wall that was black goes white as the drone flies past | tilted beams skimming over it and ray-tracing through | `scan_stabilizer` must be running — check its health line |

And in the **costmap**: if there is no grey anywhere, `track_unknown_space` is
off and every cell the lidar has never seen reads as confirmed-free. The
planner will route straight through unexplored space and find the wall the hard
way. Both that and `planner_server.allow_unknown` are set correctly in
`config/nav2_params.yaml` — if you replace that file, keep them.

---

## Never do
- Source two workspaces → two `navigate_to_pose` servers racing your goals
- Run raw `slam_toolbox online_async_launch.py` → no stabilizer → no `/scan_safe` → no map
- Run `scan_tilt_gate` → deprecated; it freezes the scan stream and starves SLAM
- Run `topic_relay_node` **and** `cmd_vel_bridge` together → two publishers on `/ap/cmd_vel`
- Run **two** `cmd_vel_bridge`s (step 4 starts one; don't also start one by hand).
  Their idle-quiet state is per-instance, so one goes silent to let ArduPilot
  hold position while the other keeps sending zeros — which sinks the drone and
  blocks takeoff, while the first one's log says it went quiet. The bridge now
  refuses to start as a duplicate, so this fails loudly instead. Want it by
  hand? `ros2 launch nidar_sim nav2.launch.py bridge:=false`
- Leave Nav2 or SLAM running while you restart Gazebo. Sim time restarts at 0,
  every TF buffer still holds stamps from the old run, and all new transforms
  are rejected as `TF_OLD_DATA ... from the past`. SLAM's `map->odom` is then
  computed from a corrupted graph — one run had it at **z = +7.2 m**, so the
  drone rendered 7 m above the floor and 4 m off origin while actually sitting
  at (0, 0) on the ground. Restart Gazebo → restart everything (step 0)
- Switch `param_profile` without `wipe:=true` (or
  `scripts/apply_param_profile.py`) → the banner says one profile, the vehicle
  flies the other. See [Param profiles](#param-profiles)
- `git pull` in `src/ardupilot_gz` without reapplying `patches/`
