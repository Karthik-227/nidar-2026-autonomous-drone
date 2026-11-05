# Competition Arenas

Three generated Gazebo worlds at Mission-Brief spec. `sim.launch.py` still flies
ardupilot_gz's `maze.sdf` unchanged; these are reached through `arena.launch.py`.

---

## 0 · Clean first, every time

```bash
~/ardu_ws/scripts/nidar-clean.sh
```

Expect `--- fastdds shm swept: N -> 0 ---` then `--- survivors (should be empty) ---`
with nothing under it. Run it between every arena switch. Never `pkill -f`.

---

## 1 · Fly

```bash
# terminal 1 — Gazebo + SITL   (must run from ~/ardu_ws: SITL reads eeprom.bin from cwd)
cd ~/ardu_ws && source install/setup.bash
ros2 launch nidar_sim arena.launch.py arena:=1
```

```bash
# terminal 2 — arm and take off
mavproxy.py --console --map --aircraft test --master=:14550
#   mode 4          (GUIDED's number - "mode guided" races the first
#                    heartbeat and can silently arm Autotune instead)
#   arm throttle
#   takeoff 1.25
```

```bash
# terminal 3 — SLAM + Nav2 + bridge + explorer
cd ~/ardu_ws && source install/setup.bash
ros2 launch nidar_sim nidar_bringup.launch.py
```

**SLAM must be up before takeoff** or slam_toolbox anchors `map` in mid-air, the
explorer reads altitude as 0.00 and rejects every frontier.

World only, no SITL:

```bash
ros2 launch nidar_sim arena.launch.py arena:=3 spawn_robot:=false
```

### Arguments

| argument | default | notes |
|---|---|---|
| `arena` | `1` | `1`/`2`/`3`, or a full world name (`nidar_arena_4`) |
| `param_profile` | `tuning` | `tuning`/`baseline`/`none`. Enforced over MAVLink at +25 s |
| `wipe` | `false` | Move `eeprom.bin` aside. Not needed any more — enforcement bypasses the eeprom |
| `use_gz_sim_gui` | `true` | `false` = headless, faster for A/B runs |
| `spawn_robot` | `true` | `false` = world only |
| `rviz` | `false` | ardupilot_gz's RViz. Off: `nidar_bringup`'s is the useful one |
| `lidar_dim` | `2` | Stack consumes a 2D `/scan` |

---

## 2 · The check that decides everything

At +25 s terminal 1 must print:

```
WP_ACC          0.9   was 2.5
WP_ACC_CNR      0.9   was 0
ATC_ANGLE_MAX   10    was 30
WP_SPD          1.0   was 10
PSC_NE_JERK     2     was 5
PSC_ANGLE_MAX   0     ok

nidar_tuning.parm applied and verified
```

Missing, or `Some writes did not land` → **stop**. The vehicle is flying baseline
limits (`ATC_ANGLE_MAX 30`, `WP_ACC 2.5`) and the map will fill with phantom
obstacles. Check any time:

```bash
python3 ~/ardu_ws/src/nidar_sim/scripts/apply_param_profile.py --show
```

---

## 3 · Geometry

| | value | brief |
|---|---|---|
| grid | 13 × 13 cells | modular, uniform |
| clear corridor | 1.00 m | ≥ 1 m |
| wall thickness | 0.10 m | — |
| cell pitch | 1.10 m | — |
| model footprint | 14.40 × 14.40 m | ≤ 15 × 15 m |
| rooms | 2 × 2 cells → 2.10 m clear | 2 m × 2 m |
| start bay | 3 × 3 cells → 3.20 m clear | — |
| wall height / net | 2.60 m / 2.50 m | ≥ 8 ft (2.4384 m) |
| launch pad | 0.61 m | 2 ft × 2 ft |
| survivors | 6, ≤1 per room | up to 6 |

Rooms are 2.10 m not 2.00 m: a uniform 1.0 m module with 0.10 m panels gives a
2-cell room 2.10 m of clear span. 2.00 m would force 0.95 m corridors, breaking
the 1 m floor. 13 × 13 is the ceiling — 14 cells is 15.5 m, over the cap.

| arena | rooms | loop-prob | stresses |
|---|---|---|---|
| **1** | 8 | 0.10 | dense maze, deep dead ends — backtracking |
| **2** | 7 | 0.20 | balanced corridors and junctions |
| **3** | 9 | 0.32 | open plan, ring routes — goal selection, revisit penalties |

Pad is at the **world origin**, in the +x/+y corner cell of the start bay, open
to the maze on both interior sides. "Did it come home?" = "is `base_link` near
(0, 0)?". Entry and exit are the same point, as the brief requires.

---

## 4 · Matching parameters

Set from the real airframe (430 mm, QAV 7" + guards) and the 1.25 m flight height.

| parameter | value | why |
|---|---|---|
| `robot_radius` | 0.22 | 430 mm airframe → 0.215 m radius |
| `inflation_radius` | 0.35 | 0.13 m standoff; leaves a 0.30 m lane in a 1.0 m corridor |
| `obstacle_max_range` | 5.0 | floor needs 11.3° > the 10° `ATC_ANGLE_MAX` limit |
| `raytrace_max_range` | 5.5 | net needs 15.3° |
| `max_laser_range` (slam) | 5.0 | matches the costmaps |
| `min_laser_range` (slam) | 0.30 | prop guards sit at 0.215 m |
| `acc_lim_x/y` | 0.6 | per-axis; L-turn diagonal = 0.85 m/s² = 4.9° lean |

Rule when altitude or the angle limit changes:

```
obstacle_max_range  <  altitude / tan(ATC_ANGLE_MAX)
```

**Survivors are 0.28 m tall.** At the 1.25 m flight height they sit 0.97 m below the
scan plane, inside `max_hit_dz_m` (0.75) — a leaning ray at ~5 m can mark one as
an obstacle. Fly 1.0 m, not lower, and detect them with the camera.

---

## 5 · Scoring

```bash
cat ~/ardu_ws/src/nidar_sim/worlds/nidar_arena_1.yaml
```

```yaml
survivors:
  - id: survivor_1
    grid_box: H2
    xy: [-5.023, -12.139]
    room: 1
```

`xy` in world/`map` coords, pad at origin. `grid_box`: columns A→ run west→east
(+x), rows 1→ run south→north (+y). That label is what the GCS must produce.

`.txt` beside it is the ASCII plan (north up, east right; digits = survivors,
letters = room cells, `P` = pad). `arenas.png` is the three plans side by side.

---

## 6 · Generate new

```bash
cd ~/ardu_ws/src/nidar_sim/scripts
python3 generate_arena.py --out ../worlds --name nidar_arena_4 --seed $RANDOM
python3 verify_arena.py ../worlds/nidar_arena_4.sdf
cd ~/ardu_ws && colcon build --packages-select nidar_sim
```

**The rebuild is mandatory** — `arena.launch.py` reads `install/`, not `src/`. A
failed `arena:=` prints every arena it did find; that list is the installed one.

Regenerate and verify all three:

```bash
~/ardu_ws/src/nidar_sim/scripts/make_arenas.sh
```

Seeds *are* the arenas: same seed + same corridor width → same map anywhere.

| flag | default | notes |
|---|---|---|
| `--seed` | *required* | reproduces an arena exactly |
| `--cells` | `13` | widen the corridor → drop cells to stay under 15 m |
| `--corridor` | `1.0` | the brief's floor |
| `--rooms` / `--survivors` | `8` / `6` | survivors ≤ rooms, one per room |
| `--loop-prob` | `0.18` | 0 = perfect maze |
| `--start-cells` | `3` | start bay side, in cells |
| `--start` | `open` | `open`/`chamber`/`porch` — §8 |
| `--entry-corner` | `ne` | §7 |

Verify and redraw:

```bash
python3 verify_arena.py ../worlds/*.sdf
python3 plot_arena.py ../worlds/nidar_arena_*.sdf -o ../worlds/arenas.png
```

`verify_arena.py` rasters the **emitted SDF**, flood-fills from the pad, and
checks pad-in-free-space, every survivor reachable, shell sealed. Run it on
anything you generate — the merge/mirror/translate stage can emit a world that
parses, looks right, and never finishes an exploration run.

---

## 7 · "Top right" depends on the viewer

`--entry-corner` names the corner on world axes: north = +y, east = +x. Default
`ne` is top-right in Gazebo and in any x-right/y-up plot.

**RViz TopDownOrtho draws +x up and +y left**, so `ne` lands top-*left* on screen.
For top-right in RViz:

```bash
python3 generate_arena.py --out ../worlds --name nidar_arena_1_se --seed 101 --entry-corner se
```

Same topology, mirrored.

---

## 8 · Why the arena is sealed

A gap in the perimeter lets the 2D lidar see out, slam_toolbox maps unbounded
free space, and the explorer correctly chases that frontier to infinity. The run
never terminates and it looks like an explorer bug.

So the perimeter is unbroken and entry/exit is an open bay cut into a corner.

* `--start chamber` — same bay behind one 1 m doorway, still inside 15 m
* `--start porch` — external vestibule, closest to the real event, costs two
  cells in one axis (14.4 × 16.6 m), which may exceed the cap

---

## 9 · Stuck? First silent one is the fault

```bash
ros2 topic list | grep -E '^/clock$|^/scan$'
ros2 topic hz /scan
ros2 topic echo /ap/pose/filtered --once
ros2 topic echo /map --once | head -5
ros2 service call /bt_navigator/get_state lifecycle_msgs/srv/GetState
```

Healthy on the pad before takeoff: `/scan` ~8.8 Hz, 1081 returns, min ~0.70 m,
max ~9.8 m. That long sightline is the bay being open — sealed in a 2 × 2 box it
would cap near 3 m. `/bt_navigator` must be `active`.

| symptom | cause |
|---|---|
| No `enforce_param_profile` block | Profile not applied — vehicle is on baseline limits. §2 |
| `arena:=4` fails, error lists 1/2/3 | Generated but not installed. `colcon build --packages-select nidar_sim` |
| `arena:=1_wide` fails | Removed. The 430 mm airframe patch made spec maps flyable |
| No `/scan`, no `/clock`, `/ap/*` missing | Bridge is namespaced by world name. Launch through `arena.launch.py` |
| `bt_navigator` `inactive`, goals rejected | Stale Fast DDS shm. §0 |
| Drone not at (0, 0) | You passed `x:`/`y:`. Don't — pad is the origin |
| Explorer leaves the building | Not one of these worlds, or a hole in the perimeter. `verify_arena.py` → `sealed=False` |
| Phantom dots in the map | Profile not enforced, or `obstacle_max_range` > `altitude/tan(ATC_ANGLE_MAX)`. §2, §4 |
| Two drones, `arm throttle` does nothing | Two stacks. §0 |

---

## 10 · Files

```
nidar_sim/
├── launch/arena.launch.py              world + SITL + bridge + profile enforcement
├── worlds/
│   ├── nidar_arena_{1,2,3}.sdf         13×13, 1.0 m corridors
│   ├── *.yaml                          survivor ground truth
│   ├── *.txt                           ASCII plan, north up
│   ├── arenas.png                      three plans side by side
│   └── README.md
├── scripts/
│   ├── generate_arena.py               worlds/*.sdf is its output
│   ├── verify_arena.py                 flood-fill check on the emitted SDF
│   ├── plot_arena.py                   redraw the plan PNGs
│   ├── make_arenas.sh                  regenerate + verify all three
│   └── apply_param_profile.py          PARAM_SET the profile, verify read-back
└── patches/
    └── ardupilot_gazebo-airframe-430mm.patch   sim collision envelope → 430 mm
```

`worlds/*.sdf` is generated output. Worlds are self-contained — no `model://`
includes, no Fuel downloads, so they load offline.

---

## Never do

* Hand-edit `worlds/*.sdf` — generated. Change the generator.
* Skip `verify_arena.py` on a generated world — a leaking shell parses fine.
* Run terminal 1 from anywhere but `~/ardu_ws` — SITL puts `eeprom.bin` in cwd.
* Trust a run with no `enforce_param_profile` block.
* Start Nav2 or SLAM before takeoff — SLAM must be up first, Nav2 after.
* `pkill -f` a pattern in your own command line. Use `scripts/nidar-clean.sh`.
