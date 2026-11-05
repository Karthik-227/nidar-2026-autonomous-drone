# The custom path-following controller

**Author: Vishnu Joshi (Aerial Guild, NIDAR teammate), built with Claude's assistance.**
Branch: `follower-controller`. Files: [`src/nidar_sim/Frontier_Exploration_Sim/`](../src/nidar_sim/Frontier_Exploration_Sim/).

This is the fix that took autonomous exploration from stalling after ~12-20
goals to a complete, unattended run. It replaces Nav2's default local
controller (`DWB`) with a small, purpose-built path follower, while keeping
everything upstream of it — SLAM, Nav2's global planner, the frontier
explorer — unchanged.

## Result

Autonomous frontier exploration of `nidar_arena_1` (14.4 m, 1 m corridors)
with no manual input: **94% of the arena mapped in 14.5 minutes** (85.5%
free / 8.8% wall / 5.7% unknown), walls single-cell thin. Before this change
the best runs managed ~12 and ~20 goals before stalling and needing a
manual restart.

## Why DWB had to go

`DWB` is a sampling controller: ~10 times a second it generates ~2000
candidate velocity commands, forward-simulates each for 1.7 s **assuming the
vehicle instantly reaches the commanded velocity**, discards any whose
footprint touches an obstacle, and scores what's left on distance to the
path and goal.

ArduPilot takes roughly 0.5 s to actually reach a commanded velocity and
coasts after that. In a 1 m corridor with a 0.30 m robot radius, the legal
lane is 0.4 m wide — about the size of the lag error itself. The consequence
wasn't bad tuning, it was that the controller's core assumption didn't hold
for this airframe:

| Symptom | Mechanism |
|---|---|
| Drone drifts into the no-go zone, then every goal fails to plan — including the one to return home | The simulated trajectory never matched the one actually flown |
| 55 s stalled at a doorway, zero movement, no error | When every "real" trajectory was ruled unsafe, a 0.03 m/s creep was always legal, so DWB "succeeded" at a motion too slow to pass the progress checker |
| Drone yawed for no reason | Yaw rate was a free variable in the search; curving trajectories scored about the same as straight ones |
| A doorway that looked open to the planner looked closed to the controller | The local costmap (raw scan hits, thicker) and SLAM's filtered map disagreed |

Every parameter change between 23 Oct and 31 Oct moved the symptom, never
the cause — because none of them were parameters. They were what a sampling
controller does under this much actuation lag.

## The pipeline — same shape, one node bypassed

```
explorer ──NavigateToPose──▶ bt_navigator ──ComputePathToPose──▶ planner_server (NavFn) ──▶ /plan
                                   │                                                            │
                                   └──FollowPath──▶ controller_server (DWB) ──▶ /cmd_vel ──▶ (nobody)
                                                                                                  │
                              path_follower.py ◀── /plan + TF(map→base_link) + /scan             │
                                   │                                                             │
                                   └──▶ /cmd_vel_follower ──▶ cmd_vel bridge ──▶ ArduPilot
```

Nav2 is launched exactly as before, and `controller_server` (DWB) keeps
running — removing it isn't an option, since `bt_navigator`'s behaviour tree
calls `follow_path` and DWB is the only thing serving that action. Leaving
it running costs nothing and keeps two of its checks for free: its **goal
checker** still tells the explorer "reached" from the drone's real TF pose,
and its **progress checker** still triggers Nav2's own stuck-recovery. DWB's
*velocity output* is simply never listened to again — the bridge to
ArduPilot now subscribes to `/cmd_vel_follower`, which the new controller
publishes instead.

## How the follower works

Every 100 ms, [`path_follower.py`](../src/nidar_sim/Frontier_Exploration_Sim/path_follower.py) (~100 lines):

1. Reads the drone's pose from TF (`map → base_link`).
2. Finds the nearest point on the current `/plan` at or after the last
   tracked index (never jumps backward on a U-turn).
3. Walks `lookahead` (0.30 m) further along the path — the **carrot**.
4. Computes `v = gain × (carrot − pose)`, capped at `v_max` (0.30 m/s),
   rotated into the drone's body frame. Yaw rate is always zero — a 360°
   lidar needs no heading.
5. A lidar guard checks for any return closer than `guard_dist` (0.35 m)
   within ±30° of the commanded direction; if so, it publishes zero instead
   (hover) regardless of what the carrot says.
6. Publishes the result on `/cmd_vel_follower`.

It holds (publishes zero) when the plan goes stale, the scan goes stale,
TF is unavailable, or the path end is reached within tolerance — and prints
a state change exactly once, so the log reads as `TRACKING idx=...` /
`HOLD: <reason>` rather than a flood.

**Why this doesn't fight the airframe's lag the way DWB did:** the carrot
always sits on the path, and the path already runs through the corridor's
centre (it's shaped by the costmap's inflation gradient). If the drone lags
or drifts off that line, the error vector points back toward the path, so
the correction grows in the direction that fixes it — the same mechanism
ArduPilot's own attitude controller uses to hold a hover. DWB's lag pushed
the drone *out* of the lane; this pulls it back *in*.

## Supporting changes that had to happen alongside it

- **`cmd_vel_stamper.py` — a mirrored lateral axis.** ROS's `base_link` is
  FLU (+y = left); ArduPilot's body frame is FRD (+y = right). The stamper
  now negates `linear.y`. Verified with an open-loop test: publishing pure
  +x / +y / +yaw and confirming the drone moves along the expected
  arrow/direction — worth doing on any new airframe or bridge before
  trusting a controller on it.
- **`nidar_bt.xml` — recovery without `Spin`/`BackUp`.** Nav2's default
  recovery tree yaws and backs up on failure. Both are ground-robot
  reflexes: a yaw-rate command on a multirotor translates the vehicle
  sideways and smears the 2D scan, and `BackUp` drives in whatever
  direction the nose happens to be pointed. Every crash before this change
  was preceded by one of the two firing. The tree here keeps only
  `ClearCostmap → Wait`.
- **`nav2_params_iris .yaml` — costmap and tolerance tuning** matched to the
  airframe's real prop radius, with inflation reaching all the way to the
  corridor centre so the planner is pulled off wall tips rather than
  hugging them.

## Tuning, if the drone weaves

All in the follower's own ROS 2 parameters (`-p name:=value`):

| Param | Default | Tune when |
|---|---|---|
| `lookahead` | 0.30 m | corners visibly cut → lower to ~0.2 |
| `gain` | 1.0 | drone swerves along straight corridors → lower to ~0.6. Measured live: at 0.6, cross-track deviation from the planned path was max 7.8 cm / mean 1.8 cm over 1000+ samples — essentially a straight line. |
| `v_max` | 0.30 m/s | overall speed cap |
| `guard_dist` / `guard_half_angle_deg` | 0.35 m / 30° | lidar e-stop cone |

## Honest open items

- One crash near the end of a full-coverage run is still undiagnosed —
  candidates are corner-cutting at the current `lookahead`, or the guard
  cone not covering a lateral approach.
- Two small pockets of the arena (5.7% of its area) remain unexplored,
  most likely due to the explorer's own revisit suppression rather than
  anything in the controller.
- Mission yaw (a camera sweep to spot survivors) is deliberately **not**
  part of this controller — navigation yaw stays at zero on purpose, and a
  future mission layer should hold position and rotate separately rather
  than mixing the two.
