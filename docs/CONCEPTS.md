# Concepts Reference — Every Idea Used in This Workspace

**NIDAR 2026–27 · Air Mouse.** A glossary you can actually learn from: what each
thing is, why it exists, and where it shows up in our code.

Read top to bottom the first time — the sections build on each other.

---

# Part 1 — ROS 2: the plumbing

### ROS 2
Not an operating system. A **set of libraries and conventions for making many
small programs talk to each other.** Instead of one giant program, we run ~15
independent processes that exchange messages. If the exploration logic crashes,
SLAM keeps mapping.

We use **ROS 2 Humble** on Ubuntu 22.04.

### Node
One running program that does one job. Ours:

| Node | Job |
| --- | --- |
| `slam_toolbox` | Builds the map, works out where the drone is |
| `scan_stabilizer` | Corrects the lidar scan for the drone's lean, and withholds rays it cannot vouch for |
| `frontier_explorer` | Decides where to fly next |
| `cmd_vel_bridge` | Translates Nav2's commands into what ArduPilot accepts |
| Nav2's ~8 nodes | Path planning, path following, obstacle maps, recovery |

### Topic
A named channel that nodes publish to and subscribe to. **Many-to-many and
anonymous** — a publisher has no idea who is listening. That is what lets us
swap components without rewiring anything.

Key topics here:

| Topic | Type | Carries |
| --- | --- | --- |
| `/scan` | `LaserScan` | Raw lidar distances |
| `/scan_safe` | `LaserScan` | Tilt-gated lidar — what SLAM and Nav2 actually read |
| `/map` | `OccupancyGrid` | The SLAM map |
| `/cmd_vel` | `Twist` | Nav2's velocity command |
| `/ap/cmd_vel` | `TwistStamped` | The same command, in ArduPilot's format |
| `/clock` | `Clock` | Simulation time |
| `/tf` | `TFMessage` | All coordinate-frame relationships |

### Message
A typed data structure. `Twist` = 6 numbers (3 linear + 3 angular velocity).
`TwistStamped` = a `Twist` **plus a header** (timestamp + frame name).

> **This exact distinction cost us a working demo.** Nav2 publishes `Twist` on
> `/cmd_vel`; ArduPilot subscribes to `TwistStamped` on `/ap/cmd_vel`.
> Different type *and* different name — so Nav2 planned perfectly and the drone
> never moved. `cmd_vel_bridge` exists solely to close that gap.

### Service and Action
- **Service** — request/response, immediate. "Clear the costmap." → "Done."
- **Action** — a long-running goal you can monitor and cancel. "Fly to (3, 5)"
  takes 20 seconds, reports progress, and can be aborted.

We use the **`NavigateToPose` action**. The explorer sends a goal, gets told
accepted/rejected, and can cancel mid-flight — which the collision watchdog does.

### QoS (Quality of Service)
Per-topic delivery rules. Two matter to us:

- **Reliability** — `RELIABLE` (retry until delivered) vs `BEST_EFFORT` (send
  once, drop if lost). Right for sensors: a lost lidar scan is worthless 0.1 s
  later anyway.
- **Durability** — `VOLATILE` (subscribers see only new messages) vs
  `TRANSIENT_LOCAL` ("latched" — a late subscriber immediately gets the last
  message). `/map` is transient-local, so a newly started node gets the map at
  once instead of waiting.

> **QoS mismatch is the nastiest bug class in ROS 2.** A subscriber may request
> *less* than a publisher offers, never more. Get it backwards and
> `ros2 topic info -v` shows a publisher and a subscriber on the same topic and
> **zero messages flow, with no error anywhere.**

### Parameters
Per-node settings, changeable at launch without editing code. All our tuning
(`commitment_radius_m`, `max_safe_tilt_deg`, …) is parameters, so we can retune
between competition runs from the command line.

### Launch files
Python scripts that start many nodes with the right parameters and remappings.
`nidar_bringup.launch.py` starts our whole autonomy stack in one command.

### Workspace, `colcon`, overlays
Source lives in `~/ardu_ws/src`, `colcon build` compiles it into
`~/ardu_ws/install`, and `source install/setup.bash` puts it on your path.

> **Overlay trap we hit:** sourcing *two* workspaces that both contain the same
> packages puts two copies of everything on the search path — which means two
> Nav2 stacks, **two `navigate_to_pose` action servers**, and goals silently
> raced between them. Worse, `colcon` bakes the workspaces that were sourced
> *at build time* into `install/setup.sh`, so deleting the extra `source` line
> is not enough — you must rebuild from a clean shell.

---

# Part 2 — Coordinate frames and TF

### Why frames exist
"The wall is 2 metres away" is meaningless without "away *from what*". Every
measurement lives in a **frame of reference**, and the system constantly
converts between them.

### Our frame tree

```
map  ──►  odom  ──►  base_link  ──►  base_scan
 │         │           │               │
 │         │           │               └─ the lidar itself
 │         │           └─ the drone's body centre
 │         └─ a smooth, drift-prone origin from onboard odometry
 └─ the fixed world origin, corrected by SLAM
```

- **`map`** — fixed to the world. Never drifts, but **jumps** when SLAM corrects
  itself (e.g. on recognising a previously visited spot).
- **`odom`** — continuous and smooth, never jumps, but **drifts** over time.
- **`base_link`** — the drone body.
- **`base_scan`** — the lidar, 7.5 cm above `base_link` on our airframe.

**Why both `map` and `odom`?** Because you need both properties and no single
frame has both. Controllers need smooth (`odom`) or the drone lurches on every
correction. Goals need drift-free (`map`) or they slide away over time. So SLAM
publishes the correction **`map → odom`**, and the two are used for different
jobs. This is standard ROS 2 practice, not a workaround.

### TF2
The library that stores these relationships over time and answers "where was
the lidar, in the map frame, at 14:32:07.412?" — **at a specific timestamp**,
not just now. That is why every sensor message carries a header stamp.

> Our tilt gate keys off **`odom → base_link`**, deliberately *not*
> `map → base_link`. It feeds SLAM, and SLAM produces `map → odom` — so using
> `map` would make it depend on the very thing that depends on it. `odom` comes
> from ArduPilot's own state estimate and exists before SLAM produces anything.

### Quaternions
A 4-number representation of 3D rotation `(x, y, z, w)`. Used instead of
roll/pitch/yaw because Euler angles suffer **gimbal lock** — an orientation
where two axes align and one degree of freedom vanishes.

We use them in two places:
- Converting a desired heading into a goal orientation.
- Extracting **tilt** for the tilt gate: `1 − 2(x² + y²)` gives the vertical
  component of the drone's own up-axis, and its arccosine is the lean angle.

---

# Part 3 — SLAM

### SLAM (Simultaneous Localisation and Mapping)
The chicken-and-egg problem at the centre of this challenge:

- To build a map, you must know where you are.
- To know where you are, you need a map.

SLAM solves both **at the same time**, incrementally. With no GPS, this is the
only way the Air Mouse mission is possible at all.

### Occupancy grid
The map format: the world as a grid of 5 cm cells, each holding one number.

| Value | Meaning |
| --- | --- |
| `0` | Free — lidar looked through it |
| `100` | Occupied — lidar hit something |
| `-1` | Unknown — never observed |

Stored as `nav_msgs/OccupancyGrid`: the cell array plus resolution, width,
height, and the world position of the bottom-left corner (`origin`).

**Everything downstream is built on this one structure** — frontier detection,
Nav2's costmaps, and the coverage figure are all just different reads of it.

### Scan matching
How SLAM localises. Take the new lidar scan, slide and rotate it against the map
so far, find the alignment that fits best — that offset is how far the drone
moved. Much more accurate than dead reckoning, which accumulates error forever.

### Loop closure
When the drone re-enters a place it has been before, SLAM recognises it and
corrects all the accumulated drift at once. This is why the `map` frame jumps,
and why we never use `map` for smooth control.

### `slam_toolbox`
Our SLAM implementation. We run **async online mode**: process scans as they
arrive, don't block if one takes too long. Key settings:

| Setting | Ours | Why |
| --- | --- | --- |
| `resolution` | 0.05 m | Matches Nav2's costmaps — avoids resampling |
| `map_update_interval` | 1.0 s | Was 5.0 s; Nav2's static layer could only be as fresh as the last map, so walls took 5 s to become obstacles |
| `min_laser_range` | 0.30 m | Below this, returns off the drone's own airframe and prop arcs enter the map as speckle |
| `max_laser_range` | 8.0 m | Edge-of-range returns are the noisiest and most tilt-sensitive |
| `scan_topic` | `/scan_safe` | The tilt-gated feed, not raw `/scan` |

---

# Part 4 — Navigation (Nav2)

### Nav2
The ROS 2 navigation stack: given a map, a position and a goal, produce
velocity commands that get there without hitting anything. We use it as-is and
feed it goals — we did not write a path planner.

### Costmap
A **grid of costs**, not just occupancy. Built in stacked layers:

| Layer | Contributes |
| --- | --- |
| **Static** | The SLAM map |
| **Obstacle** | Live lidar returns |
| **Inflation** | A cost halo grown around every obstacle |

**Global costmap** — the whole known map, used for route planning.
**Local costmap** — a 5×5 m rolling window centred on the drone, updated fast,
used for immediate obstacle avoidance.

> **Why the local costmap matters for safety:** the SLAM map lags — it is
> republished periodically, and a newly seen wall needs several scans before it
> is confidently marked. A wall can be right beside the drone and still read as
> free on `/map` for a second or two. The local costmap is fed straight from the
> live laser. Our collision watchdog reads **the local costmap**, never `/map`.

### Inflation and robot radius
A drone is not a point. If you plan for a point, you clip every corner. So:

- Cells within **`robot_radius`** (0.35 m) of a wall are **inscribed** — the
  drone physically cannot be there. Hard-blocked.
- Cells out to **`inflation_radius`** (0.45 m) get a decaying cost: legal, but
  the planner prefers to avoid them.

> **A tuning mistake worth remembering.** With `inflation_radius` at 0.65 m in
> 2.8 m corridors, 1.3 m of every corridor was inflated — so the drone was
> inside the "slow down near obstacles" band for its **entire** flight. The
> speed reduction was permanent rather than situational, and it never once flew
> at its configured cruise speed. Lesson: tune inflation against your actual
> corridor width, not in the abstract.

### Planner vs controller
- **Planner** (`NavFn`, Dijkstra/A\*) — computes a route through the global
  costmap. Runs on a new goal and on replans.
- **Controller** (Regulated Pure Pursuit) — follows that route, emitting
  velocities at 30 Hz. "Pure pursuit" aims at a point a fixed distance ahead on
  the path; "regulated" slows for tight curves.

### Behaviour Tree
Nav2's decision structure. A tree of small behaviours — *compute path*, *follow
path*, *if stuck, spin*, *if still stuck, back up* — evaluated continuously.
Easier to reason about and modify than a state machine.

### Lifecycle nodes
Nav2 nodes have explicit states: *unconfigured → inactive → active*. A
**lifecycle manager** brings them up in the right order. `Managed nodes are
active` in the log is how you know Nav2 is genuinely ready.

---

# Part 5 — Simulation

### Gazebo (Harmonic)
Physics simulator. Models the airframe, motors, aerodynamics, the maze, and a
simulated lidar that ray-casts against the world. Lets us test everything
without risking hardware.

### ArduPilot SITL
**Software In The Loop** — the *real, unmodified* ArduPilot flight-control
firmware compiled to run on a PC. Not an approximation: the same code that would
fly the physical drone. So flight modes, arming logic and failsafes all behave
exactly as they will on the real vehicle.

### `ros_gz_bridge`
Gazebo and ROS 2 use different messaging systems. This translates between them
per topic.

> **This bridge caused our single most damaging bug.** It was configured to
> forward the Gazebo topic `/clock`, which in Gazebo Harmonic **has no
> publisher** — the live one is `/world/<world>/clock`. So ROS `/clock` never
> received a message. See "Simulation time" below for why that was catastrophic.

### MAVLink and MAVProxy
**MAVLink** is the drone telemetry/command protocol. **MAVProxy** is a
ground-station tool that speaks it — we use it to arm and take off, because
those are flight-mode decisions the autonomy stack deliberately does not make.

### Micro-XRCE-DDS
How ArduPilot talks ROS 2 natively. ArduPilot runs a tiny DDS client; an
**agent** on the PC bridges it onto the normal ROS 2 network. That is where
`/ap/v1/*` topics come from.

### Simulation time (`use_sim_time`)
Simulated physics does not run at exactly real speed. So every node is told
`use_sim_time:=true` and takes time from the `/clock` topic instead of the wall
clock, keeping everything consistent.

> **The failure mode to burn into memory.** With `use_sim_time:=true` and no
> `/clock` arriving, ROS time is **frozen at zero** — and *every timer in every
> node never fires.* SLAM stops mapping, costmaps stop updating, and the
> explorer prints its startup lines and then goes **permanently silent with no
> error**, while `ros2 node list` shows it perfectly healthy. It looks exactly
> like "it isn't receiving the map."
>
> Two causes: the bridge forwarding a dead Gazebo topic (above), or **more than
> one Gazebo instance running**, each publishing `/clock`, with sim time jumping
> between them. Leftover processes survive Ctrl+C more often than you would
> think.
>
> **First diagnostic, always:** `ros2 topic hz /clock` — expect ~1000 Hz — and
> `ros2 topic info /clock` — expect exactly **one** publisher.

### `sdformat_urdf` and `robot_state_publisher`
`robot_state_publisher` reads the robot description and publishes the fixed
transforms between its parts — including **`base_link → base_scan`**, without
which `/scan` cannot be placed in the world at all.

> It was **crashing silently on every launch.** The launch file rewrote the
> model's `model://` URIs to `package://`, which `sdformat_urdf` cannot resolve
> (it parses with libsdformat's global config, which has no find-file callback).
> `base_link` lives inside the unresolved include, so the whole description
> failed and the process aborted — scrolling past in two seconds while
> everything else kept running. Result: no `base_link → base_scan`, so SLAM
> could never register the lidar. **SLAM and Nav2 both looked misconfigured for
> a reason that was neither.**

---

# Part 6 — Algorithms

### Occupancy grid mapping
Each cell accumulates evidence. A lidar beam ending in a cell is evidence of
*occupied*; a beam passing *through* a cell is evidence of *free*
(**raytracing**). Cells cross a threshold before being declared either way, so
one bad reading cannot create a wall.

> Raytracing is also the mechanism the tilt problem exploits: a beam that
> returns nothing is treated as passing through everything along its length, so
> a tilted scan **actively erases** correctly mapped walls.

### Dijkstra's algorithm
Finds shortest paths from one start to *every* reachable cell. We use it for the
wall-aware travel-distance field — flood-filling outward through free space only.

**Complexity** O(E log V); on a 400×400 grid that is ~420 ms.

> We run it on a grid **decimated by 2** (0.10 m cells), ~10× cheaper. Not
> because 420 ms misses the 3-second planning deadline — it does not — but
> because rclpy's default executor is **single-threaded**, so a 420 ms
> computation also blocks the 5 Hz collision watchdog. Starving the safety
> check for two of its own periods, every cycle, defeats the point of having it.
>
> A decimated cell counts as free only if **every** sub-cell is free — safe for
> walls, but it can close a narrow-but-passable gap. So if the decimated field
> finds nothing reachable while candidates exist, we pay for one
> full-resolution pass rather than wrongly concluding the map is finished.

### A\*
Dijkstra plus a heuristic that biases the search toward the goal. Faster when
you want *one* path. Nav2's planner uses it; we do not, because we need
distances to *all* frontiers at once, which is exactly what plain Dijkstra gives.

### Flood fill / connected components
Grouping touching cells into one region. We use it twice: clustering frontier
cells into openings, and reachability.

### Binary dilation
Growing a shape outward by *n* cells. We dilate the obstacle map to build the
safety margin for frontier filtering.

> We tried to "optimise" this with `scipy.ndimage` and with a separable
> sliding-window filter. **Both lost.** scipy cannot even be imported under
> NumPy 2.x here, and the separable version measured *slower* (2.3 ms vs
> 1.9 ms) than the naive eight-shift loop — `numpy.roll` over 160k booleans is
> already cheap. Kept the simple version. **Measure before optimising.**

### Pure pursuit
Path following by always steering toward a point a fixed "lookahead" distance
ahead on the path. Longer lookahead → smoother but cuts corners; shorter →
accurate but wobbles.

### PID control
ArduPilot's inner loops. **P** reacts to current error, **I** removes steady
offsets (e.g. wind), **D** damps overshoot. We do not tune these — ArduPilot
owns attitude control; we only send velocity targets.

---

# Part 7 — The mistakes, collected

The most portable thing in this document. Every one of these cost real time and
**every one presented as a problem in a different component than the actual
cause.**

| Symptom | Real cause |
| --- | --- |
| "SLAM is broken, no map" | `robot_state_publisher` crashed on startup — no `base_link → base_scan`, so `/scan` had nowhere to sit |
| "The node is hung / not receiving the map" | `/clock` silent → ROS time frozen at 0 → every timer dead. The node was not running *at all* |
| "Nav2 isn't marking walls as obstacles" | Global costmap's obstacle layer had default height limits `0.0–2.0 m`, checked in the **global** frame — at a 2 m takeoff, every scan point sat exactly on the limit |
| "Walls only appear when very close" | `obstacle_max_range` 2.5 m against a 10 m lidar |
| "The drone crawls everywhere" | Cost-regulated slowdown was permanently active because inflation covered half of every corridor |
| "The map glitches, walls flicker" | Tilted lidar returns nothing → raytraced as free → correctly mapped walls erased |
| "It oscillates between two goals" | Score depended only on current position; the drone's own movement flipped the winner. **Every individual decision was correct** |
| "Exploration finished early" | Anti-oscillation suppression hid a reachable region — identical in appearance to being done |
| "`ros2 run` says no executable" | `setup.py` had an empty `console_scripts` |
| Goals cancelled constantly | Collision watchdog read the costmap at the *inflation* threshold, not the *lethal* one — permanently true in a 2.8 m corridor |
| `colcon test` red | Lint suite was walking into sibling packages and importing `cv_bridge`, broken by a NumPy 2.x ABI conflict |
| "The drone just yaws at everything" | Nav2's `use_rotate_to_heading` defaults **true** — the controller stops and spins in place whenever heading error exceeds 45°. Built for differential-drive robots; a multirotor with a 360° lidar needs neither |
| Phantom diagonal walls across open floor | The tilted scan plane striking the **ground**. At 2 m altitude and 8 m range that starts at 14° of lean |
| The tilt gate never actually gated | It computed tilt from a TF lookup; when that failed it forwarded everything and said so **once**. Now reads the IMU directly and complains every 10 s while inert |
| Nav2's `controller_server` never reaches `configured` | One orphaned `ros_gz_bridge` still publishing `/clock` from a dead Gazebo. Sim time jumps between two sources and every TF buffer clears continuously |
| Node reports "my clock has stopped" | `wait_for_server(timeout_sec=5.0)` inside a timer callback on a **single-threaded executor** blocks the `/clock` subscription too. The node starves its own clock |
| Collision guard fires at 0.07 m in an open corridor | It counted cost **99** (inscribed) as an obstacle. 99 means "the robot's centre may not sit here", true within `robot_radius` of any wall — a planning constraint, not a distance. Only 100 is a real obstacle |
| Exploration "completes" at half coverage | Frontiers sitting on unreachable speckle islands were discarded as infinite-distance. You do not need to stand on a frontier, only get close enough to see past it |
| Everything looks busy, nothing progresses | **The drone had landed.** ArduPilot auto-disarms `DISARM_DELAY` seconds after touchdown; Nav2 keeps accepting goals regardless |

### The single biggest time sink: orphaned processes

Worth its own heading, because it caused more lost hours than every algorithm
bug combined, and it never looks like what it is.

Every `ros2 launch` or `ros2 run` that is not cleanly shut down leaves
processes behind. `Ctrl+C` does not reliably take them: anything started under
`setsid` outlives the terminal, and killed launch files leave orphaned
children. Counts observed in this workspace: **83** `frontier_explorer`
instances, **2** `slam_toolbox`, **2** `bt_navigator`, and a lone
`ros_gz_bridge` whose Gazebo had been dead for twenty minutes.

Each one produces a *different* mystery:

| Duplicate | Symptom |
| --- | --- |
| `ros_gz_bridge` | Two `/clock` publishers, sim time jumps backwards, every TF buffer clears continuously, Nav2 cannot configure |
| `slam_toolbox` | Two publishers of `map→odom`; every lookup fails "extrapolation into the past" and `tf2_echo` times out |
| `bt_navigator` | Two Nav2 stacks race every goal |
| `frontier_explorer` | Competing goals, each cancelling the other — the drone visibly thrashes |

**`pkill -f <pattern>` is not adequate**, for two reasons that bite constantly:

1. It matches the pattern against *every* command line **including the shell
   running the pkill**. `pkill -f frontier_explorer` from a script containing
   that word kills the script first.
2. `setsid` processes are in their own process group and survive.

Use `scripts/nidar-clean.sh`, which matches on executable paths and excludes
itself. Run it before **every** launch, not just when something looks wrong.

**The pattern:** in a distributed system, the component that *reports* the
failure is rarely the component that *caused* it. Diagnose in dataflow order —
clock, then TF, then sensors, then map, then costmap, then commands — and stop
at the first thing that is silent.

---

## Further reading

- Yamauchi (1997), *A Frontier-Based Approach for Autonomous Exploration*
- Thrun, Burgard & Fox, *Probabilistic Robotics* — SLAM and occupancy grids
- [Nav2 documentation](https://navigation.ros.org/)
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [ArduPilot ROS 2](https://ardupilot.org/dev/docs/ros2.html)
