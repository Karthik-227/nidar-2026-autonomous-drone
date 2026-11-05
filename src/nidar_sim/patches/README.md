# Patches against upstream clones in `~/ardu_ws/src`

These are local fixes to third-party packages that live alongside `nidar_sim`
in the workspace. They are kept here because `git pull` in those clones will
silently revert them, and the failures they fix do not look like their own
cause — see the "Common Issues/Errors" section of the top-level README for the
full symptom-to-cause write-up of each.

## `ardupilot_gz-nidar-fixes.patch`

Three independent fixes, all in `ArduPilot/ardupilot_gz`.

### 1. `/clock` bridged from a Gazebo topic that has no publisher

Files: `ardupilot_gz_bringup/config/*_bridge.yaml`.

The bridge forwarded gz topic `/clock`. In Gazebo Harmonic nothing publishes
that - `gz topic -i -t /clock` lists subscribers only; the live topic is
`/world/<world>/clock`. ROS `/clock` therefore never received a message.

This is the highest-impact bug in the workspace, because everything here runs
with `use_sim_time:=true` and that means ROS time comes only from `/clock`.
With no `/clock`, ROS time is pinned at zero and **every timer in every node
never fires**: slam_toolbox stops mapping, Nav2's costmaps never update (so
walls never become obstacles), and the frontier explorer prints its startup
lines and then goes permanently silent with no error. Each of those looks like
a separate bug in a different package.

Verified: bridging `/world/maze/clock` gives ROS `/clock` at ~1000 Hz, and the
explorer immediately starts finding frontiers.

### 2. sdformat URI resolution


File: `ardupilot_gz_bringup/launch/robots/iris_lidar.launch.py`.

Upstream rewrites the robot model's `model://` include URIs to
`package://<pkg>/models/<model>` before handing the description to
`robot_state_publisher`. `sdformat_urdf` parses it with libsdformat's *global*
`sdf::ParserConfig`, which has no find-file callback registered, so a
`package://` URI cannot be resolved at all. `base_link` lives inside the
unresolved `iris_with_standoffs` include, so `lidar_joint`'s parent is missing,
the URDF conversion fails, and `robot_state_publisher` aborts.

Consequence: no `base_link -> base_scan` transform, so `/scan` cannot be placed
in the robot and slam_toolbox never registers the sensor. SLAM and Nav2 both
appear misconfigured for a reason that is neither.

The patch resolves those URIs to absolute `file://` paths instead, which
libsdformat handles directly — no callback, and no dependence on `SDF_PATH` or
`GZ_SIM_RESOURCE_PATH` being set correctly.

### 3. 2D lidar angular resolution too coarse at range

File: `ardupilot_gz_description/models/lidar_2d/model.sdf`.

Upstream uses 640 samples over 360 deg. That puts successive returns 7.9 cm
apart at 8 m, which is 1.6 cells of a 0.05 m map — so a wall seen from across
the arena lands on every other cell, comes out as a dotted line, and never
accumulates enough hits to cross `occupancy_threshold`. Near walls were always
fine, because the same angular step is only 1.4 cm at 1.4 m, which is why this
reads as "walls are solid where I flew slowly and speckled everywhere else"
rather than as a sensor problem.

Raised to 1081 samples (0.333 deg/ray, 4.7 cm at 8 m — under one cell), so far
walls accumulate continuously. Cost is 1.7x the ray casts on a GPU lidar at
10 Hz.

Apply with:

```bash
cd ~/ardu_ws/src/ardupilot_gz
git apply ~/ardu_ws/src/nidar_sim/patches/ardupilot_gz-nidar-fixes.patch
cd ~/ardu_ws
colcon build --packages-select ardupilot_gz_bringup --symlink-install
```

Verify (fix 3): `ros2 topic echo /scan --field ranges --once | tr ',' '\n' | wc -l`
reports 1081, not 640.

Verify (fix 2): `robot_state_publisher` logs `got segment base_link`,
`got segment base_scan`, `got segment imu_link` and four rotors, with no
`ERROR`, and `ros2 run tf2_ros tf2_echo base_link base_scan` resolves.

## `ardupilot_gazebo-airframe-430mm.patch`

Target: `ArduPilot/ardupilot_gazebo`, `models/iris_with_standoffs/model.sdf`.

The simulated airframe was **1.6x the real one**, which made the competition
arena untestable.

The Mission Brief fixes corridors at a minimum clear width of **1 m**, and the
team's drone is **430 mm** across (QAV 7" frame, 7" HQProp 7040 bi-blades,
polycarbonate prop guards - hardware sheet), leaving 285 mm a side. Stock
`iris_with_standoffs` collides as a **704 mm** vehicle: a 543 mm body mesh, plus
rotor collision cylinders of 0.1 m radius sitting on hubs 0.252 m out. In a 1 m
corridor that leaves 148 mm a side, so the sim clipped walls the real drone
would clear, and every brief-compliant arena had to be widened to 1.4 m to fly
at all - testing a corridor the competition will not have.

The patch changes **collision geometry only**:

* `base_collision` mesh gains `<scale>0.790 0.790 1.0</scale>`, taking the body
  from 0.543 m to 0.430 m across. `z` is left at 1.0 so the landing height is
  unchanged.
* the four landing-leg collision cylinders move inward by the same 0.790, their
  `z` untouched at -0.11.
* the four rotor `<collision>` blocks are **removed**. They have to go rather
  than shrink: the hubs are at 0.252 m, outside the 0.215 m envelope, so any
  prop collision at all would strike a wall before the body did.

**Nothing that affects flight is touched** - no `<mass>`, no `<inertial>`, no
joint, and no rotor link pose. Verified by diff: zero lines matching
mass/ixx/iyy/izz/inertial/joint. Thrust is applied by ArduPilotPlugin at the
rotor link poses, which are unchanged, so the airframe flies exactly as it did
and the ArduCopter tuning still holds. Only what the vehicle *bumps into*
changed.

Cosmetic consequence, worth knowing before someone reports it as a bug: the
visual meshes are untouched, so the propellers are drawn outside the collision
envelope and will appear to pass through walls in a tight corridor. The drone is
not clipping - it is 430 mm of collision wearing a 704 mm picture.

Verify:

```bash
ros2 launch nidar_sim arena.launch.py arena:=1      # the 1 m-corridor arena
```

It should now fly a brief-spec 1 m corridor without wall contact. Before this
patch that needed `arena:=1_wide`.

Apply with:

```bash
cd ~/ardu_ws/src/ardupilot_gazebo
git apply ~/ardu_ws/src/nidar_sim/patches/ardupilot_gazebo-airframe-430mm.patch
```

No rebuild needed - `install/ardupilot_gazebo/.../model.sdf` is a symlink to the
source file.

## `ardupilot_gazebo-airframe-geometry.patch`

Target: `ArduPilot/ardupilot_gazebo`, file
`models/iris_with_standoffs/model.sdf`.

Two things, both about the model describing the wrong aircraft.

**Visuals matched the Iris, not the real airframe.** `base_collision` was
already corrected to a cylinder representing a Lumenier QAV-R 2 (302 mm motor
diagonal + 7 in props = 0.2399 m swept). The *visual* meshes were never
touched: the Iris draws motors at 0.2555 m with 0.1333 m blades, so the
rendered aircraft was 0.389 m swept - **62% wider than the body that actually
collides**. Propellers therefore visibly swept through walls the drone never
touched, which reads as "the physics is too forgiving" when the physics was
right and the picture was wrong. Each rotor visual is now offset within its
link and scaled x0.667 so the drawn prop sits at 0.151 m with a 7 in blade.

The rotor **link** poses are deliberately unchanged, so thrust is still applied
where it was and flight dynamics - and the ArduPilot tuning in
`nidar_tuning.parm` - are unaffected. This is a rendering correction.

**Collision radius 0.240 -> 0.280.** A deliberate 40 mm safety margin over the
true 0.2399 m swept radius, so the simulator collides *earlier* than the real
aircraft would and a clean sim run implies a clean real run. This value must
stay equal to `robot_radius` in `nidar_sim/config/nav2_params.yaml` (both
costmaps) and `robot_radius_m` in `frontier_explorer.py`; if they drift apart
the margin is fictional, because Nav2 would plan paths the body cannot fit.

Apply with:

```bash
cd ~/ardu_ws/src/ardupilot_gazebo
git apply ~/ardu_ws/src/nidar_sim/patches/ardupilot_gazebo-airframe-geometry.patch
cd ~/ardu_ws && colcon build --packages-select ardupilot_gazebo --symlink-install
```

Verify: `gz sdf -k model.sdf` reports `Valid.`, and in Gazebo the rendered prop
discs now sit inside the collision footprint instead of overhanging it.
