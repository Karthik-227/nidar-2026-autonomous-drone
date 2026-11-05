# Simulation Setup
This guide goes through setting up the simulation environment properly for ROS2 integration with Gazebo and Ardupilot SITL. 

This guide is made because the Ardupilot documentation wiki is old and not properly updated, and hence this goes through what you should correct in your installation.

# Installation
The complete and tested method to install is following the official documentation found [here](https://ardupilot.org/dev/docs/ros2-gazebo.html). Please note the following:
- Ensure that you follow all the prerequisites and auxiliary installations that the documentation asks you to make, after which only follow the relevant documentation commands.

- Make sure to put each command with care, often multiple commands are copied and pasted at the same time, which causes one to not notice which intermediate command failed or didnt run, as a result of which commands further down will not work as intended. 

- Some commands are dependent on the version of Ardupilot installed. For most people setting it up now, it'll be the latest, that is, Ardupilot 4.8.0 so put the commands accordingly. Generally speaking, putting the wrong command here is known to not cause much problems, but better to be on the safer side. To check your Ardupilot version, run the following command: 
```bash
cd ~/ardu_ws
cat ArduCopter/version.h
```

- Try to avoid Gemini spam for this as much as possible, most of the time it makes the problem worse somehow, rather go through the common issues section of this file. 

- The documentation so linked should be followed with care as some of the commands have some mistakes, some of which are noted down here.
    - __Missing Tilde__: Some of the commands have a missing tilde in the command ```cd ardu_ws```, which causes the command to not work as intended and cause mistakes in setup, be mindful of this. 
    - __VCS Import__: The initial VCS import command is not updated, leading to the 404 error. The correct command to run is (in Humble):
    ```vcs import --recursive --input  https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.humble.repos src```
    - __Testing__: There are multiple places where the documentation talks about testing your installation, which is unnecessary for us. Its mainly for people who want to edit the source code, so do not run these testing sections. 

# Environment
Get this right before anything else. Most of the "SLAM is broken" / "Nav2 is broken" reports in this project turned out to be environment problems, and they are much harder to recognise once you are already deep in parameter files.

Two rules:

- __Source exactly one workspace.__ Not `~/ros2_ws` *and* `~/ardu_ws`. Two workspaces holding copies of the same packages puts two of everything on `AMENT_PREFIX_PATH`, which means two `navigate_to_pose` action servers racing your goals. See the "Two workspaces sourced at once" entry in the top-level README — including the part about it being baked into `install/setup.sh` at build time, so deleting the extra `source` line is not enough on its own.
- __Point the Gazebo resource paths at the workspace you actually build.__ If `GZ_SIM_RESOURCE_PATH` / `SDF_PATH` point at a different workspace's `src`, models resolve inconsistently depending on which workspace was built last.

A `~/.bashrc` tail that satisfies both:

```bash
source /opt/ros/humble/setup.bash

export GZ_VERSION=harmonic
export ARDU_WS="$HOME/ardu_ws"

export GZ_SIM_RESOURCE_PATH="$ARDU_WS/install/ardupilot_gazebo/share/ardupilot_gazebo/models:$ARDU_WS/install/ardupilot_gz_description/share/ardupilot_gz_description/models:$HOME/.gz/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export SDF_PATH="$ARDU_WS/install/ardupilot_gazebo/share/ardupilot_gazebo/models:$ARDU_WS/install/ardupilot_gz_description/share/ardupilot_gz_description/models:$HOME/.gz/models${SDF_PATH:+:$SDF_PATH}"

# MAVProxy, ArduPilot's autotest tools, and the DDS IDL generator that
# ardupilot_sitl's build needs on PATH (see issue 4 below).
export PATH="$PATH:$HOME/.local/bin:$ARDU_WS/src/ardupilot/Tools/autotest:$ARDU_WS/Micro-XRCE-DDS-Gen/scripts"

if [ -f "$ARDU_WS/src/ardupilot/Tools/completion/completion.bash" ]; then
  source "$ARDU_WS/src/ardupilot/Tools/completion/completion.bash"
fi

if [ -f "$ARDU_WS/install/setup.bash" ]; then
  source "$ARDU_WS/install/setup.bash"
fi
```

Sanity check in a fresh terminal:

```bash
echo $AMENT_PREFIX_PATH | tr ':' '\n'   # exactly one workspace + /opt/ros/humble
echo $GZ_VERSION                        # harmonic
which microxrceddsgen mavproxy.py
```

# Common Issues
1. __White Drone__: The model spawns but renders untextured/white. The mesh files aren't being found — Gazebo resolved the model's SDF but not the `.dae`/`.stl` it references.

    __Solution__: Confirm `GZ_SIM_RESOURCE_PATH` includes `ardupilot_gazebo/share/ardupilot_gazebo/models` (the *install* space, not `src`), and that the meshes are actually installed:
    ```bash
    ls ~/ardu_ws/install/ardupilot_gazebo/share/ardupilot_gazebo/models/iris_with_standoffs/meshes
    ```
    If that directory is empty or missing, rebuild `ardupilot_gazebo` — its `CMakeLists.txt` installs `models/` into `share/`, so an interrupted build leaves the SDF present and the meshes absent.

    Note this is a *rendering* problem and a separate thing from `robot_state_publisher` dying, which produces no RViz2 robot at all rather than a white one. See the README's first Common Issues entry for that.

2. __No Drone__: The model wasn't loaded into Gazebo at all — the resource path environment variable isn't set, so the `model://` URIs in the world/model files can't be resolved.

    __Solution__: Set `GZ_SIM_RESOURCE_PATH` and `SDF_PATH` as in the Environment section above, in a *fresh* terminal (exports don't reach already-running shells). Then check the launch output for `Unable to find uri[...]` lines, which name exactly which model failed to resolve. `gz sim` also honours these, so `gz sim -v4` on the world alone is a quick way to isolate whether the problem is the world or the robot.

    If the drone is in Gazebo but not in MAVProxy's map, that is a different problem — MAVProxy talks MAVLink over UDP 14550 and knows nothing about Gazebo. Check that SITL actually started (`ros2 node list` should show the micro-ROS agent) and that nothing else is holding port 14550.

3. __Laptop Crash on building__: `colcon` builds packages in parallel, one compiler process per core, and the heavy C++ packages here (`slam_toolbox`, `ros_gz_bridge`, `ardupilot_gazebo`) each need well over a GB while compiling. On 16 cores that is enough to exhaust RAM and take the machine down.

    __Solution__: Cap the parallelism, and cap the per-package compiler jobs too:
    ```bash
    colcon build --symlink-install --parallel-workers 2 \
      --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```
    `MAKEFLAGS="-j2" colcon build --symlink-install --parallel-workers 2` is the stricter version if it still dies. Build the heavy ones on their own first (`--packages-select slam_toolbox`, then `ros_gz_bridge`), then the rest. Adding swap helps but is slow; reducing parallelism is the real fix.

4. __`colcon build` fails: `Could not find the program ['microxrceddsgen']`__: `ardupilot_sitl` runs ArduPilot's waf configure, which needs the Micro-XRCE-DDS IDL generator on `PATH` to build the DDS client.

    __Solution__: Make sure `~/ardu_ws/Micro-XRCE-DDS-Gen/scripts` is on `PATH` (see the Environment section) *in the shell you build from*, then confirm with `microxrceddsgen -version`. If the directory doesn't exist at all, it was never cloned/built — follow the DDS section of the official ROS 2 + Gazebo guide. This one bites specifically when you build from a deliberately minimal shell.

5. __`colcon build` fails: `failed to create symbolic link ... Is a directory`__: A `build/` directory created *without* `--symlink-install` being reused by a build *with* it.

    __Solution__: `rm -rf build install` and rebuild, always passing `--symlink-install` consistently.

6. __`A module that was compiled using NumPy 1.x cannot be run in NumPy 2.x`__ / `numpy.core.multiarray failed to import`: a pip-installed NumPy 2.x in `~/.local/lib/python3.10/site-packages` is shadowing the system NumPy 1.21 that Ubuntu 22.04's ROS 2 Humble packages were built against. Check with:
    ```bash
    python3 -c "import numpy; print(numpy.__version__, numpy.__file__)"
    ```
    If that prints a 2.x version from `~/.local`, every C-extension built against NumPy 1.x is affected. On this machine `scipy` is already unusable because of it:
    ```bash
    python3 -c "import scipy.ndimage"   # ImportError: numpy.core.multiarray failed to import
    ```

    __Impact on this repo__: `nidar_sim` is deliberately NumPy-only and does not import scipy, so SLAM, Nav2, the bridge and the explorer are unaffected. `rclpy`, `tf2_ros` and `sensor_msgs_py` were verified working. The risk is to anything else that reaches for a scientific Python extension.

    __Solution__: don't resolve it by downgrading system-wide — NumPy 2.x is presumably there for the perception side (the pinned `opencv-python` 5.x and the YOLO weights in `humandetection/` need it), and dropping to `numpy<2` would break that instead. Put the perception stack in its own virtualenv and leave the system interpreter on the NumPy the ROS packages expect:
    ```bash
    python3 -m venv --system-site-packages ~/venvs/perception
    source ~/venvs/perception/bin/activate
    pip install "numpy>=2" opencv-python ultralytics
    ```
    Then `pip uninstall numpy` from the user site (`python3 -m pip uninstall numpy`) so the base interpreter falls back to `/usr/lib/python3/dist-packages/numpy` (1.21). Verify with the one-liner above that it reports 1.x from `dist-packages`.

## A note on stray config copies
`~/ardu_ws/config/` currently holds another `nav2_params.yaml` and a `mapper_params_online_async.yaml`. These are not the ones the package uses. The authoritative copies live in `src/nidar_sim/config/` and are installed to `install/nidar_sim/share/nidar_sim/config/`, which is where the launch files read them from. Editing the workspace-root copies has no effect; delete them or be clear about which is which, because "I changed the parameter and nothing happened" is a very easy hour to lose.

