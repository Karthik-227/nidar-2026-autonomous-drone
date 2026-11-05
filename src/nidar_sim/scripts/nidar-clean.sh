#!/bin/bash
set -u
SELF=$$

kill_exact() {
    local pat="$1"
    local pids
    pids=$(ps -eo pid,cmd --no-headers \
           | grep -- "$pat" | grep -v " grep " \
           | awk -v me="$SELF" '$1 != me {print $1}')
    [ -n "$pids" ] && kill -9 $pids 2>/dev/null
}

for p in \
    "lib/ros_gz_bridge/parameter_bridge" \
    "lib/slam_toolbox/async_slam_toolbox_node" \
    "lib/nidar_sim/" \
    "nav2_bt_navigator/bt_navigator" \
    "nav2_controller/controller_server" \
    "nav2_planner/planner_server" \
    "nav2_smoother/smoother_server" \
    "nav2_behaviors/behavior_server" \
    "nav2_waypoint_follower/waypoint_follower" \
    "nav2_velocity_smoother/velocity_smoother" \
    "nav2_lifecycle_manager/lifecycle_manager" \
    "robot_state_publisher/robot_state_publisher" \
    "topic_tools/relay" \
    "micro_ros_agent/micro_ros_agent" \
    "lib/rviz2/rviz2" \
    "arducopter --model" \
    "mavproxy.py" \
    "ros2 launch ardupilot_gz_bringup" \
    "ros2 launch nidar_sim" \
    "ros2 launch slam_toolbox" \
    "ros2 launch nav2_bringup" \
    "ros2 run nidar_sim" \
    "path_follower.py" \
    "cmd_vel_stamper.py" \
    "gz sim" \
    "ign gazebo" ; do
    kill_exact "$p"
done
sleep 4
for p in "gz sim" "ign gazebo" "arducopter --model" "parameter_bridge" "mavproxy.py" "path_follower.py" "cmd_vel_stamper.py"; do kill_exact "$p"; done
sleep 2

shm_before=$(ls /dev/shm 2>/dev/null | grep -c fastrtps)
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
shm_after=$(ls /dev/shm 2>/dev/null | grep -c fastrtps)
echo "--- fastdds shm swept: $shm_before -> $shm_after ---"

echo "--- survivors (should be empty) ---"
ps -eo pid,cmd --no-headers \
  | grep -E "gz sim|ign gazebo|arducopter --model|async_slam_toolbox|bt_navigator|parameter_bridge|lib/nidar_sim/|mavproxy.py|ros2 launch|ros2 run nidar_sim|micro_ros_agent|rviz2|path_follower.py|cmd_vel_stamper.py" \
  | grep -v " grep " | awk -v me="$SELF" '$1 != me' | cut -c1-90
echo "--- done ---"
