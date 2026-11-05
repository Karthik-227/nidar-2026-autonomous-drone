from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    SetEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    pkg_share = get_package_share_directory('nidar_sim')
    use_sim_time = LaunchConfiguration('use_sim_time')

    def include(name, **extra):
        args = {'use_sim_time': use_sim_time}
        args.update(extra)
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_share, 'launch', name])),
            launch_arguments=args.items(),
        )

    return LaunchDescription([
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE',
            PathJoinSubstitution([pkg_share, 'config', 'fastdds_no_shm.xml'])),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo clock. Must stay true in simulation.'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description="Open RViz2 with the SLAM/Nav2 displays preloaded. The "
                        "RViz2 that ardupilot_gz_bringup opens has no Map or "
                        "Costmap display, so it cannot show the map."),
        DeclareLaunchArgument(
            'explore', default_value='true',
            description='Start autonomous frontier exploration. false = SLAM + '
                        'Nav2 + bridge only, drive it with RViz 2D Goal Pose.'),
        DeclareLaunchArgument(
            'flight_altitude', default_value='1.17',
            description='Height (m) the bridge holds the drone at, and the '
                        'height the scan filters are derived from. Take off '
                        'to the same number.'),
        DeclareLaunchArgument(
            'nav2_delay_sec', default_value='8.0',
            description='Seconds to wait after SLAM before starting Nav2, so '
                        "the global costmap's static_layer has a /map to latch."),
        DeclareLaunchArgument(
            'explore_delay_sec', default_value='14.0',
            description='Seconds to wait before starting the explorer. The '
                        'explorer has its own startup_delay_sec on top of '
                        'this; this one just avoids it logging "Nav2 action '
                        'server not available" while Nav2 is still activating.'),

        include('slam.launch.py',
                flight_altitude=LaunchConfiguration('flight_altitude')),

        TimerAction(
            period=LaunchConfiguration('nav2_delay_sec'),
            actions=[include(
                'nav2.launch.py',
                rviz=LaunchConfiguration('rviz'),
                flight_altitude=LaunchConfiguration('flight_altitude'))],
        ),

        TimerAction(
            period=LaunchConfiguration('explore_delay_sec'),
            actions=[include('exploration.launch.py', bridge='false')],
            condition=IfCondition(LaunchConfiguration('explore')),
        ),
    ])
