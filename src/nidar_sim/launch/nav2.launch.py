from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('nidar_sim')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE',
            PathJoinSubstitution(
                [pkg_share, 'config', 'fastdds_no_shm.xml'])),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo clock. Must stay true in simulation.'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [pkg_share, 'config', 'nav2_params.yaml']),
            description='Nav2 parameter file.'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Also open RViz2 with the SLAM/Nav2 displays preloaded. '
                        'The RViz2 that ardupilot_gz_bringup opens has no Map or '
                        'Costmap display at all, so it can never show the map.'),
        DeclareLaunchArgument(
            'flight_altitude', default_value='1.17',
            description='Height (m) the bridge holds the drone at. Nav2 is a '
                        '2D planner and never commands a climb rate, so '
                        'without this the drone slowly sinks and lands. It '
                        'also has to match what slam.launch.py derives its '
                        'scan filters from - see cmd_vel_bridge.py.'),
        DeclareLaunchArgument(
            'bridge', default_value='true',
            description='Also start the /cmd_vel -> /ap/v1/cmd_vel bridge. '
                        'Without a bridge (this one or topic_relay_node) the '
                        'drone will not move, however well Nav2 is planning.'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [nav2_bringup_share, 'launch', 'navigation_launch.py'])),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': 'true',
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_share, 'launch', 'rviz.launch.py'])),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),

        Node(
            package='nidar_sim',
            executable='cmd_vel_bridge',
            name='cmd_vel_bridge',
            output='screen',
            condition=IfCondition(LaunchConfiguration('bridge')),
            parameters=[{
                'use_sim_time': use_sim_time,
                'target_altitude_m': LaunchConfiguration('flight_altitude'),
            }],
        ),
    ])
