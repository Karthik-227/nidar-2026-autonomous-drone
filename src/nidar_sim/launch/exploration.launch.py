from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('nidar_sim')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE',
            PathJoinSubstitution(
                [pkg_share, 'config', 'fastdds_no_shm.xml'])),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo clock. Must stay true in simulation: '
                        'the explorer measures its own timeouts against TF '
                        'stamps, which are sim time.'),
        DeclareLaunchArgument(
            'bridge', default_value='true',
            description='Start the /cmd_vel -> /ap/v1/cmd_vel bridge. Set '
                        'false if nav2.launch.py already started one.'),
        DeclareLaunchArgument(
            'frame_id', default_value='base_link',
            description="frame_id stamped onto /ap/v1/cmd_vel. ArduPilot "
                        "rejects an empty frame."),
        DeclareLaunchArgument(
            'max_linear_vel', default_value='1.10',
            description='Horizontal SPEED cap (m/s) applied by the bridge, '
                        'direction-preserving. Matches MPPI vx_max/vy_max.'),
        DeclareLaunchArgument(
            'max_angular_vel', default_value='1.5',
            description='Backstop clamp (rad/s) applied by the bridge.'),

        Node(
            package='nidar_sim',
            executable='cmd_vel_bridge',
            name='cmd_vel_bridge',
            output='screen',
            condition=IfCondition(LaunchConfiguration('bridge')),
            parameters=[{
                'use_sim_time': use_sim_time,
                'frame_id': LaunchConfiguration('frame_id'),
                'max_linear_vel': LaunchConfiguration('max_linear_vel'),
                'max_angular_vel': LaunchConfiguration('max_angular_vel'),
            }],
        ),

        Node(
            package='nidar_sim',
            executable='frontier_explorer',
            name='frontier_explorer',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
