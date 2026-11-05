from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('nidar_sim')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo clock. Must stay true in simulation.'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution(
                [pkg_share, 'rviz', 'nidar_nav2.rviz']),
            description='RViz2 config file.'),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_nidar',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),
    ])
