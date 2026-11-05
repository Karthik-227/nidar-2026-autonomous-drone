from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('nidar_sim')

    return LaunchDescription([
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE',
            PathJoinSubstitution(
                [pkg_share, 'config', 'fastdds_no_shm.xml'])),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo clock. Must stay true in simulation.'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=PathJoinSubstitution(
                [pkg_share, 'config', 'mapper_params.yaml']),
            description='slam_toolbox parameter file.'),

        DeclareLaunchArgument(
            'stabilizer', default_value='true',
            description='Run scan_stabilizer (/scan -> /scan_safe). SLAM and '
                        'both Nav2 costmaps read /scan_safe, so turning this '
                        'off leaves them with no scan at all unless you also '
                        'repoint scan_topic in the config files. There is '
                        'almost never a reason to: an unstabilised 2D scan '
                        'from a leaning multirotor is what erases mapped '
                        'walls in the first place.'),
        DeclareLaunchArgument(
            'map_filter', default_value='true',
            description='Run map_filter (/map -> /map_filtered), which deletes '
                        'occupied components too small to be a wall. false '
                        'publishes nothing on /map_filtered, so the explorer '
                        'would see no map at all.'),
        DeclareLaunchArgument(
            'min_blob_cells', default_value='30',
            description='Smallest occupied component kept, in cells. The '
                        'arena\'s shortest wall is 1.10 m x 0.10 m = 44 cells '
                        'at 0.05 m; the largest phantom island measured was 6. '
                        '30 is deliberately conservative: below the 44 of a '
                        'complete wall, five times clear of the speckle. See '
                        'nidar_sim/map_filter.py.'),
        DeclareLaunchArgument(
            'max_hit_dz_m', default_value='1.05',
            description='How far above/below the scan plane a return may land '
                        'and still be reported as a 2D obstacle. Anything '
                        'further off-plane is emitted as NaN, which SLAM and '
                        'the costmaps ignore entirely - no mark, and no '
                        'ray-trace that could erase a wall the beam skimmed.'),
        DeclareLaunchArgument(
            'min_hit_height_agl_m', default_value='0.25',
            description='Returns this close to the ground are withheld. At '
                        '2 m altitude a beam depressed past ~12 deg reaches '
                        'the floor inside the lidar range, and a floor strike '
                        'is a confident finite return that paints a phantom '
                        'obstacle and clears every real wall on the way to it.'),
        DeclareLaunchArgument(
            'flight_altitude', default_value='1.17',
            description='The height the drone is actually being held at '
                        '(cmd_vel_bridge target_altitude_m). Only used to '
                        'derive max_hit_height_agl_m below - set them together '
                        'or set neither. nidar_bringup.launch.py passes this '
                        'down so one number drives the whole stack.'),
        DeclareLaunchArgument(
            'max_hit_height_agl_m', default_value='',
            description='Reject returns at or above this height AGL. '
                        'DERIVED FROM THE FLIGHT ALTITUDE - re-derive it if '
                        'that changes. It has to clear the highest honest '
                        'return and still sit under the top of the walls: at '
                        'the 1.25 m flight height the lidar is at 1.325 m and '
                        'max_hit_dz_m allows +/-0.75 m of it, so an honest '
                        'return reaches 2.075 m, and the walls are 2.60 m. '
                        '2.40 sits between the two. '
                        'It also has to stay UNDER the arena net, which is '
                        'real: 28 rods 0.03 m thick spanning z 2.485-2.515 m, '
                        'links of the <model name="arena"> rather than a '
                        'model of their own. A rod is a confident finite '
                        'return, and each one clipped becomes an isolated '
                        'lethal cell that inflation draws as a disc. '
                        'The old 2.20 was right about the net and wrong about '
                        'the altitude: at the bridge\'s old 2.0 m it left '
                        '0.125 m '
                        'of headroom - so every lean withheld a large, '
                        'ONE-SIDED slice of the ring. An asymmetric partial '
                        'scan is the worst input a correlative scan matcher '
                        'can get, and slam_toolbox duly slid 2.4 m and then '
                        '6.1 m off ground truth on the 16 Oct baseline. 0 '
                        'disables the check.'),
        DeclareLaunchArgument(
            'max_clear_elev_deg', default_value='10.0',
            description='A ray that returned nothing only counts as evidence '
                        'of free space if it stayed this close to horizontal. '
                        'Too tight and the drone stops clearing the space it '
                        'is flying into for the whole of every acceleration, '
                        'so anything spurious already marked there is '
                        'permanent. 8 deg over the 8 m marking range is 1.1 m '
                        'of drift. Derive it, do not guess: '
                        'degrees(asin(headroom / raytrace_max_range)) with '
                        'headroom = min(net, wall) height - altitude. In the '
                        'arena at the 1.0 m flight altitude that is '
                        '(2.50 - 1.0)/9.0 -> 9.6 deg, so the default is 8.0. '
                        'Only headroom ABOVE counts: a down-tilted ray that '
                        'reaches the floor returns the floor, so it is a hit, '
                        'not a clear. Fly higher and this must get TIGHTER '
                        '(3.2 deg at 2.0 m) as the net comes closer.'),

        Node(
            package='nidar_sim',
            executable='scan_stabilizer',
            name='scan_stabilizer',
            output='screen',
            condition=IfCondition(LaunchConfiguration('stabilizer')),
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'max_hit_dz_m': LaunchConfiguration('max_hit_dz_m'),
                'min_hit_height_agl_m': LaunchConfiguration(
                    'min_hit_height_agl_m'),
                'max_clear_elev_deg': LaunchConfiguration(
                    'max_clear_elev_deg'),
                'max_hit_height_agl_m': PythonExpression([
                    "float('",
                    LaunchConfiguration('max_hit_height_agl_m'),
                    "') if '",
                    LaunchConfiguration('max_hit_height_agl_m'),
                    "'.strip() else float('",
                    LaunchConfiguration('flight_altitude'),
                    "') + 1.15",
                ]),
            }],
        ),

        Node(
            package='nidar_sim',
            executable='map_filter',
            name='map_filter',
            output='screen',
            condition=IfCondition(LaunchConfiguration('map_filter')),
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'min_blob_cells': LaunchConfiguration('min_blob_cells'),
            }],
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                LaunchConfiguration('slam_params_file'),
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'scan_topic': PythonExpression([
                        "'/scan_safe' if '",
                        LaunchConfiguration('stabilizer'),
                        "'.lower() in ('true', '1') else '/scan'",
                    ]),
                },
            ],
        ),
    ])
