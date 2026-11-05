import importlib.util
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PROFILES = {
    'baseline': 'nidar_baseline.parm',
    'tuning': 'nidar_tuning.parm',
}

def _sim_launch_helpers(pkg_nidar):
    path = os.path.join(pkg_nidar, 'launch', 'sim.launch.py')
    spec = importlib.util.spec_from_file_location('nidar_sim_launch', path)
    if spec is None or not os.path.isfile(path):
        raise RuntimeError(
            f'{path} is missing, and arena.launch.py reuses its param_profile '
            f'and eeprom-wipe helpers. Both files are installed together by '
            f'setup.py, so this means a partial install - rerun '
            f'`colcon build --packages-select nidar_sim`.')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod

def _profile_script():
    here = Path(__file__).resolve()
    for cand in (here.parents[1] / 'scripts' / 'apply_param_profile.py',
                 here.parent.parent / 'scripts' / 'apply_param_profile.py',
                 Path.home() / 'ardu_ws' / 'src' / 'nidar_sim' / 'scripts'
                 / 'apply_param_profile.py'):
        if cand.is_file():
            return str(cand)
    raise RuntimeError(
        'apply_param_profile.py not found; cannot enforce param_profile. '
        'Looked next to this launch file and in ~/ardu_ws/src/nidar_sim/scripts.')

def _assert_no_other_stack():
    import subprocess
    try:
        out = subprocess.run(['ps', '-eo', 'pid,cmd', '--no-headers'],
                             capture_output=True, text=True, timeout=5).stdout
    except Exception:
        return
    patterns = ('arducopter', 'gz sim', 'ros_gz_bridge',
                'async_slam_toolbox_node', 'controller_server')
    found = []
    for line in out.splitlines():
        for pat in patterns:
            if pat in line and 'grep' not in line:
                found.append(f"  {line.strip()[:100]}")
                break
    if found:
        raise RuntimeError(
            'A simulation stack is ALREADY RUNNING. Starting a second one '
            'resets /clock, and every node in the first stack then sees time '
            'jump backwards and clears its TF buffer - Nav2 loses every '
            'transform and nothing in the resulting log says why.\n\n'
            'Found:\n' + '\n'.join(found[:8]) + '\n\n'
            'Clear it first:\n'
            '    ~/ardu_ws/scripts/nidar-clean.sh')

def _launch_setup(context, *args, **kwargs):
    _assert_no_other_stack()
    pkg_nidar = get_package_share_directory('nidar_sim')
    pkg_bringup = get_package_share_directory('ardupilot_gz_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    iris_lidar_launch = os.path.join(
        pkg_bringup, 'launch', 'robots', 'iris_lidar.launch.py')

    arena = LaunchConfiguration('arena').perform(context)
    world_name = arena if arena.startswith('nidar_arena_') else f'nidar_arena_{arena}'
    world_file = os.path.join(pkg_nidar, 'worlds', f'{world_name}.sdf')
    if not os.path.isfile(world_file):
        available = sorted(
            p.stem for p in Path(pkg_nidar, 'worlds').glob('nidar_arena_*.sdf'))
        raise RuntimeError(
            f'arena:={arena} resolves to {world_file}, which does not exist. '
            f'Installed arenas: {", ".join(available) or "none"}. Generate more '
            f'with scripts/generate_arena.py, then rebuild the package - worlds '
            f'are only visible here once setup.py has installed them.')

    profile = LaunchConfiguration('param_profile').perform(context)
    if profile not in PROFILES and profile != 'none':
        raise RuntimeError(
            f'param_profile:={profile} is not one of '
            f"{sorted(PROFILES) + ['none']}.")

    helpers = _sim_launch_helpers(pkg_nidar)

    actions = []
    actions.append(SetEnvironmentVariable(
        'FASTRTPS_DEFAULT_PROFILES_FILE',
        os.path.join(pkg_nidar, 'config', 'fastdds_no_shm.xml')))

    actions.append(SetEnvironmentVariable('GZ_IP', '127.0.0.1'))
    actions.append(SetEnvironmentVariable('IGN_IP', '127.0.0.1'))

    _plugin_path = os.path.join(
        get_package_prefix('ardupilot_gazebo'), 'lib', 'ardupilot_gazebo')
    actions.append(SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', _plugin_path))
    actions.append(SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', _plugin_path))
    if LaunchConfiguration('wipe').perform(context).lower() in ('true', '1'):
        actions += helpers._wipe_eeprom(context)

    defaults = helpers._upstream_defaults(context, iris_lidar_launch)
    if profile == 'none':
        actions.append(LogInfo(msg=(
            '[nidar] param_profile:=none - no nidar .parm appended.')))
    else:
        parm = os.path.join(pkg_nidar, 'config', PROFILES[profile])
        if not os.path.isfile(parm):
            raise RuntimeError(
                f'param_profile:={profile} needs {parm}, which is not '
                f'installed. Run `colcon build --packages-select nidar_sim`.')
        defaults = f'{defaults},{parm}'
        actions.append(LogInfo(msg=(
            f'[nidar] param_profile:={profile} -> appended {parm}. Only applies '
            f'to params the eeprom does not already hold; pass wipe:=true on '
            f'the first launch after a switch.')))

    actions.append(LogInfo(msg=f'[nidar] arena {world_name} -> {world_file}'))

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'),
        launch_arguments={'gz_args': f'-v4 -s -r {world_file}'}.items(),
        condition=IfCondition(LaunchConfiguration('use_gz_sim_server')),
    ))

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'),
        launch_arguments={'gz_args': '-v4 -g'}.items(),
        condition=IfCondition(LaunchConfiguration('use_gz_sim_gui')),
    ))

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(iris_lidar_launch),
        launch_arguments={
            'defaults': defaults,
            'lidar_dim': LaunchConfiguration('lidar_dim'),
            'world_name': world_name,
            'instance': LaunchConfiguration('instance'),
            'use_instance_dir': LaunchConfiguration('use_instance_dir'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('spawn_robot')),
    ))

    if profile != 'none':
        actions.append(TimerAction(
            period=25.0,
            actions=[ExecuteProcess(
                cmd=['python3', _profile_script(), profile],
                output='screen',
                name='enforce_param_profile',
            )],
        ))

    actions.append(Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', f'{Path(pkg_bringup) / "rviz" / "iris_with_lidar.rviz"}'],
        condition=IfCondition(LaunchConfiguration('rviz')),
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    ))

    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'arena', default_value='1',
            description="Which generated arena to fly: 1, 2 or 3, all at "
                        "Mission-Brief spec (1.00 m corridors). A full world name (nidar_arena_2) also "
                        "works, for arenas you generate yourself."),
        DeclareLaunchArgument(
            'param_profile', default_value='tuning',
            description="ArduPilot param profile appended to SITL's defaults: "
                        "'baseline', 'tuning', or 'none'. Same meaning as in "
                        "sim.launch.py."),
        DeclareLaunchArgument(
            'wipe', default_value='false',
            description='Move SITL eeprom.bin aside first, so the chosen '
                        'param_profile actually applies. Needed once after '
                        'every profile change. Discards ALL stored params; the '
                        'old file is kept as eeprom.bin.bak-<timestamp>.'),
        DeclareLaunchArgument(
            'lidar_dim', default_value='2',
            description='2D lidar, because the SLAM/Nav2 stack in this package '
                        'consumes a 2D /scan.'),
        DeclareLaunchArgument(
            'use_gz_sim_server', default_value='true',
            description='Run the Gazebo server.'),
        DeclareLaunchArgument(
            'use_gz_sim_gui', default_value='true',
            description='Run the Gazebo GUI. false = headless, noticeably '
                        'faster for repeated A/B runs.'),
        DeclareLaunchArgument(
            'spawn_robot', default_value='true',
            description='Spawn the iris and start SITL. false brings the world '
                        'up on its own, which is how to eyeball a freshly '
                        'generated arena without waiting for SITL.'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description="Open ardupilot_gz's RViz (robot model + raw lidar). "
                        'Defaults false here, unlike sim.launch.py: that layout '
                        'has no Map or Costmap display, so for arena work the '
                        'one nidar_bringup opens is the useful window and this '
                        'is just a second one to close.'),
        DeclareLaunchArgument(
            'instance', default_value='0',
            description='SITL instance (adds 10*instance to all port numbers).'),
        DeclareLaunchArgument(
            'use_instance_dir', default_value='False',
            description='If True, SITL keeps eeprom.bin in an <instance>/ '
                        'subdirectory of the launch directory.'),

        OpaqueFunction(function=_launch_setup),
    ])
