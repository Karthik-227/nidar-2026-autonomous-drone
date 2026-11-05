import os
import shutil
import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    SetEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    get_launch_description_from_python_launch_file,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.utilities import perform_substitutions

PROFILES = {
    'baseline': 'nidar_baseline.parm',
    'tuning': 'nidar_tuning.parm',
}

def _upstream_defaults(context, iris_lidar_launch):
    ld = get_launch_description_from_python_launch_file(iris_lidar_launch)
    for arg in ld.get_launch_arguments():
        if arg.name == 'defaults':
            return perform_substitutions(context, list(arg.default_value))
    raise RuntimeError(
        f"{iris_lidar_launch} no longer declares a 'defaults' launch argument, "
        "so nidar_sim's param_profile has nothing to append to. Upstream "
        "ardupilot_gz has changed shape - check how it passes default params "
        "to SITL now and update sim.launch.py to match.")

def _wipe_eeprom(context):
    instance = LaunchConfiguration('instance').perform(context)
    use_instance_dir = LaunchConfiguration('use_instance_dir').perform(context)

    cwd = Path.cwd()
    candidates = [cwd / 'eeprom.bin']
    if use_instance_dir.lower() in ('true', '1'):
        candidates.append(cwd / instance / 'eeprom.bin')

    actions = []
    stamp = time.strftime('%Y%m%d-%H%M%S')
    for path in candidates:
        if path.is_file():
            backup = path.with_name(f'{path.name}.bak-{stamp}')
            shutil.move(str(path), str(backup))
            actions.append(LogInfo(msg=(
                f'[nidar] wiped {path} -> {backup.name}. Every stored param is '
                f'gone, not just the profile ones; restore with '
                f'`mv {backup} {path}`.')))
        else:
            actions.append(LogInfo(
                msg=f'[nidar] no eeprom at {path}, nothing to wipe.'))
    return actions

def _launch_setup(context, *args, **kwargs):
    pkg_nidar = get_package_share_directory('nidar_sim')
    pkg_bringup = get_package_share_directory('ardupilot_gz_bringup')
    iris_lidar_launch = os.path.join(
        pkg_bringup, 'launch', 'robots', 'iris_lidar.launch.py')
    iris_maze_launch = os.path.join(
        pkg_bringup, 'launch', 'iris_maze.launch.py')

    profile = LaunchConfiguration('param_profile').perform(context)
    if profile not in PROFILES and profile != 'none':
        raise RuntimeError(
            f"param_profile:={profile} is not one of "
            f"{sorted(PROFILES) + ['none']}.")

    actions = []
    if LaunchConfiguration('wipe').perform(context).lower() in ('true', '1'):
        actions += _wipe_eeprom(context)

    defaults = _upstream_defaults(context, iris_lidar_launch)
    if profile == 'none':
        actions.append(LogInfo(msg=(
            '[nidar] param_profile:=none - no nidar .parm appended. SITL gets '
            'the upstream defaults and whatever the eeprom holds.')))
    else:
        parm = os.path.join(pkg_nidar, 'config', PROFILES[profile])
        if not os.path.isfile(parm):
            raise RuntimeError(
                f'param_profile:={profile} needs {parm}, which is not '
                f'installed. Run `colcon build --packages-select nidar_sim` - '
                f'config/*.parm has to be listed in setup.py data_files to be '
                f'installed at all.')
        defaults = f'{defaults},{parm}'
        actions.append(LogInfo(msg=(
            f'[nidar] param_profile:={profile} -> appended {parm} to SITL '
            f'defaults. Remember this only takes effect for params the eeprom '
            f'does not already hold: pass wipe:=true on the first launch after '
            f'a switch, or run scripts/apply_param_profile.py.')))

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(iris_maze_launch),
        launch_arguments={
            'defaults': defaults,
            'lidar_dim': LaunchConfiguration('lidar_dim'),
            'rviz': LaunchConfiguration('rviz'),
            'use_gz_sim_gui': LaunchConfiguration('use_gz_sim_gui'),
        }.items(),
    ))
    return actions

def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE',
            PathJoinSubstitution([get_package_share_directory("nidar_sim"), 'config', 'fastdds_no_shm.xml'])),
        DeclareLaunchArgument(
            'param_profile', default_value='tuning',
            description="Which ArduPilot param profile to append to SITL's "
                        "defaults: 'baseline' (pre-tuning values, the revert), "
                        "'tuning' (capped accel/lean, m/s^2 and deg), or "
                        "'none' (append nothing at all)."),
        DeclareLaunchArgument(
            'wipe', default_value='false',
            description='Move SITL eeprom.bin aside before starting, so the '
                        'chosen profile actually applies. Needed once after '
                        'every param_profile change - a defaults file is '
                        'ignored for any param the eeprom already stores. '
                        'Discards ALL stored params; the old file is kept as '
                        'eeprom.bin.bak-<timestamp>.'),
        DeclareLaunchArgument(
            'lidar_dim', default_value='2',
            description='2D lidar. Defaults to 2 here, unlike upstream, '
                        'because the whole SLAM/Nav2 stack in this package '
                        'consumes a 2D /scan.'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description="Open ardupilot_gz's RViz. nidar_bringup opens its own "
                        'with the Map and Costmap displays; set false to avoid '
                        'two windows.'),
        DeclareLaunchArgument(
            'use_gz_sim_gui', default_value='true',
            description='Run the Gazebo GUI. false = headless server, which is '
                        'noticeably faster for repeated A/B runs.'),
        DeclareLaunchArgument(
            'instance', default_value='0',
            description='SITL instance (adds 10*instance to all port numbers).'),
        DeclareLaunchArgument(
            'use_instance_dir', default_value='False',
            description="If True, SITL keeps eeprom.bin in an <instance>/ "
                        'subdirectory of the launch directory.'),

        OpaqueFunction(function=_launch_setup),
    ])
