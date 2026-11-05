import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'nidar_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test', 'GUI', 'humandetection']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Installed so launch files can find them via
        # get_package_share_directory() instead of a hardcoded
        # $HOME/ardu_ws/src/... path that only works on one machine.
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.parm')
            + glob('config/*.xml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        # arena.launch.py runs apply_param_profile.py to force the param
        # profile into the running vehicle, so it has to exist inside the
        # install tree - not only in a source checkout that may not be there.
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
        # The generated competition arenas. The .yaml (survivor ground truth)
        # and .txt (ASCII plan) ship beside the .sdf on purpose: a world is not
        # much use for scoring a detector without knowing where the survivors
        # actually are, and arena.launch.py lists what it found here when an
        # arena:= value does not resolve.
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf') + glob('worlds/*.yaml')
            + glob('worlds/*.txt') + glob('worlds/*.md')
            + glob('worlds/*.png')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ananth',
    maintainer_email='paiananth0838@gmail.com',
    description='NIDAR 2026-27 simulation support: Nav2 <-> ArduPilot cmd_vel '
                'bridging and autonomous frontier exploration.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # Kept under its original name: README step 6 refers to it.
            'topic_relay_node = nidar_sim.topic_relay_node:main',
            'cmd_vel_bridge = nidar_sim.cmd_vel_bridge:main',
            'frontier_explorer = nidar_sim.frontier_explorer:main',
            'scan_stabilizer = nidar_sim.scan_stabilizer:main',
            # /map -> /map_filtered, dropping occupied components too small to
            # be a wall. See nidar_sim/map_filter.py for why size is the only
            # thing that separates a phantom from a wall.
            'map_filter = nidar_sim.map_filter:main',
            # DEPRECATED, superseded by scan_stabilizer. Kept registered
            # so an old command line still resolves instead of failing
            # with 'executable not found'; it prints why on startup.
            'scan_tilt_gate = nidar_sim.scan_tilt_gate:main',
        ],
    },
)
