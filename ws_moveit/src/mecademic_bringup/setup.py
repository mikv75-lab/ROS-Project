from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mecademic_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ROS2 package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Package manifest
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Install config files (yaml + txt)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.txt') + glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='miklv',
    maintainer_email='miklv75@googlemail.com',
    description='Bringup package for Mecademic robot integration (tools, poses, workspace)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tool_manager = mecademic_bringup.tool_manager:main',
            'robot_rebuilder = mecademic_bringup.robot_rebuilder:main',
            'poses_manager = mecademic_bringup.poses_manager:main',
            'workpiece_manager = mecademic_bringup.workpiece_manager:main',
            'spawn_platform = mecademic_bringup.spawn_platform:main',
            'spawn_mesh = mecademic_bringup.spawn_mesh:main',
        ],
    },
)
