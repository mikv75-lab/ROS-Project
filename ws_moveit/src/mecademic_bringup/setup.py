from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mecademic_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=['mecademic_bringup', 'mecademic_bringup.*']),
    data_files=[
        # ROS2 package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Package manifest
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Install configs
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml') + glob('config/*.txt') + glob('config/*.rviz')),

        # Install STL resources
        (os.path.join('share', package_name, 'resource/environment'),
         glob('resource/environment/*.stl')),
        (os.path.join('share', package_name, 'resource/substrate_mounts'),
         glob('resource/substrate_mounts/*.stl')),
        (os.path.join('share', package_name, 'resource/substrates'),
         glob('resource/substrates/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='miklv',
    maintainer_email='miklv75@googlemail.com',
    description='Bringup package for Mecademic robot integration (scene, tools, workspace)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tool_manager = mecademic_bringup.tool_manager:main',
            'poses_manager = mecademic_bringup.poses_manager:main',
            'scene_manager = mecademic_bringup.scene.scene_manager_node:main',
        ],
    },
)
