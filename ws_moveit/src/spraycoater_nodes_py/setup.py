from setuptools import find_packages, setup

package_name = 'spraycoater_nodes_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vincent Mikl',
    maintainer_email='miklv75@googlemail.com',
    description='Spraycoater Python nodes (scene, tools, motion, …)',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # Startet den Scene-Node (spraycoater_nodes_py/scene.py -> main())
            'scene = spraycoater_nodes_py.scene:main',
            'poses = spraycoater_nodes_py.poses:main',
        ],
    },
)
