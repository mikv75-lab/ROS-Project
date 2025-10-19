from setuptools import setup, find_packages
from glob import glob
import os

package_name = "mecademic_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml") + glob("config/*.rviz")),
        ("share/" + package_name + "/resource/environment", glob("resource/environment/*.stl")),
        ("share/" + package_name + "/resource/substrate_mounts", glob("resource/substrate_mounts/*.stl")),
        ("share/" + package_name + "/resource/substrates", glob("resource/substrates/*.stl")),
        ("share/" + package_name + "/resource/tools", glob("resource/tools/*.stl")),
        ("share/" + package_name + "/spray_paths", glob("spray_paths/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="miklv",
    maintainer_email="miklv75@googlemail.com",
    description="Mecademic Robot Bringup: TF frames, scenes, tools, spray paths.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "tool_manager = mecademic_bringup.tool_manager:main",
            "poses_manager = mecademic_bringup.poses_manager:main",
            "scene_manager = mecademic_bringup.scene_manager:main",
            "spray_path_manager = mecademic_bringup.spray_path_manager:main",
        ],
    },
)
