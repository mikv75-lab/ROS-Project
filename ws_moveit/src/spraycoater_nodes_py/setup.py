from setuptools import find_packages, setup
import os
from glob import glob

package_name = "spraycoater_nodes_py"


def files(*patterns: str):
    """
    Helper, der eine Liste von Dateien zu mehreren Patterns zurückgibt.
    Falls nichts gefunden wird, kommt einfach eine leere Liste zurück.
    """
    paths = []
    for pattern in patterns:
        paths.extend(glob(pattern, recursive=True))
    return paths


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # ament resource index + package.xml
        (
            os.path.join("share", "ament_index", "resource_index", "packages"),
            [os.path.join("resource", package_name)],
        ),
        (os.path.join("share", package_name), ["package.xml"]),

        # ✅ alle Launch-Files (inkl. launch/common, falls vorhanden)
        (
            os.path.join("share", package_name, "launch"),
            files("launch/*.py", "launch/**/*.py"),
        ),

        # ✅ common-Verzeichnis (z.B. YAMLs, Configs, …)
        (
            os.path.join("share", package_name, "common"),
            files("common/*", "common/**/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Vincent Mikl",
    maintainer_email="miklv75@googlemail.com",
    description="Spraycoater Python nodes (scene, tools, motion, …)",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            # Kern-Nodes
            "scene = spraycoater_nodes_py.scene:main",
            "poses = spraycoater_nodes_py.poses:main",
            "spray_path = spraycoater_nodes_py.spray_path:main",
            "moveit_py = spraycoater_nodes_py.moveitpy_node:main",
            "servo = spraycoater_nodes_py.servo:main",
            # Robot-Backends
            "robot_sim = spraycoater_nodes_py.robot.sim:main",
            "robot_omron = spraycoater_nodes_py.robot.omron:main",
            # Omron TCP Bridge
            "omron_tcp_bridge = spraycoater_nodes_py.robot.omron_tcp_bridge:main",
        ],
    },
)
