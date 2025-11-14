from setuptools import find_packages, setup
import os
from glob import glob

package_name = "spraycoater_nodes_py"

def files(pattern):
    # Helper that returns [] if the folder doesn't exist (avoids setup errors)
    paths = glob(pattern, recursive=True)
    return paths if paths else []

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # ament resource index + package.xml
        (os.path.join("share", "ament_index", "resource_index", "packages"),
         [os.path.join("resource", package_name)]),
        (os.path.join("share", package_name), ["package.xml"]),

        # ✅ Install all launch files recursively
        (os.path.join("share", package_name, "launch"),
         files("launch/*.py") + files("launch/**/*.py")),
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
            "scene = spraycoater_nodes_py.scene:main",
            "poses = spraycoater_nodes_py.poses:main",
            "spray_path = spraycoater_nodes_py.spray_path:main",
            "motion = spraycoater_nodes_py.motion:main",
        ],
    },
)
