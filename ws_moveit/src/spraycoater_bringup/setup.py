from setuptools import setup, find_packages
from glob import glob

package_name = "spraycoater_bringup"

setup(
    name=package_name,
    version="0.1.0",
    # Falls du keine Python-Module außer __init__.py hast, ist das ok.
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        # ROS 2 package index
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        # package.xml
        (f"share/{package_name}", ["package.xml"]),
        # Launch files
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        # Zentrale Configs
        (f"share/{package_name}/config", glob("config/*.yaml") + glob("config/*.rviz")),
        # Zusätzliche YAMLs (z.B. Pfade)
        (f"share/{package_name}/spray_paths", glob("spray_paths/*.yaml")),
        # Ressourcen/Meshes
        (f"share/{package_name}/resource/environment", glob("resource/environment/*.stl")),
        (f"share/{package_name}/resource/substrate_mounts", glob("resource/substrate_mounts/*.stl")),
        (f"share/{package_name}/resource/substrates", glob("resource/substrates/*.stl")),
        (f"share/{package_name}/resource/tools", glob("resource/tools/*.stl")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="miklv",
    maintainer_email="miklv75@googlemail.com",
    description="Mecademic Bringup: Launch + zentrale Configs/Ressourcen.",
    license="Apache-2.0",
    # Keine console_scripts mehr hier – die liegen jetzt in mecademic_nodes_py / _cpp
    entry_points={
        "console_scripts": [],
    },
)
