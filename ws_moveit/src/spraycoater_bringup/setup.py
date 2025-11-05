from setuptools import setup
from glob import glob

package_name = "spraycoater_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],  # hier keine Python-Module installieren
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py") + glob("launch/*.py")),
        (f"share/{package_name}/config", glob("config/*.yaml") + glob("config/*.rviz")),
        (f"share/{package_name}/spray_paths", glob("spray_paths/*.yaml")),
        (f"share/{package_name}/resource/environment", glob("resource/environment/*.stl")),
        (f"share/{package_name}/resource/substrate_mounts", glob("resource/substrate_mounts/*.stl")),
        (f"share/{package_name}/resource/substrates", glob("resource/substrates/*.stl")),
        (f"share/{package_name}/resource/tools", glob("resource/tools/*.stl")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="miklv",
    maintainer_email="miklv75@googlemail.com",
    description="Spraycoater Bringup: Launch + zentrale Configs/Ressourcen.",
    license="Apache-2.0",
    entry_points={"console_scripts": []},
)
