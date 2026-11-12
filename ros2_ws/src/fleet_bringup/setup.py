from glob import glob

from setuptools import find_packages, setup

package_name = "fleet_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={"console_scripts": [
        "mission_runner = fleet_bringup.mission_runner:main",
        "fleet_monitor = fleet_bringup.fleet_monitor:main",
        "fleet_report = fleet_bringup.fleet_report:main",
    ]},
)
