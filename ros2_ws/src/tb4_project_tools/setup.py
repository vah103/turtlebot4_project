from glob import glob
from setuptools import find_packages, setup


package_name = "tb4_project_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="TurtleBot4 Project Maintainer",
    maintainer_email="maintainer@example.com",
    description="Passive, configurable status monitoring tools for TurtleBot4.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "robot_status_monitor = tb4_project_tools.robot_status_monitor:main",
        ],
    },
)

