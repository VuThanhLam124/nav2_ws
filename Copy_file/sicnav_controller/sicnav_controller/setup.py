# Replace the setup file and modify: sicnav_controller/setup.py
from setuptools import setup
import os

package_name = "sicnav_controller"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="your_email@example.com",
    description="SICNAV Controller for Navigation2",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "controller_node = sicnav_controller.controller:main",
        ],
    },
    data_files=[
        # Register marker file for ament
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "plugin.xml"]),
    ],
)
