from setuptools import find_packages, setup
import os
from glob import glob

package_name = "traffic_sign_tracker"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "models"), glob("models/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Victor Manuel Vazquez Morales",
    maintainer_email="victormvm05043@gmail.com",
    description=(
        "High-performance traffic sign perception system using YOLOv8 and a custom "
        "Centroid Tracker. This package provides real-time infrastructure detection, "
        "functional family categorization, and temporal consistency management. "
        "Developed specifically for resource-constrained hardware like the Puzzlebot."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "tracker = traffic_sign_tracker.traffic_sign_tracker:main"
        ],
    },
)
