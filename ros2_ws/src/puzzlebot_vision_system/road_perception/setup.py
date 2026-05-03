from setuptools import find_packages, setup

package_name = "road_perception"

setup(
    name=package_name,
    version="1.0.0",
    # find_packages ensures road_perception.scene, road_perception.utils, etc., are installed
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    # Consistent with your CV and package.xml
    maintainer="Victor Manuel Vazquez Morales",
    maintainer_email="victormvm05043@gmail.com",
    description=(
        "Advanced road infrastructure perception system for autonomous navigation. "
        "Implements semantic segmentation, geometric profiling via distance transforms, "
        "and egocentric path interpretation for steering target generation."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "path_interpreter = road_perception.path_interpreter:main",
        ],
    },
)