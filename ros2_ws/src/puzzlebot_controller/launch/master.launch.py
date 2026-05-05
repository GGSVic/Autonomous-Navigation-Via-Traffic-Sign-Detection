from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory

import os


def generate_launch_description():

    controller_pkg = get_package_share_directory("puzzlebot_controller")
    gazebo_pkg = get_package_share_directory("ros_gz_puzzlebot_gazebo")

    nav_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_pkg, "launch", "nav_system.launch.py")
        )
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, "launch", "main_puzzlebot_lab.launch.py")
        )
    )

    return LaunchDescription([gazebo_launch, nav_system_launch])
