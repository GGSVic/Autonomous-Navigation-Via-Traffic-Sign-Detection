#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description() -> LaunchDescription:


    debug_arg = DeclareLaunchArgument(
        "debug",
        default_value="false",
        choices=["true", "false"],
        description="Enable visual telemetry and debug mode for perception nodes.",
    )

    device_arg = DeclareLaunchArgument(
        "device_type",
        default_value="cpu",
        choices=["cpu", "cuda"],
        description="Device type for YOLO inference in traffic_sign_tracker.",
    )

    debug_mode = LaunchConfiguration("debug")
    device_type = LaunchConfiguration("device_type")


    path_interpreter = Node(
        package="road_perception",
        executable="path_interpreter",
        name="path_interpreter",
        output="screen",
        parameters=[{"debug": debug_mode}], 
    )

    traffic_sign_tracker = Node(
        package="traffic_sign_tracker",
        executable="tracker",
        name="traffic_sign_tracker",
        output="screen",
        parameters=[
            {"debug": debug_mode},
            {"device_type": device_type}
        ],
    )

    navigator_node = Node(
        package="puzzlebot_controller", 
        executable="navigator_node", 
        name="navigator_node", 
        output="screen"
    )

    return LaunchDescription([
        debug_arg, 
        device_arg,
        path_interpreter, 
        traffic_sign_tracker, 
        navigator_node
    ])