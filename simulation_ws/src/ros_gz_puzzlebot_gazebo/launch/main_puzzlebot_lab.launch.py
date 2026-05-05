from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Main orchestration launch file for the Puzzlebot competition.
    Integrates world simulation, robot state publishing, and entity spawning.
    """

    world_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ros_gz_puzzlebot_gazebo"),
                    "launch",
                    "start_puzzlebot_world.launch.py",
                ]
            )
        )
    )

    state_publisher_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ros_gz_puzzlebot_bringup"),
                    "launch",
                    "state_publisher.launch.py",
                ]
            )
        )
    )

    spawn_entity_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ros_gz_puzzlebot_bringup"),
                    "launch",
                    "spawn_puzzlebot.launch.py",
                ]
            )
        )
    )

    return LaunchDescription([world_launcher, state_publisher_launcher, spawn_entity_launcher])
