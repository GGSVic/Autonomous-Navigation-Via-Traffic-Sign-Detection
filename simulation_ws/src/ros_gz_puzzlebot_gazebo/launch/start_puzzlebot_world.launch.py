import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    Orchestrates the Gazebo Sim simulation environment.
    Provides dynamic world loading for the Puzzlebot autonomous navigation tasks.
    """

    # --- Resource Location ---
    # Resolve the shared directory for the gazebo integration package
    pkg_dir = get_package_share_directory("ros_gz_puzzlebot_gazebo")
    worlds_dir = os.path.join(pkg_dir, "worlds")
    
    # --- Dynamic World Discovery ---
    # Filter files to ensure only valid simulation environments are listed in the CLI
    available_worlds = []
    if os.path.exists(worlds_dir):
        available_worlds = [f for f in os.listdir(worlds_dir) if f.endswith(('.sdf', '.world'))]

    # --- Launch Configuration Setup ---
    world_config = LaunchConfiguration("world")
    world_path = PathJoinSubstitution([pkg_dir, "worlds", world_config])

    # --- Argument Definitions ---
    # Define the 'world' argument with dynamic choices based on the worlds directory
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="puzzletrack_v1.sdf",
        description="Specific simulation environment to load from the package.",
        choices=available_worlds if available_worlds else None
    )

    # --- Gazebo Simulation Action ---
    # Include the standard Gazebo Sim launch while passing the auto-run (-r) flag
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"
            ])
        ),
        launch_arguments={
            "gz_args": PythonExpression(["'-r ' + '", world_path, "'"])
        }.items(),
    )

    # --- Execution Manifest ---
    return LaunchDescription([
        world_arg,
        gazebo_sim
    ])