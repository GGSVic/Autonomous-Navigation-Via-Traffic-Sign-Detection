
cmake_minimum_required(VERSION 3.8)
project(ros_gz_puzzlebot_gazebo)

# Enable C++17 for compatibility with modern Gazebo headers
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)

# --- Install Logic ---

# Install specific directories to the share folder
install(DIRECTORY 
    launch 
    models 
    worlds
  DESTINATION share/${PROJECT_NAME}
)

# Export the Gazebo hooks so Gazebo knows where to find your models/plugins
# This refers to the file you mentioned: hooks/ros_gz_puzzlebot_gazebo.dsv.in
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()