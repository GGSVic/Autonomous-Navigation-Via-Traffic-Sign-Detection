cmake_minimum_required(VERSION 3.8)
project(puzzlebot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(puzzlebot_interfaces REQUIRED)

add_executable(navigator_node
  src/main.cpp
  src/controller/puzzlebot_controller.cpp
  src/navigator_system.cpp
  src/state_machine/state_machine.cpp
)

target_include_directories(navigator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(navigator_node
  rclcpp
  geometry_msgs
  puzzlebot_interfaces
)

install(TARGETS
  navigator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()