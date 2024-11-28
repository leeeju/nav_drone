cmake_minimum_required(VERSION 3.8)
project(nav2_drone_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

# Specify dependencies
set(dependencies
  rclcpp
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  pluginlib
  nav_msgs
  tf2_ros
  tf2
)

# Include headers
include_directories(
  include
)

# Add library (commented out because no source files are available)
add_library(${PROJECT_NAME}_core
  src/controller.cpp
  src/planner.cpp
  src/goal_checker.cpp
  src/progress_checker.cpp
)

# Link dependencies to the library (if library exists)
# ament_target_dependencies(${PROJECT_NAME}_core ${dependencies})

# Install headers only for now
install(
  DIRECTORY include/
  DESTINATION include/
)

# Ament package export
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
