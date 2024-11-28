cmake_minimum_required(VERSION 3.8)
project(nav2_drone_util)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_drone_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(dependencies
    rclcpp
    rclcpp_action
    tf2
    tf2_ros
    tf2_geometry_msgs
    nav_msgs
    geometry_msgs
    nav2_drone_msgs
)

# Include directories
include_directories(include)

# Add subdirectory
add_subdirectory(src)

# Install include directories
install(DIRECTORY include/
  DESTINATION include/
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export package dependencies
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
