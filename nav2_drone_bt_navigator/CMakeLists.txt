cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(nav2_drone_bt_navigator LANGUAGES CXX)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR
   CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_drone_core REQUIRED)
find_package(nav2_drone_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_drone_costmap_3d REQUIRED)
find_package(nav2_drone_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Declare ament package dependencies
set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_components
  nav2_core
  nav2_drone_core
  nav2_drone_util
  pluginlib
  behaviortree_cpp_v3
  nav2_behavior_tree
  nav2_drone_costmap_3d
  nav2_drone_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_msgs
  tf2_geometry_msgs
)

# # Build the BT navigator node
# add_executable(bt_navigator_node
#   src/bt_navigator.cpp
# )

# # Include headers
# target_include_directories(bt_navigator_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

# ament_target_dependencies(bt_navigator_node
#   ${dependencies}
# )

# # Installation rules
# install(
#   TARGETS bt_navigator_node
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )

# Export dependencies downstream
ament_export_dependencies(${dependencies})
ament_package()
