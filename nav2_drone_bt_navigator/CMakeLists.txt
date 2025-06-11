cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(nav2_drone_bt_navigator LANGUAGES CXX)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_drone_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

# Include directories
include_directories(
  include
)

# Package dependencies for ament
set(dependencies
  rclcpp
  nav2_core
  nav2_drone_core
  pluginlib
  behaviortree_cpp_v3
)

# Build BT Navigator node
add_executable(bt_navigator_node
  ${CMAKE_CURRENT_SOURCE_DIR}/src/bt_navigator_node.cpp)
ament_target_dependencies(bt_navigator_node ${dependencies})

# Install rules
install(TARGETS bt_navigator_node
  DESTINATION lib/${PROJECT_NAME}
)

# Export for downstream use
ament_export_dependencies(${dependencies})
ament_package()
