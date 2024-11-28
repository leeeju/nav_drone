cmake_minimum_required(VERSION 3.8)
project(nav2_drone_costmap_3d)

# Enable warnings for GCC and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_drone_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(angles REQUIRED)
find_package(message_filters REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Library
add_library(${PROJECT_NAME}_core SHARED
  src/costmap_3d.cpp
  src/costmap_server.cpp
)

# Include directories
target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

# Link dependencies
ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  nav2_drone_msgs
  pluginlib
  angles
  message_filters
  octomap
  octomap_msgs
  std_srvs
)

# Install targets
install(
  TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Testing setup
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Mark as ament package
ament_package()
