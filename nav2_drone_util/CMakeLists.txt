cmake_minimum_required(VERSION 3.10)
project(nav2_drone_util)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)               # Added for angular utilities
find_package(geometry_msgs REQUIRED)        # Added for PoseStamped

add_library(${PROJECT_NAME}
  src/angle_utils.cpp
  src/costmap_utils.cpp
  src/drone_utils.cpp
  src/node_thread.cpp
  src/tf_help.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  angles                                    # angular utilities
  geometry_msgs                             # PoseStamped
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  angles                                    # angular utilities
  geometry_msgs                             # PoseStamped
)

ament_package()
