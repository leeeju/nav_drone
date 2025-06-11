cmake_minimum_required(VERSION 3.10)
project(nav2_drone_costmap_3d)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(octomap REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_drone_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(nav2_drone_util REQUIRED)


add_library(${PROJECT_NAME}
  src/costmap_3d.cpp
  src/costmap_publisher.cpp
  src/layered_costmap_3d.cpp
  src/octomap_wrapper.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  nav2_costmap_2d
  octomap
  pluginlib
  tf2_ros
  nav2_drone_msgs
  octomap_msgs
  nav2_drone_util
)

target_include_directories(${PROJECT_NAME} PUBLIC
  # This package's headers
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  # Include headers from nav2_drone_util (sibling package)
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../nav2_drone_util/include>
  # Installed include path
  $<INSTALL_INTERFACE:include>
)

# --------------------------------------------------------------------
# 3) Export plugin description file
# --------------------------------------------------------------------
pluginlib_export_plugin_description_file(nav2_costmap_3d plugins.xml)

# --------------------------------------------------------------------
# 4) Install headers and library
# --------------------------------------------------------------------
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# --------------------------------------------------------------------
# 5) Export package
# --------------------------------------------------------------------
ament_package()
