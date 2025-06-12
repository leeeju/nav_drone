cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(nav2_drone_mpc_controller LANGUAGES CXX)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_drone_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav2_drone_costmap_3d REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)

set(dependencies
  rclcpp
  nav2_core
  nav2_drone_core
  pluginlib
  nav2_drone_costmap_3d
  octomap
  octomap_msgs
)

# -------------------------------------------------------------------
# MPC Controller plugin
# -------------------------------------------------------------------
add_library(mpc_controller_plugin SHARED
  src/mpc_controller.cpp
)
target_include_directories(mpc_controller_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OCTOMAP_INCLUDE_DIRS}
)
ament_target_dependencies(mpc_controller_plugin ${dependencies})
target_link_libraries(mpc_controller_plugin
  Eigen3::Eigen
  ${OCTOMAP_LIBRARIES}
)
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

# -------------------------------------------------------------------
# Histogram auxiliary library
# -------------------------------------------------------------------
add_library(histogram_lib SHARED
  src/histogram.cpp
)
target_include_directories(histogram_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OCTOMAP_INCLUDE_DIRS}
)
ament_target_dependencies(histogram_lib
  nav2_core
  nav2_drone_core
  nav2_drone_costmap_3d
  octomap
  octomap_msgs
)
target_link_libraries(histogram_lib
  Eigen3::Eigen
  ${OCTOMAP_LIBRARIES}
)

# -------------------------------------------------------------------
# Install rules
# -------------------------------------------------------------------
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS mpc_controller_plugin histogram_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(
  DIRECTORY resources/
  DESTINATION share/${PROJECT_NAME}/resources
)

# Export
ament_export_include_directories(include)
ament_export_libraries(mpc_controller_plugin histogram_lib)
ament_export_dependencies(${dependencies})

ament_package()
