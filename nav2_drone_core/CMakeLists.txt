cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(nav2_drone_core LANGUAGES CXX)

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
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_drone_costmap_3d REQUIRED)

# Library target
add_library(${PROJECT_NAME} SHARED
  src/controller.cpp
  src/goal_checker.cpp
  src/planner.cpp
  src/progress_checker.cpp
)

# Include directories for build and install
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  nav2_core
  pluginlib
  geometry_msgs
)

# Export plugin descriptions
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include/
)

# Install library
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install plugin XML
install(
  FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export package configuration for downstream
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  nav2_core
  pluginlib
  geometry_msgs
)

# Finalize package
ament_package()
