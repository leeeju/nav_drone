cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(nav2_drone_costmap_3d LANGUAGES CXX)

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
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_drone_msgs REQUIRED)

# Source files (explicitly list to avoid building stale examples)
set(SOURCE_FILES
  src/layered_costmap_3d.cpp
  src/costmap_publisher.cpp
  src/octomap_wrapper.cpp
)

# Create library
add_library(${PROJECT_NAME} SHARED
  ${SOURCE_FILES}
)

# Include headers
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  nav2_costmap_2d
  nav2_core
  pluginlib
  nav_msgs
  octomap_msgs
  nav2_drone_msgs
)

# Export plugin if present
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/plugins.xml)
  pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)
endif()

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

# Install plugin file if exists
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/plugins.xml)
  install(
    FILES plugins.xml
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# Export package configuration
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  nav2_costmap_2d
  nav2_core
  pluginlib
  nav_msgs
  nav2_drone_msgs
  octomap_msgs
)

ament_package()
