cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(nav2_drone_controller LANGUAGES CXX)

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
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Plugins library (e.g., simple goal/progress checkers)
add_library(simple_goal_checker SHARED
  plugins/simple_goal_checker.cpp
)
ament_target_dependencies(simple_goal_checker
  rclcpp
  nav2_core
  nav2_drone_core
)
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

add_library(simple_progress_checker SHARED
  plugins/simple_progress_checker.cpp
)
ament_target_dependencies(simple_progress_checker
  rclcpp
  nav2_core
  nav2_drone_core
)
# Controller action server
add_executable(controller_server
  src/controller_server.cpp
)
ament_target_dependencies(controller_server
  rclcpp
  nav2_core
  nav2_drone_core
)

# Installation
install(TARGETS
  simple_goal_checker
  simple_progress_checker
  controller_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# Install plugin description and headers
install(DIRECTORY include/
        DESTINATION include/)
install(FILES plugins.xml
        DESTINATION share/${PROJECT_NAME})

# Export for downstream
ament_export_include_directories(include)
ament_export_libraries(
  simple_goal_checker
  simple_progress_checker
  controller_server
)
ament_export_dependencies(
  rclcpp
  nav2_core
  nav2_drone_core
)

ament_package()
