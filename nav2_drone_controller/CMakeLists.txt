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
find_package(rclcpp_components REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_drone_core REQUIRED)
find_package(nav2_drone_util REQUIRED)
find_package(nav2_drone_costmap_3d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_drone_msgs REQUIRED)  # Added to include costmap metadata
find_package(nav2_msgs REQUIRED)
find_package(angles REQUIRED)

# Include local headers
include_directories(
  include
)

# Plugins: simple goal checker
add_library(simple_goal_checker SHARED
  plugins/simple_goal_checker.cpp
)
target_include_directories(simple_goal_checker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(simple_goal_checker
  rclcpp
  nav2_core
  nav2_drone_core
  nav2_drone_util
  nav2_drone_costmap_3d
  pluginlib
  geometry_msgs
  nav2_drone_msgs    # Added for CostmapMetaData
  angles
)

# Plugins: simple progress checker
add_library(simple_progress_checker SHARED
  plugins/simple_progress_checker.cpp
)
target_include_directories(simple_progress_checker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(simple_progress_checker
  rclcpp
  nav2_core
  nav2_drone_core
  nav2_drone_util
  nav2_drone_costmap_3d
  pluginlib
  geometry_msgs
  nav2_drone_msgs   # Added for CostmapMetaData
)

# Export plugin descriptions
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

# Controller action server as a component
add_executable(controller_server
  src/controller_server.cpp
)
target_include_directories(controller_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(controller_server
  rclcpp
  rclcpp_components
  nav2_core
  nav2_drone_core
  nav2_drone_util
  nav2_drone_costmap_3d
  nav2_msgs
  geometry_msgs
  nav2_drone_msgs  # Added for CostmapMetaData
)

# Register as rclcpp component
rclcpp_components_register_node(
  controller_server
  PLUGIN "nav2_drone_controller::ControllerServer"
  EXECUTABLE controller_server_node
)

# Installation rules
install(
  TARGETS
    simple_goal_checker
    simple_progress_checker
    controller_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)

install(DIRECTORY include/
        DESTINATION include)
install(FILES plugins.xml
        DESTINATION share/${PROJECT_NAME})

# Export for downstream
ament_export_include_directories(include)
ament_export_libraries(simple_goal_checker simple_progress_checker controller_server)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  nav2_core
  nav2_drone_core
  nav2_drone_util
  nav2_drone_costmap_3d
  pluginlib
  geometry_msgs
  nav2_msgs
  angles
)

ament_package()
