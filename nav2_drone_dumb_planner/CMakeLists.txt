cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(nav2_drone_dumb_planner LANGUAGES CXX)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_drone_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Dependencies for ament
set(dependencies
  rclcpp
  nav2_core
  nav2_drone_core
  pluginlib
)

# Build library
add_library(${PROJECT_NAME} SHARED
  src/dumb_planner.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# Register plugin description
pluginlib_export_plugin_description_file(${PROJECT_NAME} nav2_drone_dumb_planner.xml)

# Installation
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME})

install(FILES nav2_drone_dumb_planner.xml
        DESTINATION share/${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export for downstream use
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
