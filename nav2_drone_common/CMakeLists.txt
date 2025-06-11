cmake_minimum_required(VERSION 3.10)
project(nav2_drone_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
# find_package(launch REQUIRED)
# find_package(launch_ros REQUIRED)
# find_package(osrf_pycommon REQUIRED)
find_package(rclpy REQUIRED)

# Install Python scripts
install(
  DIRECTORY nav2_drone_common/
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# Add tests if enabled
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip linter if copyright not present
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
