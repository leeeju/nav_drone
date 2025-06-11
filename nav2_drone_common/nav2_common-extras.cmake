# Specify the keyword separator for AMENT build configuration
set(AMENT_BUILD_CONFIGURATION_KEYWORD_SEPARATOR ":")

# Validate that nav2_drone_common_DIR is set
if(NOT DEFINED nav2_drone_common_DIR)
  message(FATAL_ERROR "nav2_drone_common_DIR is not defined. Ensure that nav2_drone_common is properly sourced or built.")
endif()

# Include shared Nav2 package configuration
include("${nav2_drone_common_DIR}/nav2_package.cmake")

# Add additional custom CMake logic or utilities as needed
# For example, if there are reusable functions, macros, or configurations, add them here.

