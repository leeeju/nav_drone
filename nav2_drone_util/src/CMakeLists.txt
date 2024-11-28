add_library(${PROJECT_NAME}_core
  angle_utils.cpp
  costmap_utils.cpp
  node_thread.cpp
  robot_utils.cpp
  tf_help.cpp
)

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  rclcpp_action
  rclcpp_components
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  geometry_msgs
  nav2_drone_msgs
)

install(TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
