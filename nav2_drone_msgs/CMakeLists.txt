cmake_minimum_required(VERSION 3.8)
project(nav2_drone_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ClearCostmapAroundRobot.srv"
  "srv/ClearCostmapExceptRegion.srv"
  "srv/ClearEntireCostmap.srv"
  "srv/GetCostmap.srv"
  "srv/GetCosts.srv"
  "srv/IsPathValid.srv"
  "srv/LoadMap.srv"
  "srv/ManageLifecycleNodes.srv"
  "srv/ReloadDockDatabase.srv"
  "srv/SaveMap.srv"
  "srv/SetInitialPose.srv"
  "action/BackUp.action"
  "action/ComputePathToPose.action"
  "action/ComputePathThroughPoses.action"
  "action/DockRobot.action"
  "action/FollowGPSWaypoints.action"
  "action/FollowPath.action"
  "action/FollowWaypoints.action"
  "action/NavigateThroughPoses.action"
  "action/NavigateToPose.action"
  "action/SmoothPath.action"
  "action/Spin.action"
  "action/Wait.action"
  "msg/BehaviorTreeLog.msg"
  "msg/BehaviorTreeStatusChange.msg"
  "msg/CollisionDetectorState.msg"
  "msg/CollisionMonitorState.msg"
  "msg/Costmap.msg"
  "msg/CostmapFilterInfo.msg"
  "msg/CostmapMetaData.msg"
  "msg/CostmapUpdate.msg"
  "msg/MissedWaypoint.msg"
  "msg/Particle.msg"
  "msg/ParticleCloud.msg"
  "msg/SpeedLimit.msg"
  "msg/VoxelGrid.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs std_msgs action_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
