cmake_minimum_required(VERSION 3.10)
project(nav2_drone_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs            REQUIRED)
find_package(builtin_interfaces REQUIRED)
# find_package(geometry_msgs      REQUIRED)
find_package(nav_msgs           REQUIRED)
find_package(action_msgs        REQUIRED)
find_package(geographic_msgs    REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
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
  # "msg/Odometry.msg"
  "srv/ClearCostmapArounddrone.srv"
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
  "action/Dockdrone.action"
  "action/FollowGPSWaypoints.action"
  "action/FollowPath.action"
  "action/FollowWaypoints.action"
  "action/NavigateThroughPoses.action"
  "action/NavigateToPose.action"
  "action/SmoothPath.action"
  "action/Spin.action"
  "action/Wait.action"
  DEPENDENCIES
    std_msgs
    builtin_interfaces
    geometry_msgs
    nav_msgs
    action_msgs
    geographic_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
