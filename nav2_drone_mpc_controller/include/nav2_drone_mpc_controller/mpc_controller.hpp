#ifndef NAV2_DRONE_MPC_CONTROLLER__MPC_CONTROLLER_HPP_
#define NAV2_DRONE_MPC_CONTROLLER__MPC_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "nav2_drone_core/controller.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_drone_costmap_3d/costmap_server.hpp"
#include "nav2_drone_mpc_controller/histogram.hpp"

#include <dlib/matrix.h>

namespace nav2_drone_mpc_controller {

class MPCController : public nav2_drone_core::Controller
{
public:
  using Costmap3DServerPtr = std::shared_ptr<nav2_drone_costmap_3d::CostmapServer>;

  MPCController() = default;
  ~MPCController() override = default;

  void configure(
    rclcpp::Node::SharedPtr node,
    const std::string & name,
    const std::shared_ptr<tf2_ros::Buffer> & tf,
    const Costmap3DServerPtr & costmap) override;

  void setPath(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & speed) override;

  bool isGoalReached(
    const geometry_msgs::msg::PoseStamped & pose) override;

  void updateMap(const Costmap3DServerPtr & costmap) override;

private:
  rclcpp::Node::SharedPtr node_;
  std::string plugin_name_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  Costmap3DServerPtr costmap_;
  nav_msgs::msg::Path global_plan_;

  // MPC internals
  dlib::matrix<double, 6, 6> A;
  dlib::matrix<double, 6, 3> B;
  dlib::matrix<double, 6, 1> C;
  dlib::matrix<double, 6, 1> Q;
  dlib::matrix<double, 3, 1> R;
  dlib::matrix<double, 3, 1> lower;
  dlib::matrix<double, 3, 1> upper;
  std::shared_ptr<dlib::mpc<6,3,30>> controller;

  double lookahead_dist_;
  double min_lookahead_dist_;
  double max_lookahead_dist_;
  double lookahead_time_;
  bool use_velocity_scaled_lookahead_dist_;
  double robot_radius_;
  double safety_radius_;
  double desired_linear_vel_;

  double last_e_angle_;
  double last_z_angle_;

  // Helpers
  double getLookAheadDistance(const geometry_msgs::msg::Twist & speed) const;
  std::pair<int,int> get_ez_grid_pos(const octomap::point3d & goal) const;
  std::pair<double,double> get_ez(
    const geometry_msgs::msg::PoseStamped & curr,
    const geometry_msgs::msg::PoseStamped & targ) const;
  geometry_msgs::msg::PoseStamped getLookAheadPoint(
    double lookahead_dist,
    const geometry_msgs::msg::PoseStamped & curr) const;
};

}  // namespace nav2_drone_mpc_controller

#endif  // NAV2_DRONE_MPC_CONTROLLER__MPC_CONTROLLER_HPP_
