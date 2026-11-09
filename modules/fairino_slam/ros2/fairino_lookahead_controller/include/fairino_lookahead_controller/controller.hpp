#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "fairino_lookahead_controller/core.hpp"
#include "fairino_lookahead_controller/coordination.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav2_core/controller.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp/subscription.hpp"

namespace fairino_lookahead_controller
{
class HolonomicLookaheadController : public nav2_core::Controller
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  geometry_msgs::msg::PoseStamped transformPose(
    const geometry_msgs::msg::PoseStamped & input, const std::string & frame) const;
  bool checkCollision(
    const geometry_msgs::msg::PoseStamped & pose, const core::Velocity & command,
    const core::Velocity & measured) const;
  void reportMotion(const std::string & state, const std::string & reason);
  std::mutex mutex_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr state_pub_;
  bool obstacle_wait_{false};
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr motion_sub_;
  std::unique_ptr<core::Follower> follower_;
  std::unique_ptr<coordination::Guard> coordination_;
  core::Settings settings_;
  std::string path_frame_;
  double period_{.05}, transform_tolerance_{.3}, collision_horizon_{1.}, collision_step_{.05};
  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
  bool active_{false};
  bool plan_updated_{false};
};
}  // namespace fairino_lookahead_controller
