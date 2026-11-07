#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <type_traits>

#include "fairino_straight_planner/core.hpp"
#include "fairino_straight_planner/planner_api.hpp"
#include "nav2_core/global_planner.hpp"
#include "pluginlib/class_loader.hpp"

namespace fairino_straight_planner
{
class StraightLinePlanner : public nav2_core::GlobalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  // Nav2 overlays may backport the cancellable API independently of ROS_DISTRO.
  // Provide both overloads: the installed base selects which one is virtual.
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker);

private:
  void checkInflation() const;
  core::PreparedGrid snapshot(const core::Checkpoint & checkpoint) const;
  void validatePose(const geometry_msgs::msg::PoseStamped & pose) const;
  nav_msgs::msg::Path makePath(
    const std::vector<core::Point> & points,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const core::Checkpoint & checkpoint) const;
  std::mutex mutex_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  // Destruction order matters: instances must go away before the library loader.
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::GlobalPlanner>> loader_;
  nav2_core::GlobalPlanner::Ptr fallback_;
  std::string name_, global_frame_;
  bool active_{false};
};
static_assert(!std::is_abstract_v<StraightLinePlanner>,
  "StraightLinePlanner does not implement the installed Nav2 GlobalPlanner API");
}  // namespace fairino_straight_planner
