#include "fairino_lookahead_controller/controller.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <stdexcept>
#include <utility>
#include <vector>

#include "fairino_lookahead_controller/collision.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace fairino_lookahead_controller
{
namespace
{
double steadySeconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

void validatePoseFields(const geometry_msgs::msg::Pose & pose)
{
  const auto & p = pose.position;
  const auto & q = pose.orientation;
  for (const double value : {p.x, p.y, p.z, q.x, q.y, q.z, q.w}) {
    if (!std::isfinite(value)) {throw nav2_core::PlannerException("Non-finite pose");}
  }
}

void validatePose(const geometry_msgs::msg::Pose & pose)
{
  validatePoseFields(pose);
  const auto & q = pose.orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (std::abs(norm - 1.) > .01) {throw nav2_core::PlannerException("Invalid pose quaternion");}
}

core::Velocity planarVelocity(const geometry_msgs::msg::Twist & velocity)
{
  for (const double value : {velocity.linear.x, velocity.linear.y, velocity.linear.z,
      velocity.angular.x, velocity.angular.y, velocity.angular.z})
  {
    if (!std::isfinite(value)) {throw nav2_core::PlannerException("Non-finite measured velocity");}
  }
  return {velocity.linear.x, velocity.linear.y, velocity.angular.z};
}
}  // namespace

void HolonomicLookaheadController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto node = parent.lock();
  if (!node || !tf || !costmap_ros || !costmap_ros->getLayeredCostmap()) {
    throw nav2_core::PlannerException("Lookahead controller dependencies are unavailable");
  }
  node_ = parent; tf_ = std::move(tf); costmap_ros_ = std::move(costmap_ros);
  clock_ = node->get_clock();
  const auto number = [&](const std::string & key, double fallback) {
      nav2_util::declare_parameter_if_not_declared(node, name + "." + key,
        rclcpp::ParameterValue(fallback));
      const double value = node->get_parameter(name + "." + key).as_double();
      if (!std::isfinite(value) || value <= 0) {
        throw nav2_core::PlannerException("Invalid lookahead parameter: " + key);
      }
      return value;
    };
  nav2_util::declare_parameter_if_not_declared(node, name + ".heading_mode",
    rclcpp::ParameterValue(std::string("hold_initial")));
  const auto heading_mode = node->get_parameter(name + ".heading_mode").as_string();
  if (heading_mode == "hold_initial") {
    settings_.heading_mode = core::HeadingMode::HoldInitial;
  } else if (heading_mode == "path_tangent") {
    settings_.heading_mode = core::HeadingMode::PathTangent;
  } else {
    throw nav2_core::PlannerException("Unknown lookahead heading mode: " + heading_mode);
  }
  settings_.lookahead_distance = number("lookahead_distance", .4);
  settings_.projection_search_distance = number("projection_search_distance", 1.);
  settings_.position_gain = number("position_gain", .8);
  settings_.heading_gain = number("heading_gain", 1.5);
  settings_.max_linear = number("max_linear_speed", .06);
  settings_.max_angular = number("max_angular_speed", .1);
  settings_.linear_acceleration = number("linear_acceleration", .2);
  settings_.angular_acceleration = number("angular_acceleration", .4);
  settings_.xy_goal_tolerance = number("xy_goal_tolerance", .05);
  settings_.yaw_goal_tolerance = number("yaw_goal_tolerance", .05);
  period_ = number("control_period", .05);
  transform_tolerance_ = number("transform_tolerance", .3);
  collision_horizon_ = number("collision_horizon", 1.);
  collision_step_ = number("collision_step", .05);
  // Keep this limit aligned with navigation/runtime_policy.py::COMMAND_TIMEOUT.
  constexpr double command_timeout = .6;
  double frequency = 0.;
  if (!node->get_parameter("controller_frequency", frequency) ||
    !std::isfinite(frequency) || std::abs(frequency * period_ - 1.) > 1e-6 || period_ >= command_timeout)
  {
    throw nav2_core::PlannerException("Controller period/frequency mismatch or stale-command period");
  }
  coordination::Settings coordination_settings;
  coordination_settings.feedback_timeout = number("chassis_feedback_timeout", .4);
  coordination_settings.linear_speed = number("coordination_linear_speed", .02);
  coordination_settings.angular_speed = number("coordination_angular_speed", .04);
  coordination_settings.wait_timeout = number("coordination_wait_timeout", 15.);
  coordination_settings.direction_tolerance = number("coordination_direction_tolerance", .20);
  if (coordination_settings.linear_speed > settings_.max_linear ||
    coordination_settings.angular_speed > settings_.max_angular)
  {
    throw nav2_core::PlannerException("Coordination speed exceeds navigation speed limit");
  }
  coordination_ = std::make_unique<coordination::Guard>(coordination_settings);
  follower_ = std::make_unique<core::Follower>(settings_);
  motion_sub_ = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/fairino_navigation/chassis_motion_state",
    rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
    [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr message) {
      std::lock_guard<std::mutex> guard(mutex_);
      if (!coordination_ || !clock_) {return;}
      try {
        if (message->status.size() != 1) {
          throw std::runtime_error("Expected one chassis motion diagnostic status");
        }
        const auto & status = message->status.front();
        coordination::Diagnostic diagnostic;
        diagnostic.name = status.name;
        diagnostic.session = status.hardware_id;
        diagnostic.message = status.message;
        diagnostic.level = status.level;
        if (message->header.stamp.sec < 0 || message->header.stamp.nanosec >= 1000000000u) {
          throw std::runtime_error("Invalid chassis motion ROS stamp");
        }
        diagnostic.stamp = rclcpp::Time(
          message->header.stamp, clock_->get_clock_type()).seconds();
        for (const auto & field : status.values) {
          diagnostic.values.emplace_back(field.key, field.value);
        }
        coordination_->receive(diagnostic, clock_->now().seconds(), steadySeconds());
      } catch (const std::exception & error) {
        coordination_->invalidate(error.what());
      }
    });
  state_pub_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/fairino_navigation/controller_motion", rclcpp::QoS(10));
  target_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("lookahead_target", 1);
  path_frame_.clear();
  active_ = false;
  plan_updated_ = false;
}

void HolonomicLookaheadController::cleanup()
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = false;
  plan_updated_ = false;
  motion_sub_.reset(); coordination_.reset();
  state_pub_.reset();
  follower_.reset(); target_pub_.reset(); tf_.reset(); costmap_ros_.reset(); clock_.reset();
  path_frame_.clear();
}

void HolonomicLookaheadController::activate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!follower_) {throw nav2_core::PlannerException("Lookahead controller is not configured");}
  follower_->clear(); coordination_->clear(); path_frame_.clear();
  plan_updated_ = false;
  target_pub_->on_activate();
  state_pub_->on_activate();
  obstacle_wait_ = false;
  last_time_ = clock_->now();
  active_ = true;
}

void HolonomicLookaheadController::deactivate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = false;
  plan_updated_ = false;
  if (follower_) {follower_->clear();}
  if (coordination_) {coordination_->clear();}
  if (target_pub_) {target_pub_->on_deactivate();}
  if (state_pub_) {state_pub_->on_deactivate();}
  obstacle_wait_ = false;
  path_frame_.clear();
}

void HolonomicLookaheadController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!follower_ || path.header.frame_id.empty() || path.poses.empty()) {
    throw nav2_core::PlannerException("Lookahead path is empty or controller is not configured");
  }
  std::vector<core::Point> points;
  points.reserve(path.poses.size());
  for (const auto & pose : path.poses) {
    if (!pose.header.frame_id.empty() && pose.header.frame_id != path.header.frame_id) {
      throw nav2_core::PlannerException("Lookahead path contains mixed coordinate frames");
    }
    // Smac2D with smoothing disabled leaves intermediate orientations at the
    // default all-zero quaternion. They are not heading targets; only XY is used.
    validatePoseFields(pose.pose);
    points.push_back({pose.pose.position.x, pose.pose.position.y});
  }
  validatePose(path.poses.back().pose);
  if (!path_frame_.empty() && path_frame_ != path.header.frame_id) {
    // Progress and command vectors belong to the old path frame. Never silently
    // reuse them in a different coordinate system within the same lifecycle.
    throw nav2_core::PlannerException("Path frame changed during navigation");
  }
  try {
    // The sender supplies held current yaw or an explicit terminal yaw.
    // Smac2D preserves it with use_final_approach_orientation=false on replans.
    follower_->setPath(points, tf2::getYaw(path.poses.back().pose.orientation));
    path_frame_ = path.header.frame_id;
    plan_updated_ = true;
    obstacle_wait_ = false;
  } catch (const std::exception & error) {
    throw nav2_core::PlannerException(error.what());
  }
}

geometry_msgs::msg::PoseStamped HolonomicLookaheadController::transformPose(
  const geometry_msgs::msg::PoseStamped & input, const std::string & frame) const
{
  validatePose(input.pose);
  if (input.header.frame_id.empty()) {throw nav2_core::PlannerException("Pose frame is empty");}
  if (input.header.frame_id == frame) {return input;}
  try {
    const auto transform = tf_->lookupTransform(frame, input.header.frame_id,
      tf2::TimePointZero, tf2::durationFromSec(transform_tolerance_));
    const rclcpp::Time stamp(transform.header.stamp, clock_->get_clock_type());
    // A zero timestamp denotes an all-static TF chain.
    if (stamp.nanoseconds() != 0 && std::abs((clock_->now() - stamp).seconds()) > transform_tolerance_) {
      throw nav2_core::PlannerException("Lookahead transform is stale");
    }
    geometry_msgs::msg::PoseStamped output;
    tf2::doTransform(input, output, transform);
    validatePose(output.pose);
    return output;
  } catch (const tf2::TransformException & error) {
    throw nav2_core::PlannerException(std::string("Lookahead TF failed: ") + error.what());
  }
}

bool HolonomicLookaheadController::checkCollision(
  const geometry_msgs::msg::PoseStamped & pose, const core::Velocity & command,
  const core::Velocity & measured) const
{
  auto * map = costmap_ros_->getCostmap();
  const auto map_pose = transformPose(pose, costmap_ros_->getGlobalFrameID());
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
  if (!costmap_ros_->isCurrent()) {throw nav2_core::PlannerException("Local costmap is stale");}
  std::vector<core::Point> footprint;
  for (const auto & p : costmap_ros_->getRobotFootprint()) {footprint.push_back({p.x, p.y});}
  const collision::Grid grid{map->getCharMap(), map->getSizeInCellsX(), map->getSizeInCellsY(),
    map->getResolution(), map->getOriginX(), map->getOriginY()};
  const core::Pose start{map_pose.pose.position.x, map_pose.pose.position.y,
    tf2::getYaw(map_pose.pose.orientation)};
  const double speed = std::max(std::hypot(command.vx, command.vy), std::hypot(measured.vx, measured.vy));
  const double yaw_rate = std::max(std::abs(command.wz), std::abs(measured.wz));
  // Holding velocity for the full stopping time is conservative relative to
  // ideal constant deceleration. Include one control-period reaction delay.
  const double horizon = std::max({collision_horizon_, speed / settings_.linear_acceleration + period_,
      yaw_rate / settings_.angular_acceleration + period_});
  if (!collision::sweep_clear(grid, footprint, start, command, horizon, collision_step_) ||
    !collision::sweep_clear(grid, footprint, start, measured, horizon, collision_step_))
  {
    return false;
  }
  return true;
}

void HolonomicLookaheadController::reportMotion(
  const std::string & state, const std::string & reason)
{
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = clock_->now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = state; status.message = reason;
  status.level = state == "failed" ? 2 : 1;
  message.status.push_back(status);
  if (state_pub_) {state_pub_->publish(message);}
}

geometry_msgs::msg::TwistStamped HolonomicLookaheadController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_ || !follower_ || path_frame_.empty()) {
    throw nav2_core::PlannerException("Lookahead controller has no active path");
  }
  try {
    const auto now = clock_->now();
    const rclcpp::Time pose_stamp(pose.header.stamp, clock_->get_clock_type());
    if (pose_stamp.nanoseconds() == 0 || std::abs((now - pose_stamp).seconds()) > transform_tolerance_) {
      throw nav2_core::PlannerException("Robot pose is stale");
    }
    const auto measured = planarVelocity(velocity);
    const auto world_pose = transformPose(pose, path_frame_);
    const auto coordination_state = coordination_->prepare(clock_->now().seconds(), steadySeconds());
    double elapsed = (now - last_time_).seconds();
    if (elapsed <= 0) {
      throw nav2_core::PlannerException("Controller clock did not advance");
    }
    // Humble has no per-goal controller reset. A paused control stream, or a
    // new/replanned path after stationary feedback, must not reuse a pre-cancel
    // positive command. Moving replans keep the existing acceleration history.
    const bool stopped = measured.vx == 0. && measured.vy == 0. && measured.wz == 0.;
    if (coordination_state.reseed || elapsed >= 2. * period_ || (plan_updated_ && stopped)) {
      follower_->reset(measured);
      elapsed = period_;
    }
    plan_updated_ = false;
    // Do not turn a scheduling delay into one large acceleration step.
    const double dt = std::min(elapsed, period_);
    last_time_ = now;
    auto result = follower_->compute(
      {world_pose.pose.position.x, world_pose.pose.position.y, tf2::getYaw(world_pose.pose.orientation)},
      measured, dt, !coordination_state.waiting);
    result.command = coordination_->apply(result.desired, result.command);
    const bool clear = checkCollision(pose, result.command, measured);
    if (!clear || obstacle_wait_) {
      // Latch zero until a new plan. The BT cancels this follower then replans;
      // progress-check and other controller failures remain terminal.
      obstacle_wait_ = true;
      follower_->reset(); coordination_->stop();
      reportMotion("blocked", "局部障碍、未知区域或地图边界阻挡运动；已暂停，每 1 秒重规划");
      geometry_msgs::msg::TwistStamped zero;
      zero.header.frame_id = costmap_ros_->getBaseFrameID();
      zero.header.stamp = clock_->now();
      return zero;
    }
    // TF/costmap work may take time: validate at the actual output instant too.
    coordination_->validate(clock_->now().seconds(), steadySeconds());
    geometry_msgs::msg::PoseStamped target;
    target.header.frame_id = path_frame_; target.header.stamp = now;
    target.pose.position.x = result.target.x; target.pose.position.y = result.target.y;
    target.pose.orientation.z = std::sin(result.target_yaw / 2.);
    target.pose.orientation.w = std::cos(result.target_yaw / 2.);
    target_pub_->publish(target);
    geometry_msgs::msg::TwistStamped output;
    output.header.frame_id = costmap_ros_->getBaseFrameID(); output.header.stamp = clock_->now();
    output.twist.linear.x = result.command.vx; output.twist.linear.y = result.command.vy;
    output.twist.angular.z = result.command.wz;
    return output;
  } catch (const std::exception & error) {
    // ControllerServer emits zero on PlannerException. Do not retain a positive
    // command for an automatic retry after a TF/path/collision failure.
    follower_->reset();
    if (coordination_) {coordination_->stop();}
    reportMotion("failed", error.what());
    throw nav2_core::PlannerException(error.what());
  }
}

void HolonomicLookaheadController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!follower_) {return;}
  if (!std::isfinite(speed_limit) || speed_limit < 0) {
    follower_->reset();
    if (coordination_) {coordination_->stop();}
    throw nav2_core::PlannerException("Invalid Nav2 speed limit");
  }
  // Nav2's NO_SPEED_LIMIT value is zero; core zero instead means a real stop.
  const double limit = speed_limit == 0. ? settings_.max_linear :
    (percentage ? settings_.max_linear * speed_limit / 100. : speed_limit);
  follower_->setSpeedLimit(std::min(settings_.max_linear, limit));
}
}  // namespace fairino_lookahead_controller

PLUGINLIB_EXPORT_CLASS(fairino_lookahead_controller::HolonomicLookaheadController, nav2_core::Controller)
