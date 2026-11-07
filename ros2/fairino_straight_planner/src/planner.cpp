#include "fairino_straight_planner/planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <thread>
#include <utility>

#include "fairino_straight_planner/planner_exceptions.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace fairino_straight_planner
{
void StraightLinePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  std::lock_guard<std::mutex> guard(mutex_);
  const auto node = parent.lock();
  if (!node || !tf || !costmap_ros || !costmap_ros->getCostmap()) {
    throw nav2_core::PlannerException("StraightLinePlanner requires a node, TF and global costmap");
  }
  if (fallback_) {
    throw nav2_core::PlannerException("StraightLinePlanner is already configured");
  }
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".straight_line_enabled", rclcpp::ParameterValue(true));
  auto loader = std::make_unique<pluginlib::ClassLoader<nav2_core::GlobalPlanner>>(
    "nav2_core", "nav2_core::GlobalPlanner");
  auto fallback = loader->createSharedInstance("nav2_smac_planner/SmacPlanner2D");
  // The original namespace is intentional: all existing Smac parameters remain effective.
  fallback->configure(parent, name, tf, costmap_ros);
  node_ = parent;
  name_ = std::move(name);
  global_frame_ = costmap_ros->getGlobalFrameID();
  costmap_ros_ = std::move(costmap_ros);
  loader_ = std::move(loader);
  fallback_ = std::move(fallback);
  active_ = false;
}

void StraightLinePlanner::cleanup()
{
  std::lock_guard<std::mutex> guard(mutex_);
  active_ = false;
  if (fallback_) {fallback_->cleanup();}
  fallback_.reset();
  loader_.reset();
  costmap_ros_.reset();
  node_.reset();
}

void StraightLinePlanner::activate()
{
  std::lock_guard<std::mutex> guard(mutex_);
  if (!fallback_) {throw nav2_core::PlannerException("Planner is not configured");}
  fallback_->activate();
  active_ = true;
}

void StraightLinePlanner::deactivate()
{
  std::lock_guard<std::mutex> guard(mutex_);
  active_ = false;
  if (fallback_) {fallback_->deactivate();}
}

void StraightLinePlanner::checkInflation() const
{
  const auto layered = costmap_ros_->getLayeredCostmap();
  const double radius = layered->getInscribedRadius();
  if (!std::isfinite(radius) || radius <= 0.) {
    throw nav2_core::PlannerException("Straight planning requires a positive global footprint radius");
  }
  bool valid_inflation = false;
  for (const auto & layer : *layered->getPlugins()) {
    const auto inflation = std::dynamic_pointer_cast<nav2_costmap_2d::InflationLayer>(layer);
    if (!inflation || !inflation->isEnabled()) {continue;}
    double inflation_radius = 0.;
    if (costmap_ros_->get_parameter(layer->getName() + ".inflation_radius", inflation_radius) &&
      std::isfinite(inflation_radius) && inflation_radius >= radius)
    {
      valid_inflation = true;
    }
  }
  if (!valid_inflation) {
    throw nav2_core::PlannerException(
            "Straight planning requires enabled inflation covering the global footprint radius");
  }
}

core::PreparedGrid StraightLinePlanner::snapshot(const core::Checkpoint & checkpoint) const
{
  auto * map = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex(), std::defer_lock);
  // Bound waiting for a costmap update by the same planning budget.
  while (!lock.try_lock()) {checkpoint(); std::this_thread::yield();}
  checkpoint();
  const unsigned int width = map->getSizeInCellsX();
  const unsigned int height = map->getSizeInCellsY();
  if (width == 0 || height == 0 ||
    static_cast<std::size_t>(width) > std::numeric_limits<std::size_t>::max() / height)
  {
    throw nav2_core::PlannerException("Global costmap has invalid dimensions");
  }
  const std::size_t count = static_cast<std::size_t>(width) * height;
  std::vector<unsigned char> cells(count);
  for (std::size_t i = 0; i < count; i += 4096) {
    checkpoint();
    const auto end = std::min(count, i + 4096);
    std::copy(map->getCharMap() + i, map->getCharMap() + end, cells.begin() + i);
  }
  const core::Grid raw{
    cells.data(), width, height, map->getResolution(), map->getOriginX(), map->getOriginY()};
  const double radius = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
  lock.unlock();
  // The existing costmap already inflates known obstacles. Only unknown cells and
  // map boundaries need the additional footprint guard in the geometry core.
  return core::prepare_grid(raw, radius, checkpoint);
}

void StraightLinePlanner::validatePose(const geometry_msgs::msg::PoseStamped & pose) const
{
  const auto & p = pose.pose.position;
  const auto & q = pose.pose.orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (pose.header.frame_id != global_frame_ ||
    !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
    !std::isfinite(norm) || std::abs(norm - 1.) > 1e-3)
  {
    throw nav2_core::PlannerException("Planner poses must be finite, normalized and in the global frame");
  }
}

nav_msgs::msg::Path StraightLinePlanner::makePath(
  const std::vector<core::Point> & points,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const core::Checkpoint & checkpoint) const
{
  nav_msgs::msg::Path result;
  result.header.frame_id = global_frame_;
  const auto node = node_.lock();
  if (!node) {throw nav2_core::PlannerException("Planner node has expired");}
  result.header.stamp = node->now();
  result.poses.reserve(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (i % 64 == 0) {checkpoint();}
    geometry_msgs::msg::PoseStamped pose;
    pose.header = result.header;
    pose.pose.position.x = points[i].x;
    pose.pose.position.y = points[i].y;
    if (i + 1 < points.size()) {
      const double yaw = std::atan2(points[i + 1].y - points[i].y, points[i + 1].x - points[i].x);
      pose.pose.orientation.z = std::sin(yaw / 2.);
      pose.pose.orientation.w = std::cos(yaw / 2.);
    } else {
      // Both hold-initial and path-tangent strategies use the caller's terminal yaw.
      pose.pose.orientation = goal.pose.orientation;
    }
    result.poses.push_back(pose);
  }
  if (!result.poses.empty()) {
    result.poses.front().pose.position = start.pose.position;
    result.poses.back().pose.position = goal.pose.position;
    result.poses.back().pose.orientation = goal.pose.orientation;
  }
  return result;
}

nav_msgs::msg::Path StraightLinePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  return createPlan(start, goal, {});
}

nav_msgs::msg::Path StraightLinePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  std::lock_guard<std::mutex> guard(mutex_);
  const auto node = node_.lock();
  if (!active_ || !fallback_ || !node) {
    throw nav2_core::PlannerException("StraightLinePlanner is not active");
  }
  const bool enabled = node->get_parameter(name_ + ".straight_line_enabled").as_bool();
  if (!enabled) {
    // This is a true rollback switch: retain Smac's tolerance, endpoints,
    // orientations and path sampling without any new post-processing.
    return compat::call_create_plan(*fallback_, start, goal, cancel_checker);
  }
  try {
    const double seconds = node->get_parameter(name_ + ".max_planning_time").as_double();
    if (!std::isfinite(seconds) || seconds <= 0.) {
      throw nav2_core::PlannerException("max_planning_time must be finite and positive");
    }
    const auto began = std::chrono::steady_clock::now();
    const core::Checkpoint checkpoint = [began, seconds, cancel_checker]() {
        if (cancel_checker && cancel_checker()) {
          throw compat::PlannerCancelled("Straight planner request was cancelled");
        }
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count() >= seconds) {
          throw nav2_core::PlannerException("Straight planner exceeded max_planning_time");
        }
      };
    checkpoint();
    validatePose(start);
    validatePose(goal);
    checkInflation();
    checkpoint();
    auto prepared = snapshot(checkpoint);
    const core::Point first{start.pose.position.x, start.pose.position.y};
    const core::Point last{goal.pose.position.x, goal.pose.position.y};
    if (!core::segment_clear(prepared.view(), first, first, checkpoint) ||
      !core::segment_clear(prepared.view(), last, last, checkpoint))
    {
      throw nav2_core::PlannerException("Start or exact goal is outside the collision-free global map");
    }
    std::vector<core::Point> route;
    const bool direct = core::segment_clear(prepared.view(), first, last, checkpoint);
    if (direct) {
      route = {first, last};
    } else {
      // No costmap lock survives snapshot(): Smac takes its own mutex and then
      // the costmap lock. Holding the costmap here could invert that lock order.
      const auto raw_path = compat::call_create_plan(*fallback_, start, goal, cancel_checker);
      checkpoint();
      if (raw_path.poses.empty()) {
        throw nav2_core::PlannerException("Smac could not find a path around the obstacle");
      }
      if (raw_path.header.frame_id != global_frame_) {
        throw nav2_core::PlannerException("Smac returned a path in a different frame");
      }
      route.reserve(raw_path.poses.size() + 2);
      route.push_back(first);
      for (const auto & pose : raw_path.poses) {
        checkpoint();
        validatePose(pose);
        route.push_back({pose.pose.position.x, pose.pose.position.y});
      }
      route.push_back(last);
      prepared = snapshot(checkpoint);
      // Smac's 8-neighbour search may connect diagonal cells past a blocked
      // corner. Replace only such adjacent edges with a fully checked axial
      // elbow; never accept the original corner contact or an unchecked bridge.
      route = core::repair_grid_edges(prepared.view(), route, checkpoint);
      route = core::simplify(prepared.view(), route, checkpoint);
    }
    auto points = core::resample(
      prepared.view(), route, prepared.view().resolution, checkpoint);
    // Re-read after Smac/search/simplification. A newly observed obstacle causes
    // this plan to fail, never a return of an unvalidated shortcut.
    const auto latest = snapshot(checkpoint);
    if (points.empty()) {throw nav2_core::PlannerException("Planner produced an empty path");}
    for (std::size_t i = 0; i < points.size(); ++i) {
      const auto & next = points[std::min(i + 1, points.size() - 1)];
      if (!core::segment_clear(latest.view(), points[i], next, checkpoint)) {
        throw nav2_core::PlannerException("Global costmap changed: planned segment is no longer safe");
      }
    }
    auto result = makePath(points, start, goal, checkpoint);
    checkpoint();
    RCLCPP_DEBUG(node->get_logger(), "%s: %s, %zu vertices, %zu output poses",
      name_.c_str(), direct ? "direct line" : "Smac with visibility shortcuts",
      route.size(), result.poses.size());
    return result;
  } catch (const nav2_core::PlannerException &) {
    throw;
  } catch (const std::exception & error) {
    throw nav2_core::PlannerException(std::string("Straight planner: ") + error.what());
  }
}
}  // namespace fairino_straight_planner

PLUGINLIB_EXPORT_CLASS(fairino_straight_planner::StraightLinePlanner, nav2_core::GlobalPlanner)
