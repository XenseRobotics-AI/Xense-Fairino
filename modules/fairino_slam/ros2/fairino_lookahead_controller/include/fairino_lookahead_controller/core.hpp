#ifndef FAIRINO_LOOKAHEAD_CONTROLLER__CORE_HPP_
#define FAIRINO_LOOKAHEAD_CONTROLLER__CORE_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace fairino_lookahead_controller
{
namespace core
{

struct Point {double x{0.0}; double y{0.0};};
struct Pose {double x{0.0}; double y{0.0}; double yaw{0.0};};
struct Velocity {double vx{0.0}; double vy{0.0}; double wz{0.0};};

enum class HeadingMode {HoldInitial, PathTangent};

struct Settings
{
  double lookahead_distance{0.4};
  double position_gain{0.8};
  double heading_gain{1.5};
  double max_linear{0.06};
  double max_angular{0.10};
  double linear_acceleration{0.20};
  double angular_acceleration{0.40};
  double xy_goal_tolerance{0.05};
  double yaw_goal_tolerance{0.05};
  double projection_search_distance{1.0};
  HeadingMode heading_mode{HeadingMode::HoldInitial};
};

struct Result
{
  Point target;
  double target_yaw{0.0};
  Velocity command;
  Velocity desired;  // Body-frame target before acceleration limiting.
  double remaining_distance{0.0};
  bool terminal{false};
};

inline double wrap_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline bool finite(const Point & p) {return std::isfinite(p.x) && std::isfinite(p.y);}
inline bool finite(const Pose & p)
{
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.yaw);
}
inline bool finite(const Velocity & v)
{
  return std::isfinite(v.vx) && std::isfinite(v.vy) && std::isfinite(v.wz);
}
inline double distance(const Point & a, const Point & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

// No ROS state lives here: this exact implementation is compiled in offline tests.
class Follower
{
public:
  explicit Follower(const Settings & settings = Settings{})
  : settings_(settings), speed_limit_(settings.max_linear)
  {
    for (const double value : {
        settings.lookahead_distance, settings.position_gain, settings.heading_gain,
        settings.max_linear, settings.max_angular, settings.linear_acceleration,
        settings.angular_acceleration, settings.xy_goal_tolerance,
        settings.yaw_goal_tolerance, settings.projection_search_distance})
    {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument("Lookahead settings must be finite and positive");
      }
    }
    if (settings.heading_mode != HeadingMode::HoldInitial &&
      settings.heading_mode != HeadingMode::PathTangent)
    {
      throw std::invalid_argument("Unknown lookahead heading mode");
    }
    if (settings.xy_goal_tolerance >= settings.lookahead_distance) {
      throw std::invalid_argument("Goal tolerance must be smaller than lookahead distance");
    }
  }

  void setPath(const std::vector<Point> & points, double terminal_yaw)
  {
    if (points.empty() || !std::isfinite(terminal_yaw)) {
      throw std::invalid_argument("Path and terminal heading must be valid");
    }
    std::vector<Point> compact;
    for (const auto & point : points) {
      if (!finite(point)) {
        throw std::invalid_argument("Path contains a nonfinite position");
      }
      if (compact.empty() || distance(compact.back(), point) > epsilon_) {
        compact.push_back(point);
      }
    }
    bool identical = compact.size() == path_.size();
    if (identical) {
      for (std::size_t i = 0; i < compact.size(); ++i) {
        if (distance(compact[i], path_[i]) > epsilon_) {
          identical = false;
          break;
        }
      }
    }
    target_yaw_ = wrap_angle(terminal_yaw);
    if (identical) {
      return;  // A repeated plan must not rewind progress at a crossing.
    }
    path_ = std::move(compact);
    lengths_.assign(path_.size(), 0.0);
    for (std::size_t i = 1; i < path_.size(); ++i) {
      lengths_[i] = lengths_[i - 1] + distance(path_[i - 1], path_[i]);
      if (!std::isfinite(lengths_[i])) {
        throw std::invalid_argument("Path length is not finite");
      }
    }
    progress_ = 0.0;
    projected_ = false;
    // Replanning does not reset the commanded velocity or recapture the heading.
  }

  void setSpeedLimit(double max_linear)
  {
    if (!std::isfinite(max_linear) || max_linear < 0.0) {
      throw std::invalid_argument("Speed limit must be finite and nonnegative");
    }
    speed_limit_ = std::min(max_linear, settings_.max_linear);
  }

  void reset(const Velocity & initial = Velocity{})
  {
    if (!finite(initial)) {
      throw std::invalid_argument("Invalid reset velocity");
    }
    seed_ = initial;
    explicit_seed_ = true;
    velocity_ready_ = false;
  }

  void clear()
  {
    path_.clear();
    lengths_.clear();
    progress_ = 0.0;
    projected_ = false;
    reset();
  }

  Result compute(
    const Pose & pose, const Velocity & measured, double dt, bool advance_velocity = true)
  {
    if (!finite(pose) || !finite(measured) || !std::isfinite(dt) || dt <= 0.0) {
      throw std::invalid_argument("Pose, measured velocity and control period must be valid");
    }
    if (path_.empty()) {
      throw std::runtime_error("Cannot follow an empty path");
    }
    const Point position{pose.x, pose.y};
    const auto projection = project(position);
    if (projection.distance > settings_.lookahead_distance + epsilon_) {
      throw std::runtime_error("Robot is outside the lookahead corridor; stop and replan");
    }
    const auto selected = select(position, projection.s);
    Result result;
    result.target = selected.point;
    result.remaining_distance = lengths_.back() - projection.s;
    result.terminal = selected.terminal;

    double desired_x = settings_.position_gain * (result.target.x - pose.x);
    double desired_y = settings_.position_gain * (result.target.y - pose.y);
    double linear_limit = speed_limit_;
    const double goal_distance = distance(position, path_.back());
    result.target_yaw = followingYaw(projection.s, selected, goal_distance);
    if (result.terminal) {
      // Entering the goal region requests zero, then the slew limiter brakes.
      // StoppedGoalChecker waits for actual odometry to confirm stopped motion.
      const double stop_distance = std::max(0.0, goal_distance - settings_.xy_goal_tolerance);
      linear_limit = std::min(linear_limit,
        std::sqrt(2.0 * settings_.linear_acceleration * stop_distance));
    }
    bound(desired_x, desired_y, linear_limit);
    const double heading_error = wrap_angle(result.target_yaw - pose.yaw);
    double desired_w = std::clamp(settings_.heading_gain * heading_error,
        -settings_.max_angular, settings_.max_angular);
    if (result.terminal && goal_distance <= settings_.xy_goal_tolerance &&
      std::abs(heading_error) <= settings_.yaw_goal_tolerance)
    {
      desired_w = 0.0;
    }

    const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
    result.desired = {c * desired_x + s * desired_y, -s * desired_x + c * desired_y, desired_w};
    if (!advance_velocity) {
      // A chassis wait still updates actual-pose geometry, never a virtual ramp.
      result.command = result.desired;
      progress_ = projection.s;
      projected_ = true;
      return result;
    }
    if (!velocity_ready_) {
      const Velocity initial = explicit_seed_ ? seed_ : measured;
      previous_x_ = c * initial.vx - s * initial.vy;
      previous_y_ = s * initial.vx + c * initial.vy;
      previous_w_ = std::clamp(initial.wz, -settings_.max_angular, settings_.max_angular);
      bound(previous_x_, previous_y_, speed_limit_);
      velocity_ready_ = true;
      explicit_seed_ = false;
    }
    double change_x = desired_x - previous_x_;
    double change_y = desired_y - previous_y_;
    bound(change_x, change_y, settings_.linear_acceleration * dt);
    previous_x_ += change_x;
    previous_y_ += change_y;
    // An externally reduced speed ceiling takes priority over a smooth ramp.
    bound(previous_x_, previous_y_, speed_limit_);
    previous_w_ += std::clamp(desired_w - previous_w_,
        -settings_.angular_acceleration * dt, settings_.angular_acceleration * dt);
    previous_w_ = std::clamp(previous_w_, -settings_.max_angular, settings_.max_angular);
    result.command = {c * previous_x_ + s * previous_y_,
      -s * previous_x_ + c * previous_y_, previous_w_};
    progress_ = projection.s;
    projected_ = true;
    return result;
  }

private:
  struct Projection {double s; double distance; bool truncated{false};};
  struct Selection {Point point; bool terminal;};
  static constexpr double epsilon_ = 1e-9;

  static void bound(double & x, double & y, double limit)
  {
    const double norm = std::hypot(x, y);
    if (norm > limit && norm > 0.0) {
      x *= limit / norm;
      y *= limit / norm;
    }
  }

  Point interpolate(std::size_t segment, double t) const
  {
    const auto & a = path_[segment];
    const auto & b = path_[segment + 1];
    return {a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)};
  }

  double followingYaw(double progress, const Selection & selected, double goal_distance) const
  {
    if (settings_.heading_mode == HeadingMode::HoldInitial || path_.size() == 1) {
      return target_yaw_;
    }
    double tangent = target_yaw_;
    // Use the path projection, not the off-path robot position: lateral
    // correction remains holonomic without turning the heading toward that error.
    for (std::size_t i = 0; i + 1 < path_.size(); ++i) {
      if (lengths_[i + 1] <= progress + epsilon_) {continue;}
      const double length = lengths_[i + 1] - lengths_[i];
      const Point projected = interpolate(i,
        std::clamp((progress - lengths_[i]) / length, 0.0, 1.0));
      Point delta{selected.point.x - projected.x, selected.point.y - projected.y};
      if (std::hypot(delta.x, delta.y) <= epsilon_) {
        delta = {path_[i + 1].x - path_[i].x, path_[i + 1].y - path_[i].y};
      }
      tangent = std::atan2(delta.y, delta.x);
      break;
    }
    // A spatially nearby endpoint of a loop is not necessarily the forward tail.
    if (!selected.terminal || goal_distance > settings_.lookahead_distance) {
      return tangent;
    }
    if (goal_distance <= settings_.xy_goal_tolerance) {return target_yaw_;}
    const double blend = std::clamp(
      (settings_.lookahead_distance - goal_distance) /
      (settings_.lookahead_distance - settings_.xy_goal_tolerance), 0.0, 1.0);
    return wrap_angle(tangent + blend * wrap_angle(target_yaw_ - tangent));
  }

  Projection project(const Point & position) const
  {
    if (path_.size() == 1) {
      return {0.0, distance(position, path_.front())};
    }
    const double start = projected_ ? progress_ : 0.0;
    const double finish = std::min(lengths_.back(), start + settings_.projection_search_distance);
    Projection best{start, std::numeric_limits<double>::infinity()};
    for (std::size_t i = 0; i + 1 < path_.size(); ++i) {
      if (lengths_[i + 1] < start - epsilon_ || lengths_[i] > finish + epsilon_) {
        continue;
      }
      const double length = lengths_[i + 1] - lengths_[i];
      const double lower = std::max(0.0, (start - lengths_[i]) / length);
      const double upper = std::min(1.0, (finish - lengths_[i]) / length);
      if (lower > upper) {
        continue;
      }
      const Point delta{path_[i + 1].x - path_[i].x, path_[i + 1].y - path_[i].y};
      const double raw_t = ((position.x - path_[i].x) * delta.x +
        (position.y - path_[i].y) * delta.y) / (length * length);
      const double t = std::clamp(raw_t, lower, upper);
      const double candidate = distance(position, interpolate(i, t));
      // Track the first forward local minimum. Looking globally for the closest
      // point every cycle can jump to a later branch of a self-intersecting path.
      if (candidate > best.distance + epsilon_) {
        break;
      }
      if (candidate < best.distance - epsilon_) {
        const bool truncated = finish < lengths_.back() - epsilon_ &&
          lengths_[i] + upper * length >= finish - epsilon_ && raw_t > upper + epsilon_;
        best = {lengths_[i] + t * length, candidate, truncated};
      }
    }
    if (best.truncated) {
      throw std::runtime_error("Path progress exceeds the search window; stop and replan");
    }
    return best;
  }

  Selection select(const Point & position, double start) const
  {
    const double radius = settings_.lookahead_distance;
    for (std::size_t i = 0; i + 1 < path_.size(); ++i) {
      if (lengths_[i + 1] < start - epsilon_) {
        continue;
      }
      const Point delta{path_[i + 1].x - path_[i].x, path_[i + 1].y - path_[i].y};
      const Point offset{path_[i].x - position.x, path_[i].y - position.y};
      const double a = delta.x * delta.x + delta.y * delta.y;
      const double b = 2.0 * (offset.x * delta.x + offset.y * delta.y);
      const double c = offset.x * offset.x + offset.y * offset.y - radius * radius;
      double discriminant = b * b - 4.0 * a * c;
      if (discriminant < -epsilon_) {
        continue;
      }
      discriminant = std::sqrt(std::max(0.0, discriminant));
      const double length = lengths_[i + 1] - lengths_[i];
      for (const double t : {(-b - discriminant) / (2.0 * a),
          (-b + discriminant) / (2.0 * a)})
      {
        if (t >= -epsilon_ && t <= 1.0 + epsilon_ &&
          lengths_[i] + std::clamp(t, 0.0, 1.0) * length >= start - epsilon_)
        {
          const Point target = interpolate(i, std::clamp(t, 0.0, 1.0));
          return {target, i + 2 == path_.size() && t >= 1.0 - epsilon_};
        }
      }
    }
    // A terminal point near the robot is used only after searching every
    // remaining segment for an earlier circle crossing: never shortcut a loop.
    bool forward_tail = true;
    bool have_tail_start = false;
    Point tail_direction;
    for (std::size_t i = 0; i + 1 < path_.size(); ++i) {
      if (lengths_[i + 1] < start - epsilon_) {
        continue;
      }
      const double length = lengths_[i + 1] - lengths_[i];
      const Point begin = interpolate(i, std::clamp((start - lengths_[i]) / length, 0.0, 1.0));
      if (!have_tail_start) {
        tail_direction = {path_.back().x - begin.x, path_.back().y - begin.y};
        if (std::hypot(tail_direction.x, tail_direction.y) <= epsilon_ &&
          lengths_.back() - start > epsilon_)
        {
          forward_tail = false;
          break;
        }
        have_tail_start = true;
      }
      const Point delta{path_[i + 1].x - begin.x, path_[i + 1].y - begin.y};
      if (tail_direction.x * delta.x + tail_direction.y * delta.y < -epsilon_) {
        forward_tail = false;
        break;
      }
    }
    // With no circle crossing, use an endpoint only when the remaining tail
    // progresses toward it without folding backward along its chord. A loop
    // inside the circle has no valid 0.4 m reference: stop rather than shortcut.
    if (distance(position, path_.back()) <= radius + epsilon_ && forward_tail) {
      return {path_.back(), true};
    }
    throw std::runtime_error("No forward lookahead point; stop and replan");
  }

  Settings settings_;
  double speed_limit_;
  std::vector<Point> path_;
  std::vector<double> lengths_;
  double target_yaw_{0.0};
  double progress_{0.0};
  bool projected_{false};
  bool velocity_ready_{false};
  bool explicit_seed_{false};
  Velocity seed_;
  double previous_x_{0.0};
  double previous_y_{0.0};
  double previous_w_{0.0};
};

}  // namespace core
}  // namespace fairino_lookahead_controller

#endif  // FAIRINO_LOOKAHEAD_CONTROLLER__CORE_HPP_
