#ifndef FAIRINO_LOOKAHEAD_CONTROLLER__COORDINATION_HPP_
#define FAIRINO_LOOKAHEAD_CONTROLLER__COORDINATION_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "fairino_lookahead_controller/core.hpp"

namespace fairino_lookahead_controller
{
namespace coordination
{
constexpr const char * status_name = "fairino_slam/chassis_motion";
// Exact wire-state thresholds shared with chassis/motion_feedback.py.
constexpr double intent_epsilon = 1e-4;
constexpr double drive_epsilon = 1e-6;

struct Settings
{
  double feedback_timeout{.4};
  double linear_speed{.02};
  double angular_speed{.04};
  double wait_timeout{15.};
  double direction_tolerance{.20};
};

struct Diagnostic
{
  std::string name;
  std::string session;
  std::string message;
  unsigned level{0};
  double stamp{0.};
  std::vector<std::pair<std::string, std::string>> values;
};

struct Feedback
{
  std::string session;
  std::uint64_t sequence{0};
  double stamp{0.};
  bool ready{false};
  bool drive_wait{false};
  bool healthy{false};
  double hardware_age{0.};
  double command_peak{0.};
  double velocity_peak{0.};
  core::Velocity requested;
};

inline Feedback parse(const Diagnostic & diagnostic)
{
  if (diagnostic.name != status_name || diagnostic.session.size() != 32 ||
    !std::all_of(diagnostic.session.begin(), diagnostic.session.end(), [](char c) {
      return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
    }) || !std::isfinite(diagnostic.stamp) || diagnostic.stamp <= 0. ||
    (diagnostic.level != 0 && diagnostic.level != 2))
  {
    throw std::runtime_error("Invalid chassis motion diagnostic envelope");
  }
  std::map<std::string, std::string> values;
  for (const auto & value : diagnostic.values) {
    if (!values.emplace(value).second) {
      throw std::runtime_error("Duplicate chassis motion diagnostic field");
    }
  }
  if (values.size() != 10) {throw std::runtime_error("Unexpected chassis motion fields");}
  const auto field = [&](const std::string & key) -> const std::string & {
      const auto found = values.find(key);
      if (found == values.end()) {throw std::runtime_error("Missing chassis motion field: " + key);}
      return found->second;
    };
  if (field("schema_version") != "1") {throw std::runtime_error("Unsupported chassis motion schema");}
  const auto boolean = [&](const std::string & key) {
      const auto & text = field(key);
      if (text != "0" && text != "1") {throw std::runtime_error("Invalid chassis motion boolean");}
      return text == "1";
    };
  const auto number = [&](const std::string & key) {
      const auto & text = field(key);
      std::size_t consumed = 0;
      const double value = std::stod(text, &consumed);
      if (text.empty() || consumed != text.size() || !std::isfinite(value)) {
        throw std::runtime_error("Invalid chassis motion number: " + key);
      }
      return value;
    };
  const auto & sequence = field("sequence");
  if (sequence.empty() || sequence.front() == '0' || !std::all_of(sequence.begin(), sequence.end(), [](char c) {
      return c >= '0' && c <= '9';
    }))
  {
    throw std::runtime_error("Invalid chassis motion sequence");
  }
  Feedback result;
  result.sequence = std::stoull(sequence);
  if (result.sequence == 0) {throw std::runtime_error("Chassis motion sequence must be positive");}
  result.session = diagnostic.session;
  result.stamp = diagnostic.stamp;
  result.healthy = diagnostic.level == 0;
  result.ready = boolean("ready");
  result.drive_wait = boolean("drive_wait");
  result.hardware_age = number("hardware_age_sec");
  result.command_peak = number("drive_command_peak");
  result.velocity_peak = number("drive_velocity_peak");
  result.requested = {number("requested_vx"), number("requested_vy"), number("requested_wz")};
  if (result.hardware_age < 0. || result.command_peak < 0. || result.velocity_peak < 0.) {
    throw std::runtime_error("Negative chassis motion age or wheel speed magnitude");
  }
  const bool intent = std::max({std::abs(result.requested.vx), std::abs(result.requested.vy),
    std::abs(result.requested.wz)}) > intent_epsilon;
  const bool expected_wait = result.ready && intent && result.command_peak <= drive_epsilon;
  const std::string expected_message = !result.ready ? "not_ready" : !intent ? "idle" :
    result.drive_wait ? "drive_wait" : "drive_active";
  if (result.drive_wait != expected_wait || result.healthy != result.ready ||
    diagnostic.message != expected_message)
  {
    throw std::runtime_error("Inconsistent chassis motion diagnostic state");
  }
  return result;
}

// The subscriber and control tick call this only under the controller mutex.
// ROS time validates source timestamps; steady time bounds receipt and wait ages.
class Guard
{
public:
  struct State {bool waiting{false}; bool reseed{false};};

  explicit Guard(const Settings & settings = Settings{}) : settings_(settings)
  {
    for (const double value : {settings.feedback_timeout, settings.linear_speed,
        settings.angular_speed, settings.wait_timeout, settings.direction_tolerance})
    {
      if (!std::isfinite(value) || value <= 0.) {
        throw std::invalid_argument("Coordination settings must be finite and positive");
      }
    }
    if (settings.direction_tolerance >= 1.5707963267948966 ||
      settings.linear_speed <= intent_epsilon || settings.angular_speed <= intent_epsilon ||
      settings.wait_timeout > 60. || settings.wait_timeout <= settings.feedback_timeout)
    {
      throw std::invalid_argument("Invalid coordination angle or wait timeout");
    }
  }

  void clear()
  {
    *this = Guard(settings_);
  }

  void invalidate(const std::string & reason) {invalid_ = reason;}

  void receive(const Diagnostic & diagnostic, double ros_now, double steady_now)
  {
    try {
      const auto feedback = parse(diagnostic);
      if (!session_.empty() && feedback.session != session_) {
        session_changed_ = true;
        throw std::runtime_error("Chassis worker session changed; restart navigation");
      }
      if (session_changed_) {throw std::runtime_error("Chassis worker session changed");}
      if (received_ && feedback.sequence <= feedback_.sequence) {
        throw std::runtime_error("Chassis motion sequence repeated or moved backward");
      }
      checkTimes(feedback, ros_now, steady_now, steady_now);
      session_ = feedback.session;
      feedback_ = feedback;
      receipt_ = steady_now;
      received_ = true;
      if (feedback.healthy && feedback.ready && !feedback.drive_wait) {
        wait_started_ = -1.;  // A real observation ends the episode, even between control ticks.
        frozen_ = false;
        suppress_feedback_ = false;
      }
      invalid_.clear();
    } catch (const std::exception & error) {
      invalidate(error.what());
    }
  }

  void validate(double ros_now, double steady_now) const
  {
    if (!invalid_.empty()) {throw std::runtime_error(invalid_);}
    if (!received_) {throw std::runtime_error("Chassis motion feedback is unavailable");}
    checkTimes(feedback_, ros_now, steady_now, receipt_);
    if (!feedback_.healthy || !feedback_.ready) {
      throw std::runtime_error("Chassis motion feedback is not ready or healthy");
    }
    if (feedback_.drive_wait && wait_started_ >= 0. &&
      steady_now - wait_started_ >= settings_.wait_timeout)
    {
      throw std::runtime_error("Chassis drive wait exceeded coordination timeout");
    }
  }

  State prepare(double ros_now, double steady_now)
  {
    validate(ros_now, steady_now);
    State result{feedback_.drive_wait, was_waiting_ && !feedback_.drive_wait};
    if (result.waiting) {
      if (wait_started_ < 0.) {wait_started_ = steady_now;}
    } else {
      wait_started_ = -1.;
      frozen_ = false;
      suppress_feedback_ = false;
    }
    was_waiting_ = result.waiting;
    return result;
  }

  // A protective stop clears motion intent, but never buys another wait budget.
  void stop()
  {
    frozen_ = false;
    suppress_feedback_ = true;
  }

  core::Velocity apply(const core::Velocity & desired, const core::Velocity & normal)
  {
    if (!core::finite(desired) || !core::finite(normal)) {
      throw std::invalid_argument("Invalid coordination command");
    }
    if (!was_waiting_) {return normal;}
    if (magnitude(desired) <= epsilon_) {
      stop();
      return {};  // Reaching the goal must never revive a frozen nonzero intent.
    }
    if (!frozen_) {
      reference_ = desired;
      intent_ = !suppress_feedback_ && magnitude(feedback_.requested) > epsilon_ &&
        directionChange(feedback_.requested, desired) <= settings_.direction_tolerance ?
        feedback_.requested : desired;
      intent_ = limit(intent_, desired);
      frozen_ = true;
    } else if (directionChange(reference_, desired) > settings_.direction_tolerance ||
      (std::hypot(desired.vx, desired.vy) <= epsilon_ &&
      std::hypot(intent_.vx, intent_.vy) > epsilon_) ||
      (std::abs(desired.wz) <= epsilon_ && std::abs(intent_.wz) > epsilon_))
    {
      reference_ = desired;
      intent_ = limit(desired, desired);
      suppress_feedback_ = true;  // Old worker feedback cannot restore the old route.
    }
    auto output = limit(intent_, desired);
    // Near-zero yaw correction must not scale a useful XY request into HALT
    // (and the symmetric translation/rotation case). Keep exact goal zero above.
    const auto moving = [](const core::Velocity & value) {
        return std::max({std::abs(value.vx), std::abs(value.vy), std::abs(value.wz)}) >
               intent_epsilon;
      };
    if (!moving(output) && moving(desired)) {
      reference_ = desired;
      intent_ = limit(desired, desired);
      suppress_feedback_ = true;
      output = intent_;
    }
    if (!moving(output) && moving(desired)) {
      throw std::runtime_error("Coordination speed limits suppress the motion request");
    }
    return output;
  }

private:
  void checkTimes(
    const Feedback & feedback, double ros_now, double steady_now, double receipt) const
  {
    const double source_age = ros_now - feedback.stamp;
    const double elapsed = steady_now - receipt;
    if (!std::isfinite(ros_now) || ros_now <= 0. || !std::isfinite(steady_now) ||
      !std::isfinite(receipt) || steady_now < 0. || elapsed < 0. ||
      source_age < -.05 || source_age > settings_.feedback_timeout ||
      elapsed > settings_.feedback_timeout ||
      feedback.hardware_age + std::max({0., source_age, elapsed}) > settings_.feedback_timeout)
    {
      throw std::runtime_error("Chassis motion feedback is stale or has invalid time");
    }
  }

  double magnitude(const core::Velocity & value) const
  {
    return std::hypot(std::hypot(value.vx, value.vy) / settings_.linear_speed,
      value.wz / settings_.angular_speed);
  }

  double directionChange(const core::Velocity & a, const core::Velocity & b) const
  {
    const double norm = magnitude(a) * magnitude(b);
    if (norm <= epsilon_) {return 3.14159265358979323846;}
    const double dot = (a.vx * b.vx + a.vy * b.vy) /
      (settings_.linear_speed * settings_.linear_speed) +
      a.wz * b.wz / (settings_.angular_speed * settings_.angular_speed);
    return std::acos(std::clamp(dot / norm, -1., 1.));
  }

  core::Velocity limit(const core::Velocity & value, const core::Velocity & desired) const
  {
    double scale = 1.;
    const double speed = std::hypot(value.vx, value.vy);
    if (speed > epsilon_) {
      scale = std::min(scale,
        std::min(settings_.linear_speed, std::hypot(desired.vx, desired.vy)) / speed);
    }
    if (std::abs(value.wz) > epsilon_) {
      scale = std::min(scale,
        std::min(settings_.angular_speed, std::abs(desired.wz)) / std::abs(value.wz));
    }
    return {value.vx * scale, value.vy * scale, value.wz * scale};
  }

  static constexpr double epsilon_ = 1e-12;
  Settings settings_;
  Feedback feedback_;
  std::string session_;
  std::string invalid_;
  double receipt_{0.};
  double wait_started_{-1.};
  bool received_{false};
  bool session_changed_{false};
  bool was_waiting_{false};
  bool frozen_{false};
  bool suppress_feedback_{false};
  core::Velocity reference_;
  core::Velocity intent_;
};
}  // namespace coordination
}  // namespace fairino_lookahead_controller

#endif  // FAIRINO_LOOKAHEAD_CONTROLLER__COORDINATION_HPP_
