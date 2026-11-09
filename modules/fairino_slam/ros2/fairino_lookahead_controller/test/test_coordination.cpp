#include "fairino_lookahead_controller/coordination.hpp"

#include <functional>
#include <iostream>
#include <limits>
#include <string>

using namespace fairino_lookahead_controller;
namespace
{
void require(bool value, const std::string & message)
{
  if (!value) {throw std::runtime_error(message);}
}
void near(double value, double expected, const std::string & message, double tolerance = 1e-9)
{
  require(std::abs(value - expected) <= tolerance, message);
}
void rejects(const std::function<void()> & callback, const std::string & message)
{
  bool rejected = false;
  try {callback();} catch (const std::exception &) {rejected = true;}
  require(rejected, message);
}
coordination::Diagnostic diagnostic(
  std::uint64_t sequence, double stamp, bool waiting = false,
  core::Velocity requested = {.08, .06, .12})
{
  coordination::Diagnostic result;
  result.name = coordination::status_name;
  result.session = "0123456789abcdef0123456789abcdef";
  result.stamp = stamp;
  result.message = waiting ? "drive_wait" :
    std::max({std::abs(requested.vx), std::abs(requested.vy), std::abs(requested.wz)}) >
    coordination::intent_epsilon ? "drive_active" : "idle";
  result.values = {
    {"schema_version", "1"}, {"sequence", std::to_string(sequence)},
    {"ready", "1"}, {"drive_wait", waiting ? "1" : "0"}, {"hardware_age_sec", ".01"},
    {"drive_command_peak", waiting ? "0" : "1"}, {"drive_velocity_peak", "0"},
    {"requested_vx", std::to_string(requested.vx)},
    {"requested_vy", std::to_string(requested.vy)},
    {"requested_wz", std::to_string(requested.wz)}};
  return result;
}
void field(coordination::Diagnostic & message, const std::string & key, const std::string & value)
{
  for (auto & pair : message.values) {
    if (pair.first == key) {pair.second = value; return;}
  }
  throw std::runtime_error("Test field does not exist");
}
void accept(coordination::Guard & guard, std::uint64_t sequence, double time, bool wait,
  core::Velocity requested = {.08, .06, .12})
{
  guard.receive(diagnostic(sequence, 100. + time, wait, requested), 100. + time, time);
}
void ready_and_strict_protocol()
{
  coordination::Guard guard;
  rejects([&] {guard.prepare(100., 0.);}, "missing feedback must stop");
  accept(guard, 1, 0., false, {});
  const auto state = guard.prepare(100., 0.);
  require(!state.waiting && !state.reseed, "ready idle feedback permits first request");
  near(guard.apply({.1, 0., 0.}, {.02, 0., 0.}).vx, .02, "idle does not deadlock startup");
  for (const auto & invalid : std::vector<std::pair<std::string, std::string>>{
      {"schema_version", "2"}, {"sequence", "01"}, {"sequence", "0"}, {"sequence", "-1"}, {"sequence", "1.0"},
      {"sequence", "18446744073709551616"}, {"ready", "true"}, {"drive_wait", "2"},
      {"hardware_age_sec", "-.1"}, {"drive_command_peak", "-1"},
      {"drive_velocity_peak", "nan"}, {"requested_vx", "inf"}, {"requested_wz", "1x"}})
  {
    auto message = diagnostic(1, 100.);
    field(message, invalid.first, invalid.second);
    rejects([&] {coordination::parse(message);}, "malformed field must reject: " + invalid.first);
  }
  auto duplicate = diagnostic(1, 100.);
  duplicate.values.emplace_back("ready", "1");
  rejects([&] {coordination::parse(duplicate);}, "duplicate field must reject");
  auto missing = diagnostic(1, 100.); missing.values.pop_back();
  rejects([&] {coordination::parse(missing);}, "missing field must reject");
  auto session = diagnostic(1, 100.); session.session = std::string(32, 'g');
  rejects([&] {coordination::parse(session);}, "non-hex session must reject");
  auto unknown = diagnostic(1, 100.); unknown.values.emplace_back("unknown", "1");
  rejects([&] {coordination::parse(unknown);}, "unknown fields must reject");
  auto uppercase = diagnostic(1, 100.); uppercase.session = std::string(32, 'A');
  rejects([&] {coordination::parse(uppercase);}, "session must use lower-case hex");
  auto inconsistent = diagnostic(1, 100., true); inconsistent.message = "idle";
  rejects([&] {coordination::parse(inconsistent);}, "message must match numeric state");
  field(inconsistent, "drive_command_peak", "1");
  inconsistent.message = "drive_wait";
  rejects([&] {coordination::parse(inconsistent);}, "moving wheel command cannot be drive_wait");
  auto zero_stamp = diagnostic(1, 0.);
  rejects([&] {coordination::parse(zero_stamp);}, "zero ROS stamp must reject");
  auto level = diagnostic(1, 100.); level.level = 1;
  rejects([&] {coordination::parse(level);}, "unknown status level must reject");
}
void ordered_feedback_and_session_lock()
{
  coordination::Guard guard;
  accept(guard, 2, 0., false);
  accept(guard, 2, .1, false);
  rejects([&] {guard.prepare(100.1, .1);}, "duplicate sequence invalidates rather than refreshing");
  accept(guard, 3, .2, false);
  guard.prepare(100.2, .2);
  accept(guard, 1, .3, false);
  rejects([&] {guard.prepare(100.3, .3);}, "out of order feedback must reject");
  accept(guard, 4, .4, false);
  auto replacement = diagnostic(5, 100.5); replacement.session = std::string(32, 'a');
  guard.receive(replacement, 100.5, .5);
  rejects([&] {guard.prepare(100.5, .5);}, "worker restart must reject");
  accept(guard, 6, .6, false);
  rejects([&] {guard.prepare(100.6, .6);}, "old session cannot undo a detected restart");
  guard.clear();
  guard.receive(replacement, 100.5, .5);
  guard.prepare(100.5, .5);
}
void source_receipt_and_hardware_age()
{
  const auto check = [](double stamp, double hardware_age, double receive_ros,
      double output_ros, double output_steady, bool valid) {
      coordination::Guard guard;
      auto message = diagnostic(1, stamp);
      field(message, "hardware_age_sec", std::to_string(hardware_age));
      guard.receive(message, receive_ros, 0.);
      if (valid) {guard.prepare(output_ros, output_steady);}
      else {rejects([&] {guard.prepare(output_ros, output_steady);}, "stale time must reject");}
    };
  check(100., .01, 100., 100.38, .38, true);
  check(100., .01, 100., 100.401, .1, false);
  check(100., .01, 100., 100.1, .401, false);
  check(99.7, .2, 100., 100., 0., false);  // queued packet + preexisting sample age
  check(100., .1, 100., 100.35, .35, false);  // slow final collision/TF work
  check(100.049, .01, 100., 100., 0., true);
  check(100.051, .01, 100., 100., 0., false);
  check(.049, .01, 0., 0., 0., false);
  check(100., .01, 100., 100., -.001, false);
  coordination::Guard guard;
  auto fault = diagnostic(1, 100.); fault.level = 2;
  guard.receive(fault, 100., 0.);
  rejects([&] {guard.prepare(100., 0.);}, "hardware fault overrides waiting");
  auto not_ready = diagnostic(2, 100.); field(not_ready, "ready", "0");
  guard.receive(not_ready, 100., 0.);
  rejects([&] {guard.prepare(100., 0.);}, "disabled/non-ready chassis must stop");
}
void complete_twist_ratio_and_low_speed()
{
  coordination::Guard guard;
  accept(guard, 1, 0., true);
  require(guard.prepare(100., 0.).waiting, "wait state");
  const auto command = guard.apply({.1, .075, .15}, {.1, .075, .15});
  near(command.vx, .016, "linear vector cap");
  near(command.vy, .012, "lateral ratio");
  near(command.wz, .024, "yaw ratio preserved by common scale");
  coordination::Guard low;
  accept(low, 1, 0., true, {.001, 0., .002});
  low.prepare(100., 0.);
  const auto retained = low.apply({.1, 0., .2}, {.1, 0., .2});
  near(retained.vx, .001, "never raise an already small worker intent");
  near(retained.wz, .002, "small yaw retained");
  const auto slower = guard.apply({.004, .003, .006}, {});
  near(std::hypot(slower.vx, slower.vy), .005, "goal/external speed decrease overrides freeze");
}
void near_zero_component_and_pure_rotation()
{
  coordination::Guard guard;
  accept(guard, 1, 0., true, {.02, 0., .01});
  guard.prepare(100., 0.);
  guard.apply({.1, 0., .01}, {});
  const auto correction = guard.apply({.1, 0., .00001}, {});
  require(correction.vx > .0001, "near-zero yaw must not push translation into HALT");
  require(std::abs(correction.wz) <= .00001, "near-zero yaw is not amplified");
  coordination::Guard rotation;
  accept(rotation, 1, 0., true, {0., 0., -.2});
  rotation.prepare(100., 0.);
  const auto turning = rotation.apply({0., 0., -.2}, {});
  near(turning.vx, 0., "pure rotation never creates translation");
  near(turning.vy, 0., "pure rotation never creates lateral motion");
  near(turning.wz, -.04, "pure rotation keeps sign and angular cap");
  const auto slow = rotation.apply({0., 0., -.003}, {});
  near(slow.wz, -.003, "terminal yaw deceleration is never amplified");
  const auto zero = rotation.apply({}, {});
  near(zero.wz, 0., "zero target cancels frozen yaw");
  near(rotation.apply({0., 0., .01}, {}).wz, .01, "new target does not restore old spin");
  coordination::Settings tiny_caps; tiny_caps.linear_speed = .00011;
  coordination::Guard tiny(tiny_caps);
  accept(tiny, 1, 0., true, {.1, .1, 0.});
  tiny.prepare(100., 0.);
  rejects([&] {tiny.apply({.1, .1, 0.}, {});},
    "diagonal cap below per-component request deadband must stop rather than loop in HALT");
}
void route_changes_zero_goals_and_wait_budget()
{
  coordination::Guard guard;
  accept(guard, 1, 0., true, {.1, 0., 0.});
  guard.prepare(100., 0.);
  near(guard.apply({.1, 0., 0.}, {}).vx, .02, "initial locked request");
  const auto slight = guard.apply({.1 * std::cos(.1), .1 * std::sin(.1), 0.}, {});
  near(slight.vy, 0., "small path jitter does not chase wheel direction");
  const auto turn = guard.apply({0., .1, 0.}, {});
  near(turn.vx, 0., "real replan discards obsolete direction");
  near(turn.vy, .02, "replanned intent is small and nonzero");
  accept(guard, 2, 1., true, {.1, 0., 0.});  // old in-flight worker request
  guard.prepare(101., 1.);
  near(guard.apply({0., .1, 0.}, {}).vy, .02, "old feedback cannot restore old route");
  near(guard.apply({}, {}).vy, 0., "goal zero overrides frozen movement");
  near(guard.apply({-.1, 0., 0.}, {}).vx, -.02, "new goal never revives old direction");
  guard.stop();
  accept(guard, 3, 14.9, true);
  guard.prepare(114.9, 14.9);
  guard.apply({0., .1, 0.}, {});
  accept(guard, 4, 15., true);
  rejects([&] {guard.prepare(115., 15.);}, "replan/zero/protective stop must not extend wait");
  accept(guard, 5, 15.1, false);
  require(guard.prepare(115.1, 15.1).reseed, "release requests measured reseed");
  accept(guard, 6, 15.2, true);
  guard.prepare(115.2, 15.2);  // actual release establishes a new wait episode
  accept(guard, 7, 30.1, false);
  accept(guard, 8, 30.2, true);
  guard.prepare(130.2, 30.2);  // a release observed between control ticks also ends the episode
}
void follower_wait_does_not_integrate_velocity()
{
  core::Settings settings; settings.max_linear = .1; settings.max_angular = .2;
  core::Follower follower(settings);
  follower.setPath({{0., 0.}, {2., 0.}}, 0.);
  for (int i = 0; i < 100; ++i) {
    const auto result = follower.compute({}, {}, .1, false);
    near(result.target.x, .4, "stationary wait cannot advance carrot");
    near(result.remaining_distance, 2., "wait cannot integrate path progress");
  }
  const auto first = follower.compute({}, {}, .1);
  near(first.command.vx, .02, "paused ramp did not accumulate toward full speed");
  follower.reset({.01, 0., 0.});
  near(follower.compute({}, {.01, 0., 0.}, .1).command.vx, .03, "release ramps from measurement");
  follower.setPath({{0., 0.}}, 0.);
  const auto stopped = follower.compute({}, {}, .1, false);
  near(stopped.desired.vx, 0., "terminal wait has an exact zero target");
  near(stopped.desired.wz, 0., "terminal heading has an exact zero target");
}
void closed_loop_align_then_drive_to_goal()
{
  core::Settings settings; settings.max_linear = .1; settings.max_angular = .2;
  core::Follower follower(settings);
  const std::vector<core::Point> path{{0., 0.}, {1., 0.}};
  follower.setPath(path, 0.);
  coordination::Guard guard;
  core::Pose pose;
  core::Velocity measured, sent;
  double steering = 1.5707963267948966;
  bool aligning = false, released = false;
  int wait_ticks = 0;
  for (int i = 0; i < 400; ++i) {
    const double time = .1 * i;
    if (i % 10 == 0) {follower.setPath(path, 0.);}  // normal periodic replanning
    accept(guard, static_cast<std::uint64_t>(i + 1), time, aligning, sent);
    const auto state = guard.prepare(100. + time, time);
    if (state.reseed) {follower.reset(measured); released = true;}
    const auto result = follower.compute(pose, measured, .1, !state.waiting);
    sent = guard.apply(result.desired, result.command);
    if (state.waiting) {
      ++wait_ticks;
      require(std::hypot(sent.vx, sent.vy) > 0., "nonzero intent must keep alignment requested");
      require(std::hypot(sent.vx, sent.vy) <= .020000001, "wait intent cannot build to cruise");
      near(pose.x, 0., "actual pose remains stopped during alignment");
    }
    if (steering > .05 && std::hypot(sent.vx, sent.vy) > 0.) {
      steering = std::max(0., steering - .35 * .1);
      measured = {};
      aligning = steering > .05;
    } else {
      if (released && measured.vx == 0. && sent.vx > 0.) {
        require(sent.vx <= .020000001, "alignment release must not jump to old cruise speed");
      }
      measured = sent;
      pose.x += measured.vx * .1;
      pose.y += measured.vy * .1;
      aligning = false;
    }
    if (pose.x >= .95 && std::hypot(measured.vx, measured.vy) < .005) {
      require(wait_ticks > 35 && released, "plant must actually align before driving");
      require(pose.x <= 1.05, "goal stop remains bounded");
      return;
    }
  }
  throw std::runtime_error("closed loop failed to reach and stop at the goal");
}
void settings_reject_invalid_values()
{
  for (int member = 0; member < 5; ++member) {
    coordination::Settings settings;
    double * fields[] = {&settings.feedback_timeout, &settings.linear_speed, &settings.angular_speed,
      &settings.wait_timeout, &settings.direction_tolerance};
    *fields[member] = 0.;
    rejects([&] {coordination::Guard ignored(settings);}, "zero setting must reject");
    *fields[member] = std::numeric_limits<double>::quiet_NaN();
    rejects([&] {coordination::Guard ignored(settings);}, "NaN setting must reject");
  }
  coordination::Settings settings; settings.direction_tolerance = 1.6;
  rejects([&] {coordination::Guard ignored(settings);}, "direction tolerance must be below pi/2");
  settings = {}; settings.linear_speed = coordination::intent_epsilon;
  rejects([&] {coordination::Guard ignored(settings);}, "wait speed must exceed motion deadband");
  settings = {}; settings.angular_speed = coordination::intent_epsilon;
  rejects([&] {coordination::Guard ignored(settings);}, "angular wait must exceed motion deadband");
  settings = {}; settings.wait_timeout = 60.01;
  rejects([&] {coordination::Guard ignored(settings);}, "wait budget must remain bounded");
  settings = {}; settings.wait_timeout = .3;
  rejects([&] {coordination::Guard ignored(settings);}, "wait timeout must exceed feedback timeout");
}
}
int main()
{
  const std::vector<std::pair<std::string, std::function<void()>>> tests{
    {"ready_and_strict_protocol", ready_and_strict_protocol},
    {"ordered_feedback_and_session_lock", ordered_feedback_and_session_lock},
    {"source_receipt_and_hardware_age", source_receipt_and_hardware_age},
    {"complete_twist_ratio_and_low_speed", complete_twist_ratio_and_low_speed},
    {"near_zero_component_and_pure_rotation", near_zero_component_and_pure_rotation},
    {"route_changes_zero_goals_and_wait_budget", route_changes_zero_goals_and_wait_budget},
    {"follower_wait_does_not_integrate_velocity", follower_wait_does_not_integrate_velocity},
    {"closed_loop_align_then_drive_to_goal", closed_loop_align_then_drive_to_goal},
    {"settings_reject_invalid_values", settings_reject_invalid_values}};
  for (const auto & test : tests) {
    try {test.second(); std::cout << "PASS " << test.first << '\n';}
    catch (const std::exception & error) {
      std::cerr << "FAIL " << test.first << ": " << error.what() << '\n'; return 1;
    }
  }
  std::cout << tests.size() << " C++ coordination test groups passed\n";
}
