#include "fairino_lookahead_controller/core.hpp"

#include <functional>
#include <iostream>
#include <limits>
#include <string>

using fairino_lookahead_controller::core::Follower;
using fairino_lookahead_controller::core::HeadingMode;
using fairino_lookahead_controller::core::Point;
using fairino_lookahead_controller::core::Pose;
using fairino_lookahead_controller::core::Result;
using fairino_lookahead_controller::core::Settings;
using fairino_lookahead_controller::core::Velocity;
using fairino_lookahead_controller::core::distance;
using fairino_lookahead_controller::core::wrap_angle;

namespace
{
constexpr double pi = 3.14159265358979323846;

void require(bool value, const std::string & message)
{
  if (!value) {throw std::runtime_error(message);}
}

void near(double actual, double expected, double tolerance, const std::string & message)
{
  if (std::abs(actual - expected) > tolerance) {
    throw std::runtime_error(message + ": " + std::to_string(actual) +
            " expected " + std::to_string(expected));
  }
}

void rejects(const std::function<void()> & callback, const std::string & message)
{
  bool rejected = false;
  try {callback();} catch (const std::exception &) {rejected = true;}
  require(rejected, message);
}

void straight_circle_updates()
{
  Follower follower;
  follower.setPath({{0., 0.}, {2., 0.}}, 0.);
  auto result = follower.compute({0., 0., 0.}, {}, .05);
  near(result.target.x, .4, 1e-12, "initial carrot");
  result = follower.compute({.1, 0., 0.}, {}, .05);
  near(result.target.x, .5, 1e-12, "rolling carrot");
  near(distance({.1, 0.}, result.target), .4, 1e-12, "Euclidean lookahead");
  near(result.remaining_distance, 1.9, 1e-12, "monotonic path progress");
  require(!result.terminal, "distant goal must not be terminal");
}

void curved_path_is_not_arc_length()
{
  Follower follower;
  follower.setPath({{0., 0.}, {.2, 0.}, {.2, 1.}}, .7);
  const auto result = follower.compute({0., 0., .7}, {}, .05);
  near(result.target.x, .2, 1e-12, "corner segment interpolation");
  near(result.target.y, std::sqrt(.12), 1e-12, "circle-line root");
  near(distance({}, result.target), .4, 1e-12, "circle distance on curve");
  require(.2 + result.target.y > .4, "arc length must differ from chord");
  near(result.target_yaw, .7, 1e-12, "heading independent of path tangent");
}

void endpoint_does_not_shortcut_loop()
{
  Follower follower;
  follower.setPath({{0., 0.}, {1., 0.}, {1., 1.}, {0., 1.}, {0., .1}}, 0.);
  const auto result = follower.compute({}, {}, .05);
  near(result.target.x, .4, 1e-12, "first forward crossing before nearby end");
  near(result.target.y, 0., 1e-12, "do not drive to loop endpoint");
  require(!result.terminal, "nearby endpoint is not necessarily remaining-path endpoint");
  // The robot need not sit exactly on the first vertex. A global closest-point
  // search would choose the nearby endpoint and bypass the entire loop.
  follower.clear();
  follower.setPath({{0., 0.}, {1., 0.}, {1., 1.}, {0., 1.}, {0., .1}}, 0.);
  const auto displaced = follower.compute({0., .08, 0.}, {}, .05);
  require(displaced.target.x > .3 && !displaced.terminal,
    "first projection shortcut to spatially nearby endpoint");
  follower.clear();
  follower.setPath({{0., 0.}, {.2, 0.}, {.2, .2}, {0., .2}, {0., .02}}, 0.);
  rejects([&] {follower.compute({}, {}, .05);},
    "a folded long path entirely inside the circle must not shortcut to its endpoint");
}

void duplicate_and_single_point_paths()
{
  Follower follower;
  follower.setPath({{0., 0.}, {0., 0.}, {1., 0.}, {1., 0.}}, 0.);
  near(follower.compute({}, {}, .05).target.x, .4, 1e-12, "duplicate vertices");
  follower.setPath({{.2, .1}, {.2, .1}}, 0.);
  const auto result = follower.compute({}, {}, .05);
  near(result.target.x, .2, 1e-12, "single-point terminal x");
  near(result.target.y, .1, 1e-12, "single-point terminal y");
  require(result.terminal, "short path must use endpoint");
}

void invalid_inputs_and_outside_corridor()
{
  Follower follower;
  rejects([&] {follower.compute({}, {}, .05);}, "empty path accepted");
  rejects([&] {follower.setPath({}, 0.);}, "empty setPath accepted");
  rejects([&] {follower.setPath({{0., std::numeric_limits<double>::quiet_NaN()}}, 0.);},
    "nonfinite path accepted");
  follower.setPath({{0., 0.}, {2., 0.}}, 0.);
  rejects([&] {follower.compute({0., .41, 0.}, {}, .05);}, "outside corridor accepted");
  rejects([&] {follower.compute({1.4, 0., 0.}, {}, .05);},
    "truncated initial projection selected a backward circle crossing");
  rejects([&] {follower.compute({}, {}, 0.);}, "zero dt accepted");
  rejects([&] {follower.compute({}, {std::numeric_limits<double>::infinity(), 0., 0.}, .05);},
    "nonfinite measured velocity accepted");
  Settings settings;
  settings.linear_acceleration = -1.;
  rejects([&] {Follower invalid(settings);}, "invalid settings accepted");
}

void sideways_and_heading_wrap()
{
  Follower follower;
  follower.setPath({{0., 0.}, {0., 2.}}, 0.);
  const auto sideways = follower.compute({}, {}, .05);
  near(sideways.command.vx, 0., 1e-12, "pure lateral motion x");
  require(sideways.command.vy > 0., "holonomic lateral velocity missing");
  near(sideways.command.wz, 0., 1e-12, "must not turn to path tangent");
  follower.clear();
  follower.setPath({{0., 0.}, {1., 0.}}, -pi + .02);
  const auto wrapped = follower.compute({0., 0., pi - .02}, {}, .05);
  require(wrapped.command.wz > 0., "yaw must take short route across pi");
  near(wrapped.target_yaw, -pi + .02, 1e-12, "locked heading unchanged");
}

void measured_seed_and_acceleration_limits()
{
  Follower follower;
  follower.setPath({{0., 0.}, {1., 1.}}, 1.);
  Velocity previous{.02, 0., .03};
  for (int i = 0; i < 30; ++i) {
    const auto result = follower.compute({}, previous, .05);
    const auto command = result.command;
    require(std::hypot(command.vx, command.vy) <= .06 + 1e-12, "combined XY limit");
    require(std::abs(command.wz) <= .10 + 1e-12, "angular limit");
    require(std::hypot(command.vx - previous.vx, command.vy - previous.vy) <= .2 * .05 + 1e-12,
      "vector acceleration limit");
    require(std::abs(command.wz - previous.wz) <= .4 * .05 + 1e-12,
      "angular acceleration limit");
    previous = command;
  }
}

void replanning_preserves_velocity()
{
  Follower follower;
  follower.setPath({{0., 0.}, {2., 0.}}, .3);
  Velocity previous;
  for (int i = 0; i < 10; ++i) {
    previous = follower.compute({0., 0., .3}, previous, .05).command;
  }
  const double previous_speed = std::hypot(previous.vx, previous.vy);
  follower.setPath({{0., 0.}, {.8, .1}, {2., 0.}}, .3);
  const auto result = follower.compute({0., 0., .3}, previous, .05);
  require(std::hypot(result.command.vx, result.command.vy) >= previous_speed - .01 - 1e-12,
    "replanning reset motion");
  require(std::hypot(result.command.vx - previous.vx, result.command.vy - previous.vy) <= .01 + 1e-12,
    "replanning velocity discontinuity");
  near(result.target_yaw, .3, 1e-12, "replanning recaptured heading");
}

void repeated_path_preserves_crossing_progress()
{
  const std::vector<Point> path{{-1., -1.}, {1., 1.}, {-1., 1.}, {1., -1.}};
  Follower follower;
  follower.setPath(path, 0.);
  for (double coordinate = -.8; coordinate < 0.; coordinate += .05) {
    follower.compute({coordinate, coordinate, 0.}, {}, .05);
  }
  const auto first = follower.compute({.02, .02, 0.}, {}, .05);
  require(first.target.x > .02 && first.target.y > .02, "crossing changed path branch");
  follower.setPath(path, 0.);
  const auto second = follower.compute({.03, .03, 0.}, {}, .05);
  require(second.target.x > .03 && second.target.y > .03, "same plan reset crossing progress");
  require(second.remaining_distance <= first.remaining_distance, "progress rewound");
}

void speed_ceiling_and_emergency_reset()
{
  Follower follower;
  follower.setPath({{0., 0.}, {2., 0.}}, 0.);
  for (int i = 0; i < 10; ++i) {follower.compute({}, {}, .05);}
  follower.setSpeedLimit(.015);
  auto result = follower.compute({}, {}, .05);
  require(std::hypot(result.command.vx, result.command.vy) <= .015 + 1e-12,
    "external ceiling must take effect immediately");
  follower.setSpeedLimit(0.);
  result = follower.compute({}, {}, .05);
  near(result.command.vx, 0., 1e-12, "zero limit");
  follower.setSpeedLimit(.06);
  follower.reset();
  result = follower.compute({}, {.06, 0., 0.}, .05);
  near(result.command.vx, .01, 1e-12, "emergency reset must seed zero");
  follower.reset({.03, 0., 0.});
  result = follower.compute({}, {}, .05);
  near(result.command.vx, .04, 1e-12, "explicit body seed");
}

void goal_brakes_before_reporting_stopped()
{
  Follower follower;
  follower.setPath({{0., 0.}, {1., 0.}}, 0.);
  follower.reset({.04, 0., 0.});
  const auto result = follower.compute({.96, 0., 0.}, {.04, 0., 0.}, .05);
  require(result.terminal, "terminal region not detected");
  near(result.command.vx, .03, 1e-12, "goal abruptly zeroed rather than decelerated");
  near(follower.compute({.962, 0., 0.}, {}, .05).command.vx, .02, 1e-12, "second braking step");
}

Settings tangent_settings()
{
  Settings settings;
  settings.heading_mode = HeadingMode::PathTangent;
  return settings;
}

void tangent_uses_path_chord_and_keeps_holonomic_translation()
{
  Follower follower(tangent_settings());
  follower.setPath({{0., 0.}, {2., 0.}}, 1.2);
  const auto offset = follower.compute({.1, .2, pi / 2.}, {}, .05);
  near(offset.target_yaw, 0., 1e-12, "lateral tracking error changed path heading");
  require(offset.command.wz < 0., "rotated chassis must correct toward forward path");
  require(std::hypot(offset.command.vx, offset.command.vy) > 0.,
    "tangent mode must translate before heading alignment");
  follower.clear();
  follower.setPath({{0., 0.}, {0., 2.}}, -.7);
  const auto lateral = follower.compute({}, {}, .05);
  near(lateral.target_yaw, pi / 2., 1e-12, "vertical path tangent");
  require(lateral.command.vy > 0. && lateral.command.wz > 0.,
    "sideways translation and heading correction must coexist");
  follower.clear();
  follower.setPath({{0., 0.}, {.2, 0.}, {.2, 1.}}, -.7);
  const auto curve = follower.compute({}, {}, .05);
  near(curve.target_yaw, std::atan2(std::sqrt(.12), .2), 1e-12,
    "curve heading must use projection-to-lookahead chord");
  follower.clear();
  follower.setPath({{0., 0.}, {2., 0.}}, 1.);
  const auto degenerate = follower.compute({0., .4, 0.}, {}, .05);
  near(degenerate.target_yaw, 0., 1e-12, "zero chord must use forward segment");
  follower.clear();
  follower.setPath({{0., 0.}, {1., 0.}, {1., 1.}, {0., 1.}, {0., .1}}, pi);
  const auto loop = follower.compute({}, {}, .05);
  require(!loop.terminal, "nearby loop endpoint incorrectly selected");
  near(loop.target_yaw, 0., 1e-12, "nearby loop endpoint caused early terminal turn");
}

void tangent_blends_terminal_yaw_and_wraps_shortest_angle()
{
  for (const auto & sample : std::vector<std::pair<double, double>>{
      {.6, 0.}, {.775, pi / 4.}, {.96, pi / 2.}, {1., pi / 2.}})
  {
    Follower follower(tangent_settings());
    follower.setPath({{0., 0.}, {1., 0.}}, pi / 2.);
    const auto result = follower.compute({sample.first, 0., 0.}, {}, .05);
    require(result.terminal, "short forward tail must be terminal");
    near(wrap_angle(result.target_yaw - sample.second), 0., 1e-11,
      "terminal yaw interpolation");
    if (sample.first >= .96) {
      near(std::hypot(result.command.vx, result.command.vy), 0., 1e-12,
        "position tolerance must request zero translation");
      require(result.command.wz > 0., "arbitrary terminal yaw still needs rotation");
    }
  }
  for (double direction : {-1., 1.}) {
    const double tangent = direction * (pi - .02);
    const double terminal = -direction * (pi - .02);
    const Point goal{std::cos(tangent), std::sin(tangent)};
    Follower follower(tangent_settings());
    follower.setPath({{0., 0.}, goal}, terminal);
    const auto result = follower.compute({.775 * goal.x, .775 * goal.y, tangent}, {}, .05);
    near(wrap_angle(result.target_yaw - direction * pi), 0., 1e-11,
      "terminal interpolation must cross pi by the short arc");
    require(result.command.wz * direction > 0., "terminal wrap rotation direction");
  }
  Settings invalid = tangent_settings();
  invalid.heading_mode = static_cast<HeadingMode>(99);
  rejects([&] {Follower follower(invalid);}, "unknown core heading mode accepted");
}

void tangent_single_point_rotates_and_replanning_preserves_velocity()
{
  Follower single(tangent_settings());
  single.setPath({{0., 0.}, {0., 0.}}, 1.3);
  const auto turn = single.compute({0., 0., -.5}, {}, .05);
  require(turn.terminal, "single point must be terminal");
  near(turn.target_yaw, 1.3, 1e-12, "single point terminal heading");
  near(std::hypot(turn.command.vx, turn.command.vy), 0., 1e-12,
    "single point at current position must rotate in place");
  require(turn.command.wz > 0., "single point did not turn toward requested yaw");
  single.setPath({{0., 0.}}, -1.1);
  near(single.compute({}, {}, .05).target_yaw, -1.1, 1e-12,
    "same path must accept a changed terminal yaw");

  Follower moving(tangent_settings());
  moving.setPath({{0., 0.}, {0., 2.}}, .7);
  Velocity previous;
  for (int i = 0; i < 10; ++i) {
    previous = moving.compute({}, previous, .05).command;
  }
  near(previous.wz, .1, 1e-12, "turn did not reach configured angular limit");
  moving.setPath({{0., 0.}, {2., -.1}}, .7);
  const auto replanned = moving.compute({}, previous, .05);
  require(std::hypot(replanned.command.vx - previous.vx,
      replanned.command.vy - previous.vy) <= .01 + 1e-12,
    "tangent replan reset translation history");
  near(replanned.command.wz, .08, 1e-12,
    "tangent replan must decelerate existing turn before reversing");
  near(replanned.target_yaw, std::atan2(-.1, 2.), 1e-12,
    "replan did not update tangent");
}

void simulate_tangent(const std::vector<Point> & path, double initial_yaw,
  double terminal_yaw, const std::string & name)
{
  const Settings settings = tangent_settings();
  Follower follower(settings);
  follower.setPath(path, terminal_yaw);
  Pose pose{path.front().x, path.front().y, initial_yaw};
  Velocity measured;
  Point previous_world;
  double previous_angular = 0.;
  bool stopped = false;
  for (int tick = 0; tick < 5000; ++tick) {
    const auto result = follower.compute(pose, measured, .05);
    const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
    const Point world{c * result.command.vx - s * result.command.vy,
      s * result.command.vx + c * result.command.vy};
    require(distance(world, previous_world) <= settings.linear_acceleration * .05 + 1e-11,
      name + " translational acceleration");
    require(std::abs(result.command.wz - previous_angular) <=
      settings.angular_acceleration * .05 + 1e-11, name + " angular acceleration");
    require(std::hypot(world.x, world.y) <= settings.max_linear + 1e-11,
      name + " linear speed");
    require(std::abs(result.command.wz) <= settings.max_angular + 1e-11,
      name + " angular speed");
    if (result.terminal &&
      distance({pose.x, pose.y}, path.back()) <= settings.xy_goal_tolerance)
    {
      near(wrap_angle(result.target_yaw - terminal_yaw), 0., 1e-12,
        name + " exact final yaw inside position tolerance");
    }
    pose.x += world.x * .05;
    pose.y += world.y * .05;
    pose.yaw = wrap_angle(pose.yaw + result.command.wz * .05);
    measured = result.command;
    previous_world = world;
    previous_angular = result.command.wz;
    if (distance({pose.x, pose.y}, path.back()) <= settings.xy_goal_tolerance + 1e-7 &&
      std::abs(wrap_angle(pose.yaw - terminal_yaw)) <= settings.yaw_goal_tolerance &&
      std::hypot(world.x, world.y) < .001 && std::abs(result.command.wz) < .001)
    {
      stopped = true;
      break;
    }
  }
  require(stopped, name + " did not reach position, terminal yaw and stopped motion");
}

void tangent_straight_lateral_curve_and_turn_in_place_simulations()
{
  simulate_tangent({{0., 0.}, {1., 0.}}, pi / 2., -.8, "straight, rotated start");
  simulate_tangent({{0., 0.}, {0., 1.}}, 0., -pi + .02, "lateral initial motion");
  simulate_tangent({{0., 0.}}, pi - .02, -pi + .02, "single-point wrap");
  std::vector<Point> curve;
  for (int i = 0; i <= 50; ++i) {
    const double angle = (pi / 2.) * i / 50.;
    curve.push_back({std::sin(angle), 1. - std::cos(angle)});
  }
  simulate_tangent(curve, -.4, 2.8, "curve with independent terminal heading");
}

void simulate(const std::vector<Point> & path, double locked_yaw, const std::string & name)
{
  Follower follower;
  follower.setPath(path, locked_yaw);
  Pose pose{path.front().x, path.front().y, locked_yaw};
  Velocity measured;
  Point previous_world;
  double previous_angular = 0.;
  bool stopped = false;
  for (int tick = 0; tick < 5000; ++tick) {
    Result result;
    try {
      result = follower.compute(pose, measured, .05);
    } catch (const std::exception & error) {
      throw std::runtime_error(name + " tick=" + std::to_string(tick) + " pose=" +
              std::to_string(pose.x) + "," + std::to_string(pose.y) + ": " + error.what());
    }
    const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
    const Point world{c * result.command.vx - s * result.command.vy,
      s * result.command.vx + c * result.command.vy};
    require(distance(world, previous_world) <= .2 * .05 + 1e-11, name + " acceleration");
    require(std::abs(result.command.wz - previous_angular) <= .4 * .05 + 1e-11,
      name + " angular acceleration");
    require(std::hypot(world.x, world.y) <= .06 + 1e-11, name + " speed");
    near(wrap_angle(pose.yaw - locked_yaw), 0., 1e-10, name + " heading drift");
    if (!result.terminal) {
      near(distance({pose.x, pose.y}, result.target), .4, 1e-8, name + " carrot radius");
    }
    pose.x += world.x * .05;
    pose.y += world.y * .05;
    pose.yaw = wrap_angle(pose.yaw + result.command.wz * .05);
    measured = result.command;
    previous_world = world;
    previous_angular = result.command.wz;
    if (distance({pose.x, pose.y}, path.back()) <= .05 + 1e-7 &&
      std::hypot(world.x, world.y) < .001)
    {
      stopped = true;
      break;
    }
  }
  require(stopped, name + " did not arrive and stop");
}

void straight_lateral_and_curve_simulations()
{
  simulate({{0., 0.}, {1., 0.}}, .7, "straight world path, rotated body");
  simulate({{0., 0.}, {0., 1.}}, 0., "lateral path");
  std::vector<Point> curve;
  for (int i = 0; i <= 50; ++i) {
    const double angle = (pi / 2.) * i / 50.;
    curve.push_back({std::sin(angle), 1. - std::cos(angle)});
  }
  simulate(curve, -.4, "curved path at fixed heading");
}
}  // namespace

int main()
{
  const std::vector<std::pair<std::string, std::function<void()>>> tests{
    {"straight_circle_updates", straight_circle_updates},
    {"curved_path_is_not_arc_length", curved_path_is_not_arc_length},
    {"endpoint_does_not_shortcut_loop", endpoint_does_not_shortcut_loop},
    {"duplicate_and_single_point_paths", duplicate_and_single_point_paths},
    {"invalid_inputs_and_outside_corridor", invalid_inputs_and_outside_corridor},
    {"sideways_and_heading_wrap", sideways_and_heading_wrap},
    {"measured_seed_and_acceleration_limits", measured_seed_and_acceleration_limits},
    {"replanning_preserves_velocity", replanning_preserves_velocity},
    {"repeated_path_preserves_crossing_progress", repeated_path_preserves_crossing_progress},
    {"speed_ceiling_and_emergency_reset", speed_ceiling_and_emergency_reset},
    {"goal_brakes_before_reporting_stopped", goal_brakes_before_reporting_stopped},
    {"straight_lateral_and_curve_simulations", straight_lateral_and_curve_simulations},
    {"tangent_uses_path_chord_and_keeps_holonomic_translation",
      tangent_uses_path_chord_and_keeps_holonomic_translation},
    {"tangent_blends_terminal_yaw_and_wraps_shortest_angle",
      tangent_blends_terminal_yaw_and_wraps_shortest_angle},
    {"tangent_single_point_rotates_and_replanning_preserves_velocity",
      tangent_single_point_rotates_and_replanning_preserves_velocity},
    {"tangent_straight_lateral_curve_and_turn_in_place_simulations",
      tangent_straight_lateral_curve_and_turn_in_place_simulations},
  };
  for (const auto & test : tests) {
    try {
      test.second();
      std::cout << "PASS " << test.first << '\n';
    } catch (const std::exception & error) {
      std::cerr << "FAIL " << test.first << ": " << error.what() << '\n';
      return 1;
    }
  }
  std::cout << tests.size() << " C++ lookahead test groups passed\n";
}
