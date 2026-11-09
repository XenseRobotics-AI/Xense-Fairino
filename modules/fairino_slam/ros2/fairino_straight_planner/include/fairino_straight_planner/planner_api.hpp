#pragma once

#include <cmath>
#include <functional>
#include <type_traits>
#include <utility>

namespace fairino_straight_planner::compat
{
// Humble SmacPlanner2D leaves intermediate quaternions at zero when smoothing
// is disabled. We consume only their XY positions and regenerate all headings.
// User start/goal poses still require a normalized quaternion.
inline bool usable_orientation(double x, double y, double z, double w, bool allow_unspecified)
{
  const double norm = x * x + y * y + z * z + w * w;
  const bool unspecified = x == 0. && y == 0. && z == 0. && w == 0.;
  return std::isfinite(norm) &&
         (std::abs(norm - 1.) <= 1e-3 || (allow_unspecified && unspecified));
}

namespace detail
{
template<class Planner, class Start, class Goal, class = void>
struct HasCancelCallback : std::false_type {};

template<class Planner, class Start, class Goal>
struct HasCancelCallback<Planner, Start, Goal, std::void_t<decltype(
    std::declval<Planner &>().createPlan(
      std::declval<const Start &>(), std::declval<const Goal &>(),
      std::declval<std::function<bool()>>()))>> : std::true_type {};
}  // namespace detail

// Detect the actual installed interface: vendor Nav2 workspaces may backport the
// callback API without changing ROS_DISTRO. Keep the selected planner's result
// and exceptions unchanged. Legacy two-argument planners cannot be interrupted
// through this helper while their createPlan() call is running.
template<class Planner, class Start, class Goal>
decltype(auto) call_create_plan(
  Planner & planner, const Start & start, const Goal & goal,
  std::function<bool()> cancel_checker = {})
{
  if constexpr (detail::HasCancelCallback<Planner, Start, Goal>::value) {
    // Some Smac versions call the callback without checking whether it is empty.
    if (!cancel_checker) {cancel_checker = []() {return false;};}
    return planner.createPlan(start, goal, std::move(cancel_checker));
  } else {
    return planner.createPlan(start, goal);
  }
}
}  // namespace fairino_straight_planner::compat
