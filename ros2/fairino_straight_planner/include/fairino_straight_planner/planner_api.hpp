#pragma once

#include <functional>
#include <type_traits>
#include <utility>

namespace fairino_straight_planner::compat
{
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
