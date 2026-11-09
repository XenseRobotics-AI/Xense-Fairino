#include <limits>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#include "fairino_straight_planner/planner_api.hpp"

namespace
{
using fairino_straight_planner::compat::call_create_plan;
using Path = std::vector<int>;
struct Pose {int value;};
struct MarkerError : std::runtime_error
{
  MarkerError() : std::runtime_error("original planner exception") {}
};
void check(bool condition, const char * expression, int line)
{
  if (!condition) {
    throw std::runtime_error(std::string("line ") + std::to_string(line) + ": " + expression);
  }
}
#define CHECK(expression) check((expression), #expression, __LINE__)

struct LegacyBase
{
  virtual ~LegacyBase() = default;
  virtual Path createPlan(const Pose &, const Pose &) = 0;
};
struct CallbackBase
{
  virtual ~CallbackBase() = default;
  virtual Path createPlan(const Pose &, const Pose &, std::function<bool()>) = 0;
};
struct LegacyPlanner : LegacyBase
{
  unsigned calls{0};
  bool fail{false};
  Path createPlan(const Pose & start, const Pose & goal) override
  {
    ++calls;
    if (fail) {throw MarkerError();}
    return {start.value, 77, goal.value};
  }
};
struct CallbackPlanner : CallbackBase
{
  unsigned calls{0};
  bool callback_valid{false}, cancelled{false}, fail{false};
  Path createPlan(
    const Pose & start, const Pose & goal, std::function<bool()> cancel_checker) override
  {
    ++calls;
    callback_valid = static_cast<bool>(cancel_checker);
    cancelled = cancel_checker();  // Deliberately invoke unconditionally, like Smac.
    if (fail) {throw MarkerError();}
    return {start.value, cancelled ? -1 : 88, goal.value};
  }
};
struct BothPlanner
{
  unsigned old_calls{0}, new_calls{0};
  Path createPlan(const Pose &, const Pose &)
  {
    ++old_calls;
    return {11};
  }
  Path createPlan(const Pose &, const Pose &, std::function<bool()> cancel_checker)
  {
    ++new_calls;
    return {cancel_checker() ? 33 : 22};
  }
};
struct ReferencePlanner
{
  Path path{4, 5, 6};
  Path & createPlan(const Pose &, const Pose &, std::function<bool()>) {return path;}
};

void legacy_interface_is_selected_without_evaluating_callback()
{
  LegacyPlanner implementation;
  LegacyBase & planner = implementation;
  unsigned cancellation_checks = 0;
  const auto result = call_create_plan(
    planner, Pose{3}, Pose{9}, [&]() {++cancellation_checks; return true;});
  CHECK(result == Path({3, 77, 9}));
  CHECK(implementation.calls == 1);
  CHECK(cancellation_checks == 0);
  CHECK(call_create_plan(planner, Pose{5}, Pose{10}) == Path({5, 77, 10}));
}

void callback_interface_receives_stateful_cancellation()
{
  CallbackPlanner implementation;
  CallbackBase & planner = implementation;
  unsigned cancellation_checks = 0;
  bool cancel = false;
  std::function<bool()> callback = [&]() {++cancellation_checks; return cancel;};
  CHECK(call_create_plan(planner, Pose{3}, Pose{9}, callback) == Path({3, 88, 9}));
  CHECK(implementation.callback_valid && !implementation.cancelled);
  cancel = true;
  CHECK(call_create_plan(planner, Pose{3}, Pose{9}, callback) == Path({3, -1, 9}));
  CHECK(implementation.cancelled);
  CHECK(cancellation_checks == 2 && implementation.calls == 2);
}

void empty_callback_is_safe_and_never_cancels()
{
  CallbackPlanner implementation;
  CallbackBase & planner = implementation;
  CHECK(call_create_plan(planner, Pose{1}, Pose{2}) == Path({1, 88, 2}));
  CHECK(implementation.callback_valid && !implementation.cancelled);
  CHECK(call_create_plan(planner, Pose{1}, Pose{2}, {}) == Path({1, 88, 2}));
  CHECK(implementation.callback_valid && !implementation.cancelled);
}

void callback_overload_is_preferred_if_both_exist()
{
  BothPlanner planner;
  CHECK(call_create_plan(planner, Pose{1}, Pose{2}) == Path({22}));
  CHECK(call_create_plan(planner, Pose{1}, Pose{2}, []() {return true;}) == Path({33}));
  CHECK(planner.old_calls == 0 && planner.new_calls == 2);
}

void planner_and_callback_exceptions_keep_their_original_type()
{
  LegacyPlanner legacy;
  CallbackPlanner current;
  legacy.fail = true;
  current.fail = true;
  unsigned caught = 0;
  try {call_create_plan(legacy, Pose{1}, Pose{2});} catch (const MarkerError &) {++caught;}
  try {call_create_plan(current, Pose{1}, Pose{2});} catch (const MarkerError &) {++caught;}
  current.fail = false;
  try {
    call_create_plan(current, Pose{1}, Pose{2}, []() -> bool {throw MarkerError();});
  } catch (const MarkerError &) {++caught;}
  CHECK(caught == 3);
}

void return_value_category_is_preserved()
{
  ReferencePlanner planner;
  static_assert(std::is_same_v<decltype(call_create_plan(planner, Pose{1}, Pose{2})), Path &>);
  auto & result = call_create_plan(planner, Pose{1}, Pose{2});
  CHECK(&result == &planner.path);
  CHECK(result == Path({4, 5, 6}));
}
void smac_unspecified_heading_is_only_allowed_for_fallback_samples()
{
  using fairino_straight_planner::compat::usable_orientation;
  CHECK(usable_orientation(0., 0., 0., 1., false));
  CHECK(usable_orientation(0., 0., 0., -1., true));
  CHECK(!usable_orientation(0., 0., 0., 0., false));
  CHECK(usable_orientation(0., 0., 0., 0., true));
  CHECK(!usable_orientation(0., 0., 0., 1e-6, true));
  CHECK(!usable_orientation(0., 0., 0., 2., true));
  CHECK(!usable_orientation(std::numeric_limits<double>::quiet_NaN(), 0., 0., 1., true));
  CHECK(!usable_orientation(0., 0., 0., std::numeric_limits<double>::infinity(), true));
}
}  // namespace

int main()
{
  try {
    legacy_interface_is_selected_without_evaluating_callback();
    callback_interface_receives_stateful_cancellation();
    empty_callback_is_safe_and_never_cancels();
    callback_overload_is_preferred_if_both_exist();
    planner_and_callback_exceptions_keep_their_original_type();
    return_value_category_is_preserved();
    smac_unspecified_heading_is_only_allowed_for_fallback_samples();
    std::cout << "Planner API compatibility: 7 groups passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
