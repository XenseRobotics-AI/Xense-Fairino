#pragma once

// Newer/backported Nav2 uses a dedicated planner header. Including both headers
// can redefine PlannerException, so select exactly one from the installed API.
#if __has_include("nav2_core/planner_exceptions.hpp")
#include "nav2_core/planner_exceptions.hpp"
namespace fairino_straight_planner::compat
{
using PlannerCancelled = nav2_core::PlannerCancelled;
}
#else
#include "nav2_core/exceptions.hpp"
namespace fairino_straight_planner::compat
{
// The original two-argument API has no distinct cancellation exception type.
using PlannerCancelled = nav2_core::PlannerException;
}
#endif
