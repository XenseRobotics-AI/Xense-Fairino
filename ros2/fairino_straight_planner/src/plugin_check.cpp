#include <exception>
#include <iostream>
#include "nav2_core/global_planner.hpp"
#include "pluginlib/class_loader.hpp"

int main()
{
  try {
    // No ROS init, nodes, services, costmaps or robot interfaces are created.
    pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader(
      "nav2_core", "nav2_core::GlobalPlanner");
    auto planner = loader.createSharedInstance("fairino_straight_planner::StraightLinePlanner");
    auto fallback = loader.createSharedInstance("nav2_smac_planner/SmacPlanner2D");
    if (!planner || !fallback) {return 1;}
    std::cout << "straight planner and Smac fallback plugin loading passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "planner plugin loading failed: " << error.what() << '\n';
    return 1;
  }
}
