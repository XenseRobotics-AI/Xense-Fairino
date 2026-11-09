// Load/construct/destruct only: no ROS node, costmap, publisher, or motor command.
#include <exception>
#include <iostream>
#include "nav2_core/controller.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "pluginlib/class_loader.hpp"

int main()
{
  try {
    pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core", "nav2_core::Controller");
    auto controller = loader.createSharedInstance(
      "fairino_lookahead_controller::HolonomicLookaheadController");
    if (!controller) {return 1;}
    controller.reset();
    BT::BehaviorTreeFactory factory;
    BT::SharedLibrary library;
    factory.registerFromPlugin(library.getOSName("fairino_replan_bt_node"));
    if (factory.manifests().count("ReplanUntilClear") != 1 ||
      factory.manifests().count("CaptureTravelGoal") != 1) {return 1;}
    std::cout << "Fairino lookahead plugin loaded successfully; no ROS node or robot connection.\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
