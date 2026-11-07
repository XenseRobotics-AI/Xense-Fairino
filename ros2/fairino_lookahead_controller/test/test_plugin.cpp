// Run in a local, isolated ROS domain. No robot topics or motor interfaces are used.
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

class PluginTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("lookahead_plugin_test");
    node->declare_parameter("controller_frequency", 20.);
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("plugins", std::vector<std::string>{}),
      rclcpp::Parameter("global_frame", "map"), rclcpp::Parameter("robot_base_frame", "base_link"),
      rclcpp::Parameter("width", 4), rclcpp::Parameter("height", 4),
      rclcpp::Parameter("origin_x", -2.), rclcpp::Parameter("origin_y", -2.),
      rclcpp::Parameter("resolution", .02), rclcpp::Parameter("footprint_padding", 0.),
      rclcpp::Parameter("footprint", "[[-0.3,-0.2],[0.3,-0.2],[0.3,0.2],[-0.3,0.2]]")});
    costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
    costmap->configure();
    ASSERT_EQ(costmap->get_current_state().id(), 2u);
    tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    loader = std::make_unique<pluginlib::ClassLoader<nav2_core::Controller>>(
      "nav2_core", "nav2_core::Controller");
    controller = loader->createSharedInstance("fairino_lookahead_controller::HolonomicLookaheadController");
    controller->configure(node, "FollowPath", tf, costmap);
    controller->activate();
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(node->get_node_base_interface());
    motion = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/fairino_navigation/chassis_motion_state",
      rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile());
    motion->on_activate();
    for (int i = 0; i < 100 && motion->get_subscription_count() == 0; ++i) {
      executor->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    ASSERT_GT(motion->get_subscription_count(), 0u);
    feedback();
  }

  void TearDown() override
  {
    if (controller) {controller->deactivate(); controller->cleanup(); controller.reset();}
    motion.reset(); executor.reset();
    loader.reset(); tf.reset();
    if (costmap) {costmap->cleanup(); costmap.reset();}
    node.reset();
  }

  void publishMotion(const diagnostic_msgs::msg::DiagnosticArray & message)
  {
    for (int i = 0; i < 100 && motion->get_subscription_count() == 0; ++i) {
      executor->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    ASSERT_GT(motion->get_subscription_count(), 0u);
    motion->publish(message);
    for (int i = 0; i < 5; ++i) {
      executor->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }

  diagnostic_msgs::msg::DiagnosticArray feedbackMessage(
    bool waiting = false, double vx = 0., double vy = 0., double wz = 0.)
  {
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = node->now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "fairino_slam/chassis_motion";
    status.hardware_id = "0123456789abcdef0123456789abcdef";
    status.level = 0;
    const bool intent = std::max({std::abs(vx), std::abs(vy), std::abs(wz)}) > 1e-4;
    status.message = !intent ? "idle" : waiting ? "drive_wait" : "drive_active";
    const auto add = [&](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue pair; pair.key = key; pair.value = value;
        status.values.push_back(pair);
      };
    add("schema_version", "1"); add("sequence", std::to_string(++sequence));
    add("ready", "1"); add("drive_wait", waiting ? "1" : "0");
    add("hardware_age_sec", "0.01");
    add("drive_command_peak", waiting || !intent ? "0" : "1");
    add("drive_velocity_peak", "0");
    add("requested_vx", std::to_string(vx)); add("requested_vy", std::to_string(vy));
    add("requested_wz", std::to_string(wz));
    message.status.push_back(status);
    return message;
  }

  void feedback(bool waiting = false, double vx = 0., double vy = 0., double wz = 0.)
  {
    publishMotion(feedbackMessage(waiting, vx, vy, wz));
  }

  geometry_msgs::msg::TwistStamped compute(
    const geometry_msgs::msg::PoseStamped & input,
    const geometry_msgs::msg::Twist & measured, nav2_core::GoalChecker * checker)
  {
    feedback();
    return controller->computeVelocityCommands(input, measured, checker);
  }

  void headingMode(const std::string & mode)
  {
    controller->deactivate();
    controller->cleanup();
    node->set_parameter(rclcpp::Parameter("FollowPath.heading_mode", mode));
    controller->configure(node, "FollowPath", tf, costmap);
    controller->activate();
  }

  geometry_msgs::msg::PoseStamped pose(double x = 0., double y = 0., double yaw = 0.)
  {
    geometry_msgs::msg::PoseStamped result;
    result.header.frame_id = "map"; result.header.stamp = node->now();
    result.pose.position.x = x; result.pose.position.y = y;
    result.pose.orientation.z = std::sin(yaw / 2.); result.pose.orientation.w = std::cos(yaw / 2.);
    return result;
  }

  nav_msgs::msg::Path path(double target_yaw = 0.)
  {
    nav_msgs::msg::Path result;
    result.header.frame_id = "map"; result.header.stamp = node->now();
    result.poses = {pose(0, 0, target_yaw), pose(.5, 0, target_yaw), pose(1, 0, target_yaw)};
    // Native Smac2D paths have unset intermediate quaternions when smoothing is disabled.
    result.poses[0].pose.orientation = geometry_msgs::msg::Quaternion{};
    result.poses[1].pose.orientation = geometry_msgs::msg::Quaternion{};
    return result;
  }

  void obstacle(double x, double y, unsigned char value)
  {
    auto * map = costmap->getCostmap();
    unsigned ix, iy;
    ASSERT_TRUE(map->worldToMap(x, y, ix, iy));
    map->setCost(ix, iy, value);
  }

  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr motion;
  std::uint64_t sequence{0};
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap;
  std::shared_ptr<tf2_ros::Buffer> tf;
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::Controller>> loader;
  nav2_core::Controller::Ptr controller;
};

TEST_F(PluginTest, LifecycleAndBodyVelocity)
{
  constexpr double half_pi = 1.5707963267948966;
  controller->setPlan(path(half_pi));
  const auto result = compute(pose(0, 0, half_pi), {}, nullptr);
  EXPECT_EQ(result.header.frame_id, "base_link");
  EXPECT_NEAR(result.twist.linear.x, 0., 1e-9);
  EXPECT_LT(result.twist.linear.y, 0.);
  EXPECT_NEAR(result.twist.angular.z, 0., 1e-9);
  controller->deactivate();
  EXPECT_THROW(compute(pose(), {}, nullptr), nav2_core::PlannerException);
  controller->activate();
  EXPECT_THROW(compute(pose(), {}, nullptr), nav2_core::PlannerException);
}

TEST_F(PluginTest, RejectsInteriorObstacleUnknownAndInvalidPose)
{
  auto invalid_plan = path(); invalid_plan.poses.back().pose.orientation.w = 0.;
  EXPECT_THROW(controller->setPlan(invalid_plan), nav2_core::PlannerException);
  controller->setPlan(path());
  obstacle(.02, .02, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_DOUBLE_EQ(compute(pose(), {}, nullptr).twist.linear.x, 0.);
  obstacle(.02, .02, nav2_costmap_2d::NO_INFORMATION);
  EXPECT_DOUBLE_EQ(compute(pose(), {}, nullptr).twist.linear.x, 0.);
  obstacle(.02, .02, 0);
  auto invalid = pose(); invalid.pose.orientation.w = 0.;
  EXPECT_THROW(compute(invalid, {}, nullptr), nav2_core::PlannerException);
  auto stale = pose(); stale.header.stamp.sec -= 2;
  EXPECT_THROW(compute(stale, {}, nullptr), nav2_core::PlannerException);
}

TEST_F(PluginTest, ChecksMeasuredStoppingSweepAndSpeedLimit)
{
  controller->setPlan(path());
  geometry_msgs::msg::Twist measured; measured.linear.x = .06;
  obstacle(.35, .01, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_DOUBLE_EQ(compute(pose(), measured, nullptr).twist.linear.x, 0.);
  obstacle(.35, .01, 0);
  EXPECT_DOUBLE_EQ(compute(pose(), {}, nullptr).twist.linear.x, 0.);
  controller->setPlan(path());
  controller->setSpeedLimit(10., true);
  auto result = compute(pose(), {}, nullptr);
  EXPECT_GT(result.twist.linear.x, 0.);
  EXPECT_LE(std::hypot(result.twist.linear.x, result.twist.linear.y), .006 + 1e-10);
  controller->setSpeedLimit(0., false);  // Nav2: restore configured maximum.
  EXPECT_NO_THROW(compute(pose(), {}, nullptr));
}

TEST_F(PluginTest, TransformsPathAndLocalPoseIntoConsistentFrames)
{
  constexpr double half_pi = 1.5707963267948966;
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map"; transform.child_frame_id = "odom";
  transform.header.stamp = node->now();
  transform.transform.rotation.z = std::sin(half_pi / 2.);
  transform.transform.rotation.w = std::cos(half_pi / 2.);
  ASSERT_TRUE(tf->setTransform(transform, "offline_test", true));
  controller->setPlan(path(half_pi));
  auto input = pose(); input.header.frame_id = "odom";
  const auto result = compute(input, {}, nullptr);
  EXPECT_NEAR(result.twist.linear.x, 0., 1e-9);
  EXPECT_LT(result.twist.linear.y, 0.);
  EXPECT_NEAR(result.twist.angular.z, 0., 1e-9);
}

TEST_F(PluginTest, MovingReplanContinuesButStoppedNewPathDoesNotReuseOldSpeed)
{
  controller->setPlan(path());
  geometry_msgs::msg::Twist measured;
  for (int i = 0; i < 7; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    measured = compute(pose(), measured, nullptr).twist;
  }
  ASSERT_GT(measured.linear.x, .05);
  controller->setPlan(path());
  auto continued = compute(pose(), measured, nullptr);
  EXPECT_GT(continued.twist.linear.x, .05);
  // Model ControllerServer cancel zero followed immediately by a new path,
  // without any deactivate/activate transition or 0.3-second gap.
  controller->setPlan(path());
  const auto restarted = compute(pose(), {}, nullptr);
  EXPECT_LE(std::hypot(restarted.twist.linear.x, restarted.twist.linear.y), .01 + 1e-10);
}


TEST_F(PluginTest, TangentModeTranslatesWhileTurningAndKeepsTerminalYaw)
{
  constexpr double half_pi = 1.5707963267948966;
  headingMode("path_tangent");
  controller->setPlan(path(.7));
  const auto moving = compute(pose(0., 0., half_pi), {}, nullptr);
  EXPECT_LT(moving.twist.linear.y, 0.);
  EXPECT_LT(moving.twist.angular.z, 0.);
  EXPECT_LE(std::abs(moving.twist.angular.z), .1);
  nav_msgs::msg::Path terminal;
  terminal.header.frame_id = "map";
  terminal.poses = {pose(0., 0., 1.2)};
  controller->setPlan(terminal);
  const auto turning = compute(pose(), {}, nullptr);
  EXPECT_NEAR(turning.twist.linear.x, 0., 1e-9);
  EXPECT_NEAR(turning.twist.linear.y, 0., 1e-9);
  EXPECT_GT(turning.twist.angular.z, 0.);
}

TEST_F(PluginTest, RejectsUnknownHeadingModeAndRetainsHoldMode)
{
  EXPECT_THROW(headingMode("unsupported"), nav2_core::PlannerException);
  headingMode("hold_initial");
  controller->setPlan(path(.7));
  const auto held = compute(pose(0., 0., .7), {}, nullptr);
  EXPECT_NEAR(held.twist.angular.z, 0., 1e-9);
}

TEST_F(PluginTest, ChassisWaitKeepsSmallIntentThenReseedsAndHonorsGoalZero)
{
  controller->setPlan(path());
  feedback(true, .06, 0., 0.);
  auto waiting = controller->computeVelocityCommands(pose(), {}, nullptr);
  EXPECT_GT(waiting.twist.linear.x, 0.);
  EXPECT_LE(waiting.twist.linear.x, .02 + 1e-10);
  for (int i = 0; i < 5; ++i) {
    feedback(true, .06, 0., 0.);
    waiting = controller->computeVelocityCommands(pose(), {}, nullptr);
    EXPECT_LE(waiting.twist.linear.x, .02 + 1e-10);
  }
  feedback(false, .02, 0., 0.);
  const auto released = controller->computeVelocityCommands(pose(), {}, nullptr);
  EXPECT_LE(released.twist.linear.x, .01 + 1e-10);
  nav_msgs::msg::Path terminal;
  terminal.header.frame_id = "map"; terminal.poses = {pose()};
  controller->setPlan(terminal);
  feedback(true, .06, 0., 0.);
  const auto stopped = controller->computeVelocityCommands(pose(), {}, nullptr);
  EXPECT_DOUBLE_EQ(stopped.twist.linear.x, 0.);
  EXPECT_DOUBLE_EQ(stopped.twist.linear.y, 0.);
  EXPECT_DOUBLE_EQ(stopped.twist.angular.z, 0.);
}

TEST_F(PluginTest, ChassisWaitStillRejectsCollisionStaleFeedbackAndWorkerRestart)
{
  controller->setPlan(path());
  feedback(true, .06, 0., 0.);
  obstacle(.02, .02, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_DOUBLE_EQ(controller->computeVelocityCommands(pose(), {}, nullptr).twist.linear.x, 0.);
  obstacle(.02, .02, 0);
  feedback(true, .06, 0., 0.);
  std::this_thread::sleep_for(std::chrono::milliseconds(450));
  EXPECT_THROW(controller->computeVelocityCommands(pose(), {}, nullptr), nav2_core::PlannerException);
  feedback();
  auto replacement = feedbackMessage();
  replacement.status.front().hardware_id = std::string(32, 'a');
  publishMotion(replacement);
  EXPECT_THROW(controller->computeVelocityCommands(pose(), {}, nullptr), nav2_core::PlannerException);
}


TEST_F(PluginTest, RejectsMalformedFeedbackStampBeforeRosTimeNormalization)
{
  controller->setPlan(path());
  auto malformed = feedbackMessage();
  malformed.header.stamp.nanosec = 1000000000u;
  publishMotion(malformed);
  EXPECT_THROW(controller->computeVelocityCommands(pose(), {}, nullptr), nav2_core::PlannerException);
  feedback();
  malformed = feedbackMessage();
  malformed.header.stamp.sec = -1;
  publishMotion(malformed);
  EXPECT_THROW(controller->computeVelocityCommands(pose(), {}, nullptr), nav2_core::PlannerException);
}

TEST_F(PluginTest, ObstacleWaitRequiresNewPlanAndResumesWithBoundedAcceleration)
{
  controller->setPlan(path());
  geometry_msgs::msg::Twist measured;
  for (int i = 0; i < 7; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    measured = compute(pose(), measured, nullptr).twist;
  }
  ASSERT_GT(measured.linear.x, .05);
  obstacle(.02, .02, nav2_costmap_2d::LETHAL_OBSTACLE);
  const auto stopped = compute(pose(), measured, nullptr).twist;
  EXPECT_DOUBLE_EQ(stopped.linear.x, 0.);
  EXPECT_DOUBLE_EQ(stopped.linear.y, 0.);
  EXPECT_DOUBLE_EQ(stopped.angular.z, 0.);
  obstacle(.02, .02, 0);
  EXPECT_DOUBLE_EQ(compute(pose(), {}, nullptr).twist.linear.x, 0.);
  controller->setPlan(path());
  const auto resumed = compute(pose(), {}, nullptr).twist;
  EXPECT_GT(resumed.linear.x, 0.);
  EXPECT_LE(resumed.linear.x, .01 + 1e-10);
}
