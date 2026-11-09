// Local isolated ROS domain only. These tests never use robot or motor interfaces.
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "fairino_straight_planner/planner_api.hpp"
#include "fairino_straight_planner/planner_exceptions.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

class PlannerPluginTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void parameters(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & parent)
  {
    parent->declare_parameter("GridBased.max_planning_time", 2.);
    parent->declare_parameter("GridBased.tolerance", 0.);
    parent->declare_parameter("GridBased.allow_unknown", false);
    parent->declare_parameter("GridBased.use_final_approach_orientation", false);
    parent->declare_parameter("GridBased.smoother.max_iterations", 0);
  }

  void SetUp() override
  {
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("straight_planner_test");
    parameters(node);
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("plugins", std::vector<std::string>{"inflation_layer"}),
      rclcpp::Parameter("inflation_layer.plugin", "nav2_costmap_2d::InflationLayer"),
      rclcpp::Parameter("inflation_layer.inflation_radius", .5),
      rclcpp::Parameter("global_frame", "map"),
      rclcpp::Parameter("robot_base_frame", "base_link"),
      rclcpp::Parameter("width", 8), rclcpp::Parameter("height", 8),
      rclcpp::Parameter("origin_x", -4.), rclcpp::Parameter("origin_y", -4.),
      rclcpp::Parameter("resolution", .1), rclcpp::Parameter("footprint_padding", 0.),
      rclcpp::Parameter("footprint", "[[-0.2,-0.2],[0.2,-0.2],[0.2,0.2],[-0.2,0.2]]")});
    costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
    costmap->configure();
    ASSERT_EQ(costmap->get_current_state().id(), 2u);
    auto * map = costmap->getCostmap();
    std::fill(map->getCharMap(),
      map->getCharMap() + map->getSizeInCellsX() * map->getSizeInCellsY(), 0);
    tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    loader = std::make_unique<pluginlib::ClassLoader<nav2_core::GlobalPlanner>>(
      "nav2_core", "nav2_core::GlobalPlanner");
    planner = loader->createSharedInstance("fairino_straight_planner::StraightLinePlanner");
    planner->configure(node, "GridBased", tf, costmap);
    planner->activate();
  }

  void TearDown() override
  {
    if (reference) {reference->deactivate(); reference->cleanup(); reference.reset();}
    if (planner) {planner->deactivate(); planner->cleanup(); planner.reset();}
    reference_node.reset();
    loader.reset();
    tf.reset();
    if (costmap) {costmap->cleanup(); costmap.reset();}
    node.reset();
  }

  geometry_msgs::msg::PoseStamped pose(double x, double y, double yaw = 0.)
  {
    geometry_msgs::msg::PoseStamped result;
    result.header.frame_id = "map";
    result.header.stamp = node->now();
    result.pose.position.x = x;
    result.pose.position.y = y;
    result.pose.orientation.z = std::sin(yaw / 2.);
    result.pose.orientation.w = std::cos(yaw / 2.);
    return result;
  }

  void setCost(double x, double y, unsigned char value)
  {
    unsigned int mx, my;
    ASSERT_TRUE(costmap->getCostmap()->worldToMap(x, y, mx, my));
    costmap->getCostmap()->setCost(mx, my, value);
  }

  void expectEndpoints(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
  {
    ASSERT_FALSE(path.poses.empty());
    EXPECT_DOUBLE_EQ(path.poses.front().pose.position.x, start.pose.position.x);
    EXPECT_DOUBLE_EQ(path.poses.front().pose.position.y, start.pose.position.y);
    EXPECT_DOUBLE_EQ(path.poses.back().pose.position.x, goal.pose.position.x);
    EXPECT_DOUBLE_EQ(path.poses.back().pose.position.y, goal.pose.position.y);
    EXPECT_EQ(path.poses.back().pose.orientation, goal.pose.orientation);
    for (std::size_t i = 1; i < path.poses.size(); ++i) {
      const auto & a = path.poses[i - 1].pose.position;
      const auto & b = path.poses[i].pose.position;
      EXPECT_LE(std::hypot(b.x - a.x, b.y - a.y), .1 + 1e-10);
    }
  }

  void makeReference()
  {
    reference_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("straight_smac_reference");
    parameters(reference_node);
    reference = loader->createSharedInstance("nav2_smac_planner/SmacPlanner2D");
    reference->configure(reference_node, "GridBased", tf, costmap);
    reference->activate();
  }

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, reference_node;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap;
  std::shared_ptr<tf2_ros::Buffer> tf;
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::GlobalPlanner>> loader;
  nav2_core::GlobalPlanner::Ptr planner, reference;
};

TEST_F(PlannerPluginTest, DirectFreeSpacePreservesExactEndpointsAndRequestedYaw)
{
  const auto start = pose(-1.23, -.43, -.6), goal = pose(1.12, .71, 1.8);
  const auto path = fairino_straight_planner::compat::call_create_plan(*planner, start, goal);
  expectEndpoints(path, start, goal);
  ASSERT_GT(path.poses.size(), 2u);
  for (const auto & point : path.poses) {
    EXPECT_NEAR(
      (point.pose.position.x - start.pose.position.x) *
      (goal.pose.position.y - start.pose.position.y) -
      (point.pose.position.y - start.pose.position.y) *
      (goal.pose.position.x - start.pose.position.x), 0., 1e-10);
  }
}

TEST_F(PlannerPluginTest, SubcellDistanceIsNotCollapsedToStartCell)
{
  const auto start = pose(.11, .11), goal = pose(.12, .13, 1.2);
  expectEndpoints(fairino_straight_planner::compat::call_create_plan(*planner, start, goal), start, goal);
}

TEST_F(PlannerPluginTest, InPlaceTargetKeepsRequestedYaw)
{
  const auto start = pose(.15, .15, -.4), goal = pose(.15, .15, 2.1);
  const auto path = fairino_straight_planner::compat::call_create_plan(*planner, start, goal);
  ASSERT_FALSE(path.poses.empty());
  EXPECT_EQ(path.poses.back().pose.orientation, goal.pose.orientation);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.x, .15);
}

TEST_F(PlannerPluginTest, WallRequiresSmacDetourAndLongStraightSegments)
{
  // 253 models an already-inflated obstacle in the master costmap.
  for (double x = -.25; x < .3; x += .1) {
    for (double y = -.75; y < .8; y += .1) {setCost(x, y, 253);}
  }
  const auto start = pose(-2.03, .13), goal = pose(2.12, .13, -.7);
  const auto path = fairino_straight_planner::compat::call_create_plan(*planner, start, goal);
  expectEndpoints(path, start, goal);
  bool detour = false;
  std::size_t corners = 0;
  double previous = 0.;
  bool previous_valid = false;
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    detour = detour || std::abs(b.y - .13) > .5;
    const double direction = std::atan2(b.y - a.y, b.x - a.x);
    if (previous_valid && std::abs(std::remainder(direction - previous, 2. * 3.141592653589793)) > 1e-5) {
      ++corners;
    }
    previous = direction;
    previous_valid = true;
  }
  EXPECT_TRUE(detour);
  EXPECT_LE(corners, 5u);
}

TEST_F(PlannerPluginTest, OccupiedUnknownAndOutOfMapExactGoalsFail)
{
  const auto start = pose(-1.2, .15), goal = pose(1.15, .15);
  setCost(1.15, .15, 253);
  EXPECT_THROW(fairino_straight_planner::compat::call_create_plan(*planner, start, goal), nav2_core::PlannerException);
  setCost(1.15, .15, 255);
  EXPECT_THROW(fairino_straight_planner::compat::call_create_plan(*planner, start, goal), nav2_core::PlannerException);
  EXPECT_THROW(fairino_straight_planner::compat::call_create_plan(*planner, start, pose(6., .15)), nav2_core::PlannerException);
}

TEST_F(PlannerPluginTest, UnknownRegionAndMapBoundaryKeepFootprintClearance)
{
  setCost(1.15, .15, 255);
  EXPECT_THROW(fairino_straight_planner::compat::call_create_plan(*planner, pose(-1., .15), pose(.95, .15)), nav2_core::PlannerException);
  EXPECT_THROW(fairino_straight_planner::compat::call_create_plan(*planner, pose(-1., .15), pose(3.95, .15)), nav2_core::PlannerException);
}

TEST_F(PlannerPluginTest, CompleteWallFailsWithoutUnsafeStraightFallback)
{
  auto * map = costmap->getCostmap();
  for (unsigned int y = 0; y < map->getSizeInCellsY(); ++y) {
    map->setCost(39, y, 253);
    map->setCost(40, y, 253);
  }
  EXPECT_THROW(fairino_straight_planner::compat::call_create_plan(*planner, pose(-1.1, .15), pose(1.1, .15)), nav2_core::PlannerException);
}

TEST_F(PlannerPluginTest, InvalidPoseAndExhaustedBudgetFail)
{
  auto bad = pose(1., 0.);
  bad.header.frame_id = "odom";
  EXPECT_THROW(fairino_straight_planner::compat::call_create_plan(*planner, pose(-1., 0.), bad), nav2_core::PlannerException);
  bad = pose(1., 0.);
  bad.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(fairino_straight_planner::compat::call_create_plan(*planner, pose(-1., 0.), bad), nav2_core::PlannerException);
  node->set_parameter(rclcpp::Parameter("GridBased.max_planning_time", 1e-12));
  EXPECT_THROW(fairino_straight_planner::compat::call_create_plan(*planner, pose(-1., 0.), pose(1., 0.)), nav2_core::PlannerException);
}

TEST_F(PlannerPluginTest, DisabledOptimizationReturnsOriginalSmacPathUnchanged)
{
  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("GridBased.straight_line_enabled", false)).successful);
  makeReference();
  const auto start = pose(-1.23, -.43, -.6), goal = pose(1.12, .71, 1.8);
  const auto expected = fairino_straight_planner::compat::call_create_plan(*reference, start, goal);
  const auto actual = fairino_straight_planner::compat::call_create_plan(*planner, start, goal);
  ASSERT_FALSE(expected.poses.empty());
  ASSERT_EQ(actual.poses.size(), expected.poses.size());
  EXPECT_EQ(actual.header.frame_id, expected.header.frame_id);
  for (std::size_t i = 0; i < expected.poses.size(); ++i) {
    EXPECT_EQ(actual.poses[i].pose, expected.poses[i].pose);
    EXPECT_EQ(actual.poses[i].header.frame_id, expected.poses[i].header.frame_id);
  }
  // Re-enable on the same configured plugin; exact endpoint behavior comes back.
  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("GridBased.straight_line_enabled", true)).successful);
  expectEndpoints(fairino_straight_planner::compat::call_create_plan(*planner, start, goal), start, goal);
}

TEST_F(PlannerPluginTest, DisabledOptimizationPreservesOriginalSameCellBehavior)
{
  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("GridBased.straight_line_enabled", false)).successful);
  makeReference();
  const auto start = pose(.11, .11), goal = pose(.12, .13, 1.2);
  const auto expected = fairino_straight_planner::compat::call_create_plan(*reference, start, goal);
  const auto actual = fairino_straight_planner::compat::call_create_plan(*planner, start, goal);
  ASSERT_EQ(actual.poses.size(), expected.poses.size());
  ASSERT_FALSE(actual.poses.empty());
  EXPECT_EQ(actual.poses.back().pose, expected.poses.back().pose);
}


// These cancellation checks apply to the installed base interface. A legacy
// two-argument base cannot receive a callback; the separate API tests cover it.
TEST_F(PlannerPluginTest, CancellationStopsDirectOptimizationBeforeReturningAPath)
{
  if constexpr (fairino_straight_planner::compat::detail::HasCancelCallback<
      nav2_core::GlobalPlanner, geometry_msgs::msg::PoseStamped,
      geometry_msgs::msg::PoseStamped>::value)
  {
    EXPECT_THROW(
      fairino_straight_planner::compat::call_create_plan(
        *planner, pose(-1., .15), pose(1., .15), []() {return true;}),
      fairino_straight_planner::compat::PlannerCancelled);
  } else {
    GTEST_SKIP() << "Installed Nav2 has the legacy two-argument planner interface";
  }
}
