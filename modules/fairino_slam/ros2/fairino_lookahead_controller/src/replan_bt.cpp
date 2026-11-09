#include "fairino_lookahead_controller/replan.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/robot_utils.hpp"

namespace fairino_lookahead_controller
{

class CaptureTravelGoal : public BT::SyncActionNode
{
public:
  CaptureTravelGoal(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("travel_goal")};
  }
  BT::NodeStatus tick() override
  {
    geometry_msgs::msg::PoseStamped goal, current;
    if (!getInput("goal", goal) || goal.header.frame_id != "map") {
      throw BT::RuntimeError("Saved pose requires a map goal");
    }
    const auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    const auto tf = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    if (!nav2_util::getCurrentPose(current, *tf, "map", "base_footprint", .1)) {
      throw BT::RuntimeError("Saved pose: current robot transform unavailable");
    }
    const double age = node->now().seconds() - rclcpp::Time(current.header.stamp).seconds();
    const auto & q = current.pose.orientation;
    const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    if (!std::isfinite(age) || age < -.05 || age >= 1. ||
      !std::isfinite(norm) || std::abs(norm-1.) > 1e-3)
    {
      throw BT::RuntimeError("Saved pose: current robot transform stale or invalid");
    }
    // Keep the original goal untouched for the arrival alignment stage.
    goal.pose.orientation = current.pose.orientation;
    setOutput("travel_goal", goal);
    return BT::NodeStatus::SUCCESS;
  }
};

class ReplanNode : public ReplanUntilClear
{
public:
  ReplanNode(const std::string & name, const BT::NodeConfiguration & config)
  : ReplanUntilClear(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    executor_.add_callback_group(group_, node_->get_node_base_interface());
    rclcpp::SubscriptionOptions options; options.callback_group = group_;
    sub_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/fairino_navigation/controller_motion", rclcpp::QoS(10),
      [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {
        const double stamp = rclcpp::Time(msg->header.stamp).seconds();
        const double age = node_->now().seconds() - stamp;
        if (stamp < follow_started_ || age < -.1 || age > .6 || msg->status.size() != 1) {return;}
        const auto & state = msg->status.front();
        if (state.name == "blocked") {blocked_ = state.message;}
        if (state.name == "failed") {fault_ = state.message;}
      }, options);
    // Humble creates a separate client node and tree for each navigator.
    const std::string action = std::string(node_->get_name()).find("navigate_through_poses")
      != std::string::npos ? "navigate_through_poses" : "navigate_to_pose";
    // A multi-stage tree still has exactly one authenticated DDS publisher.
    if (!config.blackboard->get("fairino_execution_publisher", pub_)) {
      pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/fairino_navigation/execution_state/" + action, rclcpp::QoS(10));
      config.blackboard->set("fairino_execution_publisher", pub_);
    }
  }
protected:
  std::string blocked() override {executor_.spin_some(); return blocked_;}
  std::string fault() override {executor_.spin_some(); return fault_;}
  void beginFollow() override
  {
    executor_.spin_some(); blocked_.clear(); fault_.clear();
    follow_started_ = node_->now().seconds();
  }
  void report(const std::string & phase, const std::string & reason) override
  {
    diagnostic_msgs::msg::DiagnosticArray msg; msg.header.stamp = node_->now();
    diagnostic_msgs::msg::DiagnosticStatus state;
    state.name = phase; state.message = reason;
    state.level = phase == "failed" ? 2 : (phase == "waiting" ? 1 : 0);
    msg.status.push_back(state); pub_->publish(msg);
  }
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_;
  double follow_started_{0.};
  std::string blocked_, fault_;
};
}
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<fairino_lookahead_controller::ReplanNode>("ReplanUntilClear");
  factory.registerNodeType<fairino_lookahead_controller::CaptureTravelGoal>("CaptureTravelGoal");
}
