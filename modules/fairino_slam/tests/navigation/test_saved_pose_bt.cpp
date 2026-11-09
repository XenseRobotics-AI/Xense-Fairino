// ROS/BT integration with in-memory TF and fake actions. No motion publishers.
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

using Pose = geometry_msgs::msg::PoseStamped;
std::vector<Pose> planned;
void require(bool yes, const char *message) {if (!yes) {throw std::runtime_error(message);}}
class Plan : public BT::SyncActionNode {
public:
  Plan(const std::string &n, const BT::NodeConfiguration &c) : BT::SyncActionNode(n,c) {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<Pose>("goal"), BT::OutputPort<std::string>("path"),
            BT::InputPort<std::string>("planner_id")};
  }
  BT::NodeStatus tick() override {
    Pose goal; require(bool(getInput("goal",goal)), "No goal");
    planned.push_back(goal); setOutput("path",std::string("checked path"));
    return BT::NodeStatus::SUCCESS;
  }
};
class Follow : public BT::SyncActionNode {
public:
  Follow(const std::string &n, const BT::NodeConfiguration &c) : BT::SyncActionNode(n,c) {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("path"), BT::InputPort<std::string>("controller_id"),
            BT::InputPort<std::string>("goal_checker_id")};
  }
  BT::NodeStatus tick() override {return BT::NodeStatus::SUCCESS;}
};
int main(int argc,char **argv) {
  try {
    require(argc==3,"Arguments: plugin.so saved_pose.xml");
    rclcpp::init(argc,argv);
    auto node=std::make_shared<rclcpp::Node>("saved_pose_bt_test");
    auto tf=std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf->setUsingDedicatedThread(true);  // In-memory TF is seeded synchronously below.
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id="map"; transform.child_frame_id="base_footprint";
    transform.header.stamp=node->now();
    transform.transform.rotation.z=std::sin(.2);
    transform.transform.rotation.w=std::cos(.2);
    require(tf->setTransform(transform,"test",false),"Cannot seed TF");
    auto board=BT::Blackboard::create();
    board->set("node",node); board->set("tf_buffer",tf);
    Pose goal;goal.header.frame_id="map";goal.header.stamp=node->now();
    goal.pose.position.x=1.25;goal.pose.position.y=-2.5;
    goal.pose.orientation.z=std::sin(.6);goal.pose.orientation.w=std::cos(.6);
    board->set("goal",goal);
    BT::BehaviorTreeFactory factory;
    factory.registerFromPlugin(argv[1]);
    factory.registerNodeType<Plan>("ComputePathToPose");
    factory.registerNodeType<Follow>("FollowPath");
    auto tree=factory.createTreeFromFile(argv[2],board);
    require(tree.tickRoot()==BT::NodeStatus::SUCCESS,"Saved tree did not finish");
    require(planned.size()==2,"Saved navigation did not have two stages");
    require(std::abs(planned[0].pose.orientation.z-std::sin(.2))<1e-9,
            "Travel heading is not current heading");
    require(std::abs(planned[1].pose.orientation.z-std::sin(.6))<1e-9,
            "Arrival heading is not saved heading");
    require(planned[0].pose.position.x==goal.pose.position.x &&
            planned[1].pose.position.y==goal.pose.position.y,"Position changed between stages");
    require(board->get<Pose>("goal").pose.orientation.z==goal.pose.orientation.z,
            "Original goal changed");
    require(node->get_publishers_info_by_topic("/fairino_navigation/execution_state/navigate_to_pose").size()==1,
            "Multi-stage navigation created duplicate execution publishers");
    tree.haltTree();
    tf->clear();
    planned.clear();
    auto stale=transform;
    stale.header.stamp=node->now()-rclcpp::Duration::from_seconds(2.);
    tf->setTransform(stale,"test",false);
    bool rejected=false;
    try {tree.tickRoot();} catch(const BT::RuntimeError &) {rejected=true;}
    require(rejected && planned.empty(),"Stale capture dispatched planning");
    rclcpp::shutdown();
    std::cout<<"PASS saved pose plugin: actual TF capture, original goal, two stages, one publisher, stale rejection\n";
    return 0;
  } catch(const std::exception &e) {
    std::cerr<<e.what()<<'\n';
    if(rclcpp::ok()) {rclcpp::shutdown();}
    return 1;
  }
}
