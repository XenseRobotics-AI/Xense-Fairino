#pragma once
#include <chrono>
#include <string>
#include "behaviortree_cpp_v3/control_node.h"

namespace fairino_lookahead_controller
{
// ROS adapter supplies observations and publishes state; scheduling is tested
// with real BT v3 and fake action leaves, without robot access.
class ReplanUntilClear : public BT::ControlNode
{
public:
  using Clock = std::chrono::steady_clock;
  ReplanUntilClear(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ControlNode(name, config) {}
  static BT::PortsList providedPorts() {return {BT::InputPort<bool>("final_stage", true, "Emit goal completion after this stage")};}
  void halt() override
  {
    if (status() == BT::NodeStatus::RUNNING) {
      report("canceled", "已停止；不会自动恢复");
    }
    BT::ControlNode::halt();
    following_ = false;
  }
protected:
  virtual std::string blocked() {return {};}
  virtual std::string fault() {return {};}
  virtual void beginFollow() {}
  virtual void report(const std::string &, const std::string &) {}
  virtual Clock::time_point now() const {return Clock::now();}
private:
  BT::NodeStatus wait(const std::string & reason)
  {
    // Inhibit output BEFORE waiting for FollowPath cancellation.
    reason_ = reason; phase_ = "waiting";
    report(phase_, reason_);
    haltChildren(); following_ = false;
    next_plan_ = now() + std::chrono::seconds(1);
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus failed(const std::string & reason)
  {
    report("failed", reason);
    haltChildren(); following_ = false;
    return BT::NodeStatus::FAILURE;
  }
  BT::NodeStatus tick() override
  {
    using S = BT::NodeStatus;
    if (children_nodes_.size() != 2) {
      throw BT::RuntimeError("ReplanUntilClear requires planner and follower");
    }
    if (status() == S::IDLE) {
      following_ = false; next_plan_ = now();
      phase_ = "planning"; reason_ = "正在规划；保持停止";
    }
    setStatus(S::RUNNING);
    report(phase_, reason_);
    if (following_) {
      const auto error = fault();
      if (!error.empty()) {return failed("控制器保护停止：" + error);}
      // Consume terminal action results before a planner failure can enter wait.
      const auto follow_result = tickFollower();
      if (follow_result != S::RUNNING) {return follow_result;}
      const auto obstacle = blocked();
      if (!obstacle.empty()) {return wait(obstacle);}
    }
    const bool was_following = following_;
    auto * planner = children_nodes_[0];
    if (planner->status() == S::RUNNING || now() >= next_plan_) {
      const auto result = planner->executeTick();
      if (result == S::FAILURE) {
        return wait("全局路径规划失败；保持停止，每 1 秒重试，路径可用后自动继续");
      }
      if (result == S::SUCCESS) {
        haltChild(0); next_plan_ = now() + std::chrono::seconds(1);
        if (!following_) {
          beginFollow(); following_ = true; phase_ = "following";
          reason_ = "路径已更新，正在跟随；障碍检查持续生效";
          report(phase_, reason_);
        }
      } else if (result != S::RUNNING) {return failed("规划节点返回无效状态");}
    }
    if (!following_ || was_following) {return S::RUNNING;}
    return tickFollower();
  }
  BT::NodeStatus tickFollower()
  {
    using S = BT::NodeStatus;
    const auto result = children_nodes_[1]->executeTick();
    if (result == S::FAILURE) {
      return failed("路径跟随失败；本次目标已停止，请检查控制器与底盘状态");
    }
    if (result == S::SUCCESS) {
      bool final_stage = true;
      getInput("final_stage", final_stage);
      report(final_stage ? "completed" : "planning",
        final_stage ? "目标已到达" : "已到达位置，准备对齐保存的朝向");
      haltChildren(); following_ = false;
      return S::SUCCESS;
    }
    if (result != S::RUNNING) {return failed("路径跟随节点返回无效状态");}
    return S::RUNNING;
  }
  bool following_{false};
  Clock::time_point next_plan_{};
  std::string phase_, reason_;
};
}
