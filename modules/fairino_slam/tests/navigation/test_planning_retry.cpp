// Real BT v3 scheduler and deployed XML, fake actions and monotonic clock.
// Never creates ROS nodes or contacts a robot. Arguments: tree1.xml tree2.xml.
#include <chrono>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "fairino_lookahead_controller/replan.hpp"

using Status = BT::NodeStatus;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

void require(bool condition, const std::string &message) {
  if (!condition) throw std::runtime_error(message);
}
struct Scenario {
  std::vector<Status> outcomes;
  std::vector<Clock::time_point> attempts;
  std::vector<std::string> planned_goals;
  int plan_halts{0}, follow_starts{0}, follow_ticks{0}, follow_halts{0};
  bool follow_fails{false};
  std::string followed_path, phase, reason, obstacle, error;
  Clock::time_point time{};
  bool follow_succeeds{false};
};
class Planner : public BT::StatefulActionNode {
 public:
  Planner(const std::string &name, const BT::NodeConfiguration &config, Scenario *scenario)
      : BT::StatefulActionNode(name, config), scenario_(scenario) {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("goal"), BT::InputPort<std::string>("goals"),
            BT::InputPort<std::string>("planner_id"), BT::OutputPort<std::string>("path")};
  }
  Status onStart() override {
    auto index = scenario_->attempts.size();
    scenario_->attempts.push_back(scenario_->time);
    std::string goal; getInput("goal", goal); scenario_->planned_goals.push_back(goal);
    result_ = index < scenario_->outcomes.size() ? scenario_->outcomes[index] : Status::FAILURE;
    return Status::RUNNING;
  }
  Status onRunning() override {
    if (result_ != Status::RUNNING) {
      // Humble ComputePath actions clear their blackboard output on abort.
      setOutput("path", result_ == Status::SUCCESS
                           ? "path-" + std::to_string(scenario_->attempts.size()) : "");
    }
    return result_;
  }
  void onHalted() override {
    ++scenario_->plan_halts;
    setOutput("path", std::string());
  }
 private:
  Scenario *scenario_;
  Status result_{Status::RUNNING};
};
class Follow : public BT::StatefulActionNode {
 public:
  Follow(const std::string &name, const BT::NodeConfiguration &config, Scenario *scenario)
      : BT::StatefulActionNode(name, config), scenario_(scenario) {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("path"), BT::InputPort<std::string>("controller_id"),
            BT::InputPort<std::string>("goal_checker_id")};
  }
  Status onStart() override {
    ++scenario_->follow_starts;
    getInput("path", scenario_->followed_path);
    require(!scenario_->followed_path.empty(), "FollowPath started without a successful plan");
    return Status::RUNNING;
  }
  Status onRunning() override {
    ++scenario_->follow_ticks;
    std::string path;
    getInput("path", path);
    // Matches Humble FollowPath's explicit rejection of empty replacement paths.
    if (!path.empty()) scenario_->followed_path = path;
    return scenario_->follow_fails ? Status::FAILURE :
      scenario_->follow_succeeds ? Status::SUCCESS : Status::RUNNING;
  }
  void onHalted() override { ++scenario_->follow_halts; }
 private:
  Scenario *scenario_;
};
class Scheduler : public fairino_lookahead_controller::ReplanUntilClear {
 public:
  Scheduler(const std::string & name, const BT::NodeConfiguration & config, Scenario * s)
  : ReplanUntilClear(name, config), s_(s) {}
 protected:
  Clock::time_point now() const override {return s_->time;}
  std::string blocked() override {return s_->obstacle;}
  std::string fault() override {return s_->error;}
  void beginFollow() override {s_->obstacle.clear(); s_->error.clear();}
  void report(const std::string & phase, const std::string & reason) override {
    s_->phase = phase; s_->reason = reason;
  }
 private:
  Scenario * s_;
};
class Capture : public BT::SyncActionNode {
 public:
  Capture(const std::string &n, const BT::NodeConfiguration &c) : BT::SyncActionNode(n,c) {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("goal"), BT::OutputPort<std::string>("travel_goal")};
  }
  Status tick() override {
    std::string goal; getInput("goal", goal); setOutput("travel_goal",goal+"/travel");
    return Status::SUCCESS;
  }
};
struct Fixture {
  Scenario scenario;
  BT::BehaviorTreeFactory factory;
  BT::Tree tree;
  Fixture(const std::string &file, std::vector<Status> outcomes) {
    scenario.outcomes = std::move(outcomes);
    factory.registerNodeType<Capture>("CaptureTravelGoal");
    factory.registerBuilder<Scheduler>("ReplanUntilClear",
      [this](const std::string &n, const BT::NodeConfiguration &c) {
        return std::make_unique<Scheduler>(n, c, &scenario);
      });
    factory.registerBuilder<Planner>("ComputePathToPose", [this](const std::string &n, const BT::NodeConfiguration &c) {
      return std::make_unique<Planner>(n, c, &scenario);
    });
    factory.registerBuilder<Planner>("ComputePathThroughPoses", [this](const std::string &n, const BT::NodeConfiguration &c) {
      return std::make_unique<Planner>(n, c, &scenario);
    });
    factory.registerBuilder<Follow>("FollowPath", [this](const std::string &n, const BT::NodeConfiguration &c) {
      return std::make_unique<Follow>(n, c, &scenario);
    });
    auto board = BT::Blackboard::create();
    board->set("goal", std::string("target"));
    board->set("goals", std::string("targets"));
    board->set("path", std::string());
    tree = factory.createTreeFromFile(file, board);
  }
  Status step(std::chrono::milliseconds elapsed=50ms) {
    scenario.time += elapsed;
    return tree.tickRoot();
  }
  Status until(const std::function<bool(Status)> &done, int ticks=200) {
    for (int i=0;i<ticks;++i) {
      auto status=step();
      if (done(status)) return status;
      require(status==Status::RUNNING, "Unexpected tree completion");
    }
    throw std::runtime_error("Test timeout");
  }
};

void check(const std::string &file) {
  {
    Fixture f(file, {Status::SUCCESS});
    f.until([&](Status) {return f.scenario.follow_starts==1;});
    require(f.scenario.attempts.size()==1, "Immediate plan was retried");
    require(f.scenario.time.time_since_epoch()<=100ms, "Initial plan was delayed");
    f.scenario.follow_succeeds=true;
    require(f.step()==Status::SUCCESS, "Goal did not complete");
    f.tree.haltTree();
    require(f.scenario.phase=="completed", "Completion reason overwritten on halt");
  }
  {
    // Long human blockage exceeds the old three-attempt limit and progress timeout.
    Fixture f(file, std::vector<Status>(80,Status::FAILURE));
    f.scenario.outcomes.push_back(Status::SUCCESS);
    f.until([&](Status) {return f.scenario.attempts.size()==80;}, 2000);
    require(f.scenario.follow_starts==0, "Followed an invalid path");
    require(f.scenario.phase=="waiting", "No waiting reason");
    f.until([&](Status) {return f.scenario.follow_starts==1;});
    require(f.scenario.attempts.size()==81, "Did not resume after long blockage");
    for(size_t i=1;i<f.scenario.attempts.size();++i)
      require(f.scenario.attempts[i]-f.scenario.attempts[i-1]>=1s, "Retry interval below 1 second");
  }
  {
    Fixture f(file, {Status::SUCCESS,Status::FAILURE,Status::FAILURE,Status::SUCCESS});
    f.until([&](Status) {return f.scenario.phase=="waiting";});
    require(f.scenario.follow_starts==1 && f.scenario.follow_halts==1,
            "Failed replan did not cancel old follower immediately");
    auto ticks=f.scenario.follow_ticks;
    f.until([&](Status) {return f.scenario.attempts.size()==3;});
    require(f.scenario.follow_ticks==ticks, "Old path followed while waiting");
    f.until([&](Status) {return f.scenario.follow_starts==2;});
    require(f.scenario.followed_path=="path-4", "Resumed with stale path");
  }
  {
    Fixture f(file, {Status::SUCCESS,Status::SUCCESS});
    f.until([&](Status) {return f.scenario.follow_starts==1;});
    f.scenario.obstacle="local collision";
    require(f.step()==Status::RUNNING, "Local collision aborted navigation");
    require(f.scenario.follow_halts==1 && f.scenario.reason=="local collision",
            "Collision did not pause and preserve reason");
    f.until([&](Status) {return f.scenario.follow_starts==2;});
    require(f.scenario.followed_path=="path-2", "Collision recovery used old path");
  }
  {
    Fixture f(file, {Status::SUCCESS,Status::RUNNING});
    f.until([&](Status) {return f.scenario.attempts.size()==2;});
    require(f.scenario.follow_ticks>0, "Periodic healthy replanning stopped movement");
    f.scenario.follow_fails=true;
    require(f.step()==Status::FAILURE, "Controller failure was swallowed");
    require(f.scenario.plan_halts==1, "Controller failure left planner active");
  }
  {
    Fixture f(file, {Status::SUCCESS,Status::FAILURE});
    f.until([&](Status) {return f.scenario.attempts.size()==2;});
    f.scenario.follow_fails=true;
    require(f.step()==Status::FAILURE, "Simultaneous planner failure hid controller abort");
    require(f.scenario.phase=="failed", "Controller abort entered obstacle retry");
  }
  {
    Fixture f(file, {Status::SUCCESS});
    f.until([&](Status) {return f.scenario.follow_starts==1;});
    f.scenario.error="stale TF";
    f.scenario.obstacle="local collision";
    require(f.step()==Status::FAILURE, "Fault was hidden by obstacle wait");
    require(f.scenario.reason.find("stale TF")!=std::string::npos, "Fault reason lost");
    f.tree.haltTree();
    require(f.scenario.phase=="failed", "Failure reason overwritten");
  }
  {
    Fixture f(file, {});
    f.until([&](Status) {return f.scenario.phase=="waiting";});
    f.tree.haltTree();
    auto count=f.scenario.attempts.size();
    f.scenario.time+=60s;
    require(f.scenario.attempts.size()==count && f.scenario.phase=="canceled",
            "Halted delay dispatched another action");
    f.step();
    require(f.scenario.attempts.size()==count+1, "Reuse inherited old retry timer");
    f.step(); auto failed=f.scenario.time;
    f.step(999ms);
    require(f.scenario.attempts.size()==count+1, "Reuse lost its full retry interval");
    f.step(1ms);
    require(f.scenario.attempts.back()-failed>=1s, "Reuse timer is stale");
  }
  {
    Fixture f(file, {Status::RUNNING});
    f.step(); f.tree.haltTree();
    require(f.scenario.plan_halts==1 && f.scenario.follow_starts==0,
            "Cancel failed to halt pending planning");
  }
  {
    Fixture f(file, {Status::SUCCESS,Status::RUNNING});
    f.until([&](Status) {return f.scenario.attempts.size()==2;});
    f.tree.haltTree();
    require(f.scenario.plan_halts==1 && f.scenario.follow_halts==1,
            "Cancel failed to halt both concurrent actions");
  }
  std::cout<<"PASS "<<file<<": immediate, indefinite retry, stop/resume, local collision, fault, halt/reuse\n";
}

void check_saved(const std::string & file) {
  {
    Fixture f(file, {Status::SUCCESS, Status::SUCCESS});
    f.until([&](Status) {return f.scenario.follow_starts==1;});
    require(f.scenario.planned_goals.front()=="target/travel", "Travel did not use captured heading");
    f.scenario.follow_succeeds=true;
    require(f.step()==Status::RUNNING, "Position completion ended saved-pose navigation");
    require(f.scenario.phase!="completed", "Intermediate stage emitted terminal state");
    f.scenario.follow_succeeds=false;
    f.until([&](Status) {return f.scenario.follow_starts==2;});
    require(f.scenario.planned_goals.back()=="target", "Alignment lost original saved goal");
    require(f.tree.rootBlackboard()->get<std::string>("goal")=="target", "Original goal was modified");
    f.scenario.follow_succeeds=true;
    require(f.step()==Status::SUCCESS && f.scenario.phase=="completed", "Alignment did not complete goal");
  }
  {
    Fixture f(file, {Status::SUCCESS, Status::SUCCESS});
    f.until([&](Status) {return f.scenario.follow_starts==1;});
    f.tree.haltTree();
    require(f.scenario.phase=="canceled" && f.scenario.follow_starts==1,
            "Cancel started arrival alignment");
  }
  {
    Fixture f(file, {Status::SUCCESS, Status::FAILURE, Status::FAILURE,
                    Status::FAILURE, Status::FAILURE, Status::SUCCESS});
    f.until([&](Status) {return f.scenario.follow_starts==1;});
    f.scenario.follow_succeeds=true;
    f.step();
    f.scenario.follow_succeeds=false;
    f.until([&](Status) {return f.scenario.follow_starts==2;}, 160);
    require(f.scenario.attempts.size()==6, "Alignment did not retain unlimited retry");
    require(f.scenario.planned_goals.back()=="target", "Retry changed saved heading");
    f.tree.haltTree();
    require(f.scenario.phase=="canceled", "Cancel failed during alignment");
  }
  std::cout<<"PASS saved pose: stage ordering, no intermediate completion, original goal, cancel and retry\n";
}

int main(int argc,char **argv) {
  try {
    require(argc==3 || argc==4,"Arguments: tree1.xml tree2.xml [saved_pose.xml]");
    check(argv[1]); check(argv[2]); if (argc==4) {check_saved(argv[3]);} return 0;
  } catch(const std::exception &e) {
    std::cerr<<"FAIL: "<<e.what()<<'\n'; return 1;
  }
}
