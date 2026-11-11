#pragma once

#include <array>
#include <limits>
#include <mutex>
#include <map>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include "tron2_ik/box_qp.hpp"

namespace tron2_ik {

constexpr int kArmDof = 14;
constexpr int kColumnSamples = 58;  // 2 * (7 segments * 4 samples + endpoint)

struct Options {
  int max_iterations = 500;
  int qp_max_iterations = 100;
  double dt = 0.02;
  double damping = 1e-5;
  double step_size = 1.0;
  double max_step_norm = 0.15;
  double tolerance = 1e-5;
  double position_tolerance = 0.0;
  double orientation_tolerance = 0.0;
  double position_cost = 1.0;
  double orientation_cost = 1.0;
  double frame_gain = 1.0;
  double posture_weight = 0.0;
  double posture_gain = 1.0;
  // Lightweight center-column avoidance.  The URDF base_2_Link collision
  // box is checked against sampled arm-link center lines.  The box is
  // inflated by column_link_radius so the samples approximate capsules.
  bool column_avoidance = true;
  double column_activation_distance = 0.04;
  double column_min_distance = 0.01;
  double column_link_radius = 0.04;
  double column_avoidance_weight = 4.0;
  double column_avoidance_gain = 1.0;
  // Optional public-order (left seven, right seven) posture target.  An
  // empty vector means that the seed is used, matching tron2-ik's Python
  // implementation when posture regularization is enabled without an
  // explicit target.
  std::vector<double> posture_target;
  std::array<double, 2> head_position{{0.0, 0.0}};
  double left_grasper_base_position = 0.0;
  double right_grasper_base_position = 0.0;
  double left_gripper_position = 0.0;
  double right_gripper_position = 0.0;
};

struct Result {
  std::vector<double> q;
  // Expanded full URDF configuration.  Keeping this in the native result
  // avoids a second Python -> C++ call after every IK solve.
  std::vector<double> full_q;
  bool success = false;
  int iterations = 0;
  int qp_iterations = 0;
  int qp_active_constraints = 0;
  double solve_time_ms = 0.0;
  std::string solver = "eiquadprog";
  double error = 0.0;
  double left_position_error = 0.0;
  double left_orientation_error = 0.0;
  double right_position_error = 0.0;
  double right_orientation_error = 0.0;
  bool column_collision_free = true;
  double column_min_distance = std::numeric_limits<double>::infinity();
  std::string column_nearest_segment;
  std::string status;
  std::string message;
};

class Model {
 public:
  explicit Model(
      const std::string& urdf_path,
      const std::string& left_frame = "wrist_roll_L_Link",
      const std::string& right_frame = "wrist_roll_R_Link");
  Model(const std::string& urdf_path, const std::string& left_frame,
        const std::string& right_frame, const std::array<double, 16>& base_pose);

  int nq() const noexcept { return kArmDof; }
  int nv() const noexcept { return kArmDof; }
  int fullNq() const noexcept { return full_model_.nq; }
  int fullNv() const noexcept { return full_model_.nv; }
  std::vector<std::array<double, 2>> jointLimits() const;
  std::vector<double> velocityLimits() const;
  std::vector<std::string> frameNames() const;
  std::map<std::string, int> jointCoordinates() const;
  std::vector<double> fullConfiguration(const std::vector<double>& q,
                                      const Options& options);
  std::map<std::string, std::array<double, 16>> framePlacements(
      const std::vector<double>& q, const Options& options);

  std::vector<double> forwardKinematics(
      const std::vector<double>& q_arm, const std::string& side,
      const Options& options = Options{});

  Result solveIK(
      const std::array<double, 16>& left_target,
      const std::array<double, 16>& right_target,
      const std::vector<double>& q_seed,
      const Options& options);

  std::vector<Result> solveIKBatch(
      const std::vector<std::array<double, 16>>& left_targets,
      const std::vector<std::array<double, 16>>& right_targets,
      const std::vector<std::vector<double>>& q_seeds,
      const Options& options);

 private:
  Result solveIKUnlocked(
      const std::array<double, 16>& left_target,
      const std::array<double, 16>& right_target,
      const std::vector<double>& q_seed,
      const Options& options);
  void configureReducedModel(const Options& options);
  Eigen::VectorXd fullConfigurationImpl(const std::vector<double>& q,
                                       const Options& options) const;
  struct Mimic { int child; int parent; double multiplier; double offset; };
  std::vector<Mimic> mimic_;  // Topologically ordered at construction.
  std::string left_frame_name_, right_frame_name_;
  std::array<double, 6> fixed_coordinates_{};
  bool reduced_initialized_ = false;
  Eigen::VectorXd armToReduced(const std::vector<double>& q_arm) const;
  std::vector<double> reducedToArm(const Eigen::VectorXd& q_reduced) const;
  Eigen::Isometry3d poseFromArray(const std::array<double, 16>& values) const;
  std::array<double, 16> poseToArray(const Eigen::Isometry3d& pose) const;
  Eigen::Isometry3d framePose(const Eigen::VectorXd& q, pinocchio::FrameIndex frame);
  void updateKinematics(const Eigen::VectorXd& q);
  void validateOptions(const Options& options) const;
  void validateQ(const std::vector<double>& q, const char* name) const;

  struct PoseErrors {
    Eigen::Matrix<double, 12, 1> task_error;
    double left_position = 0.0;
    double left_orientation = 0.0;
    double right_position = 0.0;
    double right_orientation = 0.0;
    double max_error = 0.0;
  };

  struct ColumnMetrics {
    double min_distance = std::numeric_limits<double>::infinity();
    std::string nearest_segment;
  };

  PoseErrors evaluate(
      const Eigen::VectorXd& q,
      const Eigen::Isometry3d& left_target,
      const Eigen::Isometry3d& right_target);
  ColumnMetrics evaluateColumnAvoidance(
      const Options& options,
      Eigen::Matrix<double, kArmDof, kArmDof>* hessian,
      Eigen::Matrix<double, kArmDof, 1>* rhs,
      Eigen::Matrix<double, kColumnSamples, kArmDof>* constraints = nullptr,
      Eigen::Matrix<double, kColumnSamples, 1>* offsets = nullptr);

  pinocchio::Model full_model_;
  pinocchio::Data full_data_;
  pinocchio::Model reduced_model_;
  pinocchio::Data reduced_data_;
  pinocchio::FrameIndex left_frame_id_{0};
  pinocchio::FrameIndex right_frame_id_{0};
  std::array<std::array<pinocchio::FrameIndex, 8>, 2>
      column_arm_frame_ids_{};
  Eigen::Isometry3d base_pose_{Eigen::Isometry3d::Identity()};
  std::array<std::string, kArmDof> arm_joint_names_{{
      "proximal_pitch_L_Joint",
      "proximal_roll_L_Joint",
      "proximal_yaw_L_Joint",
      "elbow_L_Joint",
      "wrist_yaw_L_Joint",
      "wrist_pitch_L_Joint",
      "wrist_roll_L_Joint",
      "proximal_pitch_R_Joint",
      "proximal_roll_R_Joint",
      "proximal_yaw_R_Joint",
      "elbow_R_Joint",
      "wrist_yaw_R_Joint",
      "wrist_pitch_R_Joint",
      "wrist_roll_R_Joint",
  }};
  std::array<int, kArmDof> arm_q_indices_{};
  std::array<double, kArmDof> lower_limits_{};
  std::array<double, kArmDof> upper_limits_{};
  std::array<double, kArmDof> velocity_limits_{};
  std::array<int, kArmDof> full_q_indices_{};
  BoxQP<kArmDof, kColumnSamples> qp_;
  mutable std::mutex mutex_;
};

}  // namespace tron2_ik
