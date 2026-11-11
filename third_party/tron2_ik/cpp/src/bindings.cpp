#include "tron2_ik/solver.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(_native, module) {
  module.doc() = "C++ Pinocchio + constrained eiquadprog dual-arm IK";
  // Both this extension and libtron2rt compile the same kernel. Local type
  // registration allows both to be imported without duplicate-type errors.
  auto options = py::class_<tron2_ik::Options>(module, "Options", py::module_local()).def(py::init<>());
#define OPTION(name) options.def_readwrite(#name, &tron2_ik::Options::name)
  OPTION(max_iterations); OPTION(qp_max_iterations); OPTION(dt); OPTION(damping);
  OPTION(step_size); OPTION(max_step_norm); OPTION(tolerance);
  OPTION(position_tolerance); OPTION(orientation_tolerance);
  OPTION(position_cost); OPTION(orientation_cost); OPTION(frame_gain);
  OPTION(posture_weight); OPTION(posture_gain); OPTION(posture_target);
  OPTION(column_avoidance); OPTION(column_activation_distance); OPTION(column_min_distance);
  OPTION(column_link_radius); OPTION(column_avoidance_weight); OPTION(column_avoidance_gain);
  OPTION(head_position); OPTION(left_grasper_base_position); OPTION(right_grasper_base_position);
  OPTION(left_gripper_position); OPTION(right_gripper_position);
#undef OPTION
  auto result = py::class_<tron2_ik::Result>(module, "Result", py::module_local());
#define RESULT(name) result.def_readonly(#name, &tron2_ik::Result::name)
  RESULT(q); RESULT(full_q); RESULT(success); RESULT(iterations); RESULT(qp_iterations);
  RESULT(qp_active_constraints); RESULT(solve_time_ms); RESULT(solver); RESULT(error);
  RESULT(left_position_error); RESULT(right_position_error);
  RESULT(left_orientation_error); RESULT(right_orientation_error);
  RESULT(column_collision_free); RESULT(column_min_distance); RESULT(column_nearest_segment);
  RESULT(status); RESULT(message);
#undef RESULT
  py::class_<tron2_ik::Model>(module, "Model", py::module_local())
      .def(py::init<const std::string&, const std::string&, const std::string&>())
      .def(py::init<const std::string&, const std::string&, const std::string&, const std::array<double, 16>&>())
      .def_property_readonly("nq", &tron2_ik::Model::nq)
      .def_property_readonly("nv", &tron2_ik::Model::nv)
      .def_property_readonly("full_nq", &tron2_ik::Model::fullNq)
      .def_property_readonly("full_nv", &tron2_ik::Model::fullNv)
      .def_property_readonly("joint_limits", &tron2_ik::Model::jointLimits)
      .def_property_readonly("velocity_limits", &tron2_ik::Model::velocityLimits)
      .def_property_readonly("frame_names", &tron2_ik::Model::frameNames)
      .def_property_readonly("joint_coordinates", &tron2_ik::Model::jointCoordinates)
      .def("full_configuration", &tron2_ik::Model::fullConfiguration, py::call_guard<py::gil_scoped_release>())
      .def("frame_placements", &tron2_ik::Model::framePlacements, py::call_guard<py::gil_scoped_release>())
      .def("forward_kinematics", &tron2_ik::Model::forwardKinematics, py::call_guard<py::gil_scoped_release>())
      .def("solve_ik", &tron2_ik::Model::solveIK, py::call_guard<py::gil_scoped_release>())
      .def("solve_ik_batch", &tron2_ik::Model::solveIKBatch,
           py::call_guard<py::gil_scoped_release>());
}
