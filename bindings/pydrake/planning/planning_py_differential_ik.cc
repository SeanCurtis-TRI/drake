// #include "drake/bindings/pydrake/common/wrap_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/planning/planning_py.h"
// #include "drake/bindings/pydrake/pydrake_pybind.h"
// #include "drake/planning/collision_checker.h"
// #include "drake/planning/distance_and_interpolation_provider.h"
// #include "drake/planning/scene_graph_collision_checker.h"
// #include "drake/planning/unimplemented_collision_checker.h"


namespace drake {
namespace pydrake {
namespace internal {

void DefinePlanningDifferentialIk(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::planning;
  constexpr auto& doc = pydrake_doc.drake.planning;

  using DiffIkSysClass = DifferentialInverseKinematicsSystem;
  constexpr auto& diff_ik_sys_cls_doc = doc.DifferentialInverseKinematicsSystem;
  py::class_<DiffIkSysClass> diff_ik_sys_cls(
      m, "DifferentialInverseKinematicsSystem", diff_ik_sys_cls_doc.doc);
  cls  // BR
      .def(py::init<std::unique_ptr<DiffIkSysClass::Recipe>, std::string_view,
               std::shared_ptr<const CollisionChecker>, const DofMask&,
               double time_step, double K_VX, Vd_TG_limit>(),
          py::arg("recipe"), py::arg("task_frame"),
          py::arg("collision_checker"), py::arg("active_dof"),
          py::arg("time_step"), py::arg("K_VX"), py::arg("Vd_TG_limit"),
          diff_ik_sys_cls_doc.ctor.doc)
      .def("plant", &DiffIkSysClass::plant, py_rvp::reference_internal,
          diff_ik_sys_cls_doc.plant.doc)
      .def("collision_checker", &DiffIkSysClass::collision_checker,
          py_rvp::reference_internal, diff_ik_sys_cls_doc.collision_checker.doc)
      .def("active_dof", &DiffIkSysClass::active_dof,
          py_rvp::reference_internal, diff_ik_sys_cls_doc.active_dof.doc)
      .def("time_step", &DiffIkSysClass::time_step,
          diff_ik_sys_cls_doc.time_step.doc)
      .def("task_frame", &DiffIkSysClass::task_frame,
          py_rvp::reference_internal, diff_ik_sys_cls_doc.task_frame.doc)
      .def("get_input_port_position", &DiffIkSysClass::get_input_port_position,
          py_rvp::reference_internal,
          diff_ik_sys_cls_doc.get_input_port_position.doc)
      .def("get_input_port_nominal_posture",
          &DiffIkSysClass::get_input_port_nominal_posture,
          py_rvp::reference_internal,
          diff_ik_sys_cls_doc.get_input_port_nominal_posture.doc)
      .def("get_input_port_desired_cartesian_poses",
          &DiffIkSysClass::get_input_port_desired_cartesian_poses,
          py_rvp::reference_internal,
          diff_ik_sys_cls_doc.get_input_port_desired_cartesian_poses.doc)
      .def("get_input_port_desired_cartesian_velocities",
          &DiffIkSysClass::plant, py_rvp::reference_internal,
          diff_ik_sys_cls_doc.get_input_port_desired_cartesian_velocities.doc)
      .def("get_output_port_commanded_velocity",
          &DiffIkSysClass::get_output_port_commanded_velocity,
          py_rvp::reference_internal,
          diff_ik_sys_cls_doc.get_output_port_commanded_velocity.doc);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
