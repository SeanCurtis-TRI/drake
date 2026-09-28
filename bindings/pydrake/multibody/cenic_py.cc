#include "drake/bindings/generated_docstrings/multibody_cenic.h"
#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/cenic/cenic_integrator.h"

namespace drake {
namespace pydrake {

using systems::Context;
using systems::IntegratorBase;
using systems::System;

PYDRAKE_MODULE(cenic, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;
  constexpr auto& doc = pydrake_doc_multibody_cenic.drake.multibody;

  py::module_::import_("pydrake.multibody.contact_solvers");
  py::module_::import_("pydrake.multibody.plant");
  py::module_::import_("pydrake.systems.analysis");

  py::class_<CenicStepStatistics>(
      m, "CenicStepStatistics", doc.CenicStepStatistics.doc)
      .def_readonly("step_type", &CenicStepStatistics::step_type,
          doc.CenicStepStatistics.step_type.doc)
      .def_readonly("time", &CenicStepStatistics::time,
          doc.CenicStepStatistics.time.doc)
      .def_readonly("step_size", &CenicStepStatistics::step_size,
          doc.CenicStepStatistics.step_size.doc)
      .def_readonly("num_solver_iterations",
          &CenicStepStatistics::num_solver_iterations,
          doc.CenicStepStatistics.num_solver_iterations.doc)
      .def_readonly("total_linesearch_iterations",
          &CenicStepStatistics::total_linesearch_iterations,
          doc.CenicStepStatistics.total_linesearch_iterations.doc)
      .def_readonly("max_linesearch_iterations",
          &CenicStepStatistics::max_linesearch_iterations,
          doc.CenicStepStatistics.max_linesearch_iterations.doc)
      .def_readonly("mean_linesearch_iterations",
          &CenicStepStatistics::mean_linesearch_iterations,
          doc.CenicStepStatistics.mean_linesearch_iterations.doc)
      .def_readonly("max_condition_number",
          &CenicStepStatistics::max_condition_number,
          doc.CenicStepStatistics.max_condition_number.doc)
      .def_readonly("last_condition_number",
          &CenicStepStatistics::last_condition_number,
          doc.CenicStepStatistics.last_condition_number.doc)
      .def_readonly("max_e0", &CenicStepStatistics::max_e0,
          doc.CenicStepStatistics.max_e0.doc)
      .def_readonly("mean_e0", &CenicStepStatistics::mean_e0,
          doc.CenicStepStatistics.mean_e0.doc)
      .def_readonly("total_num_constraint_pairs",
          &CenicStepStatistics::total_num_constraint_pairs,
          doc.CenicStepStatistics.total_num_constraint_pairs.doc)
      .def("to_string", &CenicStepStatistics::to_string,
          doc.CenicStepStatistics.to_string.doc);

  auto bind_nonsymbolic_scalar_types = [&m](auto dummy) {
    using T = decltype(dummy);

    DefineTemplateClassWithDefault<CenicIntegrator<T>, IntegratorBase<T>>(
        m, "CenicIntegrator", GetPyParam<T>(), doc.CenicIntegrator.doc)
        .def(py::init<const System<T>&, Context<T>*>(), py::arg("system"),
            py::arg("context") = nullptr,
            // Keep alive, reference: `self` keeps `system` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `context` alive.
            py::keep_alive<1, 3>(), doc.CenicIntegrator.ctor.doc)
        .def("get_solver_parameters",
            &CenicIntegrator<T>::get_solver_parameters,
            doc.CenicIntegrator.get_solver_parameters.doc)
        .def("SetSolverParameters", &CenicIntegrator<T>::SetSolverParameters,
            py::arg("parameters"), doc.CenicIntegrator.SetSolverParameters.doc)
        .def("get_step_statistics", &CenicIntegrator<T>::get_step_statistics,
            py::return_value_policy::reference_internal,
            doc.CenicIntegrator.get_step_statistics.doc);
  };
  type_visit(bind_nonsymbolic_scalar_types, NonSymbolicScalarPack{});
}

}  // namespace pydrake
}  // namespace drake
