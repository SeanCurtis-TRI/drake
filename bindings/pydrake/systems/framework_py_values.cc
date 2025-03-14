#include "drake/bindings/pydrake/systems/framework_py_values.h"

#include <sstream>

#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/type_pack.h"
#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/bus_value.h"
#include "drake/systems/framework/subvector.h"
#include "drake/systems/framework/supervector.h"

using std::string;

namespace drake {
namespace pydrake {

namespace {
template <typename T>
void DoScalarDependentDefinitions(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  constexpr auto& doc = pydrake_doc.drake.systems;

  // Value types.
  DefineTemplateClassWithDefault<VectorBase<T>>(
      m, "VectorBase", GetPyParam<T>(), doc.VectorBase.doc)
      .def("__str__",
          [](const VectorBase<T>& self) {
            // Print out list directly.
            return py::str(py::cast(self.CopyToVector()).attr("tolist")());
          })
      .def("__repr__",
          [](const VectorBase<T>& self) {
            py::handle cls = py::cast(&self, py_rvp::reference).get_type();
            return py::str("{}({})").format(internal::PrettyClassName(cls),
                py::cast(self.CopyToVector()).attr("tolist")());
          })
      .def("__getitem__",
          overload_cast_explicit<const T&, int>(&VectorBase<T>::operator[]),
          py_rvp::reference_internal, doc.VectorBase.operator_array.doc)
      .def(
          "__setitem__",
          [](VectorBase<T>& self, int index, T& value) { self[index] = value; },
          doc.VectorBase.operator_array.doc)
      .def("size", &VectorBase<T>::size, doc.VectorBase.size.doc)
      .def("GetAtIndex",
          overload_cast_explicit<T&, int>(&VectorBase<T>::GetAtIndex),
          py::arg("index"), py_rvp::reference_internal,
          doc.VectorBase.GetAtIndex.doc)
      .def("SetAtIndex", &VectorBase<T>::SetAtIndex, py::arg("index"),
          py::arg("value"), doc.VectorBase.SetAtIndex.doc)
      .def("SetFrom", &VectorBase<T>::SetFrom, py::arg("value"),
          doc.VectorBase.SetFrom.doc)
      .def("SetFromVector", &VectorBase<T>::SetFromVector, py::arg("value"),
          doc.VectorBase.SetFromVector.doc)
      .def("SetZero", &VectorBase<T>::SetZero, doc.VectorBase.SetZero.doc)
      .def("CopyToVector", &VectorBase<T>::CopyToVector,
          doc.VectorBase.CopyToVector.doc)
      .def("PlusEqScaled",
          overload_cast_explicit<VectorBase<T>&, const T&,
              const VectorBase<T>&>(&VectorBase<T>::PlusEqScaled),
          py::arg("scale"), py::arg("rhs"), py_rvp::reference_internal,
          doc.VectorBase.PlusEqScaled.doc_2args);

  // TODO(eric.cousineau): Make a helper function for the Eigen::Ref<>
  // patterns.
  auto basic_vector =
      DefineTemplateClassWithDefault<BasicVector<T>, VectorBase<T>>(
          m, "BasicVector", GetPyParam<T>(), doc.BasicVector.doc);
  DefClone(&basic_vector);
  basic_vector
      // N.B. Place `init<VectorX<T>>` `init<int>` so that we do not
      // implicitly convert scalar-size `np.array` objects to `int` (since
      // this is normally permitted).
      .def(py::init<VectorX<T>>(), py::arg("data"),
          doc.BasicVector.ctor.doc_1args_vec)
      .def(
          py::init<int>(), py::arg("size"), doc.BasicVector.ctor.doc_1args_size)
      .def(
          "set_value",
          [](BasicVector<T>* self, const Eigen::Ref<const VectorX<T>>& value) {
            self->set_value(value);
          },
          doc.BasicVector.set_value.doc)
      .def(
          "value",
          [](const BasicVector<T>* self) -> const VectorX<T>& {
            return self->value();
          },
          py_rvp::reference_internal, doc.BasicVector.value.doc)
      .def(
          "get_value",
          [](const BasicVector<T>* self) -> Eigen::Ref<const VectorX<T>> {
            return self->get_value();
          },
          py_rvp::reference_internal, doc.BasicVector.get_value.doc)
      // TODO(eric.cousineau): Remove this once `get_value` is changed, or
      // reference semantics are changed for custom dtypes.
      .def("_get_value_copy",
          [](const BasicVector<T>* self) -> VectorX<T> {
            return self->get_value();
          })
      .def(
          "get_mutable_value",
          [](BasicVector<T>* self) -> Eigen::Ref<VectorX<T>> {
            return self->get_mutable_value();
          },
          py_rvp::reference_internal, doc.BasicVector.get_mutable_value.doc);

  DefineTemplateClassWithDefault<Supervector<T>, VectorBase<T>>(
      m, "Supervector", GetPyParam<T>(), doc.Supervector.doc);

  DefineTemplateClassWithDefault<Subvector<T>, VectorBase<T>>(
      m, "Subvector", GetPyParam<T>(), doc.Subvector.doc);

  AddValueInstantiation<BasicVector<T>>(m)  // BR
      .def(
          "set_value",
          [](Value<BasicVector<T>>* self, const T& value) {
            self->set_value(BasicVector<T>{value});
          },
          py::arg("value"))
      .def(
          "set_value",
          [](Value<BasicVector<T>>* self,
              const Eigen::Ref<const VectorX<T>>& value) {
            self->set_value(BasicVector<T>(value));
          },
          py::arg("value"));
}

void DefineBusValue(py::module m) {
  using Class = systems::BusValue;
  constexpr auto& cls_doc = pydrake_doc.drake.systems.BusValue;
  py::class_<Class> cls(m, "BusValue", cls_doc.doc);
  cls  // BR
      .def(py::init<>(), cls_doc.ctor.doc)
      .def(
          "Find",
          // Similar to OutputPort::Eval, instead of binding the function
          // call directly we'll amend it to unwrap the AbstractValue.
          [](const Class& self, std::string_view name) -> py::object {
            const AbstractValue* found_cxx = self.Find(name);
            if (found_cxx == nullptr) {
              return py::none();
            }
            py::object found_py = py::cast(
                found_cxx, py_rvp::reference_internal, py::cast(&self));
            return found_py.attr("get_value")();
          },
          py::arg("name"), cls_doc.Find.doc)
      .def("Clear", &Class::Clear, cls_doc.Clear.doc)
      .def("Set", &Class::Set, py::arg("name"), py::arg("value"),
          cls_doc.Set.doc)
      // To behave like a dict in Python, we'll bind __iterator__ to traverse
      // our keys (i.e., our names) and __getitem__ to look up by name. Unlike
      // the binding for Find() which returns None when not found, __getitem__
      // must raise a KeyError instead.
      .def(
          "__iter__",
          [](const Class& self) {
            return py::make_key_iterator(self.begin(), self.end());
          },
          // Keep alive, reference: `return` keeps `self` alive.
          py::keep_alive<0, 1>())
      .def(
          "__getitem__",
          [](py::object self, py::str key) -> py::object {
            py::object result = self.attr("Find")(key);
            if (result.is_none()) {
              throw py::key_error(std::string{key});
            }
            return result;
          },
          py::arg("key"));
  DefCopyAndDeepCopy(&cls);
  AddValueInstantiation<Class>(m);
}

}  // namespace

void DefineFrameworkPyValues(py::module m) {
  DefineBusValue(m);
  // Do templated instantiations.
  auto bind_common_scalar_types = [m](auto dummy) {
    using T = decltype(dummy);
    DoScalarDependentDefinitions<T>(m);
  };
  type_visit(bind_common_scalar_types, CommonScalarPack{});
}

}  // namespace pydrake
}  // namespace drake
