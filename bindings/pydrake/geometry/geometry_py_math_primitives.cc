/* @file This contains all of the various types in the drake::geometry namespace
 that represent the "core" geometry concepts: instances, frames, properties.
 These don't depend on any of geometry's computational structure, but, instead,
 represent the input values to all of those computations. They can be found in
 the pydrake.geometry module. */

#include <string>
#include <utility>
#include <vector>

#include "drake/bindings/generated_docstrings/geometry.h"
#include "drake/bindings/generated_docstrings/geometry_proximity.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/type_pack.h"
#include "drake/bindings/pydrake/geometry/geometry_py.h"
#include "drake/geometry/proximity/plane.h"

namespace drake {
namespace pydrake {
namespace {

// NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
using namespace drake::geometry;
constexpr auto& doc = pydrake_doc_geometry_proximity.drake.geometry;

template <typename T>
void DefinePlane(py::module m, T) {
  {
    using Class = Plane<T>;
    constexpr auto& cls_doc = doc.Plane;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "Plane", GetPyParam<T, T>(), cls_doc.doc);
    cls  // BR
        .def(py::init<const Vector3<T>&, const Vector3<T>&, bool>(),
            py::arg("normal"), py::arg("point_on_plane"),
            py::arg("already_normalized") = false, cls_doc.ctor.doc)
        .def(
            "CalcHeight",
            [](const Class* self, const Vector3<T>& Q) {
              return self->CalcHeight(Q);
            },
            py::arg("Q"), cls_doc.CalcHeight.doc)
        .def("normal", &Class::normal, py_rvp::reference_internal,
            cls_doc.normal.doc)
        .def("point_on_plane", &Class::point_on_plane,
            cls_doc.point_on_plane.doc)
        .def(py::pickle(
            [](const Class& self) {
              return std::make_tuple(self.normal(), self.point_on_plane());
            },
            [](std::tuple<Vector3<T>, Vector3<T>> data) {
              Vector3<T> normal = std::get<0>(data);
              Vector3<T> point_on_plane = std::get<1>(data);
              return Class(normal, point_on_plane);
            }));
    DefCopyAndDeepCopy(&cls);
  }
}

}  // namespace

void DefineGeometryMathPrimitives(py::module m) {
  m.doc() = "Bindings for `drake::geometry`";

  // This list must remain in topological dependency order.
  type_visit(
      [m](auto dummy) { DefinePlane(m, dummy); }, NonSymbolicScalarPack{});
}

}  // namespace pydrake
}  // namespace drake
