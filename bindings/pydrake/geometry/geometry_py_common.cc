/* @file This contains all of the various types in the drake::geometry namespace
 that represent the "core" geometry concepts: instances, frames, properties.
 These don't depend on any of geometry's computational structure, but, instead,
 represent the input values to all of those computations. They can be found in
 the pydrake.geometry module. */

#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/common/identifier_pybind.h"
#include "drake/bindings/pydrake/common/serialize_pybind.h"
#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/geometry/geometry_py.h"
#include "drake/common/overloaded.h"
#include "drake/geometry/collision_filter_declaration.h"
#include "drake/geometry/collision_filter_manager.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_properties.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/geometry_version.h"
#include "drake/geometry/in_memory_mesh.h"
#include "drake/geometry/mesh_source.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace pydrake {
namespace {

// NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
using namespace drake::geometry;
constexpr auto& doc = pydrake_doc.drake.geometry;

// TODO(jwnimmer-tri) Reformat this entire file to remove the unnecessary
// indentation.

void DefineCollisionFilterScope(py::module m) {
  {
    using Class = CollisionFilterScope;
    constexpr auto& cls_doc = doc.CollisionFilterScope;
    py::enum_<Class>(m, "CollisionFilterScope", cls_doc.doc)
        .value("kAll", Class::kAll, cls_doc.kAll.doc)
        .value("kOmitDeformable", Class::kOmitDeformable,
            cls_doc.kOmitDeformable.doc);
  }
}

void DefineCollisionFilterDeclaration(py::module m) {
  {
    using Class = CollisionFilterDeclaration;
    constexpr auto& cls_doc = doc.CollisionFilterDeclaration;

    py::class_<Class>(m, "CollisionFilterDeclaration", cls_doc.doc)
        .def(py::init(), cls_doc.ctor.doc)
        .def(py::init<CollisionFilterScope>(), py::arg("scope"),
            cls_doc.ctor.doc)
        .def("AllowBetween", &Class::AllowBetween, py::arg("set_A"),
            py::arg("set_B"), py_rvp::reference, cls_doc.AllowBetween.doc)
        .def("AllowWithin", &Class::AllowWithin, py::arg("geometry_set"),
            py_rvp::reference, cls_doc.AllowWithin.doc)
        .def("ExcludeBetween", &Class::ExcludeBetween, py::arg("set_A"),
            py::arg("set_B"), py_rvp::reference, cls_doc.ExcludeBetween.doc)
        .def("ExcludeWithin", &Class::ExcludeWithin, py::arg("geometry_set"),
            py_rvp::reference, cls_doc.ExcludeWithin.doc);
  }
}

void DefineCollisionFilterManager(py::module m) {
  {
    using Class = CollisionFilterManager;
    constexpr auto& cls_doc = doc.CollisionFilterManager;
    py::class_<Class>(m, "CollisionFilterManager", cls_doc.doc)
        .def("Apply", &Class::Apply, py::arg("declaration"), cls_doc.Apply.doc)
        .def("ApplyTransient", &Class::ApplyTransient, py::arg("declaration"),
            cls_doc.ApplyTransient.doc)
        .def("RemoveDeclaration", &Class::RemoveDeclaration,
            py::arg("filter_id"), cls_doc.RemoveDeclaration.doc)
        .def("has_transient_history", &Class::has_transient_history,
            cls_doc.has_transient_history.doc)
        .def("IsActive", &Class::IsActive, py::arg("filter_id"),
            cls_doc.IsActive.doc);
  }
}

void DefineGeometryFrame(py::module m) {
  {
    using Class = GeometryFrame;
    constexpr auto& cls_doc = doc.GeometryFrame;
    py::class_<Class> cls(m, "GeometryFrame", cls_doc.doc);
    cls  // BR
        .def(py::init<const std::string&, int>(), py::arg("frame_name"),
            py::arg("frame_group_id") = 0, cls_doc.ctor.doc)
        .def("id", &Class::id, cls_doc.id.doc)
        .def("name", &Class::name, cls_doc.name.doc)
        .def("frame_group", &Class::frame_group, cls_doc.frame_group.doc);
    DefCopyAndDeepCopy(&cls);
  }
}

void DefineGeometryInstance(py::module m) {
  {
    using Class = GeometryInstance;
    constexpr auto& cls_doc = doc.GeometryInstance;
    py::class_<Class> cls(m, "GeometryInstance", cls_doc.doc);
    cls  // BR
        .def(py::init<const math::RigidTransform<double>&, const Shape&,
                 const std::string&>(),
            py::arg("X_PG"), py::arg("shape"), py::arg("name"),
            cls_doc.ctor.doc)
        .def("id", &Class::id, cls_doc.id.doc)
        .def("pose", &Class::pose, py_rvp::reference_internal, cls_doc.pose.doc)
        .def(
            "set_pose", &Class::set_pose, py::arg("X_PG"), cls_doc.set_pose.doc)
        .def("shape", &Class::shape, py_rvp::reference_internal,
            cls_doc.shape.doc)
        .def("name", &Class::name, cls_doc.name.doc)
        .def("set_name", &Class::set_name, cls_doc.set_name.doc)
        .def("set_proximity_properties", &Class::set_proximity_properties,
            py::arg("properties"), cls_doc.set_proximity_properties.doc)
        .def("set_illustration_properties", &Class::set_illustration_properties,
            py::arg("properties"), cls_doc.set_illustration_properties.doc)
        .def("set_perception_properties", &Class::set_perception_properties,
            py::arg("properties"), cls_doc.set_perception_properties.doc)
        .def("mutable_proximity_properties",
            &Class::mutable_proximity_properties, py_rvp::reference_internal,
            cls_doc.mutable_proximity_properties.doc)
        .def("proximity_properties", &Class::proximity_properties,
            py_rvp::reference_internal, cls_doc.proximity_properties.doc)
        .def("mutable_illustration_properties",
            &Class::mutable_illustration_properties, py_rvp::reference_internal,
            cls_doc.mutable_illustration_properties.doc)
        .def("illustration_properties", &Class::illustration_properties,
            py_rvp::reference_internal, cls_doc.illustration_properties.doc)
        .def("mutable_perception_properties",
            &Class::mutable_perception_properties, py_rvp::reference_internal,
            cls_doc.mutable_perception_properties.doc)
        .def("perception_properties", &Class::perception_properties,
            py_rvp::reference_internal, cls_doc.perception_properties.doc);
    DefCopyAndDeepCopy(&cls);
  }
}

void DefineGeometryProperties(py::module m) {
  {
    using Class = GeometryProperties;
    constexpr auto& cls_doc = doc.GeometryProperties;
    py::handle abstract_value_cls =
        py::module::import("pydrake.common.value").attr("AbstractValue");
    py::class_<Class>(m, "GeometryProperties", cls_doc.doc)
        .def("HasGroup", &Class::HasGroup, py::arg("group_name"),
            cls_doc.HasGroup.doc)
        .def("num_groups", &Class::num_groups, cls_doc.num_groups.doc)
        .def(
            "GetPropertiesInGroup",
            [](const Class& self, const std::string& group_name) {
              py::dict out;
              py::object py_self = py::cast(&self, py_rvp::reference);
              for (auto& [name, abstract] :
                  self.GetPropertiesInGroup(group_name)) {
                out[name.c_str()] = py::cast(
                    abstract.get(), py_rvp::reference_internal, py_self);
              }
              return out;
            },
            py::arg("group_name"), cls_doc.GetPropertiesInGroup.doc)
        .def("GetGroupNames", &Class::GetGroupNames, cls_doc.GetGroupNames.doc)
        .def(
            "AddProperty",
            [abstract_value_cls](Class& self, const std::string& group_name,
                const std::string& name, py::object value) {
              py::object abstract = abstract_value_cls.attr("Make")(value);
              self.AddPropertyAbstract(
                  group_name, name, abstract.cast<const AbstractValue&>());
            },
            py::arg("group_name"), py::arg("name"), py::arg("value"),
            cls_doc.AddProperty.doc)
        .def(
            "UpdateProperty",
            [abstract_value_cls](Class& self, const std::string& group_name,
                const std::string& name, py::object value) {
              py::object abstract = abstract_value_cls.attr("Make")(value);
              self.UpdatePropertyAbstract(
                  group_name, name, abstract.cast<const AbstractValue&>());
            },
            py::arg("group_name"), py::arg("name"), py::arg("value"),
            cls_doc.UpdateProperty.doc)
        .def("HasProperty", &Class::HasProperty, py::arg("group_name"),
            py::arg("name"), cls_doc.HasProperty.doc)
        .def(
            "GetProperty",
            [](const Class& self, const std::string& group_name,
                const std::string& name) {
              py::object abstract =
                  py::cast(self.GetPropertyAbstract(group_name, name),
                      py_rvp::reference);
              return abstract.attr("get_value")();
            },
            py::arg("group_name"), py::arg("name"), cls_doc.GetProperty.doc)
        .def(
            "GetPropertyOrDefault",
            [](const Class& self, const std::string& group_name,
                const std::string& name, py::object default_value) {
              // For now, ignore typing. This is less efficient, but eh, it's
              // Python.
              if (self.HasProperty(group_name, name)) {
                py::object py_self = py::cast(&self, py_rvp::reference);
                return py_self.attr("GetProperty")(group_name, name);
              } else {
                return default_value;
              }
            },
            py::arg("group_name"), py::arg("name"), py::arg("default_value"),
            cls_doc.GetPropertyOrDefault.doc)
        .def("RemoveProperty", &Class::RemoveProperty, py::arg("group_name"),
            py::arg("name"), cls_doc.RemoveProperty.doc)
        .def_static("default_group_name", &Class::default_group_name,
            cls_doc.default_group_name.doc)
        .def(
            "__str__",
            [](const Class& self) {
              std::stringstream ss;
              ss << self;
              return ss.str();
            },
            "Returns formatted string.");
  }
}

void DefineGeometrySet(py::module m) {
  {
    using Class = GeometrySet;
    constexpr auto& cls_doc = doc.GeometrySet;
    constexpr char extra_ctor_doc[] = "See main constructor";
    // N.B. For containers, we use `std::vector<>` rather than abstract
    // iterators / containers.
    py::class_<Class>(m, "GeometrySet", cls_doc.doc)
        .def(py::init(), cls_doc.ctor.doc)
        .def(py::init<GeometryId>(), py::arg("geometry_id"), extra_ctor_doc)
        .def(py::init<FrameId>(), py::arg("frame_id"), extra_ctor_doc)
        .def(py::init([](std::vector<GeometryId> geometry_ids) {
          return Class(geometry_ids);
        }),
            py::arg("geometry_ids"), extra_ctor_doc)
        .def(py::init([](std::vector<FrameId> frame_ids) {
          return Class(frame_ids);
        }),
            py::arg("frame_ids"), extra_ctor_doc)
        .def(py::init([](std::vector<GeometryId> geometry_ids,
                          std::vector<FrameId> frame_ids) {
          return Class(geometry_ids, frame_ids);
        }),
            py::arg("geometry_ids"), py::arg("frame_ids"), extra_ctor_doc)
        .def(
            "Add",
            [](Class* self, const GeometryId& geometry_id) {
              self->Add(geometry_id);
            },
            py::arg("geometry_id"), cls_doc.Add.doc)
        .def(
            "Add",
            [](Class* self, const FrameId& frame_id) { self->Add(frame_id); },
            py::arg("frame_id"), cls_doc.Add.doc)
        .def(
            "Add",
            [](Class* self, std::vector<GeometryId> geometry_ids) {
              self->Add(geometry_ids);
            },
            py::arg("geometry_ids"), extra_ctor_doc)
        .def(
            "Add",
            [](Class* self, std::vector<FrameId> frame_ids) {
              self->Add(frame_ids);
            },
            py::arg("frame_ids"), extra_ctor_doc)
        .def(
            "Add",
            [](Class* self, std::vector<GeometryId> geometry_ids,
                std::vector<FrameId> frame_ids) {
              self->Add(geometry_ids, frame_ids);
            },
            py::arg("geometry_ids"), py::arg("frame_ids"), extra_ctor_doc);
  }
}

void DefineGeometryVersion(py::module m) {
  {
    using Class = GeometryVersion;
    constexpr auto& cls_doc = doc.GeometryVersion;
    py::class_<Class> cls(m, "GeometryVersion", cls_doc.doc);
    cls.def(py::init(), cls_doc.ctor.doc)
        .def(py::init<const GeometryVersion&>(), py::arg("other"),
            "Creates a copy of the GeometryVersion.")
        .def("IsSameAs", &Class::IsSameAs, py::arg("other"), py::arg("role"),
            cls_doc.IsSameAs.doc);
    DefCopyAndDeepCopy(&cls);
  }
}

void DefineIdentifiers(py::module m) {
  {
    BindIdentifier<FilterId>(m, "FilterId", doc.FilterId.doc);
    BindIdentifier<SourceId>(m, "SourceId", doc.SourceId.doc);
    BindIdentifier<FrameId>(m, "FrameId", doc.FrameId.doc);
    BindIdentifier<GeometryId>(m, "GeometryId", doc.GeometryId.doc);
  }
}

void DefineInMemoryMesh(py::module m) {
  {
    using Class = InMemoryMesh;
    constexpr auto& cls_doc = doc.InMemoryMesh;
    py::class_<Class> cls(m, "InMemoryMesh", cls_doc.doc);
    py::object ctor = m.attr("InMemoryMesh");
    cls  // BR
        .def(ParamInit<Class>())
        .def_readwrite("mesh_file", &Class::mesh_file, cls_doc.mesh_file.doc)
        .def_readwrite("supporting_files", &Class::supporting_files,
            cls_doc.supporting_files.doc)
        .def(py::pickle(
            [](const InMemoryMesh& self) {
              return py::dict(py::arg("mesh_file") = self.mesh_file,
                  py::arg("supporting_files") = self.supporting_files);
            },
            [ctor](const py::dict& kwargs) {
              return ctor(**kwargs).cast<InMemoryMesh>();
            }));
    // Note: __repr__ is defined in _geometry_extra.py.
    DefCopyAndDeepCopy(&cls);
  }
}

void DefineGeometryPropertiesSubclasses(py::module m) {
  {
    py::class_<IllustrationProperties, GeometryProperties> cls(
        m, "IllustrationProperties", doc.IllustrationProperties.doc);
    cls  // BR
        .def(py::init(), doc.IllustrationProperties.ctor.doc)
        .def(py::init<const GeometryProperties&>(), py::arg("other"),
            "Creates a copy of the properties");
    DefCopyAndDeepCopy(&cls);
  }
  {
    py::class_<PerceptionProperties, GeometryProperties> cls(
        m, "PerceptionProperties", doc.PerceptionProperties.doc);
    cls  // BR
        .def(py::init(), doc.PerceptionProperties.ctor.doc)
        .def(py::init<const GeometryProperties&>(), py::arg("other"),
            "Creates a copy of the properties");
    DefCopyAndDeepCopy(&cls);
  }
  {
    py::class_<ProximityProperties, GeometryProperties> cls(
        m, "ProximityProperties", doc.ProximityProperties.doc);
    cls  // BR
        .def(py::init(), doc.ProximityProperties.ctor.doc)
        .def(py::init<const GeometryProperties&>(), py::arg("other"),
            "Creates a copy of the properties");
    DefCopyAndDeepCopy(&cls);
  }
}

void DefineMeshSource(py::module m) {
  {
    using Class = MeshSource;
    constexpr auto& cls_doc = doc.MeshSource;
    py::class_<Class> cls(m, "MeshSource", cls_doc.doc);
    py::object ctor = m.attr("MeshSource");
    cls  // BR
        .def(py::init<std::filesystem::path>(), py::arg("path"),
            cls_doc.ctor.doc_1args_path)
        .def(py::init<InMemoryMesh>(), py::arg("mesh"),
            cls_doc.ctor.doc_1args_mesh)
        .def(py::init<const MeshSource&>(), py::arg("other"))
        .def("is_path", &Class::is_path, cls_doc.is_path.doc)
        .def("is_in_memory", &Class::is_in_memory, cls_doc.is_in_memory.doc)
        .def("description", &Class::description, cls_doc.description.doc)
        .def("extension", &Class::extension, cls_doc.extension.doc)
        .def("path", &Class::path, cls_doc.path.doc)
        .def("in_memory", &Class::in_memory, py_rvp::reference_internal,
            cls_doc.in_memory.doc)
        .def(py::pickle(
            [](const MeshSource& self) {
              if (self.is_path()) {
                return py::dict(py::arg("path") = self.path());
              }
              DRAKE_DEMAND(self.is_in_memory());
              return py::dict(py::arg("mesh") = self.in_memory());
            },
            [ctor](const py::dict& kwargs) {
              return ctor(**kwargs).cast<MeshSource>();
            }));
    // Note: __repr__ is defined in _geometry_extra.py.
    DefCopyAndDeepCopy(&cls);

    py::implicitly_convertible<std::filesystem::path, MeshSource>();
    py::implicitly_convertible<InMemoryMesh&&, MeshSource>();
  }
}

void DefineRgba(py::module m) {
  {
    using Class = Rgba;
    constexpr auto& cls_doc = doc.Rgba;
    py::class_<Class> cls(m, "Rgba", cls_doc.doc);
    cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc_0args)
        .def(py::init<double, double, double, double>(), py::arg("r"),
            py::arg("g"), py::arg("b"), py::arg("a") = 1.0,
            cls_doc.ctor.doc_4args)
        .def("r", &Class::r, cls_doc.r.doc)
        .def("g", &Class::g, cls_doc.g.doc)
        .def("b", &Class::b, cls_doc.b.doc)
        .def("a", &Class::a, cls_doc.a.doc)
        .def("set",
            py::overload_cast<double, double, double, double>(&Class::set),
            py::arg("r"), py::arg("g"), py::arg("b"), py::arg("a") = 1.0,
            cls_doc.set.doc_4args)
        .def("set",
            py::overload_cast<const Eigen::Ref<const Eigen::VectorXd>&>(
                &Class::set),
            py::arg("rgba"), cls_doc.set.doc_1args)
        .def("update", &Class::update, py::arg("r") = py::none(),
            py::arg("g") = py::none(), py::arg("b") = py::none(),
            py::arg("a") = py::none(), cls_doc.update.doc)
        .def(py::self == py::self)
        .def(py::self != py::self)
        .def(py::self * py::self)
        .def("scale_rgb", &Class::scale_rgb, py::arg("scale"),
            cls_doc.scale_rgb.doc)
        .def("__repr__", [](const Class& self) {
          return py::str("Rgba(r={}, g={}, b={}, a={})")
              .format(self.r(), self.g(), self.b(), self.a());
        });
    DefAttributesUsingSerialize(&cls);
    cls.def_property("rgba",
        // The Serialize-based binding skips the validity checking; we'll
        // add it back here by re-binding the property getter and setter.
        &Class::rgba,
        py::overload_cast<const Eigen::Ref<const Eigen::VectorXd>&>(
            &Class::set),
        "The RGBA value as a property (as np.ndarray).");
    DefCopyAndDeepCopy(&cls);
    AddValueInstantiation<Rgba>(m);
  }
}

void DefineRole(py::module m) {
  {
    constexpr auto& cls_doc = doc.Role;
    py::enum_<Role>(m, "Role", py::arithmetic(), cls_doc.doc)
        .value("kUnassigned", Role::kUnassigned, cls_doc.kUnassigned.doc)
        .value("kProximity", Role::kProximity, cls_doc.kProximity.doc)
        .value("kIllustration", Role::kIllustration, cls_doc.kIllustration.doc)
        .value("kPerception", Role::kPerception, cls_doc.kPerception.doc);
  }
}

void DefineRoleAssign(py::module m) {
  {
    constexpr auto& cls_doc = doc.RoleAssign;
    using Class = RoleAssign;
    py::enum_<Class>(m, "RoleAssign", cls_doc.doc)
        .value("kNew", Class::kNew, cls_doc.kNew.doc)
        .value("kReplace", Class::kReplace, cls_doc.kReplace.doc);
  }
}

void DefineShapes(py::module m) {
  // Shape constructors.
  {
    py::class_<Shape> shape_cls(m, "Shape", doc.Shape.doc);
    shape_cls  // BR
        .def("__repr__", [](const Shape& self) { return self.to_string(); });
    DefClone(&shape_cls);

    py::class_<Box, Shape>(m, "Box", doc.Box.doc)
        .def(py::init<double, double, double>(), py::arg("width"),
            py::arg("depth"), py::arg("height"), doc.Box.ctor.doc_3args)
        .def(py::init<const Vector3<double>&>(), py::arg("measures"),
            doc.Box.ctor.doc_1args)
        .def("width", &Box::width, doc.Box.width.doc)
        .def("depth", &Box::depth, doc.Box.depth.doc)
        .def("height", &Box::height, doc.Box.height.doc)
        .def("size", &Box::size, py_rvp::reference_internal, doc.Box.size.doc)
        .def(py::pickle(
            [](const Box& self) {
              return std::make_tuple(self.width(), self.depth(), self.height());
            },
            [](std::tuple<double, double, double> dims) {
              return Box(
                  std::get<0>(dims), std::get<1>(dims), std::get<2>(dims));
            }));

    py::class_<Capsule, Shape>(m, "Capsule", doc.Capsule.doc)
        .def(py::init<double, double>(), py::arg("radius"), py::arg("length"),
            doc.Capsule.ctor.doc_2args)
        .def(py::init<const Vector2<double>&>(), py::arg("measures"),
            doc.Capsule.ctor.doc_1args)
        .def("radius", &Capsule::radius, doc.Capsule.radius.doc)
        .def("length", &Capsule::length, doc.Capsule.length.doc)
        .def(py::pickle(
            [](const Capsule& self) {
              return std::make_pair(self.radius(), self.length());
            },
            [](std::pair<double, double> dims) {
              return Capsule(dims.first, dims.second);
            }));

    // Note: meshes used to get pickled with a single scale value (representing
    // uniform scale). While we now allow for non-uniform scaling, we still
    // need to depickle the old representation. So, we need to handle both
    // scale representations.
    using ScaleVariant = std::variant<double, Vector3<double>>;

    py::class_<Convex, Shape> convex_cls(m, "Convex", doc.Convex.doc);
    convex_cls
        .def(py::init<std::filesystem::path, double>(), py::arg("filename"),
            py::arg("scale") = 1.0, doc.Convex.ctor.doc_2args_filename_scale)
        .def(py::init<std::filesystem::path, const Vector3<double>&>(),
            py::arg("filename"), py::arg("scale3"),
            doc.Convex.ctor.doc_2args_filename_scale3)
        .def(py::init<InMemoryMesh, double>(), py::arg("mesh_data"),
            py::arg("scale") = 1.0, doc.Convex.ctor.doc_2args_mesh_data_scale)
        .def(py::init<InMemoryMesh, const Vector3<double>&>(),
            py::arg("mesh_data"), py::arg("scale3"),
            doc.Convex.ctor.doc_2args_mesh_data_scale3)
        .def(py::init<MeshSource, double>(), py::arg("source"),
            py::arg("scale") = 1.0, doc.Convex.ctor.doc_2args_source_scale)
        .def(py::init<MeshSource, const Vector3<double>&>(), py::arg("source"),
            py::arg("scale3"), doc.Convex.ctor.doc_2args_source_scale3)
        .def(py::init<const Eigen::Matrix3X<double>&, const std::string&,
                 double>(),
            py::arg("points"), py::arg("label"), py::arg("scale") = 1.0,
            doc.Convex.ctor.doc_3args_points_label_scale)
        .def(py::init<const Eigen::Matrix3X<double>&, const std::string&,
                 const Vector3<double>&>(),
            py::arg("points"), py::arg("label"), py::arg("scale3"),
            doc.Convex.ctor.doc_3args_points_label_scale3)
        .def("source", &Convex::source, doc.Convex.source.doc)
        .def("extension", &Convex::extension, doc.Convex.extension.doc)
        .def("scale", &Convex::scale, doc.Convex.scale.doc)
        .def("scale3", &Convex::scale3, doc.Convex.scale3.doc)
        .def("GetConvexHull", &Convex::GetConvexHull,
            doc.Convex.GetConvexHull.doc)
        .def(py::pickle(
            [](const Convex& self) {
              return std::make_pair(self.source(), ScaleVariant(self.scale3()));
            },
            [](std::pair<MeshSource, ScaleVariant> info) {
              return std::visit<Convex>(overloaded{[&info](auto&& scale) {
                return Convex(std::move(info.first), scale);
              }},
                  info.second);
            }));
    // Note: Convex.__repr__ is redefined in _geometry_extra.py;
    // Shape::to_string() does not properly condition strings for python.

    py::class_<Cylinder, Shape>(m, "Cylinder", doc.Cylinder.doc)
        .def(py::init<double, double>(), py::arg("radius"), py::arg("length"),
            doc.Cylinder.ctor.doc_2args)
        .def(py::init<const Vector2<double>&>(), py::arg("measures"),
            doc.Cylinder.ctor.doc_1args)
        .def("radius", &Cylinder::radius, doc.Cylinder.radius.doc)
        .def("length", &Cylinder::length, doc.Cylinder.length.doc)
        .def(py::pickle(
            [](const Cylinder& self) {
              return std::make_pair(self.radius(), self.length());
            },
            [](std::pair<double, double> dims) {
              return Cylinder(dims.first, dims.second);
            }));

    py::class_<Ellipsoid, Shape>(m, "Ellipsoid", doc.Ellipsoid.doc)
        .def(py::init<double, double, double>(), py::arg("a"), py::arg("b"),
            py::arg("c"), doc.Ellipsoid.ctor.doc_3args)
        .def(py::init<const Vector3<double>&>(), py::arg("measures"),
            doc.Ellipsoid.ctor.doc_1args)
        .def("a", &Ellipsoid::a, doc.Ellipsoid.a.doc)
        .def("b", &Ellipsoid::b, doc.Ellipsoid.b.doc)
        .def("c", &Ellipsoid::c, doc.Ellipsoid.c.doc)
        .def(py::pickle(
            [](const Ellipsoid& self) {
              return std::make_tuple(self.a(), self.b(), self.c());
            },
            [](std::tuple<double, double, double> dims) {
              return Ellipsoid(
                  std::get<0>(dims), std::get<1>(dims), std::get<2>(dims));
            }));

    py::class_<HalfSpace, Shape>(m, "HalfSpace", doc.HalfSpace.doc)
        .def(py::init<>(), doc.HalfSpace.ctor.doc)
        .def_static("MakePose", &HalfSpace::MakePose, py::arg("Hz_dir_F"),
            py::arg("p_FB"), doc.HalfSpace.MakePose.doc);

    py::class_<Mesh, Shape> mesh_cls(m, "Mesh", doc.Mesh.doc);
    mesh_cls
        .def(py::init<std::filesystem::path, double>(), py::arg("filename"),
            py::arg("scale") = 1.0, doc.Mesh.ctor.doc_2args_filename_scale)
        .def(py::init<std::filesystem::path, const Vector3<double>&>(),
            py::arg("filename"), py::arg("scale3"),
            doc.Mesh.ctor.doc_2args_filename_scale3)
        .def(py::init<InMemoryMesh, double>(), py::arg("mesh_data"),
            py::arg("scale") = 1.0, doc.Mesh.ctor.doc_2args_mesh_data_scale)
        .def(py::init<InMemoryMesh, const Vector3<double>&>(),
            py::arg("mesh_data"), py::arg("scale3"),
            doc.Mesh.ctor.doc_2args_mesh_data_scale3)
        .def(py::init<MeshSource, double>(), py::arg("source"),
            py::arg("scale") = 1.0, doc.Mesh.ctor.doc_2args_source_scale)
        .def(py::init<MeshSource, const Vector3<double>&>(), py::arg("source"),
            py::arg("scale3"), doc.Mesh.ctor.doc_2args_source_scale3)
        .def("source", &Mesh::source, doc.Mesh.source.doc)
        .def("extension", &Mesh::extension, doc.Mesh.extension.doc)
        .def("scale", &Mesh::scale, doc.Mesh.scale.doc)
        .def("scale3", &Mesh::scale3, doc.Mesh.scale3.doc)
        .def("GetConvexHull", &Mesh::GetConvexHull, doc.Mesh.GetConvexHull.doc)
        .def(py::pickle(
            [](const Mesh& self) {
              return std::make_pair(self.source(), ScaleVariant(self.scale3()));
            },
            [](std::pair<MeshSource, ScaleVariant> info) {
              return std::visit<Mesh>(overloaded{[&info](auto&& scale) {
                return Mesh(std::move(info.first), scale);
              }},
                  info.second);
            }));
    // Note: Mesh.__repr__ is redefined in _geometry_extra.py;
    // Shape::to_string() does not properly condition strings for python.

    py::class_<Sphere, Shape>(m, "Sphere", doc.Sphere.doc)
        .def(py::init<double>(), py::arg("radius"), doc.Sphere.ctor.doc)
        .def("radius", &Sphere::radius, doc.Sphere.radius.doc)
        .def(py::pickle([](const Sphere& self) { return self.radius(); },
            [](const double radius) { return Sphere(radius); }));

    py::class_<MeshcatCone, Shape>(m, "MeshcatCone", doc.MeshcatCone.doc)
        .def(py::init<double, double, double>(), py::arg("height"),
            py::arg("a") = 1.0, py::arg("b") = 1.0,
            doc.MeshcatCone.ctor.doc_3args)
        .def(py::init<const Vector3<double>&>(), py::arg("measures"),
            doc.MeshcatCone.ctor.doc_1args)
        .def("height", &MeshcatCone::height, doc.MeshcatCone.height.doc)
        .def("a", &MeshcatCone::a, doc.MeshcatCone.a.doc)
        .def("b", &MeshcatCone::b, doc.MeshcatCone.b.doc)
        .def(py::pickle(
            [](const MeshcatCone& self) {
              return std::make_tuple(self.height(), self.a(), self.b());
            },
            [](std::tuple<double, double, double> params) {
              return MeshcatCone(std::get<0>(params), std::get<1>(params),
                  std::get<2>(params));
            }));
  }
}

void DefineMiscFunctions(py::module m) {
  m.def("CalcVolume", &CalcVolume, py::arg("shape"), doc.CalcVolume.doc);

  m.def("MakePhongIllustrationProperties", &MakePhongIllustrationProperties,
      py_rvp::reference_internal, py::arg("diffuse"),
      doc.MakePhongIllustrationProperties.doc);

  m.def("AddContactMaterial",
      py::overload_cast<std::optional<double>, std::optional<double>,
          const std::optional<multibody::CoulombFriction<double>>&,
          ProximityProperties*>(&AddContactMaterial),
      py::arg("dissipation"), py::arg("point_stiffness"), py::arg("friction"),
      py::arg("properties"), doc.AddContactMaterial.doc);
  // The C++ function does not offer default arguments, but it's convenient to
  // default the optional arguments to None in Python because a caller can use
  // named arguments to disambiguate which arguments get which values.
  m.def(
      "AddContactMaterial",
      [](ProximityProperties* properties, std::optional<double> dissipation,
          std::optional<double> point_stiffness,
          const std::optional<multibody::CoulombFriction<double>>& friction) {
        AddContactMaterial(dissipation, point_stiffness, friction, properties);
      },
      py::arg("properties"), py::arg("dissipation") = std::nullopt,
      py::arg("point_stiffness") = std::nullopt,
      py::arg("friction") = std::nullopt, doc.AddContactMaterial.doc);

  // TODO(SeanCurtis-TRI): Decompose this in some meaningful way.
  // NOLINTNEXTLINE(readability/fn_size)
}

// Test-only code.
namespace testing {
// For use with `test_geometry_properties_cpp_types`.
template <typename T>
void DefGetPropertyCpp(py::module m) {
  auto func = [](const geometry::GeometryProperties& properties,
                  const std::string& group, const std::string& name) {
    return properties.GetProperty<T>(group, name);
  };
  AddTemplateFunction(m, "GetPropertyCpp", func, GetPyParam<T>());
}

// For use with test_proximity_properties. The hydroelastic compliance type is
// internal. But we want to test that the compliance type has been successfully
// defined in set of properties. If we ever move HydroelasticType out of
// internal and bind it, we can eliminate this helper.
//
// Return true if the properties indicate being compliant, false if rigid, and
// throws if the property isn't set at all (or set to undefined).
bool PropertiesIndicateCompliantHydro(
    const geometry::ProximityProperties& props) {
  using geometry::internal::HydroelasticType;
  const HydroelasticType hydro_type =
      props.GetPropertyOrDefault(geometry::internal::kHydroGroup,
          geometry::internal::kComplianceType, HydroelasticType::kUndefined);
  if (hydro_type == HydroelasticType::kUndefined) {
    throw std::runtime_error("No specification of rigid or compliant");
  }
  return hydro_type == HydroelasticType::kSoft;
}

void def_testing_module(py::module m) {
  // The get_constant_id() returns a fresh object every time, but always with
  // the same underlying get_value().
  const auto constant_id = geometry::FilterId::get_new_id();
  m.def("get_constant_id", [constant_id]() { return constant_id; });

  m.def("PropertiesIndicateCompliantHydro", &PropertiesIndicateCompliantHydro);

  // For use with `test_geometry_properties_cpp_types`.
  DefGetPropertyCpp<std::string>(m);
  DefGetPropertyCpp<bool>(m);
  DefGetPropertyCpp<double>(m);
}
}  // namespace testing

}  // namespace

void DefineGeometryCommon(py::module m) {
  m.doc() = "Bindings for `drake::geometry`";

  // This list must remain in topological dependency order.
  DefineIdentifiers(m);
  DefineRole(m);
  DefineRoleAssign(m);
  DefineRgba(m);
  DefineGeometryFrame(m);
  DefineGeometryProperties(m);
  DefineGeometryPropertiesSubclasses(m);
  DefineInMemoryMesh(m);
  DefineMeshSource(m);
  DefineShapes(m);
  DefineGeometrySet(m);
  DefineCollisionFilterScope(m);
  DefineCollisionFilterDeclaration(m);
  DefineCollisionFilterManager(m);
  DefineGeometryInstance(m);
  DefineGeometryVersion(m);
  DefineMiscFunctions(m);

  testing::def_testing_module(m.def_submodule("_testing"));
}

}  // namespace pydrake
}  // namespace drake
