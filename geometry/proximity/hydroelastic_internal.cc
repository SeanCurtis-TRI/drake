#include "drake/geometry/proximity/hydroelastic_internal.h"

#include <algorithm>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include <fmt/format.h>

#include "drake/common/text_logging.h"
#include "drake/geometry/proximity/inflate_mesh.h"
#include "drake/geometry/proximity/make_box_field.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/geometry/proximity/make_capsule_field.h"
#include "drake/geometry/proximity/make_capsule_mesh.h"
#include "drake/geometry/proximity/make_convex_field.h"
#include "drake/geometry/proximity/make_convex_hull_mesh_impl.h"
#include "drake/geometry/proximity/make_convex_mesh.h"
#include "drake/geometry/proximity/make_cylinder_field.h"
#include "drake/geometry/proximity/make_cylinder_mesh.h"
#include "drake/geometry/proximity/make_ellipsoid_field.h"
#include "drake/geometry/proximity/make_ellipsoid_mesh.h"
#include "drake/geometry/proximity/make_mesh_field.h"
#include "drake/geometry/proximity/make_mesh_from_vtk.h"
#include "drake/geometry/proximity/make_sphere_field.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"
#include "drake/geometry/proximity/mesh_to_vtk.h"
#include "drake/geometry/proximity/obj_to_surface_mesh.h"
#include "drake/geometry/proximity/polygon_to_triangle_mesh.h"
#include "drake/geometry/proximity/tessellation_strategy.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace hydroelastic {
namespace {

VolumeMesh<double> RemoveNegativeVolumes(const VolumeMesh<double>& mesh) {
  std::vector<VolumeElement> tets;
  for (int e = 0; e < mesh.num_elements(); ++e) {
    const double vol = mesh.CalcTetrahedronVolume(e);
    if (vol > 0) {
      tets.push_back(mesh.element(e));
    }
  }
  std::vector<Vector3<double>> verts = mesh.vertices();

  return VolumeMesh<double>(std::move(tets), std::move(verts));
}

// Decide whether a shape is primitive for vanished-checking purposes. (See
// Geometries::is_vanished() documentation).  This reifier expects that
// user_data will point to a single boolean flag, which is pre-set to `true`.
class IsPrimitiveChecker final : public ShapeReifier {
 private:
  using ShapeReifier::ImplementGeometry;

  // Primitives are the default. The handler here does nothing; rely on the
  // caller to have pre-set the user_data flag to true.
  void DefaultImplementGeometry(const Shape&) final {}

  // Non-primitives.

  void ImplementGeometry(const Convex&, void* user_data) final {
    *static_cast<bool*>(user_data) = false;
  }

  void ImplementGeometry(const HalfSpace&, void* user_data) final {
    *static_cast<bool*>(user_data) = false;
  }

  void ImplementGeometry(const Mesh&, void* user_data) final {
    *static_cast<bool*>(user_data) = false;
  }

  void ImplementGeometry(const MeshcatCone&, void*) final {
    DRAKE_UNREACHABLE();
  }
};

bool is_primitive(const Shape& shape) {
  bool result{true};  // The reifier default is to assume primitive.
  IsPrimitiveChecker checker;
  shape.Reify(&checker, &result);
  return result;
}

}  // namespace

using std::make_unique;

CompliantMesh::CompliantMesh(
    std::unique_ptr<VolumeMesh<double>> mesh,
    std::unique_ptr<VolumeMeshFieldLinear<double, double>> pressure,
    std::unique_ptr<TriangleSurfaceMesh<double>> collision_mesh)
    : mesh_(std::move(mesh)),
      pressure_(std::move(pressure)),
      bvh_(std::make_unique<Bvh<Obb, VolumeMesh<double>>>(*mesh_)),
      collision_mesh_(std::move(collision_mesh)) {
  DRAKE_ASSERT(mesh_.get() == &pressure_->mesh());
  tri_to_tet_ = std::make_unique<std::vector<TetFace>>();
  surface_mesh_ = std::make_unique<TriangleSurfaceMesh<double>>(
      ConvertVolumeToSurfaceMeshWithBoundaryVertices(*mesh_, nullptr,
                                                     tri_to_tet_.get()));
  surface_mesh_bvh_ =
      std::make_unique<Bvh<Obb, TriangleSurfaceMesh<double>>>(*surface_mesh_);
  mesh_topology_ = std::make_unique<VolumeMeshTopology>(*mesh_);

  if (collision_mesh_ != nullptr) {
    // Build the topology of the DynamicBVHs.
    collision_mesh_vertex_bvh_ = std::make_unique<DynamicBvh>(
        collision_mesh_->num_vertices(), [this](int i) -> Aabb {
          const Vector3<double>& v = collision_mesh_->vertex(i);
          return Aabb(v, Vector3<double>::Zero());
        });
    collision_mesh_edge_bvh_ = std::make_unique<DynamicBvh>(
        collision_mesh_->num_edges(), [this](int i) -> Aabb {
          const auto [v0_idx, v1_idx] = collision_mesh_->edge(i);
          const Vector3<double>& v0 = collision_mesh_->vertex(v0_idx);
          const Vector3<double>& v1 = collision_mesh_->vertex(v1_idx);
          Vector3<double> min_corner = v0.cwiseMin(v1);
          Vector3<double> max_corner = v0.cwiseMax(v1);

          return Aabb((min_corner + max_corner) / 2,
                      (max_corner - min_corner) / 2);
        });
    collision_mesh_face_bvh_ = std::make_unique<DynamicBvh>(
        collision_mesh_->num_elements(), [this](int i) -> Aabb {
          const SurfaceTriangle& tri = collision_mesh_->element(i);
          const Vector3<double>& v0 = collision_mesh_->vertex(tri.vertex(0));
          const Vector3<double>& v1 = collision_mesh_->vertex(tri.vertex(1));
          const Vector3<double>& v2 = collision_mesh_->vertex(tri.vertex(2));

          Vector3<double> min_corner = v0.cwiseMin(v1).cwiseMin(v2);
          Vector3<double> max_corner = v0.cwiseMax(v1).cwiseMax(v2);

          return Aabb((min_corner + max_corner) / 2,
                      (max_corner - min_corner) / 2);
        });
  }
}

CompliantMesh& CompliantMesh::operator=(const CompliantMesh& s) {
  if (this == &s) return *this;

  mesh_ = make_unique<VolumeMesh<double>>(s.mesh());
  // We can't simply copy the mesh field; the copy must contain a pointer to
  // the new mesh. So, we use CloneAndSetMesh() instead.
  pressure_ = s.pressure().CloneAndSetMesh(mesh_.get());
  bvh_ = make_unique<Bvh<Obb, VolumeMesh<double>>>(s.bvh());
  surface_mesh_ =
      std::make_unique<TriangleSurfaceMesh<double>>(s.surface_mesh());
  tri_to_tet_ = std::make_unique<std::vector<TetFace>>(s.tri_to_tet());
  surface_mesh_bvh_ = std::make_unique<Bvh<Obb, TriangleSurfaceMesh<double>>>(
      s.surface_mesh_bvh());
  mesh_topology_ = std::make_unique<VolumeMeshTopology>(s.mesh_topology());
  if (s.has_collision_mesh()) {
    collision_mesh_ =
        make_unique<TriangleSurfaceMesh<double>>(s.collision_mesh());
    collision_mesh_vertex_bvh_ =
        std::make_unique<DynamicBvh>(s.collision_mesh_vertex_bvh());
    collision_mesh_edge_bvh_ =
        std::make_unique<DynamicBvh>(s.collision_mesh_edge_bvh());
    collision_mesh_face_bvh_ =
        std::make_unique<DynamicBvh>(s.collision_mesh_face_bvh());
  }
  return *this;
}

Geometries::~Geometries() = default;

HydroelasticType Geometries::hydroelastic_type(GeometryId id) const {
  auto iter = supported_geometries_.find(id);
  if (iter != supported_geometries_.end()) return iter->second;
  return HydroelasticType::kUndefined;
}

bool Geometries::is_vanished(GeometryId id) const {
  return vanished_geometries_.contains(id);
}

void Geometries::RemoveGeometry(GeometryId id) {
  supported_geometries_.erase(id);
  compliant_geometries_.erase(id);
  rigid_geometries_.erase(id);
}

void Geometries::MaybeAddGeometry(const Shape& shape, GeometryId id,
                                  const ProximityProperties& properties) {
  const HydroelasticType type = properties.GetPropertyOrDefault(
      kHydroGroup, kComplianceType, HydroelasticType::kUndefined);
  if (type != HydroelasticType::kUndefined) {
    ReifyData data{type, id, properties};
    shape.Reify(this, &data);
  }
}

void Geometries::ImplementGeometry(const Box& box, void* user_data) {
  MakeShape(box, *static_cast<ReifyData*>(user_data));
}

void Geometries::ImplementGeometry(const Capsule& capsule, void* user_data) {
  MakeShape(capsule, *static_cast<ReifyData*>(user_data));
}

void Geometries::ImplementGeometry(const Convex& convex, void* user_data) {
  MakeShape(convex, *static_cast<ReifyData*>(user_data));
}

void Geometries::ImplementGeometry(const Cylinder& cylinder, void* user_data) {
  MakeShape(cylinder, *static_cast<ReifyData*>(user_data));
}

void Geometries::ImplementGeometry(const Ellipsoid& ellipsoid,
                                   void* user_data) {
  MakeShape(ellipsoid, *static_cast<ReifyData*>(user_data));
}

void Geometries::ImplementGeometry(const HalfSpace& half_space,
                                   void* user_data) {
  MakeShape(half_space, *static_cast<ReifyData*>(user_data));
}

void Geometries::ImplementGeometry(const Mesh& mesh, void* user_data) {
  MakeShape(mesh, *static_cast<ReifyData*>(user_data));
}

void Geometries::ImplementGeometry(const Sphere& sphere, void* user_data) {
  MakeShape(sphere, *static_cast<ReifyData*>(user_data));
}

template <typename ShapeType>
void Geometries::MakeShape(const ShapeType& shape, const ReifyData& data) {
  switch (data.type) {
    case HydroelasticType::kRigid: {
      auto hydro_geometry = MakeRigidRepresentation(shape, data.properties);
      if (hydro_geometry) AddGeometry(data.id, std::move(*hydro_geometry));
    } break;
    case HydroelasticType::kCompliant: {
      auto hydro_geometry = MakeCompliantRepresentation(shape, data.properties);
      if (hydro_geometry) {
        if (is_primitive(shape) &&
            hydro_geometry->pressure_field().is_gradient_field_degenerate()) {
          vanished_geometries_.insert(data.id);
        } else {
          AddGeometry(data.id, std::move(*hydro_geometry));
        }
      }
    } break;
    case HydroelasticType::kUndefined:
      // No action required.
      break;
  }
}

void Geometries::AddGeometry(GeometryId id, CompliantGeometry geometry) {
  DRAKE_DEMAND(hydroelastic_type(id) == HydroelasticType::kUndefined);
  supported_geometries_[id] = HydroelasticType::kCompliant;
  compliant_geometries_.insert({id, std::move(geometry)});
}

void Geometries::AddGeometry(GeometryId id, RigidGeometry geometry) {
  DRAKE_DEMAND(hydroelastic_type(id) == HydroelasticType::kUndefined);
  supported_geometries_[id] = HydroelasticType::kRigid;
  rigid_geometries_.insert({id, std::move(geometry)});
}

namespace {

// Validator interface for use with extracting valid properties. It is
// instantiated with shape (e.g., "Sphere", "Box", etc.) and compliance (i.e.,
// "rigid" or "compliant") strings (to help give intelligible error messages)
// and then attempts to extract a typed value from a set of proximity properties
// -- spewing meaningful error messages based on absence, type mismatch, and
// invalid values.
template <typename ValueType>
class Validator {
 public:
  // Parameters `shape_name` and `compliance` are only for error messages.
  Validator(const char* shape_name, const char* compliance)
      : shape_name_(shape_name), compliance_(compliance) {}

  virtual ~Validator() = default;

  // Extract an arbitrary property from the proximity properties. If no default
  // value is given (`default_value == std::nullopt)`, throws a
  // consistent error message in the case of missing or mis-typed properties.
  // Otherwise, the default value is used in place of the missing property.
  // Relies on the ValidateValue() method to validate the value.
  ValueType Extract(const ProximityProperties& props, const char* group_name,
                    const char* property_name,
                    std::optional<ValueType> default_value = std::nullopt) {
    const std::string full_property_name =
        fmt::format("('{}', '{}')", group_name, property_name);
    const bool has_default = default_value.has_value();
    if (!has_default && !props.HasProperty(group_name, property_name)) {
      throw std::logic_error(
          fmt::format("Cannot create {} {}; missing the {} property",
                      compliance(), shape_name(), full_property_name));
    }
    const ValueType value =
        has_default ? props.GetPropertyOrDefault(group_name, property_name,
                                                 *default_value)
                    : props.GetProperty<ValueType>(group_name, property_name);
    ValidateValue(value, full_property_name);
    return value;
  }

 protected:
  const char* shape_name() const { return shape_name_; }
  const char* compliance() const { return compliance_; }

  // Does the work of validating the given value. Sub-classes should throw if
  // the provided value is not valid. The first parameter is the value to
  // validate; the second is the full name of the property.
  virtual void ValidateValue(const ValueType&, const std::string&) const {}

 private:
  const char* shape_name_{};
  const char* compliance_{};
};

// Validator that extracts *strictly positive doubles*.
class PositiveDouble : public Validator<double> {
 public:
  using Validator<double>::Validator;

 protected:
  void ValidateValue(const double& value,
                     const std::string& property) const override {
    if (!(value > 0)) {
      throw std::logic_error(
          fmt::format("Cannot create {} {}; the {} property must be positive",
                      compliance(), shape_name(), property));
    }
  }
};

// Validator that extracts *non-negative doubles*, where a zero value is valid.
// In case of missing property, a value of zero is returned.
class NonNegativeDouble : public Validator<double> {
 public:
  using Validator<double>::Validator;

 protected:
  void ValidateValue(const double& value,
                     const std::string& property) const override {
    if (!(value >= 0)) {
      throw std::logic_error(fmt::format(
          "Cannot create {} {}; the {} property must be non-negative",
          compliance(), shape_name(), property));
    }
  }
};

}  // namespace

void WarnNoRigidRepresentation(std::string_view shape_type_name) {
  static const logging::Warn log_once(
      "Rigid {} shapes are not currently supported for hydroelastic "
      "contact; registration is allowed, but an error will be thrown "
      "during contact.",
      shape_type_name);
}

std::optional<RigidGeometry> MakeRigidRepresentation(
    const HalfSpace& hs, const ProximityProperties&) {
  return RigidGeometry(hs);
}

std::optional<RigidGeometry> MakeRigidRepresentation(
    const Sphere& sphere, const ProximityProperties& props) {
  PositiveDouble validator("Sphere", "rigid");
  const double edge_length = validator.Extract(props, kHydroGroup, kRezHint);
  auto mesh = make_unique<TriangleSurfaceMesh<double>>(
      MakeSphereSurfaceMesh<double>(sphere, edge_length));

  return RigidGeometry(RigidMesh(std::move(mesh)));
}

std::optional<RigidGeometry> MakeRigidRepresentation(
    const Box& box, const ProximityProperties&) {
  auto mesh = make_unique<TriangleSurfaceMesh<double>>(
      MakeBoxSurfaceMeshWithSymmetricTriangles<double>(box));
  return RigidGeometry(RigidMesh(std::move(mesh)));
}

std::optional<RigidGeometry> MakeRigidRepresentation(
    const Cylinder& cylinder, const ProximityProperties& props) {
  PositiveDouble validator("Cylinder", "rigid");
  const double edge_length = validator.Extract(props, kHydroGroup, kRezHint);
  auto mesh = make_unique<TriangleSurfaceMesh<double>>(
      MakeCylinderSurfaceMesh<double>(cylinder, edge_length));

  return RigidGeometry(RigidMesh(std::move(mesh)));
}

std::optional<RigidGeometry> MakeRigidRepresentation(
    const Capsule& capsule, const ProximityProperties& props) {
  PositiveDouble validator("Capsule", "rigid");
  const double edge_length = validator.Extract(props, kHydroGroup, kRezHint);
  auto mesh = make_unique<TriangleSurfaceMesh<double>>(
      MakeCapsuleSurfaceMesh<double>(capsule, edge_length));

  return RigidGeometry(RigidMesh(std::move(mesh)));
}

std::optional<RigidGeometry> MakeRigidRepresentation(
    const Ellipsoid& ellipsoid, const ProximityProperties& props) {
  PositiveDouble validator("Ellipsoid", "rigid");
  const double edge_length = validator.Extract(props, kHydroGroup, kRezHint);
  auto mesh = make_unique<TriangleSurfaceMesh<double>>(
      MakeEllipsoidSurfaceMesh<double>(ellipsoid, edge_length));

  return RigidGeometry(RigidMesh(std::move(mesh)));
}

std::optional<RigidGeometry> MakeRigidRepresentation(
    const Mesh& mesh_spec, const ProximityProperties&) {
  // Mesh does not use any properties.
  std::unique_ptr<TriangleSurfaceMesh<double>> mesh;

  const std::string extension = mesh_spec.extension();
  if (extension == ".obj") {
    mesh = make_unique<TriangleSurfaceMesh<double>>(
        ReadObjToTriangleSurfaceMesh(mesh_spec.source(), mesh_spec.scale3()));
  } else if (extension == ".vtk") {
    mesh = make_unique<TriangleSurfaceMesh<double>>(
        ConvertVolumeToSurfaceMesh(MakeVolumeMeshFromVtk<double>(mesh_spec)));
  } else {
    throw(std::runtime_error(fmt::format(
        "hydroelastic::MakeRigidRepresentation(): for rigid hydroelastic Mesh "
        "shapes can only use .obj or .vtk files; given: {}",
        mesh_spec.source().description())));
  }

  return RigidGeometry(RigidMesh(std::move(mesh)));
}

std::optional<RigidGeometry> MakeRigidRepresentation(
    const Convex& convex_spec, const ProximityProperties&) {
  // Simply use the Convex's GetConvexHull().
  return RigidGeometry(RigidMesh(make_unique<TriangleSurfaceMesh<double>>(
      MakeTriangleFromPolygonMesh(convex_spec.GetConvexHull()))));
}

void WarnNoCompliantRepresentation(std::string_view shape_type_name) {
  static const logging::Warn log_once(
      "Compliant {} shapes are not currently supported for hydroelastic "
      "contact; "
      "registration is allowed, but an error will be thrown during contact.",
      shape_type_name);
}

std::optional<CompliantGeometry> MakeCompliantRepresentation(
    const Sphere& sphere, const ProximityProperties& props) {
  PositiveDouble positive_validator("Sphere", "compliant");
  NonNegativeDouble non_negative_validator("Sphere", "compliant");
  const double edge_length =
      positive_validator.Extract(props, kHydroGroup, kRezHint);
  const double margin =
      non_negative_validator.Extract(props, kHydroGroup, kMargin, 2e-4);
  const double barrier =
      non_negative_validator.Extract(props, kHydroGroup, kBarrier, 1e-4);

  // To prototype the epsilon log-barrier region, we will repurpose the margin
  // parameter to create an offset surface volume mesh for use with the
  // hydroelastic contact surface query.
  if (barrier > 0) {
    auto surface_mesh = make_unique<TriangleSurfaceMesh<double>>(
        MakeSphereSurfaceMesh<double>(sphere, edge_length));

    auto extruded_mesh = make_unique<VolumeMesh<double>>(
        MakeExtrudedMesh(*surface_mesh, margin + barrier));

    // Extent field over the extruded mesh: e = 2 on the core surface
    // vertices (the first N = surface_mesh->num_vertices()), falling linearly
    // to -2*margin/barrier on the extruded layer (zero level set at distance
    // `barrier` from the core). See the detailed discussion of this field —
    // including the factor-2 relation to the paper's e in [0, 1] formulation —
    // at the Mesh variant below.
    const double surface_epsilon = -2 * margin / barrier;
    std::vector<double> extruded_values(extruded_mesh->num_vertices(),
                                        surface_epsilon);
    for (int i = 0; i < surface_mesh->num_vertices(); ++i) {
      extruded_values[i] = 2.0;
    }

    // For now assume all tetrahedra have positive volume.

    // Replace mesh with one that only has positive tetrahedra volumes. This
    // doesn't change the vertex count.
    // extruded_mesh =
    //     make_unique<VolumeMesh<double>>(RemoveNegativeVolumes(*extruded_mesh));

    // DRAKE_DEMAND(ssize(inflated_values) == extruded_mesh->num_vertices());

    auto extruded_field = make_unique<VolumeMeshFieldLinear<double, double>>(
        std::move(extruded_values), extruded_mesh.get(),
        MeshGradientMode::
            kOkOrThrow /* what MakeVolumeMeshPressureField() uses. */);

    return CompliantGeometry(CompliantMesh(std::move(extruded_mesh),
                                           std::move(extruded_field),
                                           std::move(surface_mesh)));
  } else {
    // Volumetric Hydro (no collision mesh)
    const Sphere inflated_sphere(sphere.radius() + margin);

    // If nothing is said, let's go for the *cheap* tessellation strategy.
    const TessellationStrategy strategy =
        props.GetPropertyOrDefault(kHydroGroup, "tessellation_strategy",
                                   TessellationStrategy::kSingleInteriorVertex);
    auto inflated_mesh = make_unique<VolumeMesh<double>>(
        MakeSphereVolumeMesh<double>(inflated_sphere, edge_length, strategy));

    // Store an extent field for log barrier hydro.
    auto pressure = make_unique<VolumeMeshFieldLinear<double, double>>(
        MakeSpherePressureField(inflated_sphere, inflated_mesh.get(), 1.0,
                                margin));

    return CompliantGeometry(
        CompliantMesh(std::move(inflated_mesh), std::move(pressure)));
  }
}

std::optional<CompliantGeometry> MakeCompliantRepresentation(
    const Box& box, const ProximityProperties& props) {
  NonNegativeDouble non_negative_validator("Box", "compliant");
  const double margin =
      non_negative_validator.Extract(props, kHydroGroup, kMargin, 2e-4);
  const double barrier =
      non_negative_validator.Extract(props, kHydroGroup, kBarrier, 1e-4);

  // To prototype the epsilon log-barrier region, we will repurpose the margin
  // parameter to create an offset surface volume mesh for use with the
  // hydroelastic contact surface query.
  if (barrier > 0) {
    auto surface_mesh = make_unique<TriangleSurfaceMesh<double>>(
        MakeBoxSurfaceMeshWithSymmetricTriangles<double>(box));

    auto extruded_mesh = make_unique<VolumeMesh<double>>(
        MakeExtrudedMesh(*surface_mesh, margin + barrier));

    // Extent field over the extruded mesh: e = 2 on the core surface
    // vertices (the first N = surface_mesh->num_vertices()), falling linearly
    // to -2*margin/barrier on the extruded layer (zero level set at distance
    // `barrier` from the core). See the detailed discussion of this field —
    // including the factor-2 relation to the paper's e in [0, 1] formulation —
    // at the Mesh variant below.
    const double surface_epsilon = -2 * margin / barrier;
    std::vector<double> extruded_values(extruded_mesh->num_vertices(),
                                        surface_epsilon);
    for (int i = 0; i < surface_mesh->num_vertices(); ++i) {
      extruded_values[i] = 2.0;
    }

    // For now assume all tetrahedra have positive volume.

    // Replace mesh with one that only has positive tetrahedra volumes. This
    // doesn't change the vertex count.
    // extruded_mesh =
    //     make_unique<VolumeMesh<double>>(RemoveNegativeVolumes(*extruded_mesh));

    // DRAKE_DEMAND(ssize(inflated_values) == extruded_mesh->num_vertices());

    auto extruded_field = make_unique<VolumeMeshFieldLinear<double, double>>(
        std::move(extruded_values), extruded_mesh.get(),
        MeshGradientMode::
            kOkOrThrow /* what MakeVolumeMeshPressureField() uses. */);

    return CompliantGeometry(CompliantMesh(std::move(extruded_mesh),
                                           std::move(extruded_field),
                                           std::move(surface_mesh)));
  } else {
    // Volumetric Hydro (no collision mesh).
    // Define the shape of the "inflated" hydroelastic geometry to include the
    // margin. We inflate all faces of the box a distance "margin" along the
    // outward normal.
    const Box inflated_box(box.size() +
                           Vector3<double>::Constant(2.0 * margin));

    // First, create an inflated mesh.
    auto inflated_mesh = make_unique<VolumeMesh<double>>(
        MakeBoxVolumeMeshWithMaAndSymmetricTriangles<double>(inflated_box));

    // Store an extent field.
    auto pressure = make_unique<VolumeMeshFieldLinear<double, double>>(
        MakeBoxPressureField(inflated_box, inflated_mesh.get(), 1.0, margin));

    return CompliantGeometry(
        CompliantMesh(std::move(inflated_mesh), std::move(pressure)));
  }
}

std::optional<CompliantGeometry> MakeCompliantRepresentation(
    const Cylinder& cylinder, const ProximityProperties& props) {
  const double margin = NonNegativeDouble("Cylinder", "compliant")
                            .Extract(props, kHydroGroup, kMargin, 0.0);
  const Cylinder inflated_cylinder(cylinder.radius() + margin,
                                   cylinder.length() + 2.0 * margin);

  PositiveDouble positive_validator("Cylinder", "compliant");
  const double edge_length =
      positive_validator.Extract(props, kHydroGroup, kRezHint);
  auto inflated_mesh = make_unique<VolumeMesh<double>>(
      MakeCylinderVolumeMeshWithMa<double>(inflated_cylinder, edge_length));

  const double hydroelastic_modulus =
      positive_validator.Extract(props, kHydroGroup, kElastic);
  auto pressure = make_unique<VolumeMeshFieldLinear<double, double>>(
      MakeCylinderPressureField(inflated_cylinder, inflated_mesh.get(),
                                hydroelastic_modulus, margin));

  return CompliantGeometry(
      CompliantMesh(std::move(inflated_mesh), std::move(pressure)));
}

std::optional<CompliantGeometry> MakeCompliantRepresentation(
    const Capsule& capsule, const ProximityProperties& props) {
  const double margin = NonNegativeDouble("Capsule", "compliant")
                            .Extract(props, kHydroGroup, kMargin, 0.0);
  const Capsule inflated_capsule(capsule.radius() + margin, capsule.length());

  PositiveDouble positive_validator("Capsule", "compliant");
  const double edge_length =
      positive_validator.Extract(props, kHydroGroup, kRezHint);
  auto inflated_mesh = make_unique<VolumeMesh<double>>(
      MakeCapsuleVolumeMesh<double>(inflated_capsule, edge_length));

  const double hydroelastic_modulus =
      positive_validator.Extract(props, kHydroGroup, kElastic);
  auto pressure = make_unique<VolumeMeshFieldLinear<double, double>>(
      MakeCapsulePressureField(inflated_capsule, inflated_mesh.get(),
                               hydroelastic_modulus, margin));

  return CompliantGeometry(
      CompliantMesh(std::move(inflated_mesh), std::move(pressure)));
}

std::optional<CompliantGeometry> MakeCompliantRepresentation(
    const Ellipsoid& ellipsoid, const ProximityProperties& props) {
  // If nothing is said, let's go for the *cheap* tessellation strategy.
  const TessellationStrategy strategy =
      props.GetPropertyOrDefault(kHydroGroup, "tessellation_strategy",
                                 TessellationStrategy::kSingleInteriorVertex);

  const double margin = NonNegativeDouble("Ellipsoid", "compliant")
                            .Extract(props, kHydroGroup, kMargin, 0.0);
  PositiveDouble positive_validator("Ellipsoid", "compliant");
  const double edge_length =
      positive_validator.Extract(props, kHydroGroup, kRezHint);
  const Ellipsoid inflated_ellipsoid(
      ellipsoid.a() + margin, ellipsoid.b() + margin, ellipsoid.c() + margin);
  auto inflated_mesh =
      make_unique<VolumeMesh<double>>(MakeEllipsoidVolumeMesh<double>(
          inflated_ellipsoid, edge_length, strategy));

  const double hydroelastic_modulus =
      positive_validator.Extract(props, kHydroGroup, kElastic);
  auto pressure = make_unique<VolumeMeshFieldLinear<double, double>>(
      MakeEllipsoidPressureField(inflated_ellipsoid, inflated_mesh.get(),
                                 hydroelastic_modulus, margin));

  return CompliantGeometry(
      CompliantMesh(std::move(inflated_mesh), std::move(pressure)));
}

std::optional<CompliantGeometry> MakeCompliantRepresentation(
    const HalfSpace&, const ProximityProperties& props) {
  PositiveDouble positive_validator("HalfSpace", "compliant");

  const double thickness =
      positive_validator.Extract(props, kHydroGroup, kSlabThickness);

  const double hydroelastic_modulus =
      positive_validator.Extract(props, kHydroGroup, kElastic);

  const double margin = NonNegativeDouble("HalfSpace", "compliant")
                            .Extract(props, kHydroGroup, kMargin, 0.0);

  return CompliantGeometry(
      CompliantHalfSpace{hydroelastic_modulus / thickness, margin});
}

std::optional<CompliantGeometry> MakeCompliantRepresentation(
    const Convex& convex_spec, const ProximityProperties& props) {
  const double margin = NonNegativeDouble("Convex", "compliant")
                            .Extract(props, kHydroGroup, kMargin, 0.0);
  // For zero margin, use the pre-computed convex hull for the shape.
  const TriangleSurfaceMesh<double> inflated_surface_mesh =
      MakeTriangleFromPolygonMesh(
          margin > 0 ? MakeConvexHull(convex_spec.source(),
                                      convex_spec.scale3(), margin)
                     : convex_spec.GetConvexHull());
  auto inflated_mesh = make_unique<VolumeMesh<double>>(
      MakeConvexVolumeMesh<double>(inflated_surface_mesh));

  const double hydroelastic_modulus =
      PositiveDouble("Convex", "compliant")
          .Extract(props, kHydroGroup, kElastic);

  auto pressure = make_unique<VolumeMeshFieldLinear<double, double>>(
      MakeVolumeMeshPressureField(inflated_mesh.get(), hydroelastic_modulus,
                                  margin));

  return CompliantGeometry(
      CompliantMesh(std::move(inflated_mesh), std::move(pressure)));
}

std::optional<CompliantGeometry> MakeCompliantRepresentation(
    const Mesh& mesh_spec, const ProximityProperties& props) {
  NonNegativeDouble non_negative_validator("Mesh", "compliant");
  const double margin =
      non_negative_validator.Extract(props, kHydroGroup, kMargin, 2e-4);
  const double barrier =
      non_negative_validator.Extract(props, kHydroGroup, kBarrier, 1e-4);

  drake::log()->debug(
      "Compliant mesh reification: '{}' margin={} ({}) barrier={} ({})",
      mesh_spec.source().description(), margin,
      props.HasProperty(kHydroGroup, kMargin) ? "property" : "fallback",
      barrier,
      props.HasProperty(kHydroGroup, kBarrier) ? "property" : "fallback");

  if (barrier > 0) {
    std::unique_ptr<TriangleSurfaceMesh<double>> surface_mesh;
    if (mesh_spec.extension() == ".vtk") {
      surface_mesh = make_unique<TriangleSurfaceMesh<double>>(
          ConvertVolumeToSurfaceMesh(MakeVolumeMeshFromVtk<double>(mesh_spec)));
    } else {
      surface_mesh = make_unique<TriangleSurfaceMesh<double>>(
          ReadObjToTriangleSurfaceMesh(mesh_spec.source(), mesh_spec.scale3()));
    }

    auto extruded_mesh = make_unique<VolumeMesh<double>>(
        MakeExtrudedMesh(*surface_mesh, margin + barrier));

    // Extent field over the extruded mesh (see also the sphere/box variants
    // above, which use the same construction). The first N vertices are the
    // original surface mesh (the rigid core), N = surface_mesh->num_vertices();
    // the remainder are their copies extruded outward by (margin + barrier).
    //
    // Values: e = 2 on the core, e = -2·margin/barrier on the extruded layer.
    // Linearly along the extrusion, at distance d from the core:
    //     e(d) = 2 - (2/barrier)·d,
    // so the zero level set sits at d = barrier, with the extra margin band
    // (e < 0) extending field support to d = margin + barrier.
    //
    // N.B. the paper's formulation uses e ∈ [0, 1] (1 at the core, 0 at
    // d = barrier); the code's field is exactly 2× that. The doubling is NOT
    // behavior-neutral in the barrier model (n(e) ∝ e/(1-e) is nonlinear):
    // with e_code, n(e)'s pole sits at d = barrier/2 rather than at the core,
    // and values e_code ∈ (1, 2] near the core are only well-behaved because
    // the near-rigid linear extension (RegularizedBarrierModel::n_e_tilde)
    // takes over above the transition point.
    // TODO(joemasterjohn): Reconcile with the paper's e ∈ [0, 1] formulation
    // (PAPER_NOTES §4): either renormalize here (a physics change requiring
    // re-validation of the E-campaign data) or adopt the factor-2 field in
    // the paper text.
    const double surface_epsilon = -2 * margin / barrier;
    std::vector<double> extruded_values(extruded_mesh->num_vertices(),
                                        surface_epsilon);
    for (int i = 0; i < surface_mesh->num_vertices(); ++i) {
      extruded_values[i] = 2.0;
    }

    // For now assume all tetrahedra have positive volume.

    auto extruded_field = make_unique<VolumeMeshFieldLinear<double, double>>(
        std::move(extruded_values), extruded_mesh.get(),
        MeshGradientMode::
            kOkOrThrow /* what MakeVolumeMeshPressureField() uses. */);

    return CompliantGeometry(CompliantMesh(std::move(extruded_mesh),
                                           std::move(extruded_field),
                                           std::move(surface_mesh)));
  } else {
    // Volumetric Hydro (no collision mesh). This is the upstream Drake
    // compliant Mesh representation: a .vtk file provides the volume mesh
    // directly; any other mesh format falls back to its convex hull. As with
    // the Sphere and Box volumetric branches above, the field is a normalized
    // extent field (modulus 1.0) — the ICF builder scales constraints by the
    // effective hydroelastic modulus itself.
    std::unique_ptr<VolumeMesh<double>> mesh;
    std::map<int, int> split_vertices_map;

    if (mesh_spec.extension() == ".vtk") {
      // If they've explicitly provided a .vtk file, we'll treat it as it is a
      // volume mesh. If that's not true, we'll get an error.
      mesh = make_unique<VolumeMesh<double>>(
          MakeVolumeMeshFromVtk<double>(mesh_spec));
    } else {
      // Otherwise, we'll create a compliant representation of its convex hull.
      mesh = make_unique<VolumeMesh<double>>(MakeConvexVolumeMesh<double>(
          MakeTriangleFromPolygonMesh(mesh_spec.GetConvexHull())));
    }

    auto inflated_mesh = make_unique<VolumeMesh<double>>(
        MakeInflatedMesh(*mesh, margin, &split_vertices_map));

    // N.B. The inflated mesh might have different topology than the original
    // mesh. This makes calling MakeVolumeMeshPressureField() on the inflated
    // mesh problematic. Instead, we use the original "non-inflated" mesh to
    // compute a pressure field with the given margin value and apply that to
    // the inflated mesh. If no vertices are duplicated, the mapping between
    // the two meshes is a simple one-to-one correspondence. For duplicate
    // vertices, we use the mapping provided by MakeInflatedMesh() assign the
    // same pressure values to duplicated vertices as assigned to the original.

    // Extent field computed using the original mesh but with margin.
    VolumeMeshFieldLinear<double, double> field =
        MakeVolumeMeshPressureField(mesh.get(), 1.0, margin);

    // The "inflated" field will contain pressure values at the original
    // vertices and, if added by MakeInflatedMesh(), on split vertices.
    const std::vector<double>& values = field.values();
    std::vector<double> inflated_values(values.size() +
                                        split_vertices_map.size());
    std::copy(values.begin(), values.end(), inflated_values.begin());

    // Copy values from their corresponding original vertex for split vertices.
    for (auto& [v_split, v_original] : split_vertices_map) {
      inflated_values[v_split] = values[v_original];
    }

    // Replace mesh with one that only has positive tetrahedra volumes. This
    // doesn't change the vertex count.
    inflated_mesh =
        make_unique<VolumeMesh<double>>(RemoveNegativeVolumes(*inflated_mesh));

    DRAKE_DEMAND(ssize(inflated_values) == inflated_mesh->num_vertices());

    auto inflated_field = make_unique<VolumeMeshFieldLinear<double, double>>(
        std::move(inflated_values), inflated_mesh.get(),
        MeshGradientMode::
            kOkOrThrow /* what MakeVolumeMeshPressureField() uses. */);

    return CompliantGeometry(
        CompliantMesh(std::move(inflated_mesh), std::move(inflated_field)));
  }
}

}  // namespace hydroelastic
}  // namespace internal
}  // namespace geometry
}  // namespace drake
