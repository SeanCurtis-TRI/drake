#pragma once

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <coal/collision_object.h>
#include <coal/shape/convex.h>
#include <fmt/format.h>

#include "drake/common/drake_export.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal DRAKE_NO_EXPORT {

// TODO(SeanCurtis-TRI): Given the dependencies on Coal for this file, the name
//  should reflect it so that it doesn't get included in files that will
//  eventually get included in the public API.

/* Coal's convex polytope is templated on the polygon type it stores. Drake
 always uses 32-bit triangles; see MakeCoalTriangles().  */
using CoalConvex = coal::Convex<coal::Triangle32>;

/* Converts a Drake pose into the rigid transform type that Coal's API
 consumes.  */
inline coal::Transform3s ToCoalTransform(const math::RigidTransformd& X_AB) {
  return coal::Transform3s(X_AB.rotation().matrix(), X_AB.translation());
}

/* Converts a Coal transform back into a Drake pose. The inverse of
 ToCoalTransform().  */
inline math::RigidTransformd FromCoalTransform(const coal::Transform3s& X_AB) {
  return math::RigidTransformd(math::RotationMatrixd(X_AB.getRotation()),
                               X_AB.getTranslation());
}

/* Triangulates the faces of a convex PolygonSurfaceMesh -- given in that
 class's flat [count, v₀, v₁, ..., count, ...] encoding -- into the triangle
 list that CoalConvex requires. Each n-gon becomes a fan of n - 2 triangles
 around its first vertex, which preserves the winding (and therefore the
 outward normal) of the original face.

 @pre `face_data` is the face_data() of a PolygonSurfaceMesh whose faces are
      all planar and convex (which is true of a convex hull).  */
std::vector<coal::Triangle32> MakeCoalTriangles(
    const std::vector<int>& face_data);

// TODO(SeanCurtis-TRI): Snake case this name.
/* Calculates an absolute tolerance value conditioned to a problem's
 characteristic, positive-valued `size`. The tolerance is sufficient to account
 for round-off error which arises due to transformations.
 @pre size > 0.  */
constexpr double DistanceToPointRelativeTolerance(double size) {
  return 1e-14 * std::max(1.0, size);
}

/* Class for coordinating the collision objects stored in the proximity engine
 with the geometries stored in SceneGraph. The two are related in that the
 collision object stores the GeometryId of the corresponding SceneGraph
 geometry as well as a bit to mark whether it is anchored or dynamic.

 It stores this data by compactly "encoding" them. The encoding packs a geometry
 id value with a bit indicating if the id refers to dynamic or anchored geometry
 (proximity engine segregates them). The highest-order bit indicates dynamic (1)
 or anchored (0). The remaining lower bits store the id. The data is stored in a
 pointer-sized integer. This integer is, in turn, stored directly into
 coal::CollisionObject's void* user data member.  */
class EncodedData {
 public:
  using ValueType = decltype(GeometryId::get_new_id().get_value());

  /* Constructs encoded data directly from the id and the known
   anchored/dynamic characterization.  */
  EncodedData(GeometryId id, bool is_dynamic)
      : data_(static_cast<ValueType>(id.get_value())) {
    // Make sure we haven't used so many ids that we're *using* the highest-
    // order bit -- i.e., it must be strictly positive.
    DRAKE_DEMAND(data_ > 0);
    if (is_dynamic) set_dynamic();
    // NOTE: data is encoded as anchored by default. So, an action only needs to
    // be taken in the dynamic case.
  }

  /* Constructs encoded data by extracting it from the given collision object.
   */
  explicit EncodedData(const coal::CollisionObject& coal_object)
      : data_(reinterpret_cast<ValueType>(coal_object.getUserData())) {}

  /* Constructs encoded data for the given id identified as dynamic.  */
  static EncodedData encode_dynamic(GeometryId id) { return {id, true}; }

  /* Constructs encoded data for the given id identified as anchored.  */
  static EncodedData encode_anchored(GeometryId id) { return {id, false}; }

  /* Sets the encoded data to be dynamic.  */
  void set_dynamic() { data_ |= kIsDynamicMask; }

  /* Sets the encoded data to be anchored.  */
  void set_anchored() { data_ &= ~kIsDynamicMask; }

  /* Writes the encoded data into the collision object's user data.  */
  void write_to(coal::CollisionObject* object) const {
    object->setUserData(reinterpret_cast<void*>(data_));
  }

  /* Reports true if the encoded data is marked as dynamic. False if anchored.
   */
  bool is_dynamic() const { return (data_ & kIsDynamicMask) != 0; }

  /* Reports the stored id.  */
  GeometryId id() const {
    return static_cast<GeometryId>(data_ & ~kIsDynamicMask);
  }

  /* Reports the encoded data.  */
  ValueType encoding() const { return data_; }

 private:
  // For this encoding to work we have the following requirements:
  //  1. ValueType must be at least as large as a pointer.
  //  2. ValueType must be at least as large as GeometryId. (At least as large
  //     still implies that with enough GeometryIds allocated, the high order
  //     bit could become ambiguous -- however, that would require one id for
  //     each atom in the universe).
  // These static asserts guarantee these conditions.

  // This is redundant of the declaration of ValueType; it serves as an
  // independent witness to the requirement in case the declaration changes.
  static_assert(sizeof(ValueType) >= sizeof(GeometryId),
                "The encoded data type must be at least as large as the "
                "identifier to use in EncodedData");

  static_assert(sizeof(ValueType) >= sizeof(void*),
                "The encoded data type must be at least as large as a pointer "
                "type");

  // Regardless of how large ValueType is, ultimately, we need to be able to
  // pack the encoding into a void*. So, we set the bit mask as the highest
  // order bit of something pointer sized.
  static const ValueType kIsDynamicMask = ValueType{1}
                                          << (sizeof(void*) * 8 - 1);

  // The encoded data - id and mobility type masked together.
  ValueType data_{};
};

/* Returns the name of the geometry associated with the given collision
 `object`.  */
std::string GetGeometryName(const coal::CollisionObject& object);

// TODO(joemasterjohn): Move the below mesh testing utilities to a separate
// target only used by tests.

/* Counts the unique 1-simplices (edges) in `mesh`. */
int CountEdges(const VolumeMesh<double>& mesh);

/* Counts the unique 2-simplices (faces) in the `mesh` */
int CountFaces(const VolumeMesh<double>& mesh);

/*
  Computes the generalized Euler characteristic:

   χ = k₀ - k₁ + k₂ - k₃

  Where kᵢ is the number of i-simplexes in the mesh.

  We can use χ in a necessary condition for a _conforming_
  tetrahedral mesh (any two tetrahedra intersect in their shared
  face, or shared edge, or shared vertex, or not at all. There is
  no partial overlapping of two tetrahedra.).

  For example, every tetrahedral mesh of every geometric primitive
  (Box , Cylinder, Ellipsoid, Sphere, etc.) must have χ = 1 because
  it is topologically equivalent to a solid ball.
*/
int ComputeEulerCharacteristic(const VolumeMesh<double>& mesh);

using Eigen::Vector3d;

/* Computes the signed distance to the capsule's surface from point P. The point
 is measured and expressed in the capsule's canonical frame C. Negative values
 for points *inside* the capsule, positive value for points outside.  */
double CalcDistanceToSurface(const Capsule& capsule, const Vector3d& p_CP);

// clang-format off
}  // namespace internal
// clang-format on
}  // namespace geometry
}  // namespace drake
