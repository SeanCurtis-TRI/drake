#include "drake/geometry/internal_geometry.h"

namespace drake {
namespace geometry {
namespace internal {

bool InternalGeometryBase::has_role(Role role) const {
  switch (role) {
    case Role::kProximity:
      return has_proximity_role();
    case Role::kPerception:
      return has_perception_role();
    case Role::kIllustration:
      return has_illustration_role();
    default:
      // THis should never be reached. The switch statement should be exhaustive
      // of all enumeration values.
      DRAKE_DEMAND(false);
      return false;
  }
}

InternalGeometry::InternalGeometry() : InternalGeometryBase() {}

InternalGeometry::InternalGeometry(std::unique_ptr<Shape> shape,
                                   FrameId frame_id, GeometryId geometry_id,
                                   const std::string& name,
                                   const Isometry3<double>& X_PG,
                                   InternalIndex internal_index,
                                   const optional<GeometryId>& parent_id)
    : InternalGeometryBase(std::move(shape), geometry_id, name, X_PG,
                           internal_index),
      frame_id_(frame_id),
      parent_id_(parent_id) {}

InternalAnchoredGeometry::InternalAnchoredGeometry() : InternalGeometryBase() {}

InternalAnchoredGeometry::InternalAnchoredGeometry(std::unique_ptr<Shape> shape,
                                                   GeometryId geometry_id,
                                                   const std::string& name,
                                                   const Isometry3<double> X_WG,
                                                   InternalIndex internal_index)
    : InternalGeometryBase(std::move(shape), geometry_id, name, X_WG,
                           internal_index) {}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
