#include "drake/geometry/internal_geometry.h"

namespace drake {
namespace geometry {
namespace internal {

InternalGeometry::InternalGeometry() : InternalGeometryBase() {}

InternalGeometry::InternalGeometry(std::unique_ptr<Shape> shape,
                                   FrameId frame_id, GeometryId geometry_id,
                                   const std::string& name,
                                   const Isometry3<double>& X_PG,
                                   PoseIndex pose_index,
                                   const optional<GeometryId>& parent_id)
    : InternalGeometryBase(std::move(shape), geometry_id, name, X_PG),
      frame_id_(frame_id),
      pose_index_(pose_index),
      parent_id_(parent_id) {}

InternalAnchoredGeometry::InternalAnchoredGeometry() : InternalGeometryBase() {}

InternalAnchoredGeometry::InternalAnchoredGeometry(
    std::unique_ptr<Shape> shape, GeometryId geometry_id,
    const std::string& name, const Isometry3<double> X_WG)
    : InternalGeometryBase(std::move(shape), geometry_id, name, X_WG) {}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
