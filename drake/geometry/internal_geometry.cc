#include "drake/geometry/internal_geometry.h"

namespace drake {
namespace geometry {
namespace internal {

InternalGeometry::InternalGeometry() : InternalGeometryBase() {}

InternalGeometry::InternalGeometry(std::unique_ptr<Shape> shape,
                                   FrameId frame_id, GeometryId geometry_id,
                                   const std::string& name,
                                   const Isometry3<double>& X_PG,
                                   GeometryIndex engine_index,
                                   const optional<GeometryId>& parent_id)
    : InternalGeometryBase(std::move(shape), geometry_id, name, X_PG,
                           engine_index),
      frame_id_(frame_id),
      parent_id_(parent_id) {}

InternalGeometry::InternalGeometry(std::unique_ptr<Shape> shape,
                                   FrameId frame_id, GeometryId geometry_id,
                                   const std::string& name,
                                   const Isometry3<double>& X_PG,
                                   GeometryIndex engine_index,
                                   const VisualMaterial& vis_material,
                                   const optional<GeometryId>& parent_id)
    : InternalGeometryBase(std::move(shape), geometry_id, name, X_PG,
                           engine_index, vis_material),
      frame_id_(frame_id),
      parent_id_(parent_id) {}

InternalAnchoredGeometry::InternalAnchoredGeometry() : InternalGeometryBase() {}

InternalAnchoredGeometry::InternalAnchoredGeometry(
    std::unique_ptr<Shape> shape, GeometryId geometry_id,
    const std::string& name, const Isometry3<double> X_WG,
    AnchoredGeometryIndex engine_index)
    : InternalGeometryBase(std::move(shape), geometry_id, name, X_WG,
                           engine_index) {}

InternalAnchoredGeometry::InternalAnchoredGeometry(
    std::unique_ptr<Shape> shape, GeometryId geometry_id,
    const std::string& name, const Isometry3<double> X_WG,
    AnchoredGeometryIndex engine_index, const VisualMaterial& vis_material)
    : InternalGeometryBase(std::move(shape), geometry_id, name, X_WG,
                           engine_index, vis_material) {}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
