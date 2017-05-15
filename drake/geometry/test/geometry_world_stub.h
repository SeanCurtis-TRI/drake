#pragma once

#include "drake/geometry/frame_kinematics_set.h"
#include "drake/geometry/geometry_query.h"

namespace drake {
namespace geometry {
/** @cond */

// This serves as a _mock_ GeometryWorld. GeometryWorld serves as a factory of
// various geometry-namespaced classes which should *only* be created by
// GeometryWorld. It is the _only_ class that can generate them.
template <typename T>
class GeometryWorld {
 public:
  static FrameKinematicsSet<T> MakeFKS(SourceId s_id) {
    return FrameKinematicsSet<T>(s_id);
  }

  static GeometryQuery<T> MakeQuery(
      const GeometryEngine<T>& engine, const std::vector<GeometryId>& ids,
      const std::unordered_map<GeometryId, internal::InternalGeometry>&
      geometries) {
    return GeometryQuery<T>(engine, ids, geometries);
  }
};

/** @endcode */
}  // namespace geometry
}  // namespace drake
