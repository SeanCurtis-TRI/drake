#pragma once

#include "drake/geometry/bullet_geometry_engine.h"

namespace drake {
namespace geometry {

/**
 The GeometryWorld is the structure that tracks the geometric elements in a
 simulation. The geometric elements are associated with "parent" frames; as
 the frame moves, the geometry moves with it in a rigid manner. GeometryWorld
 performs geoemtric queries on the geometry (e.g., ray intersection, minimum
 distance between geometries, intersection information, etc.)  These queries
 depend on the state of the frames.  The frame state is provided as input to
 the queries.

 As geometry is introduced into the simulator, it must be registered with
 GeometryWorld so that it can be included in the queries.
 */
template <typename T>
class DRAKE_EXPORT GeometryWorld {
 public:

 private:
  // GeometryWorld has members that are specific implmentations of the
  // GeometryEngine interface. It does this so that it can support multipl
  // geometry engine implementations simultaneously to allow for picking the
  // implementation best suited for particular queries.

  // The bullet-based geometry engine.
  BulletGeometryEngine bullet_engine_;
};
}  // namespace geometry
}  // namespace drake
