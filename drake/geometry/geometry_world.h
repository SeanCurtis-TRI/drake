#pragma once

#include "drake/geometry/bullet_geometry_engine.h"

namespace drake {
namespace geometry {

/**
 The GeometryWorld is the structure that tracks the geometric elements in a
 simulation. The geometric elements are associated with "parent" frames; as
 the frame moves, the geometry moves with it in a rigid manner. GeometryWorld
 performs geometric queries on the geometry (e.g., ray intersection, minimum
 distance between geometries, intersection information, etc.)  These queries
 depend on the state of the frames.  The frame state is provided as input to
 the queries.

 As geometry is introduced into the simulator, it must be registered with
 GeometryWorld so that it can be included in the queries.

 The GeometryWorld paradigm uses the GeometryFrame class as the interface
 between the geometry and the simulation.  Ultimately, the simulation is
 responsible for moving frames.  Different systems can use different
 mechanisms for moving frames; these are the *drivers* of the frames.

 Geometry is "hung" on a GeometryFrame in a *rigid* manner.  As the frame moves,
 the associated geometry moves with it.  The associated geometry can consist of
 a single shape, or a hierarchy of shapes (such that the whole hierarchy is
 treated as a single rigid union of shapes.)
 */
template <typename T>
class GeometryWorld {
 public:

 private:
  // GeometryWorld has members that are specific implementations of the
  // GeometryEngine interface. It does this so that it can support multiple
  // geometry engine implementations simultaneously to allow for picking the
  // implementation best suited for particular queries.
  BulletGeometryEngine bullet_engine_;
  FclGeometryEngine fcl_engine_;

};
}  // namespace geometry
}  // namespace drake
