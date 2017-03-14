#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/bullet_geometry_engine.h"
#include "drake/geometry/geometry_indexes.h"

namespace drake {
namespace geometry {

// Forward declarations.
class Geometry;

// TODO(SeanCurtis-TRI): Review this documentation to confirm that it's consistent
// with what I ended up implementing.
/**
 GeometryWorld is the structure that tracks the geometric elements in a
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
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GeometryWorld)

  /** @name Frame and Geometry Registration

   This is the interface that allows entities to inform GeometryWorld of
   simulation geometry.  The geometry could be anchored or dynamic.

   By definition, anchored geometry is fixed to the world frame. As such,
   registering anchored geometry only requires the geometry data itself and its
   pose relative to the world frame.

   Like with anchored geometry, registering dynamic geometry requires the
   geometry data and a pose. Unlike anchored geometry, the frame in which the
   pose is defined is arbitrary and can move over time. Dynamic geometry must
   be associated with an instance of GeometryKinematics -- the specification of
   the movement of the geometry's _driving_ frame measured and expressed in the
   world frame.  The GeometryKinematics instance must, in turn, be included
   in a GeometryBundle.

   This interface provides all the methods necessary to register geometry, its
   data, and define its associated frame.
   @{
   */

  // TODO(SeanCurtis-TRI): Work through the parameters for geometry and pose.
  /**
   Adds the given geometry to the world as anchored geometry.
   @param geometry      The geometry to add to the world.
   @param X_FG          A transform from the geometry's canonical space to
                        world space.
   @return The index f
   */
  GeometryIndex AddAnchoredGeometry(std::unique_ptr<Geometry> geometry,
                                    const Isometry3<Scalar>& X_WG);

  /** @} */

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
