#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/bullet_geometry_engine.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace geometry {

// Forward declarations.
class GeometryInstance;

// TODO(SeanCurtis-TRI): Review this documentation to confirm that it's consistent
// with what I ended up implementing.
/**
 GeometryWorld is the structure that coordinates the geometric elements across
 various independent subsystems in a simulation. The geometric elements are
 associated with "parent" frames; as
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

 @section geom_world_workflow Workflow for Upstream Frame Owners

 For an upstream frame owner, the expected work flow is as follows in two
 phases: initialization (a.k.a. declaration) and setting values.

 @subsection geom_world_declare_workflow For initialization/declaration:

 @code
 // Assuming the following variables are defined.
 Context& context;
 GeometryWorld* geometry_world;

 // Open a channel -- should be stored in a persistent variable, preferably a
 // member variable.
 channel_ = geometry_world->RequestChannel(context);

 // Declare moving frames and the hanging geometry. The caller is responsible
 // for saving the FrameId instances for use later.
 FrameId f0 = channel_->DeclareFrame(context);
 GeometryId g0 = channel->DeclareGeometry(context, f0, instance, pose0);
 FrameId f1 = channel_->DeclareFrame(context);
 GeometryId g1 = channel->DeclareGeometry(context, f1, instance, pose0);
 ...

 // Acquire and persist a FrameKinematicsSet instance (preferably in a member
 // variable).
 frame_kinetics_set_ = channel_->GetFrameKinematicsSet();
 @endcode

 @subsection geom_world_value_workflow For setting values during the simulation.

 @code{.cpp}
 // Assume the following variables are defined.
 const Context& context;

 // Always re-sync the frame kinematics set with the particular context.
 channel_->UpdateFrameKinematicsSet(context, &frame_kinematics_set_);
 foreach ( FrameId frame_id : frames_ ) {
    // Compute SpatialPose, SpatialVelocity, and SpatialAcceleration
    SpatialPose X_WF = ...;
    SpatialVelocity V_WF = ...;
    SpatialAcceleration A_WF = ...;
    frame_kinematics_set_.SetFrameFullKinematics(frame_id, X_WF, V_WF, A_WF);
 }
 @endcode
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

  /**
   Requests a new channel for declaring frames and geometry to GeometryWorld.
   @param context   A mutable context.
   @sa GeometryChannel
   */
  unique_ptr<GeometryChannel> RequestChannel(drake::systems::Context* context);

  /**
   Adds the given geometry to the world as anchored geometry.
   @param context       A mutable context.
   @param geometry      The geometry to add to the world.
   @param X_WG          A transform from the geometry's canonical space to
                        world space.
   @returns The index for the added geometry.
   */
  GeometryId AddAnchoredGeometry(drake::systems::Context<T>* context,
                                 std::unique_ptr<GeometryInstance> geometry,
                                 const Isometry3<T>& X_WG);

  /** @} */

  /**
   Allocates the context state that GeometryWorld requires.
   @returns  A vector of abstract values which will be accessed and managed by
             GeometryWorld.
   */
  std::vector<drake::systems::AbstractValue*> AllocateAbstractValues();

  /**
   Provides a set of frame kinematics data. GeometryWorld uses this to update
   its knowledge of the kinematics of the geometry. It is essential that this
   is called for *all* open GeometryChannel instances before invoking a query to
   prevent making queries in an inconsistent state.

   This is the only mechanism for updating the state of the geometry in
   GeometryWorld.

   Several circumstances will lead to an exception being thrown:
     - One or more of the frames has _not_ had its data set,
     - The data set does not come from a known GeometryChannel,
     - The frames in the dataset are inconsistent of the declared frames.
   @param frame_kinematics  The kinematics data for the frames in a single
                            GeometryChannel.
   */
  void UpdateFrames(const FrameKinematicsSet& frame_kinematics);

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
