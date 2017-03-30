#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
//#include "drake/geometry/bullet_geometry_engine.h"
#include "drake/geometry/frame_kinematics_set.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace geometry {

// Forward declarations.
class GeometryInstance;
template <typename T> class FrameKinematicsSet;
template <typename T> class GeometryContext;

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

 @tparam T The underlying scalar type. Must be a valid Eigen scalar.
 */
template <typename T>
class GeometryWorld {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GeometryWorld)

  /** Default constructor. */
  GeometryWorld() {}

  /** @name Registration methods

   The registration methods are how system entities inform GeometryWorld of
   geometry and its dynamic properties (e.g., does it move or is it anchored).

   Upstream entities use these methods to:
     - register themselves as geometry sources,
     - register moving frames,
     - register geometry moved by registered frames,
     - register anchored geometry,
     - remove geometry (moving and anchored), and
     - remove moving frames (and affixed geometry).
   @{
   */

  /**
   Registers a new geometry source to GeometryWorld. Receives the unique
   identifier for this new source.
   @param context   A mutable geometry context.
   */
  SourceId RegisterNewSource(GeometryContext<T>* context);

  /** Removes the active geometry source from GeometryWorld.
   Throws an exception if the identifier doesn't represent an active source.

   * @param source_id   The identifier of an active geometry source.
   * @param context     The geometry context to modify.
   */
  void RemoveSource(SourceId source_id, GeometryContext<T>* context);

  // TODO(SeanCurtis-TRI): Add metadata. E.g., name, some kind of payload, etc.
  /**
   Declares a new frame on this channel, receiving the unique id for the new
   frame.
   @param source_id     The identifier for the geometry source registering the
                        frame.
   @param context       A mutable context.
   */
  FrameId RegisterFrame(SourceId source_id, GeometryContext<T>* context);

  /**
   Declares a `geometry` instance as "hanging" from the specified frame at the
   given pose relative to the frame. The geometry is _rigidly_ affixed to the
   parent frame.

   If the `id` is not a valid frame identifier declared through this channel, an
   exception will be thrown.

   @param source_id   The identifier for the geometry source registering the
                      frame.
   @param context     A mutable context.
   @param frame_id    The id for the frame `F` to hang the geometry on.
   @param geometry    The geometry to hang.
   @param X_FG        the transform X_FG which transforms the geometry
                      from its canonical frame `G` to the parent frame `F`,
                      i.e., the geometry's pose relative to its parent.
   @return A unique identifier for the added geometry.
   */
  GeometryId RegisterGeometry(SourceId source_id,
                              GeometryContext<T>* context,
                              FrameId frame_id,
                              std::unique_ptr<GeometryInstance> geometry,
                              const Isometry3<T>& X_FG);

  /**
   Declares a `geometry` instance as "hanging" from the specified geometry's
   frame `F`, with the given pose relative to that frame. The geometry is
   _rigidly_ affixed to the parent frame.

   This method enables the owner entity to construct rigid hierarchies of posed
   geometries. This rigid structure will all be driven by the declared frame
   to which the root geometry was attached.

   If the `id` is not a valid geometry identifier declared through this channel,
   an exception will be thrown.

   @param source_id    The identifier for the geometry source registering the
                       frame.
   @param context      A mutable context.
   @param geometry_id  The id for the geometry to hang the declared geometry on.
   @param geometry     The geometry to hang.
   @param X_FG         the transform X_FG which transforms the geometry
                       from its canonical frame `G` to the parent frame `F`,
                       i.e., the geometry's pose relative to its parent.
   @return A unique identifier for the added geometry.
   */
  GeometryId RegisterGeometry(SourceId source_id,
                              GeometryContext<T>* context,
                              GeometryId geometry_id,
                              std::unique_ptr<GeometryInstance> geometry,
                              const Isometry3<T>& X_FG);

  /**
   Adds the given geometry to the world as anchored geometry.
   @param context       A mutable context.
   @param geometry      The geometry to add to the world.
   @param X_WG          A transform from the geometry's canonical space to
                        world space.
   @returns The index for the added geometry.
   */
  GeometryId RegisterAnchoredGeometry(
      GeometryContext<T>* context, std::unique_ptr<GeometryInstance> geometry,
      const Isometry3<T>& X_WG);

  /** @} */

  /** @name Evaluation methods

   These are the methods through which values for registered frames are provided
   to GeometryWorld.

   @{
   */

  /**
   Requests a frame kinematics set for the given geometry source. This should be
   invoked every time the frame kinematics values are evaluated and provided.

   Aborts if the source identifier does not reference an active source.

   @param source_id     The identifier of the evaluating geometry source.
   @param context       The context against which all declarations have been
                        made.
   */
  FrameKinematicsSet<T> GetFrameKinematicsSet(SourceId source_id,
                                              const GeometryContext<T>& context);

  /**
   Sets the kinematics _values_ from the given value set. GeometryWorld consumes
   the set of frame kinematics data to update the poses of the geometry affixed
   to the frames in the set. It is essential that this is called once for each
   registered geometry source before invoking a query. Failure to do so will
   lead to queries on a world with inconsistent state.

   This is the only mechanism for updating the state of the geometry in
   GeometryWorld.

   Several circumstances will lead to an exception being thrown:
     - One or more of the frames registered by the invoking geometry source has
       _not_ had its data set,
     - The data set does not come from a known geometry source,
     - The frames in the set are inconsistent with the registered frames.

   @param frame_kinematics  The kinematics data for the frames registered by a
                            single source.
   @param context           A mutable context; used for updating cache.
   */
  void SetFrameKinematics(const FrameKinematicsSet<T>& frame_kinematics,
                          GeometryContext<T>* context);

  /** @} */

  /**
   Allocates the context state that GeometryWorld requires.
   @returns  A vector of abstract values which will be accessed and managed by
             GeometryWorld.
   */
  std::vector<std::unique_ptr<drake::systems::AbstractValue>> AllocateAbstractValues();


 private:
  // GeometryWorld has members that are specific implementations of the
  // GeometryEngine interface. It does this so that it can support multiple
  // geometry engine implementations simultaneously to allow for picking the
  // implementation best suited for particular queries.
//  BulletGeometryEngine bullet_engine_;
//  FclGeometryEngine fcl_engine_;
};
}  // namespace geometry
}  // namespace drake
