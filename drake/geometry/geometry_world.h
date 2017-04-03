#pragma once

#include <memory>
#include <unordered_set>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
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

// TODO(SeanCurtis-TRI): Review this documentation to confirm that it's
// consistent with what I ended up implementing.
/**
 GeometryWorld is the structure that coordinates the geometric elements across
 various independent subsystems in a simulation. The geometric elements are
 associated with "parent" frames; as
 the frame moves, the geometry moves with it in a rigid manner. GeometryWorld
 performs geometric queries on the geometry (e.g., ray intersection, minimum
 distance between geometries, intersection information, etc.)  These queries
 depend on the state of the frames.  The frame state is provided to
 GeometryWorld, updating its state, prior to performing queries.

 As geometry is introduced into the simulator, it must be registered with
 GeometryWorld so that it can be included in the queries.

 GeometryWorld is ignorant of the interpretation of the geometry and of what
 mechanisms cause it to move. This knowledge is owned by a "geometry source".
 Any class that computes frame kinematics values and can specify the attached
 geometry could serve as a geometry source. These classes must register
 themselves as geometry sources with GeometryWorld. When executed in a System,
 they must be part of a LeafSystem which connects an appropriately-typed output
 port to a GeometrySystem input port.

 Geometry sources register frames with GeometryWorld, declaring the frames that
 the source owns and is responsible for computing kinematics values (pose,
 velocity, and acceleration). The geometry source can also "hang" geometry on
 those registered frames in a "rigid" manner.  As the frame moves,
 the associated geometry moves with it.  The associated geometry can consist of
 a single shape, or a hierarchy of shapes (such that the whole hierarchy is
 treated as a single rigid union of shapes).

 @section geom_world_workflow Workflow for Geometry Sources

 For a class that serves as a geometry source, the expected work flow is as
 follows in two phases: registration and evaluation.

 @subsection geom_world_declare_workflow Registration:

 @code
 // Assuming the following variables are defined.
 GeometryContext& context;
 GeometryWorld* geometry_world;

 // Register as a geometry source. It is important to save the unique SourceId
 // into a class member.
 source_id_ = geometry_world->RegisterNewSource();

 // Declare moving frames and the hanging geometry. The geometry source is
 // responsible for saving the FrameId instances for use later.
 FrameId f0 = geometry_world->RegisterFrame(context, source_id_);
 // Instantiate the geometry to hang on the frame and define its pose.
 unique_ptr<GeometryInstance> instance = ...;
 Isometry3<double> pose = ...;
 GeometryId g0 = geometry_world->RegisterGeometry(context, source_id_, f0,
                                                  move(instance), pose0);
 FrameId f1 = geometry_world->RegisterFrame(context, source_id_);
 instance.reset(...);  // Create a new GeometryInstance.
 GeometryId g1 = geometry_world->RegisterGeometry(context, source_id_, f1,
                                                  move(instance), pose0);
 // continue registering frames and geometries.
 @endcode

 @subsection geom_world_value_workflow For setting values during the simulation.

 @code{.cpp}
 // Assume the following variables are defined.
 const GeometryContext& context;
 GeometryWorld* geometry_world;

 // Acquire an instance of FrameKinematicsSet and populate it with kinematics
 // values.
 FrameKinematicsSet fks = geometry_world->GetFrameKinematicsSet(source_id_);

 foreach ( FrameId frame_id : frames_ ) {
    // Compute SpatialPose, SpatialVelocity, and SpatialAcceleration
    SpatialPose X_WF = ...;
    SpatialVelocity V_WF = ...;
    SpatialAcceleration A_WF = ...;
    fks.SetFrameFullKinematics(frame_id, X_WF, V_WF, A_WF);
 }
 geometry_world->SetFrameKinematics(context, fks);
 @endcode

 @subsection geom_world_usage_notes Notes on workflow

 These code snippets shows a general workflow as an order of operations, but
 should not be taken as a literal suggestion. It merely underscores several
 things:
   - A geometry source must register itself before doing anything else.
   - The SourceId returned is very important and should be saved as a member of
     the class. All operations on GeometryWorld depend on that unique id.
   - In order to register a geometry, the frame it hangs on must be registered
     first.
   - The geometry source is responsible for creating and defining the
     GeometryInstance. GeometryWorld merely takes ownership when passed over.
   - When evaluating frames' kinematic values for a particular context, always
     acquire a new instance of the FrameKinematicsSet from GeometryWorld.

  What these examples _don't_ cover:
    - The example shows saving FrameId values in local variables (e.g., `f0`).
      In practice, these values would be saved into a mapping structure that
      will associate the FrameId with the source of that frame's values. How
      that mapping is done is up to the class serving as geometry source.
    - It provides no details on _how_ to instantiate a GeometryInstance (see
      that class's documentation for details).
    - It doesn't give an example of building a _hierarchy_ of geometry
      instances. Each GeometryInstance includes a frame, so new geometry can be
      "hung" from previously registered geometries.
    - There are methods for _removing_ registered frames and geometries.

  Finally, the outlined work flow assumes execution independent of the System
  architecture. The workflow changes slightly when running in a Diagram. For
  details on the change, see GeometrySystem.

 @tparam T The underlying scalar type. Must be a valid Eigen scalar.
 */
template <typename T>
class GeometryWorld {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GeometryWorld)

  /** Default constructor. */
  GeometryWorld() = default;

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
   */
  SourceId RegisterNewSource();

  /** Reports if the identifier references a registered source. */
  bool SourceIsRegistered(SourceId id) const;

  // TODO(SeanCurtis-TRI): Add metadata. E.g., name, some kind of payload, etc.
  /**
   Declares a new frame on this channel, receiving the unique id for the new
   frame.

   @param context       A mutable context.
   @param source_id     The identifier for the geometry source registering the
                        frame.
   */
  FrameId RegisterFrame(GeometryContext<T>* context, SourceId source_id);

  /**
   Declares a `geometry` instance as "hanging" from the specified frame at the
   given pose relative to the frame. The geometry is _rigidly_ affixed to the
   parent frame.

   If the `id` is not a valid frame identifier declared through this channel, an
   exception will be thrown.

   @param context     A mutable context.
   @param source_id   The identifier for the geometry source registering the
                      frame.
   @param frame_id    The id for the frame `F` to hang the geometry on.
   @param geometry    The geometry to hang.
   @param X_FG        the transform X_FG which transforms the geometry
                      from its canonical frame `G` to the parent frame `F`,
                      i.e., the geometry's pose relative to its parent.
   @return A unique identifier for the added geometry.
   */
  GeometryId RegisterGeometry(GeometryContext<T>* context,
                              SourceId source_id,
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

   @param context      A mutable context.
   @param source_id    The identifier for the geometry source registering the
                       geometry.
   @param geometry_id  The id for the geometry to hang the declared geometry on.
   @param geometry     The geometry to hang.
   @param X_FG         the transform X_FG which transforms the geometry
                       from its canonical frame `G` to the parent frame `F`,
                       i.e., the geometry's pose relative to its parent.
   @return A unique identifier for the added geometry.
   */
  GeometryId RegisterGeometry(GeometryContext<T>* context,
                              SourceId source_id,
                              GeometryId geometry_id,
                              std::unique_ptr<GeometryInstance> geometry,
                              const Isometry3<T>& X_FG);

  /**
   Adds the given geometry to the world as anchored geometry.
   @param context       A mutable context.
   @param source_id     The identifier for the geometry source registering the
                        geometry.
   @param geometry      The geometry to add to the world.
   @param X_WG          A transform from the geometry's canonical space to
                        world space.
   @returns The index for the added geometry.
   */
  GeometryId RegisterAnchoredGeometry(
      GeometryContext<T>* context, SourceId source_id,
      std::unique_ptr<GeometryInstance> geometry, const Isometry3<T>& X_WG);

  /** @} */

  /** @name Removal methods

   These methods provide the interface for removing registered sources, frames,
   and geometries.
   @{
   */

  /**
   Removes the given source from the set of active sources. All registered
   frames and geometries for this source will also be removed from the world.
   After this call, the source_id will no longer accept registration of frames
   or geometry.

   The system aborts if the `source_id` is not an active source.

   @param context     The context.
   @param source_id   The identifier of the source to be deactivated and
                      removed.
   */
  void RemoveSource(GeometryContext<T>* context, SourceId source_id);

  /**
   Clears all the registered frames and geometries from this source, but leaves
   the source active for future registration of frames and geometries.

   The system aborts if the `source_id` is not an active source.

   @param context     The context.
   @param source_id   The identifier of the source to be deactivated and
                      removed.
   */
  void ClearSource(GeometryContext<T>* context, SourceId source_id);

  /**
   Removes the given frame from the the indicated source's frames. All
   registered geometries connected to this frame will also be removed from the
   world.

   The system aborts if the `source_id` is not an active source, or if the
   `frame_id` does not belong to the given source.

   @param context     The context.
   @param source_id   The identifier for the owner geometry source.
   @param frame_id    The identifier of the frame to remove.
   */
  void RemoveFrame(GeometryContext<T>* context, SourceId source_id,
                   FrameId frame_id);


  /**
   Removes the given geometry from the the indicated source's geometries. All
   registered geometries connected to this geometry will also be removed from
   the world.

   The system aborts if the `source_id` is not an active source, or if the
   `frame_id` does not belong to the given source.

   @param context     The context.
   @param source_id   The identifier for the owner geometry source.
   @param geometry_id The identifier of the geometry to remove.
   */
  void RemoveGeometry(GeometryContext<T>* context, SourceId source_id,
                      GeometryId geometry_id);
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

   @param context       The context against which all declarations have been
                        made.
   @param source_id     The identifier of the evaluating geometry source.
   */
  FrameKinematicsSet<T> GetFrameKinematicsSet(const GeometryContext<T>& context,
                                              SourceId source_id);

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

   @param context           A mutable context; used for updating cache.
   @param frame_kinematics  The kinematics data for the frames registered by a
                            single source.
   */
  void SetFrameKinematics(GeometryContext<T>* context,
                          const FrameKinematicsSet<T>& frame_kinematics);

  /** @} */

  /**
   Allocates the context state that GeometryWorld requires.
   @returns  A vector of abstract values which will be accessed and managed by
             GeometryWorld.
   */
  std::vector<std::unique_ptr<drake::systems::AbstractValue>>
  AllocateAbstractValues();

 private:
  // GeometryWorld has members that are specific implementations of the
  // GeometryEngine interface. It does this so that it can support multiple
  // geometry engine implementations simultaneously to allow for picking the
  // implementation best suited for particular queries.
//  BulletGeometryEngine bullet_engine_;
//  FclGeometryEngine fcl_engine_;

  // The set of all registered sources.
  std::unordered_set<SourceId> sources_;
};
}  // namespace geometry
}  // namespace drake
