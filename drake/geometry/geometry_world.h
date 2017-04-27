#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/frame_kinematics_set.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace geometry {

// Forward declarations.
template <typename T> class FrameKinematicsSet;
template <typename T> class GeometryContext;
template <typename T> class GeometryInstance;
template <typename T> class GeometryState;

// TODO(SeanCurtis-TRI): Review this documentation to confirm that it's
// consistent with what I ended up implementing.
/** GeometryWorld serves as the common geometric space where disparate
 subsystems can perform geometric queries on geometries introduced into the
 simulator by other subsystems. The geometries are associated with "parent"
 frames; as
 the frame moves, the geometry moves with it in a rigid manner. These parent
 frames are moved by the subsystem that introduced the geometry. GeometryWorld
 performs geometric queries on the geometry (e.g., ray intersection, minimum
 distance between geometries, intersection information, etc.)  These results
 provided by these queries depend on the kinematics of the parent frames.
 Prior to each query, the current frame kinematics are reported to GeometryWorld
 to produce a coherent view of the geometric world.

 As geometry is introduced into the simulator, it must be _registered_ with
 GeometryWorld so that it can be included in the queries.

 GeometryWorld is ignorant of the interpretation of the geometry and of what
 mechanisms cause it to move. This knowledge is owned by a "geometry source".
 Any class that computes frame kinematics values and can specify the attached
 geometry can serve as a geometry source. These classes must register
 themselves as geometry sources with GeometryWorld. When executed in a System,
 they must be part of a LeafSystem which connects an abstract-valued output
 port to a GeometrySystem input port -- the type of the abstract value is
 FrameKinematicsSet.

 Geometry sources register frames with GeometryWorld, declaring the frames that
 the source owns and is responsible for computing kinematics values (pose,
 velocity, and acceleration). The geometry source can also "hang" geometry on
 those registered frames in a _rigid_ relationship.  As the frame moves,
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
 // into a class member, e.g., source_id_.
 source_id_ = geometry_world->RegisterNewSource(&context);

 // Declare moving frames and the hanging geometry. The geometry source is
 // responsible for saving the FrameId instances for use later.
 FrameId f0 = geometry_world->RegisterFrame(context, source_id_);
 // Instantiate the geometry to hang on the frame and define its pose.
 unique_ptr<GeometryInstance> instance = ...;
 GeometryId g0 = geometry_world->RegisterGeometry(context, source_id_, f0,
                                                  move(instance));
 FrameId f1 = geometry_world->RegisterFrame(context, source_id_);
 instance.reset(...);  // Create a new GeometryInstance.
 GeometryId g1 = geometry_world->RegisterGeometry(context, source_id_, f1,
                                                  move(instance));
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
    fks.ReportFullKinematics(frame_id, X_WF, V_WF, A_WF);
 }
 geometry_world->SetFrameKinematics(context, fks);
 @endcode

 @subsection geom_world_usage_notes Notes on workflow

 These code snippets shows a general workflow as an order of operations, but
 should not be taken as literal suggestions. It merely underscores several
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

 @tparam T The underlying scalar type. Must be a valid Eigen scalar. */
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
     - register geometry moved by registered frames, and
     - register anchored geometry.
   @{ */

  // TODO(SeanCurtis-TRI): Discuss the implications of the name -- where does it
  // appear?
  /** Registers a new geometry source to GeometryWorld, receiving the unique
   identifier for this new source.
   @param context       A mutable geometry context.
   @param name          The optional name of the source. If none is provided
                        it will be named Source## where the number is the
                        value of the returned SourceId.
   @throws std::logic_error if the name duplicates a previously registered
                            source name. */
  SourceId RegisterNewSource(GeometryContext<T>* context,
                             const std::string& name = "");

  /** Reports the source name for the given source id.
   @param id  The identifier of the source.
   @return The name of the source.
   @throws std::logic_error if the id does _not_ map to an active source. */
  const std::string& get_source_name(SourceId id) {
    using std::to_string;
    auto itr = sources_.find(id);
    if (itr != sources_.end()) return itr->second;
    throw std::logic_error(
        "Querying source name for an invalid source id: " + to_string(id) +
            ".");
  }

  /** Reports if the identifier references a registered source. */
  bool SourceIsRegistered(SourceId id) const;

  // TODO(SeanCurtis-TRI): Add metadata. E.g., name, some kind of payload, etc.
  /**
   Declares a new frame on this channel, receiving the unique id for the new
   frame.
   @param context       A mutable context.
   @param source_id     The identifier for the geometry source registering the
                        frame.
   @param frame         The definition of the frame to add.
   @throws std::logic_error  If the `source_id` does _not_ map to an active
                             source. */
  FrameId RegisterFrame(GeometryContext<T>* context, SourceId source_id,
                        const GeometryFrame<T>& frame);

  /**
   Declares a `geometry` instance as "hanging" from the specified frame at the
   given pose relative to the frame. The geometry is _rigidly_ affixed to the
   parent frame.
   @param context     A mutable context.
   @param source_id   The identifier for the geometry source registering the
                      frame.
   @param frame_id    The id for the frame `F` to hang the geometry on.
   @param geometry    The geometry to hang.
   @return A unique identifier for the added geometry.
   @throws std::logic_error  1. the `source_id` does _not_ map to an active
                             source, or
                             2. the `frame_id` doesn't belong to the source, or
                             3. The `geometry` is equal to `nullptr`. */
  GeometryId RegisterGeometry(GeometryContext<T>* context,
                              SourceId source_id,
                              FrameId frame_id,
                              std::unique_ptr<GeometryInstance<T>> geometry);

  /**
   Declares a `geometry` instance as "hanging" from the specified geometry's
   frame `F`, with the given pose relative to that frame. The geometry is
   _rigidly_ affixed to the parent frame.

   This method enables the owner entity to construct rigid hierarchies of posed
   geometries. This rigid structure will all be driven by the declared frame
   to which the root geometry is registered.

   @param context      A mutable context.
   @param source_id    The identifier for the geometry source registering the
                       geometry.
   @param geometry_id  The id for the geometry to hang the declared geometry on.
   @param geometry     The geometry to hang.
   @return A unique identifier for the added geometry.
   @throws std::logic_error 1. the `source_id` does _not_ map to an active
                            source, or
                            2. the `geometry_id` doesn't belong to the source,
                            or
                            3. the `geometry` is equal to `nullptr`. */
  GeometryId RegisterGeometry(GeometryContext<T>* context,
                              SourceId source_id,
                              GeometryId geometry_id,
                              std::unique_ptr<GeometryInstance<T>> geometry);

  /**
   Adds the given geometry to the world as anchored geometry.
   @param context       A mutable context.
   @param source_id     The identifier for the geometry source registering the
                        geometry.
   @param geometry      The geometry to add to the world.
   @returns The index for the added geometry.
   @throws std::logic_error  If the `source_id` does _not_ map to an active
                             source. */
  GeometryId RegisterAnchoredGeometry(
      GeometryContext<T>* context, SourceId source_id,
      std::unique_ptr<GeometryInstance<T>> geometry);

  /** @} */

  /** @name Removal methods

   These methods provide the interface for removing registered frames and
   geometries.
   @{ */

  /**
   Clears all the registered frames and geometries from this source, but leaves
   the source active for future registration of frames and geometries.

   The system aborts if the `source_id` is not an active source.

   @param context     The context.
   @param source_id   The identifier of the source to be deactivated and
                      removed.
   @throws std::logic_error  If the `source_id` does _not_ map to an active
                             source. */
  void ClearSource(GeometryContext<T>* context, SourceId source_id);

  /**
   Removes the given frame from the the indicated source's frames. All
   registered geometries connected to this frame will also be removed from the
   world.

   The system aborts if the `source_id` is not an active source, or if the
   `frame_id` does not belong to the given source.

   @param context     The context.
   @param source_id   The identifier for the owner geometry source.
   @param frame_id    The identifier of the frame to remove. */
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
   @param geometry_id The identifier of the geometry to remove. */
  void RemoveGeometry(GeometryContext<T>* context, SourceId source_id,
                      GeometryId geometry_id);
  /** @} */

  /** @name Evaluation methods

   These are the methods through which values for registered frames are provided
   to GeometryWorld.

   @{ */

  /**
   Requests a frame kinematics set for the given geometry source. This should be
   invoked every time the frame kinematics values are evaluated and provided.

   Aborts if the source identifier does not reference an active source.

   @param source_id     The identifier of the evaluating geometry source. */
  FrameKinematicsSet<T> GetFrameKinematicsSet(SourceId source_id);

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
   @throws std::logic_error If the frame kinematics data is missing any data for
                            registered frames, or includes frame ids that were
                            not registered with the associated source. */
  void SetFrameKinematics(GeometryContext<T>* context,
                          const FrameKinematicsSet<T>& frame_kinematics);

  /** @} */

  /** @name       System-compatible methods

   These methods help bridge the GeometryWorld into the System architecture.
   @{ */

  /**
   Allocates the context state that GeometryWorld requires.
   @returns  A vector of abstract values which will be accessed and managed by
             GeometryWorld. */
  std::vector<std::unique_ptr<drake::systems::AbstractValue>>
  AllocateAbstractValues();

  /**
   Extracts a mutable geometry state from the given GeometryContext. Ultimately,
   this should be a private utility method, but it is made public to facilitate
   testing.
   @param context   The context containing the state.
   @return A pointer to the mutable GeometryState instance. */
  GeometryState<T>* get_mutable_state(GeometryContext<T>* context);

  /**
   Extracts a read-only geometry state from the given GeometryContext.
   Ultimately, this should be a private utility method, but it is made public to
   facilitate testing.
   @param context   The context containing the state.
   @return A const reference to the GeometryState instance. */
  const GeometryState<T>& get_state(const GeometryContext<T>& context);

  /** @} */

 private:
  // Tests to see if the source_id is an active source identifier. Throws an
  // exception if not.
  void AssertValidSource(SourceId source_id) const;

  // The set of all registered sources and their recorded names.
  std::unordered_map<SourceId, std::string> sources_;
};
}  // namespace geometry
}  // namespace drake
