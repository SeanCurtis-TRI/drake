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

namespace drake {
namespace geometry {

// Forward declarations.
template <typename T> class FrameKinematicsSet;
template <typename T> class GeometryInstance;
template <typename T> class GeometryState;
template <typename T> struct NearestPair;
template <typename T> struct PointProximity;
struct GeometryPair;

// TODO(SeanCurtis-TRI): Review this documentation to confirm that it's
// consistent with what I ended up implementing.
/** GeometryWorld serves as the common geometric space where disparate
 subsystems can perform geometric queries on geometries introduced into the
 simulator by other subsystems. The geometries are associated with "parent"
 frames; as
 the frame moves, the geometry moves with it in a rigid manner. These parent
 frames are moved by the subsystem that introduced the geometry. GeometryWorld
 performs geometric queries on the geometry (e.g., ray intersection, minimum
 distance between geometries, contact/collision detection, etc.)  The results
 provided by these queries depend on the kinematics of the parent frames.
 Prior to each query, the current frame kinematics are reported to GeometryWorld
 to produce a coherent view of the geometric world.

 As geometry is introduced into the simulator, it must be _registered_ with
 GeometryWorld so that it can be included in the queries.

 GeometryWorld is ignorant of the interpretation of the geometry and of what
 mechanisms cause it to move. This knowledge is owned by a "geometry source".
 A geometry source registers the frames that will be moving, and the geometries
 that are moved by them. All data registered with GeometryWorld is associated
 with a source identifier. Conceptually, a single geometry source registers
 geometries and then provides subsequent kinematics values to move the geometry.
 In practice, the responsibilities can be split so long as the _source
 identifier_ is shared across the split. It is important to note, that all
 manipulation of registered data is keyed on the registering source id. The
 balance of this discussion assumes a monolithic geometry source responsible for
 registration _and_ producing kinematics values.

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
 GeometryState state;
 GeometryWorld* geometry_world;

 // Register as a geometry source. It is important to save the unique SourceId
 // into a class member, e.g., source_id_.
 source_id_ = geometry_world->RegisterNewSource(&state, "source_name");

 // Declare moving frames and the hanging geometry. The geometry source is
 // responsible for saving the FrameId instances for use later.
 FrameId f0 = geometry_world->RegisterFrame(&state, source_id_);
 // Instantiate the geometry to hang on the frame and define its pose.
 unique_ptr<GeometryInstance> instance = ...;
 GeometryId g0 = geometry_world->RegisterGeometry(&state, source_id_, f0,
                                                  move(instance));
 FrameId f1 = geometry_world->RegisterFrame(&state, source_id_);
 instance.reset(...);  // Create a new GeometryInstance.
 GeometryId g1 = geometry_world->RegisterGeometry(&state, source_id_, f1,
                                                  move(instance));
 // continue registering frames and geometries.
 @endcode

 @subsection geom_world_value_workflow For setting values during the simulation.

 @code{.cpp}
 // Assume the following variables are defined.
 const GeometryState state;
 GeometryWorld* geometry_world;

 // Acquire an instance of FrameKinematicsSet and populate it with kinematics
 // values.
 FrameKinematicsSet fks = geometry_world->GetFrameKinematicsSet(state,
                                                                source_id_);

 foreach (FrameId frame_id : frames_) {
    // Compute SpatialPose, SpatialVelocity, and SpatialAcceleration
    SpatialPose X_WF = ...;
    SpatialVelocity V_WF = ...;
    SpatialAcceleration A_WF = ...;
    fks.ReportFullKinematics(frame_id, X_WF, V_WF, A_WF);
 }
 geometry_world->SetFrameKinematics(state, fks);
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
    - There are methods for _removing_ registered frames and geometries which
      are not illustrated here.

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
   @param state         A mutable geometry state for this geometry world.
   @param name          The optional name of the source. If none is provided
                        it will be named Source## where the number is the
                        value of the returned SourceId.
   @throws std::logic_error if the name duplicates a previously registered
                            source name. */
  SourceId RegisterNewSource(GeometryState<T>* state,
                             const std::string& name = "");

  /** Reports the source name for the given source id.
   @param id  The identifier of the source.
   @return The name of the source.
   @throws std::logic_error if the id does _not_ map to an active source. */
  const std::string& get_source_name(const GeometryState<T>& state,
                                     SourceId id) const {
    return state.get_source_name(id);
  }

  /** Reports if the identifier references a registered source. */
  bool SourceIsRegistered(const GeometryState<T>& state, SourceId id) const;

  // TODO(SeanCurtis-TRI): Add metadata. E.g., name, some kind of payload, etc.
  /**
   Declares a new frame on this channel, receiving the unique id for the new
   frame.
   @param state         A mutable geometry state for this geometry world.
   @param source_id     The identifier for the geometry source registering the
                        frame.
   @param frame         The definition of the frame to add.
   @returns  A newly allocated frame id.
   @throws std::logic_error  If the `source_id` does _not_ map to an active
                             source. */
  FrameId RegisterFrame(GeometryState<T>* state, SourceId source_id,
                        const GeometryFrame<T>& frame);

  /** Registers a new frame for the given source as a child of a previously
      registered frame. The id of the new frame is returned.
   @param state        A mutable geometry state for this geometry world.
   @param source_id    The id of the source for which this frame is allocated.
   @param parent_id    The id of the parent frame.
   @param frame        The frame to register.
   @returns  A newly allocated frame id.
   @throws std::logic_error  1. If the `source_id` does _not_ map to an active
                             source, or
                             2. If the `parent_id` does _not_ map to a known
                             frame or does not belong to the source. */
  FrameId RegisterFrame(GeometryState<T>* state, SourceId source_id,
                        FrameId parent_id, const GeometryFrame<T>& frame);

  /**
   Declares a `geometry` instance as "hanging" from the specified frame at the
   given pose relative to the frame. The geometry is _rigidly_ affixed to the
   parent frame.
   @param state       A mutable geometry state for this geometry world.
   @param source_id   The identifier for the geometry source registering the
                      frame.
   @param frame_id    The id for the frame `F` to hang the geometry on.
   @param geometry    The geometry to hang.
   @return A unique identifier for the added geometry.
   @throws std::logic_error  1. the `source_id` does _not_ map to an active
                             source, or
                             2. the `frame_id` doesn't belong to the source, or
                             3. The `geometry` is equal to `nullptr`. */
  GeometryId RegisterGeometry(GeometryState<T>* state,
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

   @param state        A mutable geometry state for this geometry world.
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
  GeometryId RegisterGeometry(GeometryState<T>* state,
                              SourceId source_id,
                              GeometryId geometry_id,
                              std::unique_ptr<GeometryInstance<T>> geometry);

  /**
   Adds the given geometry to the world as anchored geometry.
   @param state         A mutable geometry state for this geometry world.
   @param source_id     The identifier for the geometry source registering the
                        geometry.
   @param geometry      The geometry to add to the world.
   @returns The index for the added geometry.
   @throws std::logic_error  If the `source_id` does _not_ map to an active
                             source. */
  GeometryId RegisterAnchoredGeometry(
      GeometryState<T>* state, SourceId source_id,
      std::unique_ptr<GeometryInstance<T>> geometry);

  /** @} */

  /** @name Removal methods

   These methods provide the interface for removing registered frames and
   geometries.
   @{ */

  /**
   Clears all the registered frames and geometries from this source, but leaves
   the source active for future registration of frames and geometries.
   @param state       A mutable geometry state for this geometry world.
   @param source_id   The identifier of the source to be deactivated and
                      removed.
   @throws std::logic_error  If the `source_id` does _not_ map to an active
                             source. */
  void ClearSource(GeometryState<T>* state, SourceId source_id);

  /**
   Removes the given frame from the the indicated source's frames. All
   registered geometries connected to this frame will also be removed from the
   world.
   @param state       A mutable geometry state for this geometry world.
   @param source_id   The identifier for the owner geometry source.
   @param frame_id    The identifier of the frame to remove.
   @throws std::logic_error If:
                            1. The `source_id` is not an active source, or
                            2. the `frame_id` doesn't belong to the source. */
  void RemoveFrame(GeometryState<T>* state, SourceId source_id,
                   FrameId frame_id);

  /**
   Removes the given geometry from the the indicated source's geometries. All
   registered geometries connected to this geometry will also be removed from
   the world.
   @param state       A mutable geometry state for this geometry world.
   @param source_id   The identifier for the owner geometry source.
   @param geometry_id The identifier of the geometry to remove.
   @throws std::logic_error If:
                            1. The `source_id` is not an active source, or
                            2. the `geometry_id` doesn't belong to the source.
   */
  void RemoveGeometry(GeometryState<T>* state, SourceId source_id,
                      GeometryId geometry_id);
  /** @} */

  /** @name Run-time query methods

   These methods define the interface for updating the state of geometry world
   and performing queries on that state.
   @{ */

  /**
   Requests a frame kinematics set for the given geometry source. This should be
   invoked every time the frame kinematics values are evaluated and provided.

   Aborts if the source identifier does not reference an active source.

   @param state         The geometry state for this geometry world.
   @param source_id     The identifier of the evaluating geometry source. */
  FrameKinematicsSet<T> GetFrameKinematicsSet(const GeometryState<T>& state,
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

   @param state             A mutable geometry state for this geometry world.
   @param frame_kinematics  The kinematics data for the frames registered by a
                            single source.
   @throws std::logic_error If the frame kinematics data is missing any data for
                            registered frames, or includes frame ids that were
                            not registered with the associated source. */
  void SetFrameKinematics(GeometryState<T>* state,
                          const FrameKinematicsSet<T>& frame_kinematics);

  /** @} */

  /** Creates a default-initialized instance of geometry state to serve as an
   operand of  %GeometryWorld operations. This serves as the portable data
   repository for all GeometryWorld operations. */
  std::unique_ptr<GeometryState<T>> CreateState();

  //----------------------------------------------------------------------------
  /** @name                   Proximity Queries

   These queries represent _proximity_ queries -- queries to determine what is
   near by, or what is closest. This is not about overlapping/penetration --
   the proximity of overlapping/penetrating objects should be zero.

   These queries are _not_ affected by collision filtering. */

  //@{

  /** Computes the pair-wise nearest points for all geometries in the world.

   The output vector will *not* be cleared. Contact information will merely be
   added to the vector.

   @param[in]   state           The state against which to queries will be made.
   @param[out]  near_points     A vector containing `O(N²)` pairs, where there
                                are `N` geometries in the world.
   @returns True if the operation was successful. */
  bool ComputePairwiseClosestPoints(
      const GeometryState<T>& state,
      std::vector<NearestPair<T>>* near_points) const;

  // NOTE: This maps to Model::closestPointsAllToAll().
  /** Computes the pair-wise nearest points for all geometries in the given set.

   The output vector will *not* be cleared. Contact information will merely be
   added to the vector.

   @param[in]   state           The state against which to queries will be made.
   @param[in]   ids_to_check    A vector of `N` geometry ids for which the
                                pair-wise points are computed.
   @param[out]  near_points     A vector containing O(N²) pairs.
   @returns True if the operation was successful. */
  bool ComputePairwiseClosestPoints(
      const GeometryState<T>& state,
      const std::vector<GeometryId>& ids_to_check,
      std::vector<NearestPair<T>>* near_points) const;

  // NOTE: This maps to Model::closestPointsPairwise().
  /** Computes the pair-wise nearest points for the explicitly indicated pairs
   of geomeries.

   The output vector will *not* be cleared. Contact information will merely be
   added to the vector.

   @param[in]   state           The state against which to queries will be made.
   @param[in]  pairs        A vector of `N` geometry pairs. The closest points
                            for each pair will be computed.
   @param[out] near_points  A vector of `N` NearestPair values will be added
                            to the vector, one for each input pair.
   @returns True if the operation was successful. */
  bool ComputePairwiseClosestPoints(
      const GeometryState<T>& state,
      const std::vector<GeometryPair>& pairs,
      std::vector<NearestPair<T>>* near_points) const;
  // TODO(SeanCurtis-TRI): Add a version that takes *frame* pairs.

  // NOTE: This maps to Model::collisionDetectFromPoints().
  /** Determines the nearest body/element to a point for a set of points.

   @param[in]   state           The state against which to queries will be made.
   @param[in]   points        An ordered list of `N` points represented
                              column-wise by a `3 x N` Matrix.
   @param[out]  near_bodies   A vector of `N` PointProximity instances such that
                              the iᵗʰ instance reports the nearest body/element
                              to the iᵗʰ point.
   @returns True if the operation was successful. */
  bool FindClosestGeometry(
      const GeometryState<T>& state,
      const Eigen::Matrix3Xd &points,
      std::vector<PointProximity<T>> *near_bodies) const;
#if 0
  // NOTE: This maps to Model::collidingPoints().
  /** Determines which of the given list of `points` are no farther than
   `distance` meters from _any_ collision geometry.

   In other words, the index `i` is included in the returned vector of indices
   iff a sphere of radius `distance`, located at `input_points[i]` collides with
   any collision element in the model.

   @param[in]   points        An ordered list of `N` points represented
                              column-wise by a `3 x N` Matrix.
   @param[in]   distance      The maximum distance from a point that is allowed.
   @param[out]  results       A vector of indices into `points`. Each index
                              indicates that the corresponding point is within
                              closer than `distance` meters away from some
                              geometry. The vector will _not_ be cleared and
                              the indexes will be added to the current values.
   @returns True if the operation was successful. */
  bool FindGeometryProximalPoints(const Matrix3X<T>& points, double distance,
                                  std::vector<size_t>* results) const;

  /** Given a vector of `points` in the world coordinate frame, reports if _any_
   of those `points` lie within a specified `distance` of any collision geometry
   in the model.

   In other words, this method tests if any of the spheres of radius
   `distance` located at `input_points[i]` collides with any part of
   the model. This method returns as soon as any of these spheres collides
   with the model. Points are not checked against one another but only against
   the existing model.

   @param[in]   points    The list of points to check for collisions against the
                          model.
   @param[in]   distance  The radius of a control sphere around each point used
                          to check for collisions with the model.
  @return True if any point is closer than `distance` units to collision
          geometry. */
  bool IsAnyGeometryNear(const Matrix3X<T>& points,
                         double distance) const;

  //@}

  //----------------------------------------------------------------------------
  /** @name                Collision Queries

   These queries detect _collisions_ between geometry. Two geometries collide
   if they overlap each other and are not explicitly excluded through
   @ref collision_filter_concepts "collision filtering". These algorithms find
   those colliding cases, characterize them, and report the essential
   characteristics of that collision.  */

  //@{

  // NOTE: This maps to Model::ComputeMaximumDepthCollisionPoints().
  /** Computes the contact across all elements in the world. Only reports
   results for elements in *contact*; if two elements are separated, there will
   be no result for that pair.

   This method is affected by collision filtering; element pairs that have
   been filtered will not produce contacts, even if their collision geometry is
   penetrating.

   The output vector will *not* be cleared. Contact information will merely be
   added to the vector.

   @param[out]  contacts    All contacts will be aggregated in this structure.
   @returns True if the operation was successful. */
  bool ComputeContact(std::vector<Contact<T>>* contacts) const;

  //@}

  //----------------------------------------------------------------------------
  /** @name                  Ray-casting Queries

   These queries perform ray-cast queries. Ray-cast queries report what, if
   anything, lies in a particular direction from a query point.
   */

  //@{

  // NOTE: This maps to Model::collisionRaycast().
  /** Cast one or more rays against the scene geometry.

   @param[in]  origin           A `3 x N` matrix where each column specifies the
                                position of a ray's origin in the world frame.
                                If `origin` is `3 x 1`, the same origin is used
                                for all rays.
   @param[in]  ray_endpoint     A `3 x N` matrix where each column specifies a
                                point *away* from the corresponding origin
                                through which the ray passes.
   @param[out] distances        A `N`-length vector of distance values. The
                                `iᵗʰ` value is the distance along the `iᵗʰ` ray.
                                The value is negative if the ray didn't hit any
                                surface.
   @param[out] normals          A `3 x N` matrix of values, where the `iᵗʰ`
                                column is the normal of the surface where the
                                `iᵗʰ` ray intersected. Values are undefined if
                                the `iᵗʰ` distance is negative.
   @returns True if the operation was successful.
   */
  bool CastRays(const Matrix3X<T>& origin,
                const Matrix3X<T>& ray_endpoint,
                Eigen::VectorXd* distances, Matrix3X<T>* normals) const;

  //@}

  // TODO(SeanCurtis-TRI): The list of Model functions not yet explicitly
  //  accounted for:
  //      potentialCollisionPoints -- really frigging weird; could include
  //        multiple penetrating points, but also non penetrating points.
  //        a) This is not called outside of tests.
  //        b) This seems to be a *very* bullet-specific method.
  //
#endif

 private:
  // Tests to see if the source_id is an active source identifier. Throws an
  // exception if not.
  void AssertValidSource(const GeometryState<T>& state,
                         SourceId source_id) const;
};
}  // namespace geometry
}  // namespace drake
