#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/frame_id_vector.h"
#include "drake/geometry/frame_kinematics_vector.h"
#include "drake/geometry/geometry_engine.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_index.h"
#include "drake/geometry/internal_frame.h"
#include "drake/geometry/internal_geometry.h"

namespace drake {
namespace geometry {

// forward declarations
class FrameIdVector;
template <typename T> struct GeometryFrame;
template <typename T> class GeometrySystem;
template <typename T> class GeometryWorld;

/** @name Structures for maintaining the entity relationships
 @{ */

/** Collection of unique frame ids. */
using FrameIdSet = std::unordered_set<FrameId>;

//@}

/**
 The context-dependent state of GeometryWorld. This serves as an AbstractValue
 in the context. GeometryWorld's time-dependent state includes more than just
 values; objects can be added to or removed from the world over time. Therefore,
 GeometryWorld's context-dependent state includes values and structure -- the
 topology of the world.

 @tparam T The underlying scalar type. Must be a valid Eigen scalar. */
template <typename T>
class GeometryState {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GeometryState)

  /** Default constructor. */
  GeometryState();

  /** @name        State introspection.

   Various methods that allow reading the state's properties and values.
  @{ */

  /** Reports the number of active sources -- whether they have frames or not.
   */
  int get_num_sources() const {
    return static_cast<int>(source_frame_id_map_.size());
  }

  /** Reports the total number of frames -- across all sources. */
  int get_num_frames() const { return static_cast<int>(frames_.size()); }

  /** Reports the total number of geometries. */
  int get_num_geometries() const {
    return static_cast<int>(geometries_.size());
  }

  /** Iterator through the keys of an unordered map. */
  template <typename K, typename V>
  class MapKeyIterator {
   public:
    class iterator {
      friend class MapKeyIterator;
     public:
      const K& operator*() const { return itr_->first; }
      const iterator& operator++() { ++itr_; return *this; }
      bool operator!=(const iterator& other) { return itr_ != other.itr_; }

     protected:
      explicit iterator(typename std::unordered_map<K, V>::const_iterator itr)
          : itr_(itr) {}

     private:
      typename std::unordered_map<K, V>::const_iterator itr_;
    };

    explicit MapKeyIterator(std::unordered_map<K, V> map) : map_(map) {}
    iterator begin() const { return iterator(map_.begin()); }
    iterator end() const { return iterator(map_.end()); }

   private:
    std::unordered_map<K, V> map_;
  };

  // TODO(SeanCurtis-TRI): Kill this
  /** Provides a range iterator for all of the source ids in the world. The
   order is not generally guaranteed; but it will be consistent as long as there
   are no changes to the topology. This is intended to be used as:
   @code
   for (SourceId id : state.get_source_ids()) {
    ...
   }
   @endcode  */
  MapKeyIterator<SourceId, FrameIdSet> get_source_ids() const {
    return MapKeyIterator<SourceId, FrameIdSet> (source_frame_id_map_);
  }

  /** Provides a range iterator for all of the geometry ids in the world. The
   order is not generally guaranteed; but it will be consistent as long as there
   are no changes to the topology. This is intended to be used as:
   @code
   for (GeometryId id : state.get_geometry_ids()) {
    ...
   }
   @endcode  */
  MapKeyIterator<GeometryId, internal::InternalGeometry>
  get_geometry_ids() const {
    return MapKeyIterator<GeometryId, internal::InternalGeometry>(geometries_);
  }

  /** Provides a range iterator for all of the frame ids in the world. The
   order is not generally guaranteed; but it will be consistent as long as there
   are no changes to the topology. This is intended to be used as:
   @code
   for (FrameId id : state.get_frame_ids()) {
    ...
   }
   @endcode  */
  MapKeyIterator<FrameId, internal::InternalFrame>
  get_frame_ids() const {
    return MapKeyIterator<FrameId, internal::InternalFrame>(frames_);
  }

  /** Reports the frame group for the given frame.
   @internal This is equivalent to the old "model instance id". */
  int get_frame_group(FrameId frame_id) const {
    return frames_.at(frame_id).get_frame_group();
  }

  /** Reports the name of the frame. */
  const std::string& get_frame_name(FrameId frame_id) const {
    return frames_.at(frame_id).get_name();
  }

  /** Reports the pose of the geometry with the given id. */
  const Isometry3<T>& get_pose_in_world(GeometryId geometry_id) const {
    return X_WG_[geometries_.at(geometry_id).get_engine_index()];
  }

  /** Reports the pose of the frame with the given id. */
  const Isometry3<T>& get_pose_in_parent(FrameId frame_id) const {
    return X_PF_[frames_.at(frame_id).get_pose_index()];
  }

  /** Reports true if the given `source_id` references an active source. */
  bool source_is_active(SourceId source_id) const;

  /** Reports the source name for the given source id.
   @param id  The identifier of the source.
   @return The name of the source.
   @throws std::logic_error if the id does _not_ map to an active source. */
  const std::string& get_source_name(SourceId id) const;

  /** Reports the pose, relative to the registered _frame_, for the geometry
   the given identifier refers to.
   @param geometry_id     The id of the queried geometry.
   @return The geometry's pose relative to its frame.
   @throws std::logic_error  If the `geometry_id` does _not_ map to a valid
                             GeometryInstance. */
  Isometry3<T> GetPoseInFrame(GeometryId geometry_id) const;

  /** Reports the pose, relative to the registered parent, for the geometry
   the given identifier refers to. If the geometry was registered directly to
   a frame, this _must_ produce the same pose as GetPoseInFrame().
   @param geometry_id     The id of the queried geometry.
   @return The geometry's pose relative to its registered parent.
   @throws std::logic_error  If the `geometry_id` does _not_ map to a valid
                             GeometryInstance. */
  Isometry3<T> GetPoseInParent(GeometryId geometry_id) const;

  //@}

  /** @name       Relationship queries

   Various methods that map identifiers for one type of entity to its related
   entities.
   @{ */

  /** Reports if the given frame id was registered to the given source id.
   @param frame_id      The query frame id.
   @param source_id     The query source id.
   @returns True if `frame_id` was registered on `source_id`.
   @throws std::logic_error  If the `frame_id` does _not_ map to a frame or the
                             identified source is not active. */
  bool BelongsToSource(FrameId frame_id, SourceId source_id) const;

  /** Reports if the given geometry id was ultimately registered to the given
   source id.
   @param geometry_id      The query geometry id.
   @param source_id     The query source id.
   @returns True if `geometry_id` was registered on `source_id`.
   @throws std::logic_error  If the `geometry_id` does _not_ map to a valid
                             geometry or the identified source is not active */
  bool BelongsToSource(GeometryId geometry_id, SourceId source_id) const;

  /** Retrieves the frame id on which the given geometry id is declared.
   @param geometry_id   The query geometry id.
   @returns An optional FrameId based on a successful lookup.
   @throws std::logic_error  If the `geometry_id` does _not_ map to a geometry
                             which belongs to an existing frame.*/
  FrameId GetFrameId(GeometryId geometry_id) const;

  /** Returns the set of frames registered to the given source.
   @param source_id     The identifier of the source to query.
   @return  The set of frames associated with the id.
   @throws std::logic_error If the `source_id` does _not_ map to an active
                            source. */
  const FrameIdSet& GetFramesForSource(SourceId source_id) const;

  //@}

  /** @name        State management

   The methods that modify the state including: adding/removing entities from
   the state, modifying values in the state, etc.
   @{ */

  /** Registers a new, named source into the state.
    @param name          The optional name of the source. If none or the empty
                        string is provided it will be named "Source_##" where
                        the number is the value of the returned SourceId.
   @trhows std::logic_error is thrown if the name is _not_ unique. */
  SourceId RegisterNewSource(const std::string& name = "");

  /** Removes  all frames and geometry registered from the identified source.
   The source remains active and further frames and geometry can be registered
   on it.
   @param source_id     The identifier for the source to clear.
   @throws std::logic_error  If the `source_id` does _not_ map to an active
                             source. */
  void ClearSource(SourceId source_id);

  /** Removes the given frame from the the indicated source's frames. All
   registered geometries connected to this frame will also be removed from the
   world.
   @param source_id     The identifier for the owner geometry source.
   @param frame_id      The identifier of the frame to remove.
   @throws std::logic_error  1. If the `source_id` does _not_ map to an active
                             source, or
                             2. the `frame_id` does not map to a valid frame, or
                             3. the `frame_id` maps to a frame that does not
                             belong to the indicated source. */
  void RemoveFrame(SourceId source_id, FrameId frame_id);

  /** Removes the given geometry from the the indicated source's frames. Any
   geometry that was hung from the indicated geometry will _also_ be removed.
   @param source_id     The identifier for the owner geometry source.
   @param geometry_id   The identifier of the frame to remove.
   @throws std::logic_error  1. If the `source_id` does _not_ map to an active
                             source, or
                             2. the `geometry_id` does not map to a valid
                             geometry, or
                             3. the `geometry_id` maps to a geometry that does
                             not belong to the indicated source. */
  void RemoveGeometry(SourceId source_id, GeometryId geometry_id);

  /** Registers a new frame for the given source, the id of the new frame is
   returned.
   @param source_id    The id of the source for which this frame is allocated.
   @param frame        The frame to register.
   @returns  A newly allocated frame id.
   @throws std::logic_error  If the `source_id` does _not_ map to an active
                             source. */
  FrameId RegisterFrame(SourceId source_id, const GeometryFrame<T>& frame);

  /** Registers a new frame for the given source as a child of a previously
   registered frame. The id of the new frame is returned.
   @param source_id    The id of the source for which this frame is allocated.
   @param parent_id    The id of the parent frame.
   @param frame        The frame to register.
   @returns  A newly allocated frame id.
   @throws std::logic_error  1. If the `source_id` does _not_ map to an active
                             source, or
                             2. If the `parent_id` does _not_ map to a known
                             frame or does not belong to the source. */
  FrameId RegisterFrame(SourceId source_id, FrameId parent_id,
                        const GeometryFrame<T>& frame);

  /** Registers a GeometryInstance with the state. The state takes ownership of
   the geometry and associates it with the given frame and source. Returns the
   new identifier for the GeometryInstance.
   @param source_id    The id of the source to which the frame and geometry
                       belongs.
   @param frame_id     The id of the frame on which the geometry is to hang.
   @param geometry     The geometry to get the id for. The state takes
                       ownership of the geometry.
   @returns  A newly allocated geometry id.
   @throws std::logic_error  1. the `source_id` does _not_ map to an active
                             source, or
                             2. the `frame_id` doesn't belong to the source, or
                             3. The `geometry` is equal to `nullptr`. */
  GeometryId RegisterGeometry(
      SourceId source_id, FrameId frame_id,
      std::unique_ptr<GeometryInstance<T>> geometry);

  /** Registers a GeometryInstance with the state. Rather than hanging directly
   from a _frame_, the instance hangs on another geometry instance. The input
   `geometry` instance's pose is assumed to be relative to that parent geometry
   instance. The state takes ownership of the geometry and associates it with
   the given geometry parent (and, ultimately, the parent geometry's frame) and
   source. Returns the new identifier for the input `geometry`.
   @param source_id    The id of the source on which the geometry is being
                       declared.
   @param geometry_id  The parent geometry for this geometry.
   @param geometry     The geometry to get the id for. The state takes
                       ownership of the geometry.
   @returns  A newly allocated geometry id.
   @throws std::logic_error 1. the `source_id` does _not_ map to an active
                            source, or
                            2. the `geometry_id` doesn't belong to the source,
                            or
                            3. the `geometry` is equal to `nullptr`. */
  GeometryId RegisterGeometryWithParent(
      SourceId source_id, GeometryId geometry_id,
      std::unique_ptr<GeometryInstance<T>> geometry);

  /** Registers a GeometryInstance with the state as anchored geometry. This
   registers geometry which "hangs" from the world frame and never moves.
   The `geometry`'s pose value is relative to the world frame. The state takes
   ownership of the geometry and associates it with the given source. Returns
   the new identifier for the GeometryInstance.
   @param source_id    The id of the source on which the geometry is being
                       declared.
   @param geometry     The geometry to get the id for. The state takes
                       ownership of the geometry.
   @returns  A newly allocated geometry id.
   @throws std::logic_error  If the `source_id` does _not_ map to an active
                             source. */
  GeometryId RegisterAnchoredGeometry(
      SourceId source_id,
      std::unique_ptr<GeometryInstance<T>> geometry);

  //@}

  /** Sets the kinematic poses for the frames indicated by the given ids. This
   method assumes that the `ids` have already been validated by
   ValidateFrameIds().
   @param ids   The ids of the frames whose poses are being set.
   @param poses The frame pose values.
   @throws std::logic_error  if the poses don't "match" the ids. */
  void SetFramePoses(const FrameIdVector& ids, const FramePoseSet<T>& poses);

  /** Sets the kinematic velocities for the frames indicated by the given ids.
   This method assumes that the `ids` have already been validated by
   ValidateFrameIds().
   @param ids        The ids of the frames whose poses are being set.
   @param velocities The frame velocity values.
   @throws std::logic_error  if the velocities don't "match" the ids.  */
  void SetFrameVelocities(const FrameIdVector& ids,
                          const FrameVelocitySet<T>& velocities);

  /** Method that performs any final book-keeping/updating on the state after
   _all_ of the stat's frames have had their poses updated. */
  void FinalizePoseUpdate() { geometry_engine_->UpdateWorldPoses(X_WG_); }

  /** Method that performs any final book-keeping/updating on the state after
   _all_ of the stat's frames have had their poses updated. */
  void FinalizeVelocityUpdate() {
    throw std::runtime_error("Not implemented.");
  }

  // TODO(SeanCurtis-TRI): Why are these public?  Testing?

  // TODO(SeanCurtis-TRI): When *does* this get invoked?
  /** Confirms that the set of ids provided include _all_ of the frames
   registered to the set's source id and that no extra frames are included.
   @param ids The id set to validate.
   @throws std::logic_error if the set is inconsistent with known topology. */
  void ValidateFrameIds(const FrameIdVector& ids) const;

  /** Confirms that the pose data is consistent with the set of ids.
   @param ids       The id set to test against.
   @param poses     The poses to test.
   @throws  std::logic_error if the two data sets don't have matching source ids
                             or matching size. */
  void ValidateFramePoses(const FrameIdVector& ids,
                          const FramePoseSet<T>& poses) const;

  /** Confirms that the velocity data is consistent with the set of ids.
   @param ids       The id set to test against.
   @param poses     The velocities to test.
   @throws  std::logic_error if the two data sets don't have matching source ids
                             or matching size. */
  void ValidateFrameVelocities(const FrameIdVector& ids,
                               const FrameVelocitySet<T>& velocities) const;

  /** Informs the state that all kinematics data has been set (via calls to
   SetFrameKinematics()). Allows the state to update internal bookkeeping. */
  void FinalizeKinematicsUpdate() { geometry_engine_->UpdateWorldPoses(X_WG_); }

  /** Finds the identifier for parent geometry of the given geometry_id. The
   optional will be invalid if geometry_id's parent is the frame itself.
   @param geometry_id   The identifier for the geometry whose parent is to be
                        found.
   @returns The _optional_ parent of the queried geometry_id. Will be valid if
            `geometry_id` has a geometry parent, invalid if it has a frame
            parent.
   @throws std::logic_error If geometry_id is _not_ a valid geometry id. */
  optional<GeometryId> FindParentGeometry(GeometryId geometry_id) const;

 private:
  // Allow geometry dispatch to peek into GeometryState.
  friend void DispatchLoadMessage(const GeometryState<T>&);

  // Allow GeometryWorld unique access to the state members to perform queries.
  friend class GeometryWorld<T>;

  // Allow GeometrySystem unique access to the state members to perform queries.
  friend class GeometrySystem<T>;

  // Friend declaration so that the internals of the state can be confirmed in
  // unit tests.
  template <class U> friend class GeometryStateTester;

  // Gets the source id for the given frame id. Throws std::logic_error if the
  // frame belongs to no registered source.
  SourceId get_source_id(FrameId frame_id) const;

  // Does the work of registering geometry. Attaches the given geometry to the
  // identified frame (which must belong to the identified source). The geometry
  // can have an optional parent.
  // Throws an exception as documented in RegisterGeometry().
  GeometryId RegisterGeometryHelper(
      SourceId source_id, FrameId frame_id,
      std::unique_ptr<GeometryInstance<T>> geometry,
      optional<GeometryId> parent = {});

  // The origin from where an invocation of RemoveFrameUnchecked was called.
  // The origin changes the work that is required.
  enum class RemoveFrameOrigin {
    SOURCE,     // Invoked by ClearSource().
    FRAME,      // Invoked by RemoveFrame().
    RECURSE     // Invoked by recursive call in RemoveGeometryUnchecked.
  };

  // Performs the work necessary to remove the identified frame from
  // GeometryWorld. The amount of work depends on the context from which this
  // method is invoked:
  //
  //  - ClearSource(): ClearSource() is deleting *all* frames and geometries.
  //    It explicitly iterates through the frames (regardless of hierarchy).
  //    Thus, recursion is unnecessary, removal from parent references is
  //    likewise unnecessary (and actually wrong).
  //  - RemoveFrame(): The full removal is necessary; recursively remove child
  //    frames (and child geometries), removing references to this id from
  //    the source and its parent frame (if not the world).
  //   - RemoveFrameUnchecked(): This is the recursive call; it's parent
  //    is already slated for removal, so parent references can be left alone,
  //    but recursion is necessary.
  void RemoveFrameUnchecked(FrameId frame_id, RemoveFrameOrigin caller);

  // The origin from where an invocation of RemoveGeometryUnchecked was called.
  // The origin changes the work that is required.
  enum class RemoveGeometryOrigin {
    FRAME,      // Invoked by RemoveFrame().
    GEOMETRY,   // Invoked by RemoveGeometry().
    RECURSE     // Invoked by recursive call in RemoveGeometryUnchecked.
  };

  // Performs the work necessary to remove the identified geometry from
  // GeometryWorld. The amount of work depends on the context from which this
  // method is invoked:
  //
  //  - RemoveFrame(): RemoveFrame() is deleting *all* geometry attached to the
  //    frame. It explicitly iterates through those geometries. Thus,
  //    recursion is unnecessary, removal from parent references is likewise
  //    unnecessary (and actually wrong).
  //   - RemoveGeometry(): The full removal is necessary; recursively remove
  //    children and remove this geometry from the child lists of its parent
  //    frame and, if exists, parent geometry.
  //   - RemoveGeometryUnchecked(): This is the recursive call; it's parent
  //    is already slated for removal, so parent references can be left alone.
  void RemoveGeometryUnchecked(GeometryId geometry_id,
                               RemoveGeometryOrigin caller);

  // Recursively updates the frame and geometry _pose_ information for the tree
  // rooted at the given frame, whose parent's pose in the world frame is given
  // as `X_WP`.
  void UpdatePosesRecursively(const internal::InternalFrame& frame,
                              const Isometry3<T>& X_WP,
                              const FrameIdVector& ids,
                              const FramePoseSet<T>& poses);

  // Recursively updates the frame and geometry _velocity_ values for the tree
  // rooted at the given frame, whose parent's pose and velocity in the world
  // frame is given as `V_WP`, respectively. This assumes that the
  // poses have already been recursively updated.
  void UpdateVelocitiesRecursively(const internal::InternalFrame& frame,
                                   const Isometry3<T>& X_WP,
                                   const FrameIdVector& ids,
                                   const FrameVelocitySet<T>& poses) {
    // TODO(SeanCurtis-TRI): THe interface on this needs to change. I need the
    // parent velocity as well.
    throw std::runtime_error("Not implemented");
  }

  // TODO(SeanCurtis-TRI): Several design issues on this:
  //  1. It should *ideally* be const.
  //  2. Can I guarantee that it's always 0?
  // The frame identifier for the world frame.
  FrameId kWorldFrame;

  // ---------------------------------------------------------------------
  // Maps representing the registered state of sources, frames and geometries,
  // and their relationships. This data should only change at major discrete
  // events where frames/geometries are introduced and removed. They do *not*
  // depend on time-dependent input values (e.g., System::InputPort).

  // The active geometry sources and the frame ids that have been registered
  // on them.
  std::unordered_map<SourceId, FrameIdSet> source_frame_id_map_;

  // The active geometry sources and the frame ids that have the world frame as
  // the parent frame. For a completely flat hierarchy, this contains the same
  // values as the corresponding entry in source_frame_id_map_.
  std::unordered_map<SourceId, FrameIdSet> source_root_frame_map_;

  // The active geometry sources and the _anchored_ geometries that have been
  // registered on them. These don't fit in the frame hierarchy because they do
  // not belong to dynamic frames.
  std::unordered_map<SourceId, std::unordered_set<GeometryId>>
      source_anchored_geometry_map_;

  // The active geometry source names. Each name is unique and the keys in this
  // map should be identical to those in source_frame_id_map_ and
  // source_root_frame_map_.
  std::unordered_map<SourceId, std::string> source_names_;

  // The frame data, keyed on unique frame identifier.
  std::unordered_map<FrameId, internal::InternalFrame> frames_;

  // The geometry data, keyed on unique geometry identifiers.
  std::unordered_map<GeometryId, internal::InternalGeometry> geometries_;

  // The _anchored_ geometry data, keyed on the unique geometry identifiers.
  std::unordered_map<GeometryId, AnchoredGeometryIndex> anchored_geometries_;

  // This *implicitly* maps each extant geometry engine index to its
  // corresponding unique geometry identifier. It assumes that the index in the
  // vector *is* the index in the engine.
  // It should be an invariant that:
  // geometries_[geometry_index_id_map_[i]].get_engine_index() == i is true.
  std::vector<GeometryId> geometry_index_id_map_;

  // This *implicitly* maps each extant anchored geometry engine index to its
  // corresponding unique geometry identifier. It assumes that the index in the
  // vector *is* the index in the engine.
  // It should be an invariant that:
  // geometries_[geometry_index_id_map_[i]].get_engine_index() == i is true.
  std::vector<GeometryId> anchored_geometry_index_id_map_;

  // The pose of each geometry relative to the frame to which it belongs. Each
  // geometry has an "engine index". That geometry's pose is stored in this
  // vector at that engine index. Because the geometries are rigidly fixed to
  // frames, these values are a property of the topology and _not_ the time-
  // dependent frame kinematics.
  std::vector<Isometry3<T>> X_FG_;

  // The pose of each geometry relative to the *world* frame. The invariant
  // X_FG_.size() == X_WG_.size() should always be true. This vector contains
  // the values from the last update invocation and is the write target of the
  // next invocation to update.
  std::vector<Isometry3<T>> X_WG_;

  // ---------------------------------------------------------------------
  // These values depend on time-dependent input values (e.g., current frame
  // poses.

  // TODO(SeanCurtis-TRI): This is currently conceptual. Ultimately, this needs
  //  to appear in both the context (as an input) and the cache (as an output).
  //  the discrete callback should allow me to swap a successful cache update
  //  to the context for the next step.
  // Map from the frame id to the *current* pose of the frame it identifies, F,
  // relative to its parent frame, P: X_PF, where X_PF is measured and expressed
  std::vector<Isometry3<T>> X_PF_;

  // The underlying geometry engine. The topology of the engine does *not*
  // change with respect to time. But its values do. This straddles the two
  // worlds, maintaining its own persistent topological state and derived
  // time-dependent state. This *could* be constructed from scratch at each
  // evaluation based on the previous data, but its internal data structures
  // rely on temporal coherency to speed up the calculations. Thus we persist
  // and copy it.
  copyable_unique_ptr<GeometryEngine<T>> geometry_engine_;
};
}  // namespace geometry
}  // namespace drake
