#pragma once

#include <memory>
#include <unordered_map>
#include <unordered_set>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_index.h"

namespace drake {
namespace geometry {

// forward declarations
template <typename T> class FrameKinematicsSet;

/** @name Structures for maintaining the entity relationships
 @{ */
/** A collection of unique frame ids. */
using FrameIdSet = std::unordered_set<FrameId>;

/** A map between a source identifier and the frame identifiers it owns. */
using SourceFrameMap = std::unordered_map<SourceId, FrameIdSet>;

/** A map from a frame id to it's assigned source id. */
using FrameSourceMap = std::unordered_map<FrameId, SourceId>;

/** A collection of unique frame ids. */
using GeometryIdSet = std::unordered_set<GeometryId>;

/** A map between a frame identifier and the frame identifiers it owns. */
using FrameGeometryMap = std::unordered_map<FrameId, GeometryIdSet>;

/** A map between a geometry identifier and a set of related geometry ids. The
 "relationship" depends on the usage. */
using GeometryGeometryMap = std::unordered_map<GeometryId, GeometryIdSet>;

/** A map from a geometry id to it's assigned frame id. */
using GeometryFrameMap = std::unordered_map<GeometryId, FrameId>;

/** A map from a geometry id to index in the engine. */
using GeometryMap = std::unordered_map<GeometryId, GeometryIndex>;

/** A map from a geometry id to X_FG, the pose of the geometry relative to its
 driving frame. */
template <typename T>
using GeometryPoseMap = std::unordered_map<GeometryId, Isometry3<T>>;

//@}

// forward declaration
template <typename T> class GeometryWorld;

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

  GeometryState() = default;

  /** @name        State introspection.

   Various methods that allow reading the state's properties and values.
  @{ */

  /** Reports the number of active sources -- whether they have frames or not.
   */
  int get_num_sources() const {
    return static_cast<int>(source_frame_map_.size());
  }

  /** Reports the total number of frames -- across all sources. */
  int GetNumFrames() const;

  /** Reports the total number of geometries. */
  int get_num_geometries() const {
    return static_cast<int>(geometry_poses_.size());
  }

  /** Reports true if the given `source_id` references an active source. */
  bool source_is_active(SourceId source_id) const;

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

  /** Retrieves the geometry source id on which the given frame id is
   registered.
   @param frame_id      The query frame id.
   @returns The identifier of the source that registered this frame.
   @throws std::logic_error  If the `frame_id` does _not_ map to a frame which
                             belongs to an active source. */
  SourceId GetSourceId(FrameId frame_id) const;

  /** Retrieves the geometry source id on which the given geometry id is
   ultimately registered.
   @param geometry_id      The query geometry id.
   @returns The identifier of the source that registered this geometry.
   @throws std::logic_error  If the `geometry_id` does _not_ map to a geometry
                             which belongs to an active source. */
  SourceId GetSourceId(GeometryId geometry_id) const;

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

  /** Adds and activates a new geometry source to the state.
   @throws std::logic_error  If `source_id` maps to a pre-existing, active
                             source. */
  void RegisterNewSource(SourceId source_id);

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
   @returns  A newly allocated frame id.
   @throws std::logic_error  If the `source_id` does _not_ map to an active
                             source. */
  FrameId RegisterFrame(SourceId source_id);

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

  /** Performs the work for confirming the frame values provided in the
   kinematics set cover the expected set of frames (and no more).
   @param frame_kinematics      The input frame kinematics data.
   @throws std::logic_error 1. Frames registered on the source are not given
                               kinematics values, or
                            2. frames _not_ registered on the source _are_
                               included in the set. */
  void ValidateKinematicsSet(
      const FrameKinematicsSet<T>& frame_kinematics) const;

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
  // Removes the frame without doing any ownership tests. It does _not_ remove
  // the frame from the source_frame_map_; it assumes that the caller will do
  // so. This prevents invalidating iterators into that set.
  void RemoveFrameUnchecked(SourceId source_id, FrameId frame_id);

  // Removes the geometry without doing any ownership test. It does _not_ remove
  // the geometry from the frame_geometry_map_; it assumes that the caller will
  // do so. This prevents invalidating iterators into that set. The output
  // set accumulates all of the geometries that have been removed recursively
  // so that the caller knows which geometries to remove from
  // frame_geometry_map_.
  void RemoveGeometryUnchecked(FrameId frame_id, GeometryId geometry_id,
                               std::unordered_set<GeometryId>* children);

  // This functionality can only be invoked by GeometryWorld.
  // The active geometry sources and the frames that have been registered
  // on them.
  SourceFrameMap source_frame_map_;

  // The map between frames and the sources on which they were registered.
  FrameSourceMap frame_source_map_;

  // Map from frame ids to the geometry that has been hung on it.
  FrameGeometryMap frame_geometry_map_;

  // Map from geometry ids to the frame it has been hung on.
  GeometryFrameMap geometry_frame_map_;

  // Map from a geometry id to the geometries that have been hung from it.
  GeometryGeometryMap linked_geometry_;

  // Map from geometry id to the _pose_ of the geometry it references.
  GeometryPoseMap<T> geometry_poses_;
};
}  // namespace geometry
}  // namespace drake
