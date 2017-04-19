#pragma once

#include <unordered_set>
#include <unordered_map>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/geometry_ids.h"

namespace drake {
namespace geometry {

/** @name Maps for maintaining the entity relationships
 @{
 */
/** A collection of unique frame ids. */
using FrameIdSet = std::unordered_set<FrameId>;

/** A map between a source identifier and the frame identifiers it owns. */
using SourceFrameMap = std::unordered_map<SourceId, FrameIdSet>;

/** A map from a frame id to it's assigned channel id. */
using FrameSourceMap = std::unordered_map<FrameId, SourceId>;

/** A collection of unique frame ids. */
using GeometryIdSet = std::unordered_set<GeometryId>;

/** A map between a channel identifier and the frame identifiers it owns. */
using FrameGeometryMap = std::unordered_map<FrameId, GeometryIdSet>;

/** A map from a frame id to it's assigned channel id. */
using GeometryFrameMap = std::unordered_map<GeometryId, FrameId>;

/** @} */

// forward declaration
template <typename T> class GeometryWorld;

/**
 The context-dependent state of GeometryWorld.

 @tparam T The underlying scalar type. Must be a valid Eigen scalar.
 */
template <typename T>
class GeometryState {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GeometryState)

  GeometryState() = default;

  /** Retrieves the geometry source id on which the given frame id is
   registered.

   An exception is thrown if the id does _not_ map to an active source.

   @param frame_id      The query frame id.
   @returns The identifier of the source that registered this frame.
   */
  SourceId GetSourceId(FrameId frame_id) const;

  /** Retrieves the geometry source id on which the given geometry id is
   ultimately registered.

   An exception is thrown if the id does _not_ map to an active source.

   @param frame_id      The query geometry id.
   @returns The identifier of the source that registered this geometry.
   */
  SourceId GetSourceId(GeometryId geometry_id) const;

  /** Retrieves the frame id on which the given geometry id is declared.

   @param geometry_id   The query geometry id.
   @returns An optional FrameId based on a successful lookup.
   */
  FrameId GetFrameId(GeometryId geometry_id) const;

  // Only allow GeometryWorld to construct the set.
  friend class GeometryWorld<T>;

 private:
  // This functionality can only be invoked by GeometryWorld.

  // Returns the set of frames registered to the given source.
  const FrameIdSet& GetFramesForSource(SourceId source_id) const;

  // Removes the indicated source, removing all frames and geometry registered
  // on that source identifier.
  //
  // Throws an exception if the source is not currently active.
  void RemoveSource(SourceId source_id);

  // Removes  all frames and geometry registered from the given source_id.
  //
  // Throws an exception if the source is not currently active.
  void ClearSource(SourceId source_id);

  // Requests a new FrameId for the given geometry source.
  //
  // If the source_id is currently known, it is *added*. This must trust that
  // the provided source_id is a meaningful, registered identifier.
  //
  // @param source_id    The id of the source for which this frame is allocated.
  // @returns  A newly allocated frame id.
  FrameId RequestFrameIdForSource(SourceId source_id);

  // Requests a new GeometryId for the given frame and channel. Throws an
  // exception if the channel doesn't exist, or the frame doesn't belong to the
  // channel, or if the frame does not belong to the channel.
  //
  // @param source_id    The id of the channel on which the geometry is being
  //                     declared.
  // @param frame_id      The id of the frame on which the geometry is to hang.
  // @returns  A newly allocated geometry id.
  GeometryId RequestGeometryIdForFrame(SourceId source_id, FrameId frame_id);

 private:
  // The active geometry sources and the frames that have been registered
  // on them.
  SourceFrameMap source_frame_map_;

  // The map between frames and the sources on which they were registered.
  FrameSourceMap frame_source_map_;

  // Map from frame ids to the geometry that has been hung on it.
  FrameGeometryMap frame_geometry_map_;

  // Map from geometry ids to the frame it has been hung on.
  GeometryFrameMap geometry_frame_map_;
};
}  // namespace geometry
}  // namespace drake
