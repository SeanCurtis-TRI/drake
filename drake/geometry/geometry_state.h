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

/** A map between a channel identifier and the frame identifiers it owns. */
using ChannelFrameMap = std::unordered_map<ChannelId, FrameIdSet>;

/** A map from a frame id to it's assigned channel id. */
using FrameChannelMap = std::unordered_map<FrameId, ChannelId>;

/** A collection of unique frame ids. */
using GeometryIdSet = std::unordered_set<GeometryId>;

/** A map between a channel identifier and the frame identifiers it owns. */
using FrameGeometryMap = std::unordered_map<FrameId, GeometryIdSet>;

/** A map from a frame id to it's assigned channel id. */
using GeometryFrameMap = std::unordered_map<GeometryId, FrameId>;

/** @} */

/**
 The context-dependent state of GeometryWorld.

 @tparam T The underlying scalar type. Must be a valid Eigen scalar.
 */
template <typename T>
class GeometryState {
 public:

  GeometryState();

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GeometryState)

  /** @name  Allocation of entity identifiers

   These methods provide new identifiers for channels, frames, and geometries.
   There is a hierarchy of identifiers and relationships. These methods
   guarantee that the identifiers are provisioned in a consistent manner
   such that frames belong to channels, and geometries belong to frames.
   @{
   */

  /** Distributes the next available channel id.
   @returns A newly allocated channel id.
   */
  ChannelId RequestChannelId();

  /** Reports if the channel with the given id is open. */
  bool IsChannelOpen(ChannelId id);

  /** Requests a new FrameId for the given channel. Throws an exception if the
   channel id is not an open channel.

   @param channel_id    The id of the channel for which this frame is allocated.
   @returns  A newly allocated frame id.
   */
  FrameId RequestFrameIdForChannel(ChannelId channel_id);

  /** Requests a new GeometryId for the given frame and channel. Throws an
   exception if the channel doesn't exist, or the frame doesn't belong to the
   channel, or if the frame does not belong to the channel.

   @param channel_id    The id of the channel on which the geometry is being
                        declared.
   @param frame_id      The id of the frame on which the geometry is to hang.
   @returns  A newly allocated geometry id.
   */
  GeometryId RequestGeometryIdForFrame(ChannelId channel_id, FrameId frame_id);

  /** @} */

  /** @name  Querying for parent identifiers
   Given an identifier of an entity (i.e., geometry or frame), these queries
   provide information about the entities above it in the hierarchy.

   These queries are considered to be internal queries -- queries made by
   classes which belong to the geometry namespace and not to be made from
   outside the namespace. As such, bad queries (queries for which there is not
   valid result identifier) should be considered systemic bugs. In this case,
   an exception is thrown.

   <!--
   todo(SeanCurtis-TRI): Determine if optional<id> is the right way to go.
   These use the (as of C++14 experimental) `optional<T>` class. See
   http://en.cppreference.com/w/cpp/utility/optional for details.
   -->
   @{
   */

  /** Retrieves the channel id on which the given frame id is declared.

   An exception is thrown if the id does _not_ map to an open channel.
   <!--
   todo(SeanCurtis-TRI): Determine if optional<id> is the right way to go.
   The result is "optional". If the given id doesn't actually map to a channel,
   the result value will _not_ be valid.
   -->

   @param frame_id      The query frame id.
   @returns An optional ChannelId based on a successful lookup.
   */
  ChannelId GetChannelId(FrameId frame_id);

  /** Retrieves the channel id on which the given geometry id is ultimately
   declared.

   An exception is thrown if the id does _not_ map to declared frame.
   <!--
   todo(SeanCurtis-TRI): Determine if optional<id> is the right way to go.
   The result is "optional". If the given id doesn't actually map to a channel,
   the result value will _not_ be valid.
   -->

   @param frame_id      The query geometry id.
   @returns An optional ChannelId based on a successful lookup.
   */
  ChannelId GetChannelId(GeometryId geometry_id);

  /** Retrieves the frame id on which the given geometry id is declared.

   An exception is thrown if the id does _not_ map to an open channel.
   <!--
   todo(SeanCurtis-TRI): Determine if optional<id> is the right way to go.
   The result is "optional". If the given id doesn't actually map to a channel,
   the result value will _not_ be valid.
   -->

   @param geometry_id   The query geometry id.
   @returns An optional FrameId based on a successful lookup.
   */
  FrameId GetFrameId(GeometryId geometry_id);

  /** @} */

 private:
  // The open channels and the frames that have been declared on them.
  ChannelFrameMap channel_frame_map_;

  // The map between frames and the channel on which they were declared.
  FrameChannelMap frame_channel_map_;

  // Map from frame ids to the geometry that has been hung on it.
  FrameGeometryMap frame_geometry_map_;

  // Map from geometry ids to the frame it has been hung on.
  GeometryFrameMap geometry_frame_map_;
};
}  // namespace geometry
}  // namespace drake
