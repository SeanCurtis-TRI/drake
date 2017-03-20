#pragma once

#include <unordered_set>
#include <unordered_map>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/geometry_ids.h"

namespace drake {
namespace geometry {

/** A collection of unique frame ids. */
using FrameIdSet = std::unordered_set<FrameId>;

/** A map between a channel identifier and the frame identifiers it owns. */
using ChannelFrameIdMap = std::unordered_map<ChannelId, FrameIdSet>;

/**
 The context-dependent state of GeometryWorld.

 @tparam T The underlying scalar type. Must be a valid Eigen scalar.
 */
template <typename T>
class GeometryState {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GeometryState)

  /** Distributes the next available channel id.
   @returns A newly allocated channel id.
   */
  ChannelId RequestChannelId();

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

 private:
  // Retrieves the FrameIdSet or throws an exception if the channel id is not
  // for an open channel.
  FrameIdSet& GetFrameIdSetOrThrow(ChannelId id);

  // The open channels and the frames that have been declared on them.
  ChannelFrameIdMap channel_map_;
};
}  // namespace geometry
}  // namespace drake
