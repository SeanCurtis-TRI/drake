#include "drake/geometry/geometry_state.h"

#include <sstream>

namespace drake {
namespace geometry {

// NOTE: Given that the next_X_id_ counters are all int64_t, we're not worrying
// about overflow.

template <typename T>
ChannelId GeometryState<T>::RequestChannelId() {
  ChannelId id = ChannelId::get_new_id();
  channel_map_[id];
  return id;
}

template <typename T>
FrameId GeometryState<T>::RequestFrameIdForChannel(ChannelId channel_id) {
  FrameIdSet& set = GetFrameIdSetOrThrow(channel_id);
  FrameId frame_id = FrameId::get_new_id();
  set.insert(frame_id);
  return frame_id;
}

template <typename T>
GeometryId GeometryState<T>::RequestGeometryIdForFrame(ChannelId channel_id,
                                                       FrameId frame_id) {
  FrameIdSet& set = GetFrameIdSetOrThrow(channel_id);
  FrameIdSet::iterator itr = set.find(frame_id);
  if (itr != set.end()) {
    GeometryId id = GeometryId::get_new_id();
    // TODO(SeanCurtis-TRI): Associate this id with the frame somehow.
    return id;
  }
  std::stringstream ss;
  ss << "Referenced frame " << frame_id << " for channel " << channel_id << ".";
  ss << " But the frame doesn't belong to the channel.";
  throw std::runtime_error(ss.str());
}

template <typename T>
FrameIdSet& GeometryState<T>::GetFrameIdSetOrThrow(ChannelId id) {
  ChannelFrameIdMap::iterator itr = channel_map_.find(id);
  if (itr != channel_map_.end()) {
    return itr->second;
  }
  std::stringstream ss;
  ss << "Referenced channel " << id << " is *not* open.";
  throw std::runtime_error(ss.str());
}

}  // namespace geometry
}  // namespace drake
