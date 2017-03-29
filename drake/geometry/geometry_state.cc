#include "drake/geometry/geometry_state.h"

#include <sstream>

namespace drake {
namespace geometry {

//-----------------------------------------------------------------------------

// These utility methods help streamline the desired semantics of map lookups.
// We want to search for a key and throw an exception (with a meaningful
// message) if not found.

// Definition of error message for a missing key lookup.
template <class Key>
std::string get_missing_id_message(const Key& key) {
  // TODO(SeanCurtis-TRI): Use pretty print to get the key name.
  return "Error in map look up of unexpected key type";
}

// The look up and error-throwing method.
template <class Key, class Value>
Value& GetValueOrThrow(const Key& key, std::unordered_map<Key, Value>* map) {
  auto itr = map->find(key);
  if (itr != map->end()) {
    return itr->second;
  }
  throw std::runtime_error(get_missing_id_message(key));
}

// Specializations for missing key based on key types.
template <>
std::string get_missing_id_message<ChannelId>( const ChannelId& key) {
  std::stringstream ss;
  ss << "Referenced channel " << key << " is not open.";
  return ss.str();
}

template <>
std::string get_missing_id_message<FrameId>( const FrameId& key) {
  std::stringstream ss;
  ss << "Referenced frame " << key << " has not been declared.";
  return ss.str();
}

template <>
std::string get_missing_id_message<GeometryId>( const GeometryId& key) {
  std::stringstream ss;
  ss << "Referenced geometry " << key << " does not belong to a known frame.";
  return ss.str();
}

//-----------------------------------------------------------------------------

template <typename T>
GeometryState<T>::GeometryState() {}

// NOTE: Given that the next_X_id_ counters are all int64_t, we're not worrying
// about overflow.

template <typename T>
ChannelId GeometryState<T>::RequestChannelId() {
  ChannelId id = ChannelId::get_new_id();
  channel_frame_map_[id];   // Initialize an empty set.
  return id;
}

template <typename T>
void GeometryState<T>::CloseChannel(ChannelId channel_id) {
  FrameIdSet& frames = GetValueOrThrow(channel_id, &channel_frame_map_);
  for (auto frame_id : frames) {
    GeometryIdSet& geometries = GetValueOrThrow(frame_id, &frame_geometry_map_);
    for (auto geometry_id : geometries) {
      // TODO(SeanCurtis-TRI): Do the further work to remove the geometry:
      //  1. Delete the instance.
      geometry_frame_map_.erase(geometry_id);
    }
    frame_channel_map_.erase(frame_id);
    frame_geometry_map_.erase(frame_id);
  }
  channel_frame_map_.erase(channel_id);
}

template <typename T>
bool GeometryState<T>::ChannelIsOpen(ChannelId id) const {
  return channel_frame_map_.find(id) != channel_frame_map_.end();
}

template <typename T>
FrameId GeometryState<T>::RequestFrameIdForChannel(ChannelId channel_id) {
  FrameIdSet& set = GetValueOrThrow(channel_id, &channel_frame_map_);
  FrameId frame_id = FrameId::get_new_id();
  set.insert(frame_id);
  frame_channel_map_[frame_id] = channel_id;
  frame_geometry_map_[frame_id];  // Initialize an empty set.
  return frame_id;
}

template <typename T>
GeometryId GeometryState<T>::RequestGeometryIdForFrame(ChannelId channel_id,
                                                       FrameId frame_id) {
  FrameIdSet& set = GetValueOrThrow(channel_id, &channel_frame_map_);
  FrameIdSet::iterator itr = set.find(frame_id);
  if (itr != set.end()) {
    GeometryId id = GeometryId::get_new_id();
    GeometryIdSet& geometry_set = GetValueOrThrow(frame_id,
                                                  &frame_geometry_map_);
    geometry_set.insert(id);
    geometry_frame_map_[id] = frame_id;
    return id;
  }
  std::stringstream ss;
  ss << "Referenced frame " << frame_id << " for channel " << channel_id << ".";
  ss << " But the frame doesn't belong to the channel.";
  throw std::runtime_error(ss.str());
}

template <typename T>
ChannelId GeometryState<T>::GetChannelId(FrameId frame_id) {
  return GetValueOrThrow(frame_id, &frame_channel_map_);
}

template <typename T>
ChannelId GeometryState<T>::GetChannelId(GeometryId geometry_id) {
  FrameId frame_id = GetFrameId(geometry_id);
  return GetChannelId(frame_id);
}

template <typename T>
FrameId GeometryState<T>::GetFrameId(GeometryId geometry_id) {
  return GetValueOrThrow(geometry_id, &geometry_frame_map_);
}

// Explicitly instantiates on the most common scalar types.
template class GeometryState<double>;

}  // namespace geometry
}  // namespace drake

