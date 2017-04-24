#include "drake/geometry/geometry_state.h"

#include <memory>
#include <queue>
#include <string>
#include <utility>

#include "drake/geometry/frame_kinematics_set.h"
#include "drake/geometry/geometry_instance.h"

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
const Value& GetValueOrThrow(const Key& key,
                       const std::unordered_map<Key, Value>* map) {
  auto itr = map->find(key);
  if (itr != map->end()) {
    return itr->second;
  }
  throw std::logic_error(get_missing_id_message(key));
}

// The look up and error-throwing method.
template <class Key, class Value>
Value& GetMutableValueOrThrow(const Key& key,
                              std::unordered_map<Key, Value>* map) {
  auto itr = map->find(key);
  if (itr != map->end()) {
    return itr->second;
  }
  throw std::logic_error(get_missing_id_message(key));
}

// Specializations for missing key based on key types.
template <>
std::string get_missing_id_message<SourceId>(const SourceId& key) {
  std::stringstream ss;
  ss << "Referenced geometry source " << key << " is not active.";
  return ss.str();
}

template <>
std::string get_missing_id_message<FrameId>(const FrameId& key) {
  std::stringstream ss;
  ss << "Referenced frame " << key << " has not been registered.";
  return ss.str();
}

template <>
std::string get_missing_id_message<GeometryId>(const GeometryId& key) {
  std::stringstream ss;
  ss << "Referenced geometry " << key << " does not belong to a known frame.";
  return ss.str();
}

//-----------------------------------------------------------------------------

template <typename T>
int GeometryState<T>::GetNumFrames() const {
  int count = 0;
  for (auto pair : source_frame_map_) {
    count += static_cast<int>(pair.second.size());
  }
  return count;
}
template <typename T>
Isometry3<T> GeometryState<T>::GetPoseInFrame(GeometryId geometry_id) const {
  using std::to_string;
  auto itr = geometry_poses_.find(geometry_id);
  if (itr != geometry_poses_.end()) {
    return itr->second;
  }
  throw std::logic_error("Requesting a geometry pose with an invalid geometry "
                             "identifier: " + to_string(geometry_id) + ".");
}

template <typename T>
Isometry3<T> GeometryState<T>::GetPoseInParent(GeometryId geometry_id) const {
  Isometry3<T> X_FG = GetPoseInFrame(geometry_id);
  if (optional<GeometryId> parent_id = FindParentGeometry(geometry_id)) {
    const Isometry3<double>& X_FP = geometry_poses_.at(*parent_id);
    return X_FP.inverse() * X_FG;
  } else {
    return X_FG;
  }
}

template <typename T>
bool GeometryState<T>::source_is_active(SourceId source_id) const {
  return source_frame_map_.find(source_id) != source_frame_map_.end();
}

template <typename T>
void GeometryState<T>::RegisterNewSource(SourceId source_id) {
  if (source_frame_map_.find(source_id) == source_frame_map_.end()) {
    // Indicate it is "active" by adding it to the source-frame map with an an
    // empty frame set.
    source_frame_map_[source_id];
  } else {
    using std::to_string;
    throw std::logic_error("Trying to register a new source in the geometry "
                           "state with the id of a previously existing state: "
                           + to_string(source_id) + ".");
  }
}

template <typename T>
void GeometryState<T>::ClearSource(SourceId source_id) {
  FrameIdSet& frames = GetMutableValueOrThrow(source_id, &source_frame_map_);
  for (auto frame_id : frames) {
    RemoveFrameUnchecked(source_id, frame_id);
  }
  source_frame_map_[source_id].clear();
}

template <typename T>
void GeometryState<T>::RemoveFrame(SourceId source_id, FrameId frame_id) {
  using std::to_string;
  if (GetSourceId(frame_id) != source_id) {
    throw std::logic_error("Trying to remove frame " + to_string(frame_id) +
                           " from source " + to_string(source_id) +
                           ". But the frame doesn't belong to that source.");
  }
  RemoveFrameUnchecked(source_id, frame_id);
  source_frame_map_[source_id].erase(frame_id);
}

template <typename T>
void GeometryState<T>::RemoveGeometry(SourceId source_id,
                                      GeometryId geometry_id) {
  using std::to_string;
  if (GetSourceId(geometry_id) != source_id) {
    throw std::logic_error(
        "Trying to remove geometry " + to_string(geometry_id) + " from "
        "source " + to_string(source_id) + ". But the geometry doesn't belong"
        " to that source.");
  }
  FrameId frame_id = GetFrameId(geometry_id);
  std::unordered_set<GeometryId> removed_geometries;
  RemoveGeometryUnchecked(frame_id, geometry_id, &removed_geometries);
  for (auto id : removed_geometries) {
    frame_geometry_map_[frame_id].erase(id);
  }
}

// NOTE: Given that the new_id_() methods are all int64_t, we're not worrying
// about overflow.

template <typename T>
FrameId GeometryState<T>::RegisterFrame(SourceId source_id) {
  FrameIdSet& set = GetMutableValueOrThrow(source_id, &source_frame_map_);
  FrameId frame_id = FrameId::get_new_id();
  set.insert(frame_id);
  frame_source_map_[frame_id] = source_id;
  frame_geometry_map_[frame_id];  // Initialize an empty set.
  return frame_id;
}

template <typename T>
GeometryId GeometryState<T>::RegisterGeometry(
    SourceId source_id, FrameId frame_id,
    std::unique_ptr<GeometryInstance<T>> geometry) {
  using std::to_string;
  if (geometry == nullptr) {
    throw std::logic_error(
        "Registering null geometry to frame " + to_string(frame_id) +
        ", on source " + to_string(source_id) + ".");
  }
  FrameIdSet& set = GetMutableValueOrThrow(source_id, &source_frame_map_);
  FrameIdSet::iterator itr = set.find(frame_id);
  if (itr != set.end()) {
    GeometryId id = GeometryId::get_new_id();
    // Configure the topology.
    GeometryIdSet& geometry_set = GetMutableValueOrThrow(frame_id,
                                                         &frame_geometry_map_);
    geometry_set.insert(id);
    geometry_frame_map_[id] = frame_id;
    geometry_poses_[id] = geometry->get_pose();
    return id;
  }
  throw std::logic_error("Referenced frame " + to_string(frame_id) +
                         " for source " + to_string(source_id) +
                         ". But the frame doesn't belong to the source.");
}

template <typename T>
GeometryId GeometryState<T>::RegisterGeometryWithParent(
    SourceId source_id, GeometryId geometry_id,
    std::unique_ptr<GeometryInstance<T>> geometry) {
  // The error condition is that geometry_id doesn't belong to source_id or
  // if the source isn't active.  This is decomposed into two equivalent tests
  // (implicitly):
  //    1. Failure if the geometry_id doesn't exist at all, otherwise
  //    2. Failure if the frame it belongs to doesn't belong to the source (or
  //       active source).

  using std::to_string;
  if (geometry == nullptr) {
    throw std::logic_error(
        "Registering null geometry to geometry " + to_string(geometry_id) +
            ", on source " + to_string(source_id) + ".");
  }

  // Failure condition 1.
  FrameId frame_id = GetFrameId(geometry_id);
  // Transform pose relative to geometry, to pose relative to frame.
  Isometry3<T> X_FG = geometry_poses_[geometry_id] * geometry->get_pose();
  geometry->set_pose(X_FG);
  // Failure condition 2.
  GeometryId new_id = RegisterGeometry(source_id, frame_id, move(geometry));
  linked_geometry_[geometry_id].insert(new_id);
  geometry_poses_[new_id] = X_FG;
  return new_id;
}

template <typename T>
GeometryId GeometryState<T>::RegisterAnchoredGeometry(
    SourceId source_id,
    std::unique_ptr<GeometryInstance<T>> geometry) {
  throw std::runtime_error("Not implemented yet!");
}

template <typename T>
SourceId GeometryState<T>::GetSourceId(FrameId frame_id) const {
  return GetValueOrThrow(frame_id, &frame_source_map_);
}

template <typename T>
SourceId GeometryState<T>::GetSourceId(GeometryId geometry_id) const {
  FrameId frame_id = GetFrameId(geometry_id);
  return GetSourceId(frame_id);
}

template <typename T>
FrameId GeometryState<T>::GetFrameId(GeometryId geometry_id) const {
  return GetValueOrThrow(geometry_id, &geometry_frame_map_);
}

template <typename T>
const FrameIdSet& GeometryState<T>::GetFramesForSource(
    SourceId source_id) const {
  return GetValueOrThrow(source_id, &source_frame_map_);
}

template <typename T>
void GeometryState<T>::ValidateKinematicsSet(
    const FrameKinematicsSet<T>& frame_kinematics) const {
  using std::to_string;
  SourceId source_id = frame_kinematics.get_source_id();
  auto& frames = GetFramesForSource(source_id);
  const int ref_frame_count = static_cast<int>(frames.size());
  if (ref_frame_count != frame_kinematics.get_frame_count()) {
    // TODO(SeanCurtis-TRI): Determine if more specific information is required.
    // e.g., which frames are missing/added.
    throw std::logic_error(
        "Disagreement in expected number of frames (" +
        std::to_string(frames.size()) + ") and the given number of frames (" +
        std::to_string(frame_kinematics.get_frame_count()) + ").");
  } else {
    for (auto id : frame_kinematics.get_frame_ids()) {
      if (frames.find(id) == frames.end()) {
        throw std::logic_error(
            "Frame id provided in kinematics data (" + to_string(id) +
            ") does not belong to the source (" +
            to_string(source_id) +
            "). At least one required frame id is also missing.");
      }
    }
  }
}

template <typename T>
optional<GeometryId> GeometryState<T>::FindParentGeometry(
    GeometryId geometry_id) const {
  // TODO(SeanCurtis-TRI): Determine if it's worth saving enough topology info
  // in order to reconstruct the original pose relative to the registered
  // parent efficiently. Currently, I only have parent-to-child relationships.
  // I would also need child-to-parent lookups. Without the upward links, I
  // have to do a painful search.
  FrameId frame_id = GetFrameId(geometry_id);
  auto geometries = GetValueOrThrow(frame_id, &frame_geometry_map_);
  std::queue<GeometryId> next;
  std::unordered_set<GeometryId> visited;
  for (GeometryId g_id : geometries) {
    if (g_id == geometry_id) continue;
    auto itr = linked_geometry_.find(g_id);
    if (itr != linked_geometry_.end()) {
      // This node has children.
      auto children = itr->second;
      if (children.find(geometry_id) != children.end()) {
        return g_id;
      }
    }
  }
  return {};
}

template <typename T>
void GeometryState<T>::RemoveFrameUnchecked(SourceId source_id,
                                            FrameId frame_id) {
  GeometryIdSet& geometries = GetMutableValueOrThrow(frame_id,
                                                     &frame_geometry_map_);
  std::unordered_set<GeometryId> removed_geometries;
  for (auto geometry_id : geometries) {
    RemoveGeometryUnchecked(frame_id, geometry_id, &geometries);
  }
  // NOTE: Not using removed_geometries, because we're throwing out the whole
  // set.
  frame_source_map_.erase(frame_id);
  frame_geometry_map_.erase(frame_id);
}

template <typename T>
void GeometryState<T>::RemoveGeometryUnchecked(
    FrameId frame_id, GeometryId geometry_id,
    std::unordered_set<GeometryId>* children) {
  children->insert(geometry_id);
  geometry_frame_map_.erase(geometry_id);
  {
    auto child_itr = linked_geometry_.find(geometry_id);
    if (child_itr != linked_geometry_.end()) {
      for (auto child_id : child_itr->second) {
        RemoveGeometryUnchecked(frame_id, child_id, children);
      }
      linked_geometry_.erase(child_itr);
    }
  }
  geometry_poses_.erase(geometry_id);
}

// Explicitly instantiates on the most common scalar types.
template class GeometryState<double>;

}  // namespace geometry
}  // namespace drake

