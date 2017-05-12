#include "drake/geometry/geometry_state.h"

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "drake/geometry/frame_kinematics_set.h"
#include "drake/geometry/geometry_engine_stub.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"

namespace drake {
namespace geometry {

using internal::InternalFrame;
using internal::InternalGeometry;
using std::make_pair;
using std::make_unique;
using std::move;

//-----------------------------------------------------------------------------

// These utility methods help streamline the desired semantics of map lookups.
// We want to search for a key and throw an exception (with a meaningful
// message) if not found.

// Helper method for consistently determining the presence of a key in a
// container and throwing a consistent exception type if absent.
// Searches for a key value in a "findable" object. To be findable, the source
// must have find(const Key&) and end() methods that return types that can
// be equality compared, such that if they are equal, the key is *not* present
// in the source. The exception message is produced by the given functor,
// make_message().
template <class Key, class Findable>
void FindOrThrow(const Key& key, const Findable& source,
                 std::function<std::string()> make_message) {
  if (source.find(key) == source.end()) throw std::logic_error(make_message());
}
// Definition of error message for a missing key lookup.
template <class Key>
std::string get_missing_id_message(const Key& key) {
  // TODO(SeanCurtis-TRI): Use NiceTypeName to get the key name.
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

// TODO(SeanCurtis-TRI): Replace stub engine with a real engine.
template <typename T>
GeometryState<T>::GeometryState()
    : kWorldFrame(FrameId::get_new_id()),
      geometry_engine_(make_unique<GeometryEngineStub<T>>()) {}

template <typename T>
Isometry3<T> GeometryState<T>::GetPoseInFrame(GeometryId geometry_id) const {
  auto& geometry = GetValueOrThrow(geometry_id, &geometries_);
  return X_FG_[geometry.get_engine_index()];
}

template <typename T>
Isometry3<T> GeometryState<T>::GetPoseInParent(GeometryId geometry_id) const {
  Isometry3<T> X_FG = GetPoseInFrame(geometry_id);
  if (optional<GeometryId> parent_id = FindParentGeometry(geometry_id)) {
    GeometryIndex parent_index = geometries_.at(*parent_id).get_engine_index();
    const Isometry3<double>& X_FP = X_FG_[parent_index];
    return X_FP.inverse() * X_FG;
  } else {
    return X_FG;
  }
}

template <typename T>
bool GeometryState<T>::source_is_active(SourceId source_id) const {
  return source_frame_id_map_.find(source_id) != source_frame_id_map_.end();
}

template <typename T>
void GeometryState<T>::RegisterNewSource(SourceId source_id) {
  if (source_frame_id_map_.find(source_id) == source_frame_id_map_.end()) {
    source_frame_id_map_[source_id];
  } else {
    using std::to_string;
    throw std::logic_error("Trying to register a new source in the geometry "
                           "state with the id of a previously existing state: "
                           + to_string(source_id) + ".");
  }
}

template <typename T>
void GeometryState<T>::ClearSource(SourceId source_id) {
  FrameIdSet& frames = GetMutableValueOrThrow(source_id, &source_frame_id_map_);
  for (auto frame_id : frames) {
    RemoveFrameUnchecked(frame_id, RemoveFrameOrigin::SOURCE);
  }
  source_frame_id_map_[source_id].clear();
  source_root_frame_map_[source_id].clear();
}

template <typename T>
void GeometryState<T>::RemoveFrame(SourceId source_id, FrameId frame_id) {
  using std::to_string;
  if (GetSourceId(frame_id) != source_id) {
    throw std::logic_error("Trying to remove frame " + to_string(frame_id) +
                           " from source " + to_string(source_id) +
                           ". But the frame doesn't belong to that source.");
  }
  RemoveFrameUnchecked(frame_id, RemoveFrameOrigin::FRAME);
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
  RemoveGeometryUnchecked(geometry_id, RemoveGeometryOrigin::GEOMETRY);
}

// NOTE: Given that the new_id_() methods are all int64_t, we're not worrying
// about overflow.

template <typename T>
FrameId GeometryState<T>::RegisterFrame(SourceId source_id,
                                        const GeometryFrame<T>& frame) {
  return RegisterFrame(source_id, kWorldFrame, frame);
}

template <typename T>
FrameId GeometryState<T>::RegisterFrame(SourceId source_id, FrameId parent_id,
                                        const GeometryFrame<T>& frame) {
  using std::to_string;
  FrameId frame_id = FrameId::get_new_id();

  FrameIdSet& f_set = GetMutableValueOrThrow(source_id, &source_frame_id_map_);
  if (parent_id != kWorldFrame) {
    FindOrThrow(parent_id, f_set, [parent_id, source_id]() {
      return "Indicated parent id " + to_string(parent_id) + " does not belong "
          "to the indicated source id " + to_string(source_id) + ".";
    });
    frames_[parent_id].add_child(frame_id);
  } else {
    // The parent is the world frame; register it as a root frame.
    source_root_frame_map_[source_id].insert(frame_id);
  }
  PoseIndex pose_index(X_PF_.size());
  X_PF_.emplace_back(frame.pose);
  f_set.insert(frame_id);
  frames_.emplace(frame_id,
                  InternalFrame(source_id, frame_id, frame.name,
                                frame.frame_group, pose_index, parent_id));
  return frame_id;
}

template <typename T>
GeometryId GeometryState<T>::RegisterGeometry(
    SourceId source_id, FrameId frame_id,
    std::unique_ptr<GeometryInstance<T>> geometry) {
  return RegisterGeometryHelper(source_id, frame_id, move(geometry));
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
  InternalGeometry& parent_geometry =
      GetMutableValueOrThrow(geometry_id, &geometries_);
  FrameId frame_id = parent_geometry.get_frame_id();
  // Transform pose relative to geometry, to pose relative to frame.
  Isometry3<T> X_FG = X_FG_[parent_geometry.get_engine_index()] *
      geometry->get_pose();
  geometry->set_pose(X_FG);
  // Failure condition 2.
  GeometryId new_id = RegisterGeometryHelper(source_id, frame_id,
                                             move(geometry), geometry_id);
  parent_geometry.add_child(new_id);
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
  auto& frame = GetValueOrThrow(frame_id, &frames_);
  return frame.get_source_id();
}

template <typename T>
SourceId GeometryState<T>::GetSourceId(GeometryId geometry_id) const {
  FrameId frame_id = GetFrameId(geometry_id);
  return GetSourceId(frame_id);
}

template <typename T>
FrameId GeometryState<T>::GetFrameId(GeometryId geometry_id) const {
  auto& geometry = GetValueOrThrow(geometry_id, &geometries_);
  return geometry.get_frame_id();
}

template <typename T>
const FrameIdSet& GeometryState<T>::GetFramesForSource(
    SourceId source_id) const {
  return GetValueOrThrow(source_id, &source_frame_id_map_);
}

template <typename T>
void GeometryState<T>::SetFrameKinematics(
    const FrameKinematicsSet<T>& frame_kinematics) {
  ValidateKinematicsSet(frame_kinematics);
  const Isometry3<T> world_pose = Isometry3<T>::Identity();
  for (auto frame_id :
       source_root_frame_map_[frame_kinematics.get_source_id()]) {
    UpdateKinematics(frames_[frame_id], world_pose, frame_kinematics);
  }
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
      FindOrThrow(id, frames, [id, source_id]() {
        return "Frame id provided in kinematics data (" + to_string(id) + ") "
            "does not belong to the source (" + to_string(source_id) +
            "). At least one required frame id is also missing.";
      });
    }
  }
}

template <typename T>
optional<GeometryId> GeometryState<T>::FindParentGeometry(
    GeometryId geometry_id) const {
  auto& geometry = GetValueOrThrow(geometry_id, &geometries_);
  return geometry.get_parent();
}

template <typename T>
GeometryId GeometryState<T>::RegisterGeometryHelper(
    SourceId source_id, FrameId frame_id,
    std::unique_ptr<GeometryInstance<T>> geometry,
    optional<GeometryId> parent) {
  using std::to_string;
  if (geometry == nullptr) {
    throw std::logic_error(
        "Registering null geometry to frame " + to_string(frame_id) +
            ", on source " + to_string(source_id) + ".");
  }
  FrameIdSet& set = GetMutableValueOrThrow(source_id, &source_frame_id_map_);

  FindOrThrow(frame_id, set, [frame_id, source_id]() {
    return "Referenced frame " + to_string(frame_id) + " for source " +
        to_string(source_id) + ". But the frame doesn't belong to the source.";
  });

  GeometryId geometry_id = GeometryId::get_new_id();

  // Pass the geometry to the engine.

  // TODO(SeanCurtis-TRI): I need to map from geometry index to geometry id.
  // Either the geometry engine needs to know the id so that it can simply
  // communicate that back, or I need a map in the state that allows me to get
  // id from index.
  GeometryIndex engine_index =
      geometry_engine_->AddDynamicGeometry(geometry->release_shape());

  // Configure topology.
  frames_[frame_id].add_child(geometry_id);
  // TODO(SeanCurtis-TRI): Get name from geometry instance (when available).
  geometries_.emplace(geometry_id, InternalGeometry(frame_id, geometry_id,
                                                    "no_name", engine_index,
                                                    parent));
  // TODO(SeanCurtis-TRI): I expect my rigid poses are growing at the same
  // rate as in my engine. This seems fragile.
  DRAKE_ASSERT(static_cast<int>(X_FG_.size()) == engine_index);
  DRAKE_ASSERT(static_cast<int>(geometry_index_id_map_.size()) == engine_index);
  X_WG_.push_back(Isometry3<T>::Identity());
  X_FG_.emplace_back(geometry->get_pose());
  geometry_index_id_map_.push_back(geometry_id);
  return geometry_id;
}

template <typename T>
void GeometryState<T>::RemoveFrameUnchecked(FrameId frame_id,
                                            RemoveFrameOrigin caller) {
  auto& frame = GetMutableValueOrThrow(frame_id, &frames_);

  if (caller != RemoveFrameOrigin::SOURCE) {
    // Recursively delete the child frames.
    for (auto child_id : *frame.get_mutable_child_frames()) {
      RemoveFrameUnchecked(child_id, RemoveFrameOrigin::RECURSE);
    }

    // Remove the frames from the source.
    SourceId source_id = frame.get_source_id();
    auto& frame_set = GetMutableValueOrThrow(source_id, &source_frame_id_map_);
    frame_set.erase(frame_id);
    // This assumes that source_id in source_frame_id_map_ implies the existence
    // of an id set in source_root_frame_map_. It further relies on the
    // behavior that erasing a non-member of the set does nothing.
    source_root_frame_map_[source_id].erase(frame_id);
  }
  // Now delete the geometry on this.
  std::unordered_set<GeometryId> removed_geometries;
  for (auto child_id : *frame.get_mutable_child_geometries()) {
    RemoveGeometryUnchecked(child_id, RemoveGeometryOrigin::FRAME);
  }

  // TODO(SeanCurtis-TRI): Remove the pose should, ideally, coalesce the
  // memory. This is place holder for that act.
  X_PF_[frame.get_pose_index()].setIdentity();

  if (caller == RemoveFrameOrigin::FRAME) {
    // Only the root needs to explicitly remove itself from a possible parent
    // frame.
    FrameId parent_frame_id = frame.get_parent_frame_id();
    if (parent_frame_id != kWorldFrame) {
      auto& parent_frame = GetMutableValueOrThrow(parent_frame_id, &frames_);
      parent_frame.remove_child(frame_id);
    }
  }

  // Remove from the frames.
  frames_.erase(frame_id);
}

template <typename T>
void GeometryState<T>::RemoveGeometryUnchecked(GeometryId geometry_id,
                                               RemoveGeometryOrigin caller) {
  auto& geometry = GetValueOrThrow(geometry_id, &geometries_);

  if (caller != RemoveGeometryOrigin::FRAME) {
    // Clear children
    for (auto child_id : geometry.get_child_geometries()) {
      RemoveGeometryUnchecked(child_id, RemoveGeometryOrigin::RECURSE);
    }

    // Remove the geometry from its frame's list of geometries.
    auto& frame = GetMutableValueOrThrow(geometry.get_frame_id(), &frames_);
    frame.remove_child(geometry_id);
  }

  GeometryIndex engine_index = geometry.get_engine_index();
  X_FG_[engine_index].setIdentity();
  auto moved_index = geometry_engine_->RemoveGeometry(engine_index);
  if (moved_index) {
    // The geometry engine moved a geometry into the removed `engine_index`.
    // Update the state's knowledge of this.
    GeometryId moved_id = geometry_index_id_map_[*moved_index];
    geometries_[moved_id].set_engine_index(engine_index);
    geometry_index_id_map_[engine_index] = moved_id;
  }

  if (caller == RemoveGeometryOrigin::GEOMETRY) {
    // Only the root needs to explicitly remove itself from a possible parent
    // geometry.
    if (auto parent_id = geometry.get_parent()) {
      auto& parent_geometry =
          GetMutableValueOrThrow(*parent_id, &geometries_);
      parent_geometry.remove_child(geometry_id);
    }
  }

  // Remove from the geometries.
  geometries_.erase(geometry_id);
}

template <typename T>
void GeometryState<T>::UpdateKinematics(
    const internal::InternalFrame& frame, const Isometry3<T>& X_WP,
    const FrameKinematicsSet<T>& frame_kinematics) {
  const auto frame_id = frame.get_id();
  const auto& pose = frame_kinematics.GetPose(frame_id);
  Isometry3<T> X_PF = pose.get_isometry();
  X_PF_[frame.get_pose_index()] = X_PF;  // Also cache this transform.
  Isometry3<T> X_WF = X_WP * X_PF;

  // Update the geometry which belong to *this* frame.
  for (auto child_id : frame.get_child_geometries()) {
    auto& child_geometry = geometries_[child_id];
    auto child_index = child_geometry.get_engine_index();
    X_WG_[child_index] = X_WF * X_FG_[child_index];
  }

  // Update each child frame.
  for (auto child_id : frame.get_child_frames()) {
    auto& child_frame = frames_[child_id];
    UpdateKinematics(child_frame, X_WF, frame_kinematics);
  }
}

// Explicitly instantiates on the most common scalar types.
template class GeometryState<double>;

}  // namespace geometry
}  // namespace drake

