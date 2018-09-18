#include "drake/geometry/geometry_state.h"

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "drake/common/autodiff.h"
#include "drake/common/default_scalars.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/proximity_engine.h"
#include "drake/geometry/render/render_engine_vtk.h"
#include "drake/geometry/utilities.h"

namespace drake {
namespace geometry {

using internal::GeometryStateCollisionFilterAttorney;
using internal::InternalAnchoredGeometry;
using internal::InternalFrame;
using internal::InternalGeometry;
using internal::InternalGeometryBase;
using internal::ProximityEngine;
using std::make_pair;
using std::make_unique;
using std::move;
using std::to_string;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;

namespace {

// TODO(SeanCurtis-TRI): This is copied from proximity_engine.cc. Refactor this
// into a single location for re-use.

// ADL-reliant helper functions for converting Isometry<T> to Isometry<double>.
const Isometry3<double>& convert(const Isometry3<double>& transform) {
  return transform;
}

template <class VectorType>
Isometry3<double> convert(
    const Isometry3<Eigen::AutoDiffScalar<VectorType>>& transform) {
  Isometry3<double> result;
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      result.matrix()(r, c) = ExtractDoubleOrThrow(transform.matrix()(r, c));
    }
  }
  return result;
}

}  // namespace

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
                 const std::function<std::string()>& make_message) {
  if (source.find(key) == source.end()) throw std::logic_error(make_message());
}
// Definition of error message for a missing key lookup.
template <class Key>
std::string get_missing_id_message(const Key& key) {
  // TODO(SeanCurtis-TRI): Use NiceTypeName to get the key name.
  return "Error in map look up of unexpected key type";
}

// The look up and error-throwing method for const values.
template <class Key, class Value>
const Value& GetValueOrThrow(const Key& key,
                             const std::unordered_map<Key, Value>& map) {
  auto itr = map.find(key);
  if (itr != map.end()) {
    return itr->second;
  }
  throw std::logic_error(get_missing_id_message(key));
}

// The look up and error-throwing method for mutable values.
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
  ss << "Referenced geometry source " << key << " is not registered.";
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
  ss << "Referenced geometry " << key << " has not been registered.";
  return ss.str();
}

//-----------------------------------------------------------------------------

template <typename T>
GeometryState<T>::GeometryState()
    : geometry_engine_(make_unique<internal::ProximityEngine<T>>()),
      low_render_engine_(make_unique<render::RenderEngineVtk>()) {}

template <typename T>
bool GeometryState<T>::source_is_registered(SourceId source_id) const {
  return source_frame_id_map_.find(source_id) != source_frame_id_map_.end();
}

template <typename T>
int GeometryState<T>::get_frame_group(FrameId frame_id) const {
  FindOrThrow(frame_id, frames_, [frame_id]() {
    return "No frame group available for invalid frame id: " +
           to_string(frame_id);
  });
  return frames_.at(frame_id).get_frame_group();
}

template <typename T>
const std::string& GeometryState<T>::get_frame_name(FrameId frame_id) const {
  FindOrThrow(frame_id, frames_, [frame_id]() {
    return "No frame name available for invalid frame id: " +
           to_string(frame_id);
  });
  return frames_.at(frame_id).get_name();
}

template <typename T>
const std::string& GeometryState<T>::get_name(GeometryId geometry_id) const {
  const InternalGeometryBase* geometry = GetGeometry(geometry_id);
  if (geometry != nullptr) return geometry->name();

  throw std::logic_error("No geometry available for invalid geometry id: " +
      to_string(geometry_id));
}

template <typename T>
GeometryId GeometryState<T>::GetGeometryFromName(
    FrameId frame_id, Role role, const std::string& name) const {
  const std::string canonical_name = detail::CanonicalizeStringName(name);

  GeometryId result;
  int count = 0;
  std::string frame_name;

  if (frame_id == InternalFrame::get_world_frame_id()) {
    frame_name = "world";
    for (const auto& pair : anchored_geometries_) {
      const InternalAnchoredGeometry& geometry = pair.second;
      if (geometry.has_role(role) && geometry.name() == canonical_name) {
        ++count;
        result = pair.first;
      }
    }
  } else {
    const InternalFrame& frame = GetValueOrThrow(frame_id, frames_);
    frame_name = frame.get_name();
    for (GeometryId geometry_id : frame.get_child_geometries()) {
      const InternalGeometry& geometry = geometries_.at(geometry_id);
      if (geometry.has_role(role) && geometry.name() == canonical_name) {
        ++count;
        result = geometry_id;
      }
    }
  }

  if (count == 1) return result;
  if (count < 1) {
    throw std::logic_error("The frame '" + frame_name + "' (" +
        to_string(frame_id) + ") has no geometry with the role '" +
        to_string(role) + "' and the canonical name '" + canonical_name + "'");
  }
  // This case should only be possible for unassigned geometries.
  DRAKE_DEMAND(role == Role::kUnassigned);
  throw std::logic_error("The frame '" + frame_name + "' (" +
      to_string(frame_id) + ") has multiple geometries with the role '" +
      to_string(role) + "' and the canonical name '" + canonical_name + "'");
}

template <typename T>
const Isometry3<T>& GeometryState<T>::get_pose_in_world(
    FrameId frame_id) const {
  FindOrThrow(frame_id, frames_, [frame_id]() {
    return "No world pose available for invalid frame id: " +
           to_string(frame_id);
  });
  return X_WF_[frames_.at(frame_id).internal_index()];
}

template <typename T>
const Isometry3<T>& GeometryState<T>::get_pose_in_world(
    GeometryId geometry_id) const {
  // TODO(SeanCurtis-TRI): This is a BUG! If you pass in the id of an
  // anchored geometry, this will throw an exception. See
  // https://github.com/RobotLocomotion/drake/issues/9145.
  FindOrThrow(geometry_id, geometries_, [geometry_id]() {
    return "No world pose available for invalid geometry id: " +
           to_string(geometry_id);
  });
  return X_WG_[geometries_.at(geometry_id).internal_index()];
}

template <typename T>
const Isometry3<T>& GeometryState<T>::get_pose_in_parent(
    FrameId frame_id) const {
  FindOrThrow(frame_id, frames_, [frame_id]() {
    return "No pose available for invalid frame id: " + to_string(frame_id);
  });
  return X_PF_[frames_.at(frame_id).internal_index()];
}

template <typename T>
const std::string& GeometryState<T>::get_source_name(SourceId id) const {
  auto itr = source_names_.find(id);
  if (itr != source_names_.end()) return itr->second;
  throw std::logic_error(
      "Querying source name for an invalid source id: " + to_string(id) + ".");
}

template <typename T>
const Isometry3<double>& GeometryState<T>::GetPoseInFrame(
    GeometryId geometry_id) const {
  const auto& geometry = GetValueOrThrow(geometry_id, geometries_);
  return X_FG_[geometry.internal_index()];
}

template <typename T>
const Isometry3<double>& GeometryState<T>::GetPoseInParent(
    GeometryId geometry_id) const {
  const auto& geometry = GetValueOrThrow(geometry_id, geometries_);
  return geometry.pose_in_parent();
}

template <typename T>
const ProximityProperties* GeometryState<T>::get_proximity_properties(
    GeometryId id) const {
  const InternalGeometryBase* geometry = GetGeometry(id);
  if (geometry != nullptr) return geometry->proximity_properties();
  return nullptr;
}

template <typename T>
const IllustrationProperties* GeometryState<T>::get_illustration_properties(
    GeometryId id) const {
  const InternalGeometryBase* geometry = GetGeometry(id);
  if (geometry != nullptr) return geometry->illustration_properties();
  return nullptr;
}

template <typename T>
const PerceptionProperties* GeometryState<T>::get_perception_properties(
    GeometryId id) const {
  const InternalGeometryBase* geometry = GetGeometry(id);
  if (geometry != nullptr) return geometry->perception_properties();
  return nullptr;
}

template <typename T>
int GeometryState<T>::NumGeometryWithRole(FrameId frame_id, Role role) const {
  int count = 0;
  if (frame_id == internal::InternalFrame::get_world_frame_id()) {
    for (const auto& pair : anchored_geometries_) {
      if (pair.second.has_role(role)) ++count;
    }
  } else {
    FindOrThrow(frame_id, frames_, [frame_id, role]() {
      return "Cannot report number of geometries with the " + to_string(role) +
          " role for invalid frame id: " + to_string(frame_id);
    });
    const InternalFrame& frame = frames_.at(frame_id);
    for (GeometryId id : frame.get_child_geometries()) {
      if (geometries_.at(id).has_role(role)) ++count;
    }
  }
  return count;
}

template <typename T>
SourceId GeometryState<T>::RegisterNewSource(const std::string& name) {
  SourceId source_id = SourceId::get_new_id();
  const std::string final_name =
      name != "" ? name : "Source_" + to_string(source_id);

  // The user can provide bad names, _always_ test.
  for (const auto& pair : source_names_) {
    if (pair.second == final_name) {
      throw std::logic_error(
          "Registering new source with duplicate name: " + final_name + ".");
    }
  }

  source_frame_id_map_[source_id];
  source_root_frame_map_[source_id];
  source_anchored_geometry_map_[source_id];
  source_names_[source_id] = final_name;
  return source_id;
}

template <typename T>
FrameId GeometryState<T>::RegisterFrame(SourceId source_id,
                                        const GeometryFrame& frame) {
  return RegisterFrame(source_id, InternalFrame::get_world_frame_id(), frame);
}

template <typename T>
FrameId GeometryState<T>::RegisterFrame(SourceId source_id, FrameId parent_id,
                                        const GeometryFrame& frame) {
  FrameId frame_id = frame.id();

  if (frames_.count(frame_id) > 0) {
    throw std::logic_error(
        "Registering frame with an id that has already been registered: " +
            to_string(frame_id));
  }

  FrameIdSet& f_set = GetMutableValueOrThrow(source_id, &source_frame_id_map_);
  if (parent_id != InternalFrame::get_world_frame_id()) {
    FindOrThrow(parent_id, f_set, [parent_id, source_id]() {
      return "Indicated parent id " + to_string(parent_id) + " does not belong "
          "to the indicated source id " + to_string(source_id) + ".";
    });
    frames_[parent_id].add_child(frame_id);
  } else {
    // The parent is the world frame; register it as a root frame.
    source_root_frame_map_[source_id].insert(frame_id);
  }

  DRAKE_ASSERT(X_PF_.size() == frame_index_to_frame_map_.size());
  InternalIndex internal_index(X_PF_.size());
  X_PF_.emplace_back(frame.pose());
  X_WF_.emplace_back(Isometry3<double>::Identity());
  frame_index_to_frame_map_.push_back(frame_id);
  f_set.insert(frame_id);
  int clique = GeometryStateCollisionFilterAttorney::get_next_clique(
      geometry_engine_.get_mutable());
  frames_.emplace(frame_id, InternalFrame(source_id, frame_id, frame.name(),
                                          frame.frame_group(), internal_index,
                                          parent_id, clique));
  return frame_id;
}

template <typename T>
GeometryId GeometryState<T>::RegisterGeometry(
    SourceId source_id, FrameId frame_id,
    std::unique_ptr<GeometryInstance> geometry) {
  if (geometry == nullptr) {
    throw std::logic_error(
        "Registering null geometry to frame " + to_string(frame_id) +
            ", on source " + to_string(source_id) + ".");
  }

  GeometryId geometry_id = geometry->id();
  if (geometries_.count(geometry_id) > 0) {
    throw std::logic_error(
        "Registering geometry with an id that has already been registered: " +
            to_string(geometry_id));
  }

  FrameIdSet& set = GetMutableValueOrThrow(source_id, &source_frame_id_map_);

  FindOrThrow(frame_id, set, [frame_id, source_id]() {
    return "Referenced frame " + to_string(frame_id) + " for source " +
        to_string(source_id) + ", but the frame doesn't belong to the source.";
  });

  // Configure topology.
  // TODO(SeanCurtis-TRI): Once geometry roles are implemented, test for
  // uniqueness of the canonical name in that role for the given frame.

  InternalFrame& frame = frames_[frame_id];
  frame.add_child(geometry_id);

  // TODO(SeanCurtis-TRI): Enforcing the invariant that the indices are
  // compactly distributed. Is there a more robust way to do this?
  DRAKE_ASSERT(geometry_index_id_map_.size() == X_FG_.size());
  InternalIndex internal_index(static_cast<int>(X_FG_.size()));
  X_WG_.push_back(Isometry3<T>::Identity());
  X_FG_.emplace_back(geometry->pose());
  geometry_index_id_map_.push_back(geometry_id);

  geometries_.emplace(
      geometry_id,
      InternalGeometry(geometry->release_shape(), frame_id, geometry_id,
                       geometry->name(), geometry->pose(), internal_index));

  return geometry_id;
}

template <typename T>
GeometryId GeometryState<T>::RegisterGeometryWithParent(
    SourceId source_id, GeometryId geometry_id,
    std::unique_ptr<GeometryInstance> geometry) {
  // There are three error conditions in the doxygen:.
  //    1. geometry == nullptr,
  //    2. source_id is not a registered source, and
  //    3. geometry_id doesn't belong to source_id.
  //
  // Only #1 is tested directly. #2 and #3 are tested implicitly during the act
  // of registering the geometry.

  if (geometry == nullptr) {
    throw std::logic_error(
        "Registering null geometry to geometry " + to_string(geometry_id) +
            ", on source " + to_string(source_id) + ".");
  }

  // This confirms that geometry_id exists at all.
  InternalGeometry& parent_geometry =
      GetMutableValueOrThrow(geometry_id, &geometries_);
  FrameId frame_id = parent_geometry.frame_id();

  // This implicitly confirms that source_id is registered (condition #2) and
  // that frame_id belongs to source_id. By construction, geometry_id must
  // belong to the same source as frame_id, so this tests  condition #3.
  GeometryId new_id =
      RegisterGeometry(source_id, frame_id, move(geometry));

  // RegisterGeometry stores X_PG into X_FG_ (having assumed that  the
  // parent was a frame). This replaces the stored X_PG value with the
  // semantically correct value X_FG by concatenating X_FP with X_PG.

  // Transform pose relative to geometry, to pose relative to frame.
  const InternalGeometry& new_geometry = geometries_[new_id];
  // The call to `RegisterGeometry()` above stashed the pose X_PG into the
  // X_FG_ vector assuming the parent was the frame. Replace it by concatenating
  // its pose in parent, with its parent's pose in frame. NOTE: the pose is no
  // longer available from geometry because of the `move(geometry)`.
  const Isometry3<double>& X_PG = X_FG_[new_geometry.internal_index()];
  const Isometry3<double>& X_FP = X_FG_[parent_geometry.internal_index()];
  X_FG_[new_geometry.internal_index()] = X_FP * X_PG;

  geometries_[new_id].set_parent_id(geometry_id);
  parent_geometry.add_child(new_id);
  return new_id;
}

template <typename T>
GeometryId GeometryState<T>::RegisterAnchoredGeometry(
    SourceId source_id,
    std::unique_ptr<GeometryInstance> geometry) {
  if (geometry == nullptr) {
    throw std::logic_error(
        "Registering null anchored geometry on source "
        + to_string(source_id) + ".");
  }

  GeometryId geometry_id = geometry->id();
  if (anchored_geometries_.count(geometry_id) > 0) {
    throw std::logic_error(
        "Registering anchored geometry with an id that has already been "
        "registered: " +
        to_string(geometry_id));
  }

  auto& set = GetMutableValueOrThrow(source_id, &source_anchored_geometry_map_);

  set.emplace(geometry_id);

  // TODO(SeanCurtis-TRI): Once geometry roles are implemented, test for
  // uniqueness of the canonical name in that role for the given frame.
  // NOTE: It is important to test for name validity *before* adding this
  // geometry to the frame.

  InternalIndex internal_index(anchored_geometries_.size());
  anchored_geometries_.emplace(
      geometry_id,
      InternalAnchoredGeometry(geometry->release_shape(), geometry_id,
                               geometry->name(), geometry->pose(),
                               internal_index));

  return geometry_id;
}

template <typename T>
bool GeometryState<T>::IsValidGeometryName(
    FrameId frame_id, Role role, const std::string& candidate_name) const {
  FindOrThrow(frame_id, frames_, [frame_id]() {
    return "Given frame id is not valid: " + to_string(frame_id);
  });
  const std::string name = detail::CanonicalizeStringName(candidate_name);
  if (name.empty()) return false;
  return NameIsUnique(frame_id, role, name);
}

template <typename T>
void GeometryState<T>::AssignRole(SourceId source_id,
                                  GeometryId geometry_id,
                                  ProximityProperties properties) {
  AssignRoleInternal(source_id, geometry_id, std::move(properties),
                     Role::kProximity);

  InternalGeometryBase* geometry = GetMutableGeometry(geometry_id);
  // This *must* be no-null, otherwise the role assignment would have failed.
  DRAKE_DEMAND(geometry != nullptr);

  // NOTE: This is evidence that the division between anchored and dynamic
  // geometry *at this level* may be erroneous.
  auto dynamic_geometry = dynamic_cast<InternalGeometry*>(geometry);
  if (dynamic_geometry != nullptr) {
    // Pass the geometry to the engine.
    const InternalIndex internal_index = dynamic_geometry->internal_index();
    ProximityIndex index =
        geometry_engine_->AddDynamicGeometry(geometry->shape(), internal_index);
    dynamic_geometry->set_proximity_index(index);
    DRAKE_DEMAND(static_cast<int>(X_WG_proximity_.size()) == index);
    X_WG_proximity_.push_back(dynamic_geometry->internal_index());

    InternalFrame& frame = frames_[dynamic_geometry->frame_id()];

    int child_count = static_cast<int>(frame.get_child_geometries().size());
    if (child_count > 1) {
      // Multiple children does *not* imply the need for collision filtering.
      // Only operate on those children which have a proximity role.
      // TODO(SeanCurtis-TRI): Perhaps refactor this elsewhere?
      std::vector<GeometryId> proximity_geometries;
      proximity_geometries.reserve(child_count);
      std::copy_if(frame.get_child_geometries().begin(),
                   frame.get_child_geometries().end(),
                   std::back_inserter(proximity_geometries),
                   [this](GeometryId id) {
                     return geometries_[id].has_proximity_role();
                   });
      int proximity_count = static_cast<int>(proximity_geometries.size());

      if (proximity_count > 1) {
        // Filter collisions between geometries affixed to the same frame. We
        // only add a clique to a frame's geometries when there are *multiple*
        // child geometries.
        ProximityEngine<T>& engine = *geometry_engine_.get_mutable();
        if (proximity_count > 2) {
          // Assume all previous geometries have already had the clique
          // assigned.
          GeometryStateCollisionFilterAttorney::set_dynamic_geometry_clique(
              &engine, internal_index, frame.clique());
        } else {  // proximity_count == 2.
          // We *now* have multiple child geometries with proximity role --
          // assign to clique.
          for (GeometryId child_id : proximity_geometries) {
            InternalIndex child_index = geometries_[child_id].internal_index();
            GeometryStateCollisionFilterAttorney::set_dynamic_geometry_clique(
                &engine, child_index, frame.clique());
          }
        }
      }
    }
  } else {
    auto anchored_geometry = dynamic_cast<InternalAnchoredGeometry*>(geometry);
    // If it's not dynamic, it must be anchored.
    DRAKE_DEMAND(anchored_geometry != nullptr);
    ProximityIndex index = geometry_engine_->AddAnchoredGeometry(
        geometry->shape(), geometry->pose_in_parent(),
        geometry->internal_index());
    anchored_geometry->set_proximity_index(index);
    DRAKE_DEMAND(
        static_cast<int>(anchored_geometry_index_id_map_.size()) == index);
    // TODO(SeanCurtis-TRI): Apparently this is only anchored geometries that
    // *have collision roles*.
    anchored_geometry_index_id_map_.push_back(geometry_id);
  }
}

template <typename T>
void GeometryState<T>::AssignRole(SourceId source_id,
                                  GeometryId geometry_id,
                                  PerceptionProperties properties) {
  AssignRoleInternal(source_id, geometry_id, std::move(properties),
                     Role::kPerception);

  // TODO(SeanCurtis-TRI): Add every render engine when they become available.
  InternalGeometryBase* geometry = GetMutableGeometry(geometry_id);
  // This *must* be no-null, otherwise the role assignment would have failed.
  DRAKE_DEMAND(geometry != nullptr);
  // NOTE: Pose in parent is fragile; if an anchored geometry is registered
  // relative to *another* geometry, this is the pose relative to the geometry
  // and *not* the frame.
  RenderIndex index = low_render_engine_->RegisterVisual(
      geometry->shape(), *geometry->perception_properties(),
      geometry->pose_in_parent());
  geometry->set_render_index(index);
  auto dynamic_geometry = dynamic_cast<InternalGeometry*>(geometry);
  if (dynamic_geometry != nullptr) {
    // Save the geometry's internal index in its render index slot.
    // NOTE: These are only the indices of *dynamic* geometries with perception
    // roles. As such, we have no guarantee that they'll grow in lockstep.
    // This is in stark contrast to the dynamic/anchored dichotomy in the
    // proximity role.
    X_WG_perception_.push_back(geometry->internal_index());
  }
}

template <typename T>
void GeometryState<T>::AssignRole(SourceId source_id,
                                  GeometryId geometry_id,
                                  IllustrationProperties properties) {
  AssignRoleInternal(source_id, geometry_id, std::move(properties),
                     Role::kIllustration);
  // NOTE: No need to assign to any engines.
}

template <typename T>
bool GeometryState<T>::BelongsToSource(FrameId frame_id,
                                       SourceId source_id) const {
  // Confirm that the source_id is valid; use the utility function to confirm
  // source_id is valid and throw an exception with a known message.
  GetValueOrThrow(source_id, source_frame_id_map_);
  // If valid, test the frame.
  return get_source_id(frame_id) == source_id;
}

template <typename T>
bool GeometryState<T>::BelongsToSource(GeometryId geometry_id,
                                       SourceId source_id) const {
  // Geometry could be anchored. This also implicitly tests that source_id is
  // valid and throws an exception if not.
  const auto& anchored_geometries =
      GetValueOrThrow(source_id, source_anchored_geometry_map_);
  if (anchored_geometries.find(geometry_id) != anchored_geometries.end()) {
    return true;
  }
  // If not anchored, geometry must be dynamic. If this fails, the geometry_id
  // is not valid and an exception is thrown.
  const auto& geometry = GetValueOrThrow(geometry_id, geometries_);
  return BelongsToSource(geometry.frame_id(), source_id);
}

template <typename T>
FrameId GeometryState<T>::GetFrameId(GeometryId geometry_id) const {
  const auto& geometry = GetValueOrThrow(geometry_id, geometries_);
  return geometry.frame_id();
}

template <typename T>
const FrameIdSet& GeometryState<T>::GetFramesForSource(
    SourceId source_id) const {
  return GetValueOrThrow(source_id, source_frame_id_map_);
}

template <typename T>
void GeometryState<T>::ExcludeCollisionsWithin(const GeometrySet& set) {
  // There is no work to be done if:
  //   1. the set contains a single frame and no geometries -- geometries *on*
  //      that single frame have already been handled, or
  //   2. there are no frames and a single geometry.
  if ((set.num_frames() == 1 && set.num_geometries() == 0) ||
      (set.num_frames() == 0 && set.num_geometries() == 1)) {
    return;
  }

  std::unordered_set<InternalIndex> dynamic;
  std::unordered_set<InternalIndex> anchored;
  CollectIndices(set, &dynamic, &anchored);

  geometry_engine_->ExcludeCollisionsWithin(dynamic, anchored);
}

template <typename T>
void GeometryState<T>::ExcludeCollisionsBetween(const GeometrySet& setA,
                                                const GeometrySet& setB) {
  std::unordered_set<InternalIndex> dynamic1;
  std::unordered_set<InternalIndex> anchored1;
  CollectIndices(setA, &dynamic1, &anchored1);
  std::unordered_set<InternalIndex> dynamic2;
  std::unordered_set<InternalIndex> anchored2;
  CollectIndices(setB, &dynamic2, &anchored2);

  geometry_engine_->ExcludeCollisionsBetween(dynamic1, anchored1, dynamic2,
                                             anchored2);
}

template <typename T>
void GeometryState<T>::RenderColorImage(const render::CameraProperties& camera,
                                        FrameId parent_frame,
                                        const Isometry3<double>& X_PC,
                                        ImageRgba8U* color_image_out,
                                        bool show_window) const {
  render::RenderEngine* engine = GetRenderEngineOrThrow(camera.fidelity);
  const Isometry3<double> X_WC = GetDoubleWorldPose(parent_frame) * X_PC;
  engine->UpdateViewpoint(X_WC);
  engine->RenderColorImage(camera, color_image_out, show_window);
}

template <typename T>
void GeometryState<T>::RenderDepthImage(
    const render::DepthCameraProperties& camera,
    FrameId parent_frame, const Isometry3<double>& X_PC,
    ImageDepth32F* depth_image_out) const {
  render::RenderEngine* engine = GetRenderEngineOrThrow(camera.fidelity);
  const Isometry3<double> X_WC = GetDoubleWorldPose(parent_frame) * X_PC;
  engine->UpdateViewpoint(X_WC);
  engine->RenderDepthImage(camera, depth_image_out);
}

template <typename T>
void GeometryState<T>::RenderLabelImage(const render::CameraProperties& camera,
                                        FrameId parent_frame,
                                        const Isometry3<double>& X_PC,
                                        ImageLabel16I* label_image_out,
                                        bool show_window) const {
  render::RenderEngine* engine = GetRenderEngineOrThrow(camera.fidelity);
  const Isometry3<double> X_WC = GetDoubleWorldPose(parent_frame) * X_PC;
  engine->UpdateViewpoint(X_WC);
  engine->RenderLabelImage(camera, label_image_out, show_window);
}

template <typename T>
std::unique_ptr<GeometryState<AutoDiffXd>> GeometryState<T>::ToAutoDiffXd()
    const {
  return std::unique_ptr<GeometryState<AutoDiffXd>>(
      new GeometryState<AutoDiffXd>(*this));
}

template <typename T>
void GeometryState<T>::CollectIndices(
    const GeometrySet& geometry_set, std::unordered_set<InternalIndex>* dynamic,
    std::unordered_set<InternalIndex>* anchored) {
  for (auto frame_id : geometry_set.frames()) {
    auto iterator = frames_.find(frame_id);
    if (iterator == frames_.end()) {
      throw std::logic_error(
          "Geometry set includes a frame id that doesn't belong to the "
          "SceneGraph: " + to_string(frame_id));
    }

    const auto& frame = iterator->second;
    for (auto geometry_id : frame.get_child_geometries()) {
      InternalGeometry& geometry = geometries_[geometry_id];
      if (geometry.has_proximity_role()) {
        dynamic->insert(geometry.internal_index());
      }
    }
  }

  for (auto geometry_id : geometry_set.geometries()) {
    if (geometries_.count(geometry_id) == 1) {
      InternalGeometry& geometry = geometries_[geometry_id];
      if (geometry.has_proximity_role()) {
        dynamic->insert(geometry.internal_index());
      }
    } else if (anchored_geometries_.count(geometry_id) == 1) {
      InternalAnchoredGeometry& geometry = anchored_geometries_[geometry_id];
      if (geometry.has_proximity_role()) {
        anchored->insert(geometry.internal_index());
      }
    } else {
      throw std::logic_error(
          "Geometry set includes a geometry id that doesn't belong to the "
          "SceneGraph: " + to_string(geometry_id));
    }
  }
}

template <typename T>
void GeometryState<T>::SetFramePoses(const FramePoseVector<T>& poses) {
  // TODO(SeanCurtis-TRI): Down the road, make this validation depend on
  // ASSERT_ARMED.
  ValidateFrameIds(poses);
  const Isometry3<T> world_pose = Isometry3<T>::Identity();
  for (auto frame_id : source_root_frame_map_[poses.source_id()]) {
    UpdatePosesRecursively(frames_[frame_id], world_pose, poses);
  }
}

template <typename T>
template <typename ValueType>
void GeometryState<T>::ValidateFrameIds(
    const FrameKinematicsVector<ValueType>& kinematics_data) const {
  SourceId source_id = kinematics_data.source_id();
  auto& frames = GetFramesForSource(source_id);
  const int ref_frame_count = static_cast<int>(frames.size());
  if (ref_frame_count != kinematics_data.size()) {
    // TODO(SeanCurtis-TRI): Determine if more specific information is required.
    // e.g., which frames are missing/added.
    throw std::runtime_error(
        "Disagreement in expected number of frames (" +
        to_string(frames.size()) + ") and the given number of frames (" +
        to_string(kinematics_data.size()) + ").");
  }
  for (auto id : frames) {
    if (!kinematics_data.has_id(id)) {
      throw std::runtime_error(
          "Registered frame id (" + to_string(id) + ") belonging to source " +
          to_string(source_id) +
          " was not found in the provided kinematics data.");
    }
  }
}

template <typename T>
void GeometryState<T>::FinalizePoseUpdate() {
  geometry_engine_->UpdateWorldPoses(X_WG_, X_WG_proximity_);
  for (RenderIndex i(0); i < X_WG_perception_.size(); ++i) {
    const InternalIndex internal_index = X_WG_perception_[i];
    low_render_engine_->UpdateVisualPose(convert(X_WG_[internal_index]), i);
  }
}

template <typename T>
SourceId GeometryState<T>::get_source_id(FrameId frame_id) const {
  const auto& frame = GetValueOrThrow(frame_id, frames_);
  return frame.get_source_id();
}

template <typename T>
void GeometryState<T>::UpdatePosesRecursively(
    const internal::InternalFrame& frame, const Isometry3<T>& X_WP,
    const FramePoseVector<T>& poses) {
  const auto frame_id = frame.get_id();
  const auto& X_PF = poses.value(frame_id);
  // Cache this transform for later use.
  X_PF_[frame.internal_index()] = X_PF;
  Isometry3<T> X_WF = X_WP * X_PF;
  // TODO(SeanCurtis-TRI): Replace this when we have a transform object that
  // allows proper multiplication between an AutoDiff type and a double type.
  // For now, it allows me to perform the multiplication by multiplying the
  // fully-defined transformation (with [0 0 0 1] on the bottom row).
  X_WF.makeAffine();
  X_WF_[frame.internal_index()] = X_WF;
  // Update the geometry which belong to *this* frame.
  for (auto child_id : frame.get_child_geometries()) {
    auto& child_geometry = geometries_[child_id];
    auto child_index = child_geometry.internal_index();
    // TODO(SeanCurtis-TRI): See note above about replacing this when we have a
    // transform that supports autodiff * double.
    X_FG_[child_index].makeAffine();
    // TODO(SeanCurtis-TRI): These matrix() shenanigans are here because I can't
    // assign a an Isometry3<double> to an Isometry3<AutoDiffXd>. Replace this
    // when I can.
    X_WG_[child_index].matrix() = X_WF.matrix() * X_FG_[child_index].matrix();
  }

  // Update each child frame.
  for (auto child_id : frame.get_child_frames()) {
    auto& child_frame = frames_[child_id];
    UpdatePosesRecursively(child_frame, X_WF, poses);
  }
}

template <typename T>
const InternalGeometryBase* GeometryState<T>::GetGeometry(GeometryId id) const {
  const auto& dynamic_iterator = geometries_.find(id);
  if (dynamic_iterator != geometries_.end()) {
    return &dynamic_iterator->second;
  }

  const auto& anchored_iterator = anchored_geometries_.find(id);
  if (anchored_iterator != anchored_geometries_.end()) {
    return &anchored_iterator->second;
  }
  return nullptr;
}

template <typename T>
InternalGeometryBase* GeometryState<T>::GetMutableGeometry(
    GeometryId id) {
  const InternalGeometryBase* geometry = GetGeometry(id);
  return const_cast<InternalGeometryBase*>(geometry);
}

template <typename T>
bool GeometryState<T>::NameIsUnique(FrameId id, Role role,
                                    const std::string& name) const {
  bool unique = true;
  if (id == InternalFrame::get_world_frame_id()) {
    for (const auto& pair : anchored_geometries_) {
      const InternalAnchoredGeometry& geometry = pair.second;
      if (geometry.has_role(role) && geometry.name() == name) {
        unique = false;
        break;
      }
    }
  } else {
    const InternalFrame& frame = GetValueOrThrow(id, frames_);
    for (GeometryId geometry_id : frame.get_child_geometries()) {
      const InternalGeometry& geometry = geometries_.at(geometry_id);
      if (geometry.has_role(role) && geometry.name() == name) {
        unique = false;
        break;
      }
    }
  }
  return unique;
}

template <typename T>
void GeometryState<T>::ThrowIfNameExistsInRole(FrameId id, Role role,
                                               const std::string& name) const {
  if (!NameIsUnique(id, role, name)) {
    throw std::logic_error("The name " + name + " has already been used by "
        "a geometry with the '" + to_string(role) + "' role.");
  }
}

template <typename T>
template <typename PropertyType>
void GeometryState<T>::AssignRoleInternal(SourceId source_id,
                                          GeometryId geometry_id,
                                          PropertyType properties, Role role) {
  if (!BelongsToSource(geometry_id, source_id)) {
    throw std::logic_error("Given geometry id " + to_string(geometry_id) +
        " does not belong to the given source id " +
        to_string(source_id));
  }
  InternalGeometryBase* geometry = GetMutableGeometry(geometry_id);
  // Must be non-null, otherwise, we never would've gotten past the
  // `BelongsToSource()` call.
  DRAKE_DEMAND(geometry != nullptr);

  if (!geometry->has_role(role)) {
    // Only test for name uniqueness if this geometry doesn't already have the
    // specified role. This is here for two reasons:
    //   1. If the role has already been assigned, we want that error to
    //      have precedence -- i.e., the name is irrelevant if the role has
    //      already been assigned. We rely on SetRole() to detect and throw.
    //   2. We don't want this to *follow* SetRole(), because we only want to
    //      set the role if the name is unique -- testing after would leave the
    //      role assigned.
    ThrowIfNameExistsInRole(geometry->frame_id(), role, geometry->name());
  }
  geometry->SetRole(std::move(properties));
}

template <typename T>
Isometry3<double> GeometryState<T>::GetDoubleWorldPose(FrameId frame_id) const {
  if (frame_id == InternalFrame::get_world_frame_id()) {
    return Isometry3<double>::Identity();
  }
  const internal::InternalFrame& frame = GetValueOrThrow(frame_id, frames_);
  return convert(X_WF_[frame.internal_index()]);
}

}  // namespace geometry
}  // namespace drake

// TODO(SeanCurtis-TRI): Currently assumes that "non-symbolic" implies
// AutoDiffXd. Update things appropriately when more non-symbolic scalars
// are available.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::GeometryState)
