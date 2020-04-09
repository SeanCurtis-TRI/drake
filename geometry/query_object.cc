#include "drake/geometry/query_object.h"

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/geometry/scene_graph.h"

namespace drake {
namespace geometry {

using math::RigidTransform;
using math::RigidTransformd;
using render::CameraProperties;
using render::DepthCameraProperties;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;

template <typename T>
QueryObject<T>::QueryObject(const QueryObject& query_object) {
  *this = query_object;
}

template <typename T>
QueryObject<T>& QueryObject<T>::operator=(const QueryObject<T>& query_object) {
  if (this == &query_object) return *this;

  DRAKE_DEMAND(query_object.is_copyable());

  context_ = nullptr;
  scene_graph_ = nullptr;
  state_.reset();

  if (query_object.state_) {
    // Share the underlying baked state.
    state_ = query_object.state_;
  } else if (query_object.context_ && query_object.scene_graph_) {
    // Create a new baked state; make sure the source is fully updated.
    query_object.FullPoseUpdate();
    state_ = std::make_shared<GeometryState<T>>(query_object.geometry_state());
  }
  inspector_.set(state_.get());
  // If `query_object` is default, then this will likewise be default.

  return *this;
}

template <typename T>
const RigidTransform<T>& QueryObject<T>::X_WF(FrameId id) const {
  ThrowIfNotCallable();

  FullPoseUpdate();
  const GeometryState<T>& state = geometry_state();
  return state.get_pose_in_world(id);
}

template <typename T>
const RigidTransform<T>& QueryObject<T>::X_PF(FrameId id) const {
  ThrowIfNotCallable();

  FullPoseUpdate();
  const GeometryState<T>& state = geometry_state();
  return state.get_pose_in_parent(id);
}

template <typename T>
const RigidTransform<T>& QueryObject<T>::X_WG(GeometryId id) const {
  ThrowIfNotCallable();

  FullPoseUpdate();
  const GeometryState<T>& state = geometry_state();
  return state.get_pose_in_world(id);
}

template <typename T>
const GeometryState<T>& QueryObject<T>::geometry_state() const {
  // Some extra insurance in case some query *hadn't* called this.
  DRAKE_ASSERT_VOID(ThrowIfNotCallable());
  if (context_) {
    return scene_graph_->geometry_state(*context_);
  } else {
    return *state_;
  }
}

template <typename T>
void QueryObject<T>::FullPoseUpdate() const {
  // TODO(SeanCurtis-TRI): Modify this when the cache system is in place.
  if (scene_graph_) scene_graph_->FullPoseUpdate(*context_);
}

}  // namespace geometry
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::QueryObject)
