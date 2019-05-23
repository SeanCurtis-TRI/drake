#include "drake/geometry/query_object.h"

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/geometry/scene_graph.h"

namespace drake {
namespace geometry {

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
std::vector<PenetrationAsPointPair<double>>
QueryObject<T>::ComputePointPairPenetration() const {
  ThrowIfNotCallable();

  // TODO(SeanCurtis-TRI): Modify this when the cache system is in place.
  FullPoseUpdate();
  const GeometryState<T>& state = geometry_state();
  return state.ComputePointPairPenetration();
}

template <typename T>
std::vector<ContactSurface<T>>
QueryObject<T>::ComputeContactSurfaces() const {
  ThrowIfNotCallable();

  // TODO(SeanCurtis-TRI): Modify this when the cache system is in place.
  FullPoseUpdate();
  const GeometryState<T>& state = geometry_state();
  return state.ComputeContactSurfaces();
}

template <typename T>
std::vector<SignedDistancePair<T>>
QueryObject<T>::ComputeSignedDistancePairwiseClosestPoints(
    const double max_distance) const {
  ThrowIfNotCallable();

  // TODO(SeanCurtis-TRI): Modify this when the cache system is in place.
  FullPoseUpdate();
  const GeometryState<T>& state = geometry_state();
  return state.ComputeSignedDistancePairwiseClosestPoints(max_distance);
}

template <typename T>
std::vector<SignedDistanceToPoint<T>>
QueryObject<T>::ComputeSignedDistanceToPoint(
    const Vector3<T>& p_WQ,
    const double threshold) const {
  ThrowIfNotCallable();

  FullPoseUpdate();
  const GeometryState<T>& state = geometry_state();
  return state.ComputeSignedDistanceToPoint(p_WQ, threshold);
}

template <typename T>
void QueryObject<T>::RenderColorImage(const CameraProperties& camera,
                                      const RigidTransformd& X_WC,
                                      ImageRgba8U* color_image_out,
                                      bool show_window) const {
  ThrowIfNotCallable();

  // TODO(SeanCurtis-TRI): Modify this when the cache system is in place.
  FullPoseUpdate();
  const GeometryState<T>& state = geometry_state();
  return state.RenderColorImage(camera, X_WC, color_image_out, show_window);
}

template <typename T>
void QueryObject<T>::RenderColorImage(const CameraProperties& camera,
                                      FrameId parent_frame,
                                      const RigidTransformd& X_PC,
                                      ImageRgba8U* color_image_out,
                                      bool show_window) const {
  ThrowIfNotCallable();

  // TODO(SeanCurtis-TRI): Modify this when the cache system is in place.
  FullPoseUpdate();
  const GeometryState<T>& state = geometry_state();
  return state.RenderColorImage(camera, parent_frame, X_PC, color_image_out,
                                show_window);
}

template <typename T>
void QueryObject<T>::RenderDepthImage(
    const DepthCameraProperties& camera, const RigidTransformd& X_WC,
    ImageDepth32F* depth_image_out) const {
  ThrowIfNotCallable();

  // TODO(SeanCurtis-TRI): Modify this when the cache system is in place.
  FullPoseUpdate();
  const GeometryState<T>& state = geometry_state();
  return state.RenderDepthImage(camera, X_WC, depth_image_out);
}

template <typename T>
void QueryObject<T>::RenderDepthImage(const DepthCameraProperties& camera,
                                      FrameId parent_frame,
                                      const RigidTransformd& X_PC,
                                      ImageDepth32F* depth_image_out) const {
  ThrowIfNotCallable();

  // TODO(SeanCurtis-TRI): Modify this when the cache system is in place.
  FullPoseUpdate();
  const GeometryState<T>& state = geometry_state();
  return state.RenderDepthImage(camera, parent_frame, X_PC, depth_image_out);
}

template <typename T>
void QueryObject<T>::RenderLabelImage(const CameraProperties& camera,
                                      const RigidTransformd& X_WC,
                                      ImageLabel16I* label_image_out,
                                      bool show_window) const {
  ThrowIfNotCallable();

  // TODO(SeanCurtis-TRI): Modify this when the cache system is in place.
  FullPoseUpdate();
  const GeometryState<T>& state = geometry_state();
  return state.RenderLabelImage(camera, X_WC, label_image_out, show_window);
}

template <typename T>
void QueryObject<T>::RenderLabelImage(const CameraProperties& camera,
                                      FrameId parent_frame,
                                      const RigidTransformd& X_PC,
                                      ImageLabel16I* label_image_out,
                                      bool show_window) const {
  ThrowIfNotCallable();

  // TODO(SeanCurtis-TRI): Modify this when the cache system is in place.
  FullPoseUpdate();
  const GeometryState<T>& state = geometry_state();
  return state.RenderLabelImage(camera, parent_frame, X_PC, label_image_out,
                                show_window);
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

}  // namespace geometry
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::QueryObject)
