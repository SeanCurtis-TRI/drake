#include "drake/geometry/query_object.h"

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/geometry/scene_graph.h"

namespace drake {
namespace geometry {

template <typename T>
QueryObject<T>::QueryObject(const QueryObject&)
    : context_{nullptr}, scene_graph_{nullptr} {}

template <typename T>
QueryObject<T>& QueryObject<T>::operator=(const QueryObject<T>&) {
  context_ = nullptr;
  scene_graph_ = nullptr;
  return *this;
}

template <typename T>
const Isometry3<T>& QueryObject<T>::GetPoseInWorld(FrameId frame_id) const {
  ThrowIfDefault();
  // TODO(SeanCurtis-TRI): Modify this when the cache system is in place.
  scene_graph_->FullPoseUpdate(*context_);
  const GeometryState<T>& state = context_->get_geometry_state();
  return state.get_pose_in_world(frame_id);
}

template <typename T>
const Isometry3<T>& QueryObject<T>::GetPoseInWorld(
    GeometryId geometry_id) const {
  ThrowIfDefault();
  // TODO(SeanCurtis-TRI): Modify this when the cache system is in place.
  scene_graph_->FullPoseUpdate(*context_);
  const GeometryState<T>& state = context_->get_geometry_state();
  return state.get_pose_in_world(geometry_id);
}

template <typename T>
std::vector<PenetrationAsPointPair<double>>
QueryObject<T>::ComputePointPairPenetration() const {
  ThrowIfDefault();

  // TODO(SeanCurtis-TRI): Modify this when the cache system is in place.
  scene_graph_->FullPoseUpdate(*context_);
  const GeometryState<T>& state = geometry_state();
  return state.ComputePointPairPenetration();
}

template <typename T>
std::vector<SignedDistancePair<double>>
QueryObject<T>::ComputeSignedDistancePairwiseClosestPoints() const {
  ThrowIfDefault();

  // TODO(SeanCurtis-TRI): Modify this when the cache system is in place.
  scene_graph_->FullPoseUpdate(*context_);
  const GeometryState<T>& state = context_->get_geometry_state();
  return state.ComputeSignedDistancePairwiseClosestPoints();
}

template <typename T>
void QueryObject<T>::RenderColorImage(const render::CameraProperties& camera,
                                      const Isometry3<double>& X_WC,
                                      render::ImageRgba8U* color_image_out,
                                      bool show_window) const {
  ThrowIfDefault();

  // TODO(SeanCurtis-TRI): Modify this when the cache system is in place.
  scene_graph_->FullPoseUpdate(*context_);
  const GeometryState<T>& state = context_->get_geometry_state();
  return state.RenderColorImage(camera, X_WC, color_image_out, show_window);
}

template <typename T>
void QueryObject<T>::RenderColorImage(const render::CameraProperties& camera,
                                      FrameId parent_frame,
                                      const Isometry3<double>& X_PC,
                                      render::ImageRgba8U* color_image_out,
                                      bool show_window) const {
  ThrowIfDefault();

  // TODO(SeanCurtis-TRI): Modify this when the cache system is in place.
  scene_graph_->FullPoseUpdate(*context_);
  const GeometryState<T>& state = context_->get_geometry_state();
  return state.RenderColorImage(camera, parent_frame, X_PC, color_image_out,
                                show_window);
}

template <typename T>
void QueryObject<T>::RenderDepthImage(
    const render::DepthCameraProperties& camera,
    const Isometry3<double>& X_WC,
    render::ImageDepth32F* depth_image_out) const {
  ThrowIfDefault();

  // TODO(SeanCurtis-TRI): Modify this when the cache system is in place.
  scene_graph_->FullPoseUpdate(*context_);
  const GeometryState<T>& state = context_->get_geometry_state();
  return state.RenderDepthImage(camera, X_WC, depth_image_out);
}

template <typename T>
void QueryObject<T>::RenderDepthImage(
    const render::DepthCameraProperties& camera,
    FrameId parent_frame, const Isometry3<double>& X_PC,
    render::ImageDepth32F* depth_image_out) const {
  ThrowIfDefault();

  // TODO(SeanCurtis-TRI): Modify this when the cache system is in place.
  scene_graph_->FullPoseUpdate(*context_);
  const GeometryState<T>& state = context_->get_geometry_state();
  return state.RenderDepthImage(camera, parent_frame, X_PC, depth_image_out);
}

template <typename T>
void QueryObject<T>::RenderLabelImage(const render::CameraProperties& camera,
                                      const Isometry3<double>& X_WC,
                                      render::ImageLabel16I* label_image_out,
                                      bool show_window) const {
  ThrowIfDefault();

  // TODO(SeanCurtis-TRI): Modify this when the cache system is in place.
  scene_graph_->FullPoseUpdate(*context_);
  const GeometryState<T>& state = context_->get_geometry_state();
  return state.RenderLabelImage(camera, X_WC, label_image_out, show_window);
}

template <typename T>
void QueryObject<T>::RenderLabelImage(const render::CameraProperties& camera,
                                      FrameId parent_frame,
                                      const Isometry3<double>& X_PC,
                                      render::ImageLabel16I* label_image_out,
                                      bool show_window) const {
  ThrowIfDefault();

  // TODO(SeanCurtis-TRI): Modify this when the cache system is in place.
  scene_graph_->FullPoseUpdate(*context_);
  const GeometryState<T>& state = context_->get_geometry_state();
  return state.RenderLabelImage(camera, parent_frame, X_PC, label_image_out,
                                show_window);
}

template <typename T>
const GeometryState<T>& QueryObject<T>::geometry_state() const {
  // TODO(SeanCurtis-TRI): Handle the "baked" query object case.
  DRAKE_DEMAND(scene_graph_ != nullptr);
  DRAKE_DEMAND(context_ != nullptr);
  return context_->get_geometry_state();
}

}  // namespace geometry
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::QueryObject)
