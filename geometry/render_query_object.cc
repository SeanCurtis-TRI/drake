#include "drake/geometry/render_query_object.h"

#include "drake/common/default_scalars.h"
#include "drake/geometry/geometry_state.h"
#include "drake/geometry/input_image.h"
#include "drake/geometry/scene_graph.h"

namespace drake {
namespace geometry {

using math::RigidTransformd;
using render::CameraProperties;
using render::DepthCameraProperties;
using std::unordered_map;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;

// TODO (SeanCurtis-TRI): Give this access to the input images.

template <typename T>
void RenderQueryObject<T>::RenderColorImage(
    const CameraProperties& camera, FrameId parent_frame,
    const RigidTransformd& X_PC, bool show_window,
    ImageRgba8U* color_image_out) const {
  this->ValidateAndUpdate();
  UpdateInputImages(camera.renderer_name);
  const GeometryState<T>& state = this->geometry_state();
  return state.RenderColorImage(camera, parent_frame, X_PC, show_window,
                                color_image_out);
}

template <typename T>
void RenderQueryObject<T>::RenderDepthImage(
    const DepthCameraProperties& camera, FrameId parent_frame,
    const RigidTransformd& X_PC, ImageDepth32F* depth_image_out) const {
  this->ValidateAndUpdate();
  UpdateInputImages(camera.renderer_name);
  const GeometryState<T>& state = this->geometry_state();
  return state.RenderDepthImage(camera, parent_frame, X_PC, depth_image_out);
}

template <typename T>
void RenderQueryObject<T>::RenderLabelImage(
    const CameraProperties& camera, FrameId parent_frame,
    const RigidTransformd& X_PC, bool show_window,
    ImageLabel16I* label_image_out) const {
  this->ValidateAndUpdate();
  UpdateInputImages(camera.renderer_name);
  const GeometryState<T>& state = this->geometry_state();
  return state.RenderLabelImage(camera, parent_frame, X_PC, show_window,
                                label_image_out);
}

template <typename T>
void RenderQueryObject<T>::DoBake() {
  // Pull all input images and give all render engines a chance to update their
  // internal texture representations.
  UpdateInputImages();
}

template <typename T>
void RenderQueryObject<T>::UpdateInputImages(
    const std::string renderer_name) const {
  const_cast<GeometryState<T>&>(this->geometry_state())
      .UpdateInputImages(CollectLiveInputImages(), renderer_name);
}

template <typename T>
unordered_map<ImageId, const InputImage*>
RenderQueryObject<T>::CollectLiveInputImages() const {
  this->ThrowIfNotCallable();
  const SceneGraph<T>* sg = this->scene_graph();
  // No scene graph --> already baked and no work to do.
  if (sg == nullptr) return {};

  DRAKE_DEMAND(this->context() != nullptr);

  const GeometryState<T>& state = this->geometry_state();
  const unordered_map<ImageId, internal::InputImageDeclaration>&
      declared_images = state.input_image_set().images();
  unordered_map<ImageId, const InputImage*> input_images;
  for (const auto& [image_id, declaration] : declared_images) {
    const InputImage* image =
        &sg->image_input_port(image_id).template Eval<InputImage>(
            *this->context());
    input_images.insert({image_id, image});
  }
  return input_images;
}

}  // namespace geometry
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::RenderQueryObject)
