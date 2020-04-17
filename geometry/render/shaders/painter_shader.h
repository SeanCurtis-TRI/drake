#pragma once

#include <functional>
#include <optional>
#include <string>
#include <variant>

#include <vtkActor.h>

#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/render/render_label.h"
#include "drake/systems/sensors/color_palette.h"

namespace drake {
namespace geometry {
namespace render {
namespace shaders {

/** Maps a painter shader, as defined by perception properties, to the VTK
 implementation. The painter shader uses a mask image to combine two sources:
 canvas and paint. Where the mask image is white, the fragment value comes from
 the canvas source, where black, the paint source. Specifically, the resultant
 value will be:

    `out = mask.r * canvas + (1 - mask.R) * paint`

 where `mask.r` is the red channel of the mask image in the range [0, 1] and
 `canvas` and `paint` are the source values. The mask is a texture image and the
 sources are _optionally_ texture images. And the values of the various colors
 are evaluated at the fragment's texture coordinates.

 The PainterShader requires full specification of the appearance of the object
 in RGB images -- that means both a canvas and a paint _color_ source must be
 provided. It can optionally apply to label images as well. If both a canvas
 label _and_ paint label are provided, the mask will affect the label image
 as well.

 The painter shader is configured with the following information:

   - Specification of whether the mask is static (a single fixed texture owned
     by the render engine) or dynamic (owned by an external entity and provided
     as an InputImage).
   - Specification of the source colors either as an RGBA color or a static
     texture map.
   - Optionally, a pair of render labels corresponding to canvas and paint
     fragments, respectively. The label values _cannot_ be
     RenderLabel::kDoNotRender.
*/
class PainterShader {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PainterShader);

  /** Maybe construct a PainterShader from the specification found in the
   given `properties`.

   @return A %PainterShader if `properties` contain a fully specified painter
           shader or `nullopt` if no painter shader specified.
   @throws std::runtime_error if the properties specify a malformed painter
           shader.  */
  static std::optional<PainterShader> Make(
      const PerceptionProperties& properties);

  /** Reports if the shader uses a dynamic mask.  */
  bool is_dynamic() const { return is_dynamic_; }

  /** Reports the name of the mask -- if is_dynamic() returns `true`, this is
   the name of the expected input image, if `false`, a path to an image file. */
  const std::string& mask_name() const { return mask_name_; }

  /** Assigns this shader to the given `actor`.  */
  void AssignRgbaToActor(vtkActor* actor) const;

  bool has_labels() const { return canvas_label_.has_value(); }

  /** Assigns this shader to the given _label_ `actor` encoding the defined
   labels as rgb color using the given `label_encoder`.  */
  void AssignLabelToActor(
      vtkActor* actor,
      const std::function<systems::sensors::ColorD(RenderLabel)>& label_encoder)
      const;

 private:
  PainterShader(bool is_dynamic, std::string mask_name,
                std::variant<Eigen::Vector4d, std::string> canvas,
                std::variant<Eigen::Vector4d, std::string> paint,
                std::optional<RenderLabel> canvas_label = {},
                std::optional<RenderLabel> paint_label = {})
      : is_dynamic_(is_dynamic),
        mask_name_(std::move(mask_name)),
        canvas_color_(std::move(canvas)),
        paint_color_(std::move(paint)),
        canvas_label_(canvas_label),
        paint_label_(paint_label) {}

  // Assigns the mask texture to the actor, based on whether the mask is static
  // or dynamic.
  void AssignMaskTextureToActor(vtkActor* actor) const;

  std::string canvas_shader_string() const;

  std::string paint_shader_string() const;

  static std::string color_shader_string(
      const std::string& texture_name,
      const std::variant<Eigen::Vector4d, std::string>& color);

 private:
  bool is_dynamic_{};
  std::string mask_name_;
  std::variant<Eigen::Vector4d, std::string> canvas_color_;
  std::variant<Eigen::Vector4d, std::string> paint_color_;
  std::optional<RenderLabel> canvas_label_;
  std::optional<RenderLabel> paint_label_;
};

}  // namespace shaders
}  // namespace render
}  // namespace geometry
}  // namespace drake
