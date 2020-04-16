#pragma once

#include <string>
#include <variant>

#include <vtkActor.h>

#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_roles.h"

namespace drake {
namespace geometry {
namespace render {
namespace shaders {

/** Maps a painter shader, as defined by perception properties, to the VTK
 implementation. The painter shader uses a mask image to combine two color
 sources: canvas and paint. Where the mask image is white, color from the
 canvas is shown, and where black, paint. Specifically, the resultant color
 will be:

    `out = mask.r * canvas + (1 - mask.R) * paint`

 where `mask.r` is the red channel of the mask image in the range [0, 1] and
 `canvas` and `paint` are the source colors. The mask is a texture image and the
 sources are _optionally_ texture images. And the values of the various colors
 are evaluated at the fragment's texture coordinates.

 The painter shader is configured with the following information:

   - Knowledge of whether the mask is static or dynamic.
   - Specification of the source colors either as an RGBA color or a static
     texture map.
*/
class PainterShader {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PainterShader);

  PainterShader(bool is_dynamic, std::string mask_name,
                std::variant<Eigen::Vector4d, std::string> canvas,
                std::variant<Eigen::Vector4d, std::string> paint)
      : is_dynamic_(is_dynamic),
        mask_name_(std::move(mask_name)),
        canvas_color_(std::move(canvas)),
        paint_color_(std::move(paint)) {}

        /** Maybe construct a PainterShader from the specification found in the
         given `properties`.

         @return A %PainterShader if `properties` contain a fully specified
         painter shader or `nullopt` if no painter shader specified.
         @throws std::runtime_error if the properties specify a malformed
         painter shader.  */
        static std::optional<PainterShader> Make(
            const PerceptionProperties& properties);

  /** Reports if the shader uses a dynamic mask.  */
  bool is_dynamic() const { return is_dynamic_; }

  /** Reports the name of the mask -- if is_dynamic() returns `true`, this is
   the name of the expected input image, if `false`, a path to an image file. */
  const std::string& mask_name() const { return mask_name_; }

  /** Assigns this shader to the given `actor`.  */
  void AssignToActor(vtkActor* actor) const;

 private:
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
};

}  // namespace shaders
}  // namespace render
}  // namespace geometry
}  // namespace drake
