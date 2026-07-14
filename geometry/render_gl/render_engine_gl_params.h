#pragma once

#include <vector>

#include "drake/common/name_value.h"
#include "drake/geometry/render/light_parameter.h"
#include "drake/geometry/render/render_label.h"
#include "drake/geometry/rgba.h"

namespace drake {
namespace geometry {

/** Construction parameters for RenderEngineGl.  */
struct RenderEngineGlParams {
  /** Passes this object to an Archive.
  Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(default_diffuse));
    a->Visit(DRAKE_NVP(default_clear_color));
    a->Visit(DRAKE_NVP(lights));
    a->Visit(DRAKE_NVP(cast_shadows));
    a->Visit(DRAKE_NVP(shadow_map_size));
  }

  /** Default diffuse color to apply to a geometry when none is otherwise
   specified in the (phong, diffuse) property.  */
  Rgba default_diffuse{0.9, 0.7, 0.2, 1.0};

  /** The default background color for color images.  */
  Rgba default_clear_color{204 / 255., 229 / 255., 255 / 255., 1.0};

  /** Lights in the scene. More than five lights is an error. If no lights are
   defined, a single directional light, fixed to the camera frame, is used. */
  std::vector<render::LightParameter> lights;

  /** If true, all spot and directional lights that have valid directions cast
   shadows in color images. Point lights do not cast shadows. Geometry whose
   `(phong, diffuse)` alpha is less than one receives shadows but does not cast
   them; alpha in a diffuse texture is not inspected. Enabling this adds one
   additional scene render per shadow-casting light. */
  bool cast_shadows{false};

  /** The width and height, in pixels, of each shadow map. All shadow-casting
   lights use the same resolution. Larger maps improve fidelity at the cost of
   GPU memory and rendering time.
   @pre shadow_map_size is positive and does not exceed the OpenGL
   implementation's maximum allowable texture size. */
  int shadow_map_size{256};
};

}  // namespace geometry
}  // namespace drake
