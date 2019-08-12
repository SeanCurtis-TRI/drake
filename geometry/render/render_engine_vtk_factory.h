#pragma once

#include <memory>

#include "drake/common/drake_variant.h"
#include "drake/geometry/render/render_engine.h"

namespace drake {
namespace geometry {
namespace render {

/** The rendering mode used by RenderEngineVtk when producing color images.
 The OpenGL mode is fastest but produces the most primitive images. The ray
 tracer model has similar fidelity to OpenGL but also produces hard-edged
 shadows. The path tracer mode produces complex global illumination and depends
 on the samples per pixel.

 in which the RenderEngineVTK performs. The ray tracer can
 produce hard shadows and doesn't depend on the samples per pixel value. The
 path tracer produces complex global illumination and depends on the samples per
 pixel.  */
enum class VtkColorMode {
  kGl,
  kRayTracer,
  kPathTracer
};

struct VtkGlParams {};
struct VtkRaytraceParams {};
struct VtkPathtraceParams {
  /** The number of illumination samples per pixel. Higher numbers introduce
   higher quality at increased cost. Only has an effect if mode is
   OsprayMode::kPathTracer.  */
  int samples_per_pixel{1};
};

/** Construction parameters for the RenderEngineVtk.  */
struct RenderEngineVtkParams  {
  /** The render mode for the render engine to use for color images.  */
  VtkColorMode color_mode{VtkColorMode::kGl};

  /** The (optional) label to apply when none is otherwise specified.  */
  optional<RenderLabel> default_label{};

  /** The (optional) rgba color to apply to the (phong, diffuse) property when
    none is otherwise specified. Note: currently the alpha channel is unused
    by RenderEngineVtk.  */
  optional<Eigen::Vector4d> default_diffuse{};

  // TODO(SeanCurtis-TRI): Reconcile this with a specified background image.
  /** The rgb color for the environment background (each channel in the range
   [0, 1]). The default value (in byte values) would be [204, 229, 255].  */
  Eigen::Vector3d background_color{204 / 255., 229 / 255., 255 / 255.};

  /** @name  Color render mode parameters.

   The parameters used for the various color image render modes.  */
  //@{
  VtkGlParams gl_params;
  VtkRaytraceParams raytrace_params;
  VtkPathtraceParams pathtrace_params;
  //@}
};

/** Constructs a RenderEngine implementation which uses a VTK-based OpenGL
 renderer.

 @anchor render_engine_vtk_properties
 <h2>Geometry perception properties</h2>

 This RenderEngine implementation looks for the following properties when
 registering visual geometry, categorized by rendered image type.

 <h3>RGB images</h3>

 @todo Document the variations based on the render mode.

 | Group name | Property Name | Required |  Property Type  | Property Description |
 | :--------: | :-----------: | :------: | :-------------: | :------------------- |
 |    phong   | diffuse       | no¹      | Eigen::Vector4d | The rgba² value of the object surface. |
 |    phong   | diffuse_map   | no³      | std::string     | The path to a texture to apply to the geometry.⁴ |

 ¹ If no diffuse value is given, a default rgba value will be applied. The
   default color is a bright orange. This default value can be changed to a
   different value at construction. <br>
 ² WARNING: The alpha channel is currently ignored. <br>
 ³ If no path is specified, or the file cannot be read, the diffuse rgba value
   is used (or its default).
 ⁴ %RenderEngineVtk implements a legacy feature for associating textures with
   _meshes_. If _no_ `(phong, diffuse_map)` property is provided (or it refers
   to a file that doesn't exist), for a mesh named `/path/to/mesh.obj`,
   %RenderEngineVtk will search for a file `/path/to/mesh.png` (replacing "obj"
   with "png"). If that image exists, it will be used as a texture on the mesh
   object.

 <h3>Depth images</h3>

 No specific properties required.

 <h3>Label images</h3>

 | Group name | Property Name |   Required    |  Property Type  | Property Description |
 | :--------: | :-----------: | :-----------: | :-------------: | :------------------- |
 |   label    | id            | configurable⁵ |  RenderLabel    | The label to render into the image. |

 ⁵ %RenderEngineVtk has a default render label value that is applied to any
 geometry that doesn't have a (label, id) property at registration. If a value
 is not explicitly specified, %RenderEngineVtk uses RenderLabel::kUnspecified
 as this default value. It can be explicitly set upon construction. The possible
 values for this default label and the ramifications of that choice are
 documented @ref render_engine_default_label "here".

 <h3>Geometries accepted by %RenderEngineVtk</h3>

 As documented in RenderEngine::RegisterVisual(), a RenderEngine implementation
 can use the properties found in the PerceptionProperties to determine whether
 it _accepts_ a shape provided for registration. %RenderEngineVtk makes use of
 defaults to accept _all_ geometries (assuming the properties pass validation,
 e.g., render label validation).
 <!-- TODO(SeanCurtis-TRI): Change this policy to be more selective when other
      renderers with different properties are introduced. -->
 */
std::unique_ptr<RenderEngine> MakeRenderEngineVtk(
    const RenderEngineVtkParams& params);

}  // namespace render
}  // namespace geometry
}  // namespace drake
