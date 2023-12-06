#pragma once

#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/name_value.h"
#include "drake/geometry/render/light_parameter.h"
#include "drake/geometry/render/render_label.h"

namespace drake {
namespace geometry {

// TODO(SeanCurtis-TRI): When CubeMap is implemented, replace NullTexture with
// CubeMap.

/** (Internal use only) A place holder indicating that no texture has been
 provided for environment map (and, therefore, no environment map). */
struct NullTexture {
  /** Passes this object to an Archive.
   Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive*) {}
};

struct EquirectangularMap {
  /** Passes this object to an Archive.
   Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(path));
  }

  // TODO(SeanCurtis-TRI): It would be nice if this supported package. To that
  // end proper URIs including file:// or even data://, I suppose.
  /** The path to the map file. */
  std::string path;
};

struct EnvironmentMap {
  /** Passes this object to an Archive.
   Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(skybox));
    a->Visit(DRAKE_NVP(texture));
  }

  /** If true, the environment map will be rendered in a sky box. If false, it
   won't be visible in the background, but it will illuminate objects. */
  bool skybox{true};

  /** The texture to use for the environment map. If none is defined, there is
   no environment map.

   In yaml, this must be spelled out with the type tag. E.g.,

   @code{yaml}
     environment_map:
       skybox: true
       texture: !EquirectangularMap
         path: /path/to/environment_image.hdr
   @endcode

   @experimental
   @note This will change from a simple file path to a more comprehensive URI
   soon. So, be aware you will have to change /path/image.png to
   file:///path/image.png in the near future. */
  std::variant<NullTexture, EquirectangularMap> texture;
};

/** Construction parameters for the RenderEngineVtk.  */
struct RenderEngineVtkParams {
  /** Passes this object to an Archive.
  Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(default_diffuse));
    a->Visit(DRAKE_NVP(default_clear_color));
    a->Visit(DRAKE_NVP(lights));
    a->Visit(DRAKE_NVP(environment_map));
    a->Visit(DRAKE_NVP(exposure));
    a->Visit(DRAKE_NVP(cast_shadows));
    a->Visit(DRAKE_NVP(shadow_map_size));
  }

  /** The (optional) rgba color to apply to the (phong, diffuse) property when
    none is otherwise specified. Note: currently the alpha channel is unused
    by RenderEngineVtk.  */
  std::optional<Eigen::Vector4d> default_diffuse{};

  /** The rgb color to which the color buffer is cleared (each
   channel in the range [0, 1]). The default value (in byte values) would be
   [204, 229, 255].  */
  Eigen::Vector3d default_clear_color{204 / 255., 229 / 255., 255 / 255.};

  /** Lights in the scene. If no lights are defined, a single directional light,
   fixed to the camera frame, is used.

   Note: RenderEngineVtk does not have a hard-coded limit on the number of
         lights; but more lights increases rendering cost.
   Note: the attenuation values have no effect on VTK *directional* lights. */
  std::vector<render::LightParameter> lights;

  /** An optional environment map. When providing the environment map, the
   render engine will be configured for physically-based rendering (PBR); all
   materials will be promoted to be a PBR material. This may change the
   appearance of any geometries introduced as primitives or .obj meshes.

   Furthermore, if an environment map is specified, it *replaces* the default
   lighting (the map itself illuminates the scene). The usual camera head lamp
   will not be present. Lights *can* be explicitly added to combine with the
   environment map. */
  std::optional<EnvironmentMap> environment_map;

  /** Exposure is an aspect of "tone mapping" (as described in
   <a href="https://www.kitware.com/pbrj1/">VTK's description of its PBR
   capabilities</a>). Drake uses the GenericFilmic tone mapper and exposes
   the *exposure* property.

   The most common use for the `exposure` parameter is to combine a environment
   with shadow-casting lights. If the environment map "contains" a great deal
   of energy, it may overpower the strength of your lights. By reducing the
   exposure, the shadows caused by the lights will become more apparent. */
  double exposure{1};

  /** If `true`, lights will cast shadows.
   <!-- TODO: Figure out the limits on casting shadows. -->
   <!-- TODO: Should this be a per-light basis? That might be tricky, but I
    can use shadow attenuation to omit lights from casting shadows. -->
  */
  bool cast_shadows{true};

  /** The size of texture map to use for shadow maps. Note: this is a global
   setting. All shadow casting lights will use a map of the same size. Larger
   map sizes increase GPU memory usage and rendering times. */
  int shadow_map_size{256};
};

}  // namespace geometry
}  // namespace drake
