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

// TODO: How are 
// The ordering is inferred from the VTK code at
// https://examples.vtk.org/site/Cxx/Shaders/CubeMap/. They should be connected
// to the texture's inputs in this order (0 - 5, inclusive).

/** Defines the environment map as a cube map, with the images associated with
 the six faces of the cube explicitly enumerated. The relationship between the
 images is described in multiple online sources, (e.g., @ref Wikipedia
 https://en.wikipedia.org/wiki/Cube_mapping ).
 
 Note: definitions of e.g., "right" or "left" are only significant in that they
 communicate a relationship between the images. It is problematic to define
 an absolute orientation for these values. For example, "front" could be
 in the +Wy direction in one application or in the +Wx depending in another. */
struct CubeMap {
  /** Passes this object to an Archive.
   Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(right));
    a->Visit(DRAKE_NVP(left));
    a->Visit(DRAKE_NVP(top));
    a->Visit(DRAKE_NVP(bottom));
    a->Visit(DRAKE_NVP(front));
    a->Visit(DRAKE_NVP(back));
  }

  std::string right;
  std::string left;
  std::string top;
  std::string bottom;
  std::string front;
  std::string back;
};

/** Defines the environment map with a single image. The environment map is
 rendered using equirectangluar coordinates. */
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

/** The definition of an environment map. */
struct EnvironmentMap {
  /** Passes this object to an Archive.
   Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(skybox));
    a->Visit(DRAKE_NVP(texture));
  }

  /** If true, the environment map will be rendered in a sky box. If false, it
   won't be visible in the background, but it will still illuminate objects. */
  bool skybox{true};

  /* The encoding of the environment texture -- it can be as either a single
   equirectangular image, or a set of six cube face images. */
  std::variant<EquirectangularMap, CubeMap> texture;
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
  }

  /** (Deprecated.) The default_label is no longer configurable. <br>
   This will be removed from Drake on or after 2023-12-01. */
  std::optional<render::RenderLabel> default_label;

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

  /** Either no environment map, or an equirectangular environment map specified
   by its file path. It should either be a path to a .png/.jpg file or a high
   dynamic range image like .hdr. If an environment map is provided, it replaces
   the default lighting (the map provides illumination). That means the usual
   camera head lamp will not be present. Lights can be explicitly added to
   combine with the environment map. */
  std::optional<EnvironmentMap> environment_map;
};

}  // namespace geometry
}  // namespace drake
