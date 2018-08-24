
#pragma once

#include <Eigen/Dense>

#include "drake/geometry/geometry_index.h"
#include "drake/geometry/render/camera_properties.h"
#include "drake/geometry/render/color_palette.h"
#include "drake/geometry/render/image.h"
#include "drake/geometry/render_material.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace render {

/** The engine for performing rasterization operations on geometry. This
 includes rgb images, depth images, and, more generally, operations that can
 be performed in the OpenGL shader pipeline. The coordinate system of
 %RenderEngine's viewpoint `R` is `X-right`, `Y-down` and `Z-forward`
 with respect to the rendered images.

 Output image format:
   - RGB (ImageRgba8U) : the RGB image has four channels in the following
     order: red, green, blue, and alpha. Each channel is represented by
     a uint8_t.

   - Depth (ImageDepth32F) : the depth image has a depth channel represented
     by a float. For a point in space `P`, the value stored in the depth
     channel holds *the Z-component of the position vector `p_RP`.*
     Note that this is different from the range data used by laser
     range finders (like that provided by DepthSensor) in which the depth
     value represents the distance from the sensor origin to the object's
     surface.
<!--
     TODO(SeanCurtis-TRI): Update when shaders can swap simple depth values
     for actual distance calculations.
-->

   - Label (ImageLabel16I) : the label image has single channel represented
     by a int16_t. The value stored in the channel holds a RenderLabel value
     which corresponds to an object class in the scene. Pixels attributable to
     no geometry contain the RenderLabel::empty_label() value. A special class
     has already been defined for "terrain": RenderLabel::terrain_label(). */
class RenderEngine : public ShapeReifier {
 public:
  // TODO(SeanCurtis-TRI): Figure out what I'm doing with constructor and
  // cloning.

  virtual ~RenderEngine() {}

  /** Adds a flat terrain to the render engine; it renders to a default render
   color and the RenderLabel::terrain_label() label value. */
  virtual void AddFlatTerrain() = 0;

  /** Registers a shape specification and returns the index of the corresponding
   render geometry. The geometry can be uniquely referenced in this engine (and
   copies of this engine) by its geometry index.

   @param shape     The shape specification to add to the render engine.
   @param material  The material to apply to the shape.
   @returns A unique index for the resultant render geometry.
   @throws std::runtime_error if the shape is an unsupported type. */
  virtual RenderIndex RegisterVisual(const Shape& shape,
                                     const RenderMaterial& material) = 0;

  // TODO(SeanCurtis-TRI): I need a super-secret RegisterVisual in which the
  // index is specified.

  /** Updates the pose of a render geometry with given pose X_WG.

   @param X_WG          The pose of the render geometry in the world frame.
   @param geometry_id   The index of the render geometry whose pose is being
                        set. */
  virtual void UpdateVisualPose(const Eigen::Isometry3d& X_WG,
                                RenderIndex geometry_id) const = 0;

  /** Updates the renderer's viewpoint with given pose X_WR.

   @param X_WR  The pose of renderer's viewpoint in the world coordinate
                system. */
  virtual void UpdateViewpoint(const Eigen::Isometry3d& X_WR) const = 0;

  // TODO(SeanCurtis-TRI): Determine if I like the fact that the images are
  // output parameters when I have a void.  Do, I *know* the image is the right
  // size?

  /** Renders and outputs the rendered color image.

   @param camera                The intrinsic properties of the camera.
   @param[out] color_image_out  The rendered color image.
   @param show_window           If true, the render window will be displayed. */
  virtual void RenderColorImage(const CameraProperties& camera,
                                ImageRgba8U* color_image_out,
                                bool show_window = false) const = 0;

  /** Renders and outputs the rendered depth image.

   @param camera                The intrinsic properties of the camera.
   @param[out] depth_image_out  The rendered depth image.
   @param show_window           If true, the render window will be displayed. */
  virtual void RenderDepthImage(const DepthCameraProperties& camera,
                                ImageDepth32F* depth_image_out,
                                bool show_window = false) const = 0;

  /** Renders and outputs the rendered label image.

   @param camera                The intrinsic properties of the camera.
   @param[out] label_image_out  The rendered label image.
   @param show_window           If true, the render window will be displayed. */
  virtual void RenderLabelImage(const CameraProperties& camera,
                                ImageLabel16I* label_image_out,
                                bool show_window = false) const = 0;
};

}  // namespace render
}  // namespace geometry
}  // namespace drake
