#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/drake_optional.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/dev/geometry_index.h"
#include "drake/geometry/dev/geometry_roles.h"
#include "drake/geometry/dev/render/camera_properties.h"
#include "drake/geometry/dev/render/gl/buffer_dim.h"
#include "drake/geometry/dev/render/gl/opengl_context.h"
#include "drake/geometry/dev/render/gl/opengl_geometry.h"
#include "drake/geometry/dev/render/gl/shader_program.h"
#include "drake/geometry/dev/render/render_engine.h"
#include "drake/math/rigid_transform.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace geometry {
namespace dev {
namespace render {
namespace gl {

/** Optimized simple renderer based on direct calls to the OpenGL API.  */
class RenderEngineGl final : public RenderEngine {
 public:
  RenderEngineGl();

  RenderEngineGl& operator=(const RenderEngineGl&) = delete;
  RenderEngineGl(RenderEngineGl&&) = delete;
  RenderEngineGl& operator=(RenderEngineGl&&) = delete;

  ~RenderEngineGl() override;

  /** Inherits RenderEngine::Clone().  */
  std::unique_ptr<RenderEngine> Clone() const override;

  /** Inherits RenderEngine::AddFlatTerrain().  */
  void AddFlatTerrain() override;

  /** Inherits RenderEngine::RegisterVisual().  */
  RenderIndex RegisterVisual(const Shape& shape,
                             const PerceptionProperties& properties,
                             const Eigen::Isometry3d& X_FG) override;

  /** Inherits RenderEngine::RemoveVisual().  */
  optional<RenderIndex> RemoveVisual(RenderIndex index) override;

  /** Inherits RenderEngine::UpdateVisualPose().  */
  void UpdateVisualPose(const Eigen::Isometry3d& X_WG,
                        RenderIndex index) override;

  /** Inherits RenderEngine::UpdateViewpoint().  */
  void UpdateViewpoint(const Eigen::Isometry3d& X_WC) const override;

  /** Inherits RenderEngine::RenderColorImage().  */
  void RenderColorImage(const CameraProperties& camera,
                        systems::sensors::ImageRgba8U* color_image_out,
                        bool show_window) const override;

  /** Inherits RenderEngine::RenderDepthImage().  */
  void RenderDepthImage(
      const DepthCameraProperties& camera,
      systems::sensors::ImageDepth32F* depth_image_out) const override;

  /** Inherits RenderEngine::RenderLabelImage().  */
  void RenderLabelImage(const CameraProperties& camera,
                        systems::sensors::ImageLabel16I* label_image_out,
                        bool show_window) const override;

  /** @name    Shape reification  */
  //@{
  void ImplementGeometry(const Sphere& sphere, void* user_data) override;
  void ImplementGeometry(const Cylinder& cylinder, void* user_data) override;
  void ImplementGeometry(const HalfSpace& half_space, void* user_data) override;
  void ImplementGeometry(const Box& box, void* user_data) override;
  void ImplementGeometry(const Mesh& mesh, void* user_data) override;
  void ImplementGeometry(const Convex& convex, void* user_data) override;
  //@}

 private:
  // Copy constructor used for cloning.
  RenderEngineGl(const RenderEngineGl& other) = default;

  // Render inverse depth of the object at a specific pose in the camera frame.
  RenderTarget RenderAt(const Eigen::Matrix4f& X_CM,
                        const DepthCameraProperties& camera) const;

  // Obtain the depth image of rendered from a specific object pose. This is
  // slow because it reads the buffer back from the GPU.
  void GetDepthImage(systems::sensors::ImageDepth32F* depth_image_out,
                     const RenderTarget& target) const;

  // Provide triangle mesh definitions of the various geometries supported by
  // this renderer: sphere, cylinder, half space, box, and mesh.
  OpenGlGeometry GetSphere();
  OpenGlGeometry GetCylinder();
  OpenGlGeometry GetHalfSpace();
  OpenGlGeometry GetBox();
  OpenGlGeometry GetMesh(const std::string& filename);

  // Infrastructure for setting up the frame buffer object.
  RenderTarget SetupFBO(const DepthCameraProperties& camera);

  // Configure the model view and projection matrices.
  void SetGLProjectionMatrix(const DepthCameraProperties& camera);
  void SetGLModelViewMatrix(const Eigen::Matrix4f& X_CM) const;

  // Configure the OpenGL properties dependent on the camera properties.
  void SetCameraProperties(const DepthCameraProperties& camera);
  // Infrastructure for defining geometry -- vertex arrays, buffers, etc.
  using VertexBuffer =
      Eigen::Matrix<GLfloat, Eigen::Dynamic, 3, Eigen::RowMajor>;
  using IndexBuffer = Eigen::Matrix<GLuint, Eigen::Dynamic, 3, Eigen::RowMajor>;

  OpenGlGeometry SetupVAO(const VertexBuffer& vertices,
                          const IndexBuffer& indices);

  // The cached value transformation between camera and world frame.
  mutable math::RigidTransformd X_CW_;

  // All clones of this context share the same underlying opengl_context_. They
  // share geometry and frame buffer objects. The following structs are either
  // shared, or copy safe w.r.t. the shared context.
  std::shared_ptr<OpenGlContext> opengl_context_;

  // All of the objects below here *depend* on the OpenGL context. Right now,
  // I'm having each instance of the renderer share these OpenGl objects and
  // the context. If I'm going to do that, I may be better off making them
  // part of the Context rather than part of the renderer.
  std::shared_ptr<ShaderProgram> shader_program_;

  // One OpenGLGeometry per primitive type -- allow for instancing.
  OpenGlGeometry sphere_;
  OpenGlGeometry cylinder_;
  OpenGlGeometry half_space_;
  OpenGlGeometry box_;

  std::shared_ptr<std::unordered_map<std::string, OpenGlGeometry>> meshes_;

  std::shared_ptr<std::unordered_map<BufferDim, RenderTarget>> frame_buffers_;

  // Mapping from RenderIndex to the visual data associated with that geometry.
  // This is copied so independent renderers can have different *instances* but
  // the instances still refer to the same, shared, underlying geometry.
  std::vector<OpenGlInstance> visuals_;
};

}  // namespace gl
}  // namespace render
}  // namespace dev
}  // namespace geometry
}  // namespace drake
