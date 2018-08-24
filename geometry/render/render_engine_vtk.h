#pragma once

#include <memory>

#include <vtkActor.h>
#include <vtkAutoInit.h>
#include <vtkImageExport.h>
#include <vtkNew.h>
#include <vtkPolyDataAlgorithm.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>
#include <vtkWindowToImageFilter.h>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/render/render_engine.h"

#ifndef DRAKE_DOXYGEN_CXX
// This, and the ModuuleInitVtkRenderingOpenGL2, provide the basis for enabling
// VTK's OpenGL2 infrastructure.
VTK_AUTOINIT_DECLARE(vtkRenderingOpenGL2)
#endif

namespace drake {
namespace geometry {
namespace render {

#ifndef DRAKE_DOXYGEN_CXX
namespace detail {
struct ModuleInitVtkRenderingOpenGL2 {
  ModuleInitVtkRenderingOpenGL2(){
    VTK_AUTOINIT_CONSTRUCT(vtkRenderingOpenGL2)
  }
};
}  // namespace detail
#endif

/** Implementation of the RenderEngine using the VTK OpenGL renderer. */
class RenderEngineVtk : public RenderEngine,
                        private detail::ModuleInitVtkRenderingOpenGL2 {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RenderEngineVtk);

  RenderEngineVtk();

  std::unique_ptr<RenderEngineVtk> Clone() const;

  // TODO(SeanCurtis-TRI): Figure out what I'm doing with constructor and
  // cloning.

  /** Inherits RenderEngine::AddFlatTerrain().  */
  void AddFlatTerrain() override;

  /** Inherits RenderEngine::RegisterVisual().  */
  RenderIndex RegisterVisual(const Shape& shape,
                             const RenderMaterial& material) override;

  // TODO(SeanCurtis-TRI): I need a super-secret RegisterVisual in which the
  // index is specified.

  /** Inherits RenderEngine::RegisterVisual().  */
  void UpdateVisualPose(const Eigen::Isometry3d& X_WG,
                        RenderIndex geometry_id) const override;

  /** Inherits RenderEngine::UpdateViewpoint().  */
  void UpdateViewpoint(const Eigen::Isometry3d& X_WR) const override;

  /** Inherits RenderEngine::RenderColorImage().  */
  void RenderColorImage(const CameraProperties& camera,
                        ImageRgba8U* color_image_out,
                        bool show_window = false) const override;

  /** Inherits RenderEngine::RenderDepthImage().  */
  void RenderDepthImage(const DepthCameraProperties& camera,
                        ImageDepth32F* depth_image_out,
                        bool show_window = false) const override;

  /** Inherits RenderEngine::RenderLabelImage().  */
  void RenderLabelImage(const CameraProperties& camera,
                        ImageLabel16I* label_image_out,
                        bool show_window = false) const override;
  
  /** @name    Shape reification  */
  //@{
  void ImplementGeometry(const Sphere& sphere, void* user_data) override;
  void ImplementGeometry(const Cylinder& cylinder, void* user_data) override;
  void ImplementGeometry(const HalfSpace& half_space, void* user_data) override;
  void ImplementGeometry(const Box& box, void* user_data) override;
  void ImplementGeometry(const Mesh& mesh, void* user_data) override;
  //@}

  /** Returns the sky's color in an RGB image. */
  const ColorI& get_sky_color() const;

  /** Returns flat terrain's color in an RGB image. */
  const ColorI& get_flat_terrain_color() const;

 private:
  // Performs the common setup for all shape types.
  void ImplementGeometry(vtkPolyDataAlgorithm* source, void* user_data);

  std::vector<vtkSmartPointer<vtkActor>> rgbd_actors_;
  std::vector<vtkSmartPointer<vtkActor>> label_actors_;

  vtkNew<vtkRenderer> rgbd_renderer_;
  vtkNew<vtkRenderer> label_renderer_;
  vtkNew<vtkRenderWindow> rgbd_window_;
  vtkNew<vtkRenderWindow> label_window_;
  vtkNew<vtkWindowToImageFilter> rgb_filter_;
  vtkNew<vtkWindowToImageFilter> d_filter_;
  vtkNew<vtkWindowToImageFilter> label_filter_;
  vtkNew<vtkImageExport> rgb_exporter_;
  vtkNew<vtkImageExport> d_exporter_;
  vtkNew<vtkImageExport> label_exporter_;

  // TODO(SeanCurtis-TRI): Do away with this color palette; RenderLabel values
  // should be mangled into an rgb color and then the rgb color should be
  // mangled back into a 16-bit int without doing lookups. The labels can be
  // mapped to human-distinguishable colors as a post-processing operation.
  const ColorPalette color_palette_;
};

}  // namespace render
}  // namespace geometry
}  // namespace drake
