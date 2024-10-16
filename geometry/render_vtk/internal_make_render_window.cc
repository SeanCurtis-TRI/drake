#include "drake/geometry/render_vtk/internal_make_render_window.h"

// These include files incorporate `#define ...` madness that causes
// RenderEngineVtk to not even compile anymore because some keyword(s)
// and/or identifier(s) have been replaced, so we must segregate this
// helper function into its own little translation unit.

// To ease build system upkeep, we annotate VTK includes with their deps.
#if defined(__APPLE__)
#include <vtkCocoaRenderWindow.h>  // vtkRenderingOpenGL2
#else
#include <vtkXOpenGLRenderWindow.h>    // vtkRenderingOpenGL2
#include <vtkglad/include/glad/glx.h>  // vtkglad
#endif

#include "drake/common/drake_assert.h"
#include "drake/common/unused.h"

namespace drake {
namespace geometry {
namespace render_vtk {
namespace internal {

vtkSmartPointer<vtkRenderWindow> MakeRenderWindow() {
  vtkSmartPointer<vtkRenderWindow> result;
#if defined(__APPLE__)
  result = vtkSmartPointer<vtkCocoaRenderWindow>::New();
#else
  static const int kVersion = []() {
    // Open the library at most once per process.
    return gladLoaderLoadGLX(nullptr, 0);
  }();
  unused(kVersion);
  result = vtkSmartPointer<vtkXOpenGLRenderWindow>::New();
#endif
  DRAKE_DEMAND(result != nullptr);
  return result;
}

}  // namespace internal
}  // namespace render_vtk
}  // namespace geometry
}  // namespace drake
