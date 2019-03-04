#pragma once

#include <memory>

#include "drake/geometry/dev/render/gl/opengl_includes.h"

namespace drake {
namespace geometry {
namespace dev {
namespace render {
namespace gl {

/// Handle OpenGL context initialization, clean-up and generic OpenGL queries.
/// TODO(duy): provide mechanisms to switch back and forth from different
/// contexts.
class OpenGlContext {
 public:
  /// Constructor. Open an X display and initialize an OpenGL's context. The
  /// display will be open and ready for offscreen rendering, but no window is
  /// visible.
  explicit OpenGlContext(bool debug = false);
  ~OpenGlContext();

  /// Makes this context current or throws.
  void make_current() const;

  bool is_initialized() const;

 private:
  // Note: we are dependent on `GL/glx.h` but don't want to let that bleed into
  // other code. So, we pimpl this up so that glx.h lives only in the
  // implementation.
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gl
}  // namespace render
}  // namespace dev
}  // namespace geometry
}  // namespace drake
