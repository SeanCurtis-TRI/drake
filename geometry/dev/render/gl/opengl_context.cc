#include "drake/geometry/dev/render/gl/opengl_context.h"

#include <cstring>
#include <stdexcept>
#include <string>

#include <GL/glx.h>

namespace drake {
namespace geometry {
namespace dev {
namespace render {
namespace gl {
namespace {

// Helper function for loading OpenGL extension functions.
template <class F>
F* GetGLXFunctionARB(const char* func_name) {
  // We must copy the string to a GLubyte buffer to avoid strict aliasing rules.
  // https://gist.github.com/shafik/848ae25ee209f698763cffee272a58f8
  GLubyte gl_func_name[128] = {};
  std::memcpy(gl_func_name, func_name, strlen(func_name) + 1);
  return reinterpret_cast<F*>(glXGetProcAddressARB(gl_func_name));
}

void gl_debug_callback(GLenum, GLenum type, GLuint, GLenum severity, GLsizei,
                       const GLchar* message, const void*) {
  fprintf(stderr,
          "GL CALLBACK: %s type = 0x%x, severity = 0x%x, message = %s\n",
          (type == GL_DEBUG_TYPE_ERROR ? "** GL ERROR **" : ""), type, severity,
          message);
}

}  // namespace

class OpenGlContext::Impl {
 public:
  explicit Impl(bool debug = false) {
    // See Offscreen Rendering section here:
    // https://sidvind.com/index.php?title=Opengl/windowless

    // Open a display.
    display_ = XOpenDisplay(0);
    if (display_ == nullptr) throw std::runtime_error("Fail to open display");

    // Get framebuffer configs.
    const int kVisualAttribs[] = {None};
    int fb_count = 0;
    GLXFBConfig* fb_configs = glXChooseFBConfig(
        display_, DefaultScreen(display_), kVisualAttribs, &fb_count);
    if (fb_configs == nullptr) {
      XCloseDisplay(display_);
      throw std::runtime_error("Fail to get FBConfig");
    }

    // Create an OpenGL context.
    const int kContextAttribs[] = {GLX_CONTEXT_MAJOR_VERSION_ARB, 4,
                                   GLX_CONTEXT_MINOR_VERSION_ARB, 5, None};
    auto glXCreateContextAttribsARB =
        GetGLXFunctionARB<GLXContext(Display*, GLXFBConfig, GLXContext, Bool,
                                     const int*)>("glXCreateContextAttribsARB");

    // NOTE: The consts True and False come from gl/glx.h (indirectly), but
    // ultimately from X11/Xlib.h.
    context_ = glXCreateContextAttribsARB(display_, fb_configs[0], 0, True,
                                          kContextAttribs);
    if (context_ == nullptr) {
      XCloseDisplay(display_);
      throw std::runtime_error("Fail to create OpenGL context.");
    }

    XFree(fb_configs);
    XSync(display_, False);

    // Make it the current context.
    if (!glXMakeContextCurrent(display_, None, None, context_)) {
      glXDestroyContext(display_, context_);
      XCloseDisplay(display_);
      throw std::runtime_error("Cannot make context current");
    }

    // Debug.
    if (debug) {
      printf("vendor: %s\n",
             reinterpret_cast<const char*>(glGetString(GL_VENDOR)));
      glEnable(GL_DEBUG_OUTPUT);
      glDebugMessageCallback(gl_debug_callback, 0);
    }
  }

  ~Impl() {
    if (display_ != nullptr || context_ != nullptr) {
      glXDestroyContext(display_, context_);
      XCloseDisplay(display_);
    }
  }

  void make_current() const {
    if (glXGetCurrentContext() != context_ &&
        !glXMakeContextCurrent(display_, None, None, context_)) {
      throw std::runtime_error("Cannot make context current");
    }
  }

  bool is_initialized() const {
    return display_ != nullptr && context_ != nullptr;
  }

 private:
  Display* display_{nullptr};
  GLXContext context_{nullptr};
};

OpenGlContext::OpenGlContext(bool debug)
    : impl_(new OpenGlContext::Impl(debug)) {}

OpenGlContext::~OpenGlContext() = default;

void OpenGlContext::make_current() const { impl_->make_current(); }

bool OpenGlContext::is_initialized() const { return impl_->is_initialized(); }

}  // namespace gl
}  // namespace render
}  // namespace dev
}  // namespace geometry
}  // namespace drake
