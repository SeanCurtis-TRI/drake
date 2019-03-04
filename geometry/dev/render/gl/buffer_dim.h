#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/hash.h"
#include "drake/geometry/dev/render/gl/opengl_includes.h"

namespace drake {
namespace geometry {
namespace dev {
namespace render {
namespace gl {

/** Simple struct for recording the dimensions of a render target. */
class BufferDim {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BufferDim)

  BufferDim(int width, int height) : width_(width), height_(height) {}

  int width() const { return width_; }
  int height() const { return height_; }

  /** Implements the @ref hash_append concept.  */
  template <class HashAlgorithm>
  friend void hash_append(HashAlgorithm& hasher,
                          const BufferDim& dim) noexcept {
    using drake::hash_append;
    hash_append(hasher, dim.width_);
    hash_append(hasher, dim.height_);
  }

  bool operator==(const BufferDim& dim) const {
    return width_ == dim.width_ && height_ == dim.height_;
  }

 private:
  int width_{-1};
  int height_{-1};
};

/** The collection of OpenGL objects which define a render target --
 essentially, a 2D image that the depth data gets rendered to.  */
struct RenderTarget {
  GLuint frame_buffer;
  GLuint texture;
  GLuint render_buffer;
};

}  // namespace gl
}  // namespace render
}  // namespace dev
}  // namespace geometry
}  // namespace drake

namespace std {

/// Provides std::hash<BufferDim>.
template <>
struct hash<drake::geometry::dev::render::gl::BufferDim>
    : public drake::DefaultHash {};
#if 0
#if defined(__GLIBCXX__)
// https://gcc.gnu.org/onlinedocs/libstdc++/manual/unordered_associative.html
template <class T>
struct __is_fast_hash<hash<drake::geometry::dev::render::gl::BufferDim>>
    : std::false_type {};
#endif
#endif
}  // namespace std
