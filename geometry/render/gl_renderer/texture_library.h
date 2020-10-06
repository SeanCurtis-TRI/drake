#pragma once

#include <map>
#include <optional>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/render/gl_renderer/opengl_context.h"

namespace drake {
namespace geometry {
namespace render {
namespace internal {

/* Stores a set of OpenGl textures, referenced by their in-filesystem name.
 Its purpose is to guarantee that an image in the file system is only stored
 once in an OpenGl context. As such, a %TextureLibrary is associated with an
 instance of OpenGlContext.

 This class has been designed such that the textures are loaded implicitly, as
 such requesting the OpenGL texture id associated with a file name is called
 on a const method but may change the underlying state.  */
class TextureLibrary {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(TextureLibrary)

  explicit TextureLibrary(const OpenGlContext* context);

  /* Returns the unique OpenGl identifier for the texture with the given name.
   Errors in loading the texture are processed *silently*. This will attempt
   to load the image if a texture hasn't already been added with the given
   `file_name`.

   @param file_name  The absolute path to the file containing the texture data.
   @returns A valid OpenGl identifier if the texture has been successfully
            added to the library, std::nullopt if not.  */
  std::optional<GLuint> GetTextureId(const std::string& file_name);

 private:
  const OpenGlContext* context_{};

  /* The mapping from requested texture file name to its OpenGl id (as existing
   in context_).  */
  std::map<std::string, GLuint> textures_;
};

}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake
