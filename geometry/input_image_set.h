#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/geometry_ids.h"

namespace drake {
namespace geometry {
namespace internal {

/* The collection of all registered input image _declarations_ for a SceneGraph
 instance. This doesn't include the images themselves, as they are provided by
 the corresponding input ports.  */
class InputImageSet {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(InputImageSet)

  InputImageSet() = default;

  /* Adds the declaration of an input port to the set. The newly declared input
   image must have a name that is unique from all previous image declarations.
   @throws std::runtime_error if `name` is used by any previous input image.
   */
  void AddInputImage(SourceId source_id, ImageId image_id, int port_index,
                     const std::string& name);

  /* Reports the name of the input image for the given id.
   @throws std::runtime_error if the id doesn't represent a declared image. */
  const std::string name(ImageId id) const;

  /* Searches for an input image with the given name. If found, returns its id,
   otherwise, returns `nullopt`.  */
  std::optional<ImageId> FindIdByName(const std::string& name) const;

  /* Reports the name of the input image for the given id.
   @throws std::runtime_error if the id doesn't represent a declared image. */
  int port_index(ImageId id) const;

 private:
  struct ImageDeclaration {
    SourceId source_id;
    ImageId image_id;
    int port_index;
    std::string name;
  };

  std::unordered_map<ImageId, ImageDeclaration> images_;
};

}  // internal
}  // geometry
}  // drake
