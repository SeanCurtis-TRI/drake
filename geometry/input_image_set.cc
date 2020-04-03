#include "drake/geometry/input_image_set.h"

#include <fmt/ostream.h>
#include <fmt/format.h>

namespace drake {
namespace geometry {
namespace internal {

void InputImageSet::AddInputImage(SourceId source_id, ImageId image_id,
                                  int port_index, const std::string& name) {
  for (const auto& [id, image_declaration] : images_) {
    if (image_declaration.name == name) {
      throw std::runtime_error(
          fmt::format("Can't add an input image with name '{}'; it already "
                      "exists for id {}",
                      name, id));
    }
  }
  images_.insert({image_id,
                  ImageDeclaration{source_id, image_id, port_index, name}});
}

const std::string InputImageSet::name(ImageId id) const {
  const auto iter = images_.find(id);
  if (iter != images_.end()) return iter->second.name;

  throw std::runtime_error(fmt::format(
      "Cannot report input image name for invalid image id: {}", id));
}

std::optional<ImageId> InputImageSet::FindIdByName(
    const std::string& name) const {
  for (const auto& [id, image_declaration] : images_) {
    if (image_declaration.name == name) {
      return id;
    }
  }
  return std::nullopt;
}

int InputImageSet::port_index(ImageId id) const {
  const auto iter = images_.find(id);
  if (iter != images_.end()) return iter->second.port_index;

  throw std::runtime_error(fmt::format(
      "Cannot report input image port index for invalid image id: {}", id));
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
