#include "drake/geometry/rgba.h"

#include <cmath>
#include <stdexcept>

#include <fmt/format.h>

namespace drake {
namespace geometry {

void Rgba::set(const Eigen::Ref<const Eigen::VectorXd>& rgba) {
  // Check the size (and pad it to 4).
  Eigen::Vector4d new_value;
  if (rgba.size() == 3) {
    new_value.head(3) = rgba;
    new_value[3] = 1.0;
  } else if (rgba.size() == 4) {
    new_value = rgba;
  } else {
    throw std::runtime_error(fmt::format(
        "Rgba must contain either 3 or 4 elements (given [{}])", rgba.size()));
  }

  // Check the domain.
  for (int i = 0; i < 4; ++i) {
    if (!(new_value[i] >= 0 && new_value[i] <= 1.0)) {
      throw std::runtime_error(fmt::format(
          "Rgba values must be within the range [0, 1]. Values provided: "
          "(r={}, g={}, b={}, a={})",
          new_value[0], new_value[1], new_value[2], new_value[3]));
    }
  }

  value_ = new_value;
}

std::array<uint8_t, 4> Rgba::to_bytes() const {
  auto to_byte = [](double channel) {
    return static_cast<uint8_t>(std::round(channel * 255));
  };
  return {to_byte(r()), to_byte(g()), to_byte(b()), to_byte(a())};
}

std::string Rgba::to_string() const {
  return fmt::format("({}, {}, {}, {})", this->r(), this->g(), this->b(),
                     this->a());
}

}  // namespace geometry
}  // namespace drake
