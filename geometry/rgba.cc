#include "drake/geometry/rgba.h"

namespace drake {
namespace geometry {

std::ostream& operator<<(std::ostream& out, const Rgba& rgba) {
  out << "Rgba(" << rgba.r() << ", " << rgba.g() << ", " << rgba.b() << ", "
      << rgba.a() << ")";
  return out;
}

}  // namespace geometry
}  // namespace drake
