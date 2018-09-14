#include "drake/geometry/geometry_properties.h"

namespace drake {
namespace geometry {

// NOTE: Because we actually use string as the lookup, we don't want to keep
// creating a new string every time the default group name is accessed.
// NOLINTNEXTLINE(runtime/string)
const std::string GeometryProperties::kDefaultGroup("__^default^__");

}  // namespace geometry
}  // namespace drake
