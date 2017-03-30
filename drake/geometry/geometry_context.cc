#include "drake/geometry/geometry_context.h"

namespace drake {
namespace geometry {

// Explicitly instantiates on the most common scalar types.
template class GeometryContext<double>;

}  // namespace geometry
}  // namespace drake
