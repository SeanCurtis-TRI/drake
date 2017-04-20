#include "drake/geometry/geometry_instance.h"

namespace drake {
namespace geometry {

template <typename T>
GeometryInstance<T>::GeometryInstance(const Isometry3<T>& X_PG) : X_FG_(X_PG) {}

// Explicitly instantiates on the most common scalar types.
template class GeometryInstance<double>;

}  // namespace geometry
}  // namespace drake
