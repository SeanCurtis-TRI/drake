#include "drake/geometry/geometry_context.h"

namespace drake {
namespace geometry {

template <typename T>
GeometryState<T>& GeometryContext<T>::get_mutable_geometry_state() {
  // This is somewhat fragile; it is predicated on *knowing* that the geometry
  // state abstract value was added first.
  return this->get_mutable_state()
      ->template get_mutable_abstract_state<GeometryState<T>>(0);
}

template <typename T>
const GeometryState<T>& GeometryContext<T>::get_geometry_state() const {
  // This is somewhat fragile; it is predicated on *knowing* that the geometry
  // state abstract value was added first.
  return this->get_state().template get_abstract_state<GeometryState<T>>(0);
}

// Explicitly instantiates on the most common scalar types.
template class GeometryContext<double>;

}  // namespace geometry
}  // namespace drake
