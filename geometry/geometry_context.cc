#include "drake/geometry/geometry_context.h"

#include "drake/common/autodiff.h"

namespace drake {
namespace geometry {

template <typename T>
GeometryState<T>& GeometryContext<T>::get_mutable_geometry_state() {
  return this->get_mutable_state()
      .template get_mutable_abstract_state<GeometryState<T>>(
          kGeometryStateIndex);
}

template <typename T>
const GeometryState<T>& GeometryContext<T>::get_geometry_state() const {
  return this->get_state().template get_abstract_state<GeometryState<T>>(
      kGeometryStateIndex);
}

// Explicitly instantiates on the most common scalar types.
template class GeometryContext<double>;
template class GeometryContext<AutoDiffXd>;

}  // namespace geometry
}  // namespace drake
