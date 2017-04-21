#include "drake/geometry/geometry_engine_stub.h"

#include <utility>

namespace drake {
namespace geometry {

using std::unique_ptr;
using std::vector;

template <typename T>
GeometryEngineStub<T>::GeometryEngineStub() : GeometryEngine<T>() {}



template <typename T>
GeometryIndex GeometryEngineStub<T>::AddDynamicGeometry(
    unique_ptr<GeometryInstance<T>> geometry) {
  GeometryIndex index(geometries_.size());
  geometries_.emplace_back(std::move(geometry));
  return index;
}


template <typename T>
GeometryIndex GeometryEngineStub<T>::AddAnchoredGeometry(
    std::unique_ptr<GeometryInstance<T>> geometry) {
  throw std::runtime_error("Not implemented yet.");
}

template <typename T>
void GeometryEngineStub<T>::UpdateWorldPoses(const vector<Isometry3<T>>& X_WP) {
}

// Explicitly instantiates on the most common scalar types.
template class GeometryEngineStub<double>;

}  // namespace geometry
}  // namespace drake
