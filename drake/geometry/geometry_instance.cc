#include "drake/geometry/geometry_instance.h"

#include <utility>

namespace drake {
namespace geometry {

template <typename T>
GeometryInstance<T>::GeometryInstance(const Isometry3<T>& X_PG,
                                      std::unique_ptr<Shape> shape)
    : X_FG_(X_PG), shape_(std::move(shape)) {}

template <typename T>
GeometryInstance<T>::GeometryInstance(const Isometry3<T>& X_PG,
                                      std::unique_ptr<Shape> shape,
                                      const VisualMaterial& vis_material)
    : X_FG_(X_PG), shape_(std::move(shape)), visual_material_(vis_material) {}

// Explicitly instantiates on the most common scalar types.
template class GeometryInstance<double>;

}  // namespace geometry
}  // namespace drake
