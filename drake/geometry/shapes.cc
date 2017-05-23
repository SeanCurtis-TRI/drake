#include "drake/geometry/shapes.h"

#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {

// -------------          HalfSpace

HalfSpace::HalfSpace(const Vector3<double>& normal,
                     const Vector3<double>& point) : Shape(HALF_SPACE) {
  normal_ = normal.normalized();
  d_ = -normal_.dot(point);
}

}  // namespace geometry
}  // namespace drake
