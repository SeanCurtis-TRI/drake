#include "drake/geometry/shapes/half_space.h"

namespace drake {
namespace geometry {

HalfSpace::HalfSpace(const Vector3<double>& normal,
                     const Vector3<double>& point) : Shape(kHalfSpace),
                                                     point_(point) {
  normal_ = normal.normalized();
  d_ = -normal_.dot(point);
}

}  // namespace geometry
}  // namespace drake
