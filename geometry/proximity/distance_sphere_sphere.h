#pragma once

#include <utility>

#include <fcl/fcl.h>

#include "drake/geometry/proximity/distance_shape_to_shape_base.h"
#include "drake/geometry/proximity/proximity_utilities.h"

namespace drake {
namespace geometry {
namespace internal {
namespace signed_distance {

template <typename T>
Witness<T> Compute(
    const fcl::Sphered& A, const math::RigidTransform<T>& X_FA,
    const fcl::Sphered& B, const math::RigidTransform<T>& X_FB) {
  const Vector3<T>& p_FB_F = X_FB.translation();
  const Vector3<T>& p_FA_F = X_FA.translation();
  const Vector3<T> p_BA_F = p_FA_F - p_FB_F;
  const T dist_BA = p_BA_F.norm();
  // The gradient of the signed distance of B always points away from B's
  // origin. More particularly, the gradient evaluated at Ca (the witness point
  // on the surface of A) it must point in the direction of the center of B to
  // the center of A.
  //
  // If the sphere centers are coincident (to within a tolerance), we
  // arbitrarily set the gradient vector as documented in
  // query_object.h (QueryObject::ComputeSignedDistanceToPoint).
  using std::max;
  const double tolerance =
      DistanceToPointRelativeTolerance(max(A.radius, B.radius));
  // Unit vector in x-direction of B's frame.
  const Vector3<T> Bx_F = X_FB.rotation().col(0);
  // Gradient vector expressed in B's frame.
  Vector3<T> grad_B_F = (dist_BA > tolerance) ? p_BA_F / dist_BA : Bx_F;

  T distance = p_BA_F.dot(grad_B_F) - T(A.radius) - T(B.radius);
  const Vector3<T> p_ACa_F = p_FA_F - A.radius * grad_B_F;
  Vector3<T> p_ACa_A = X_FA.inverse() * p_ACa_F;
  const Vector3<T> p_BCb_F = p_FB_F + B.radius * grad_B_F;
  Vector3<T> p_BCb_B = X_FB.inverse() * p_BCb_F;
  return Witness<T>{std::move(p_ACa_A), std::move(p_BCb_B),
                    std::move(grad_B_F), std::move(distance)};
}

}  // namespace signed_distance
}  // namespace internal
}  // namespace geometry
}  // namespace drake
