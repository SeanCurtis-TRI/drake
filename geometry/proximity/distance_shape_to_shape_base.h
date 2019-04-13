#pragma once

#include <string>

#include <fmt/format.h>

#include "drake/common/eigen_types.h"
#include "drake/common/nice_type_name.h"
#include "drake/geometry/proximity/proximity_utilities.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {
namespace signed_distance {

/** The witness data for a signed distance value between two geometries A and
 B. The "witness" concept refers to a pair of points: one on the surface of each
 geometry. The signed distance and the gradient of the signed distance is
 expressed in terms of those witness points.

 See SignedDistancePair for more details.

 @tparam T The scalar type.
 */
template<typename T>
struct Witness {
  /** The witness point on geometry A's surface, expressed in A's frame. */
  Vector3<T> p_ACa_A;

  /** The witness point on geometry B's surface, expressed in B's frame. */
  Vector3<T> p_BCb_B;

  /** ∇φ_B(c_B) expressed in the world frame. */
  Vector3<T> nhat_BA_F;

  /** The signed distance between p_ACa and p_BCb. */
  T distance{};
};

/** Computes the signed distance (and witness data) between two shapes. See
 SignedDistanceWitness for details on the witness data.
 By default, the query is unsupported for every (`ShapeA`, `ShapeB`, `T`) triple
 and they must be explicitly whitelisted via specialization to be supported.

 @param shape_A     The first shape A.
 @param X_FA        The pose of shape A in some frame F.
 @param shape_B     The second shape B.
 @param X_FB        The pose of shape B in some frame F.
 @return The signed distance witness data.
 @tparam T          The scalar type to use for the computation.
 @tparam ShapeA     The type of the first shape.
 @tparam ShapeB     The type of the second type.
 */
template <typename T, typename ShapeA, typename ShapeB>
Witness<T> Compute(const ShapeA& shape_A,
                   const math::RigidTransform<T>& /* X_FA */,
                   const ShapeB& shape_B,
                   const math::RigidTransform<T>& /* X_FB */) {
  throw std::logic_error(fmt::format(
      "Signed distance computations are not supported for scalar "
      "'{}' and the shape pair ({}, {})",
      NiceTypeName::Get<T>(), ShapeName(shape_A), ShapeName(shape_B)));
}

}  // namespace signed_distance
}  // namespace internal
}  // namespace geometry
}  // namespace drake
