#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/shapes/shape.h"

namespace drake {
namespace geometry {

/** Definition of a half space. In its canonical frame, the plan defining the
 boundary of the half space is that frame's z = 0 plane. By implication, the
 plane's normal points in the +z direction and the origin lies on the plane.
 Other shapes are considered to be penetrating the half space if there exists
 a point on the test shape that lies on the side of the plane opposite the
 normal. */
class HalfSpace final : public Shape {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(HalfSpace)

  /** Constructor.
   @param normal    A vector normal to the plane. It points in the "outside"
                    direction. It is _not_ assumed to be unit length. It is
                    measured and expressed in the _world_ frame.
   @param X_WP      A point that lies on the plane, measured and expressed in
                    the _world_ frame.  */
  HalfSpace(const Vector3<double>& normal, const Vector3<double>& X_WP);

  /** Reports the signed distance from the given point `p` to the half-spaces
   plane boundary. Positive values indicate *outside* the half-space. */
  double get_signed_distance(const Vector3<double>& p) const {
    return normal_.dot(p) + d_;
  }

  /** Reports the half-space's outward-pointing normal. */
  const Vector3<double>& get_normal() const { return normal_; }

  /** Returns a point on the plane. */
  const Vector3<double>& get_point_on_plane() const { return point_; }

  /** Given a plane `normal` and a point `X_FP` on the plane, both expressed in
   frame F, creates the transform `X_FC` from the half-space's canonical space
   to frame F.
   @tparam T       The underlying scalar type. Must be a valid Eigen scalar.
   @param normal   A vector perpendicular to the half-space's plane boundary.
                   Must be a non-zero vector.
   @param X_FP     A point lying on the half-space's plane boundary.
   @retval `X_FC` the pose of the canonical half-space in frame F.
   @throws std::logic_error if the normal is a zero-vector. */
  template <typename T>
  static Isometry3<T> MakePose(const Vector3<T>& normal,
                               const Vector3<T>& X_FP) {
    throw std::runtime_error("Not implemented!");
  }

 protected:
  Shape* DoClone() const override {
    return new HalfSpace(*this);
  }

 private:
  // A point on the plane.
  Vector3<double> point_;
  // Defines the implicit equation of the plane: P(x) = dot(N, x) + d
  Vector3<double> normal_;
  double d_;
};

}  // namespace geometry
}  // namespace drake
