#pragma once

#include <utility>

#include <fcl/fcl.h>

#include "drake/geometry/proximity/distance_shape_to_shape_base.h"
#include "drake/geometry/proximity/proximity_utilities.h"
#include "drake/geometry/utilities.h"

namespace drake {
namespace geometry {
namespace internal {
namespace signed_distance {

/** A special version of the sign function; Sign(0) --> 1.  */
double Sign(const double& x) { return x < 0.0 ? -1 : 1; }

/** A utility class for determining the nearest feature on a box B to a point Q
 _and_ the point on that feature, N, closest to Q.

 For point Q (as p_BQ) in the box's frame, one can classify the point on the
 three axis-aligned intervals bounded by the box. On each axis, the point is
 either outside the interval, on the boundary of the interval, or inside the
 interval. (Numerically, "on the boundary" is considered w.r.t. a tolerance.)

 Ignoring the permutation of axes, there are a fixed number of classification
 combinations. Each combination can be directly mapped to the _type_ of feature
 that is closest. The table below shows the mapping. The contents of the row
 and _not_ the order determine the nearest feature type. Permuting the order
 merely affects which of the features of that type is actually nearest.

 |  X  |  Y  |  Z  | Nearest Feature Type  |
 | :-: | :-: | :-: | :-------------------: |
 |  O  |  O  |  O  | Vertex                |
 |  O  |  O  |  I  | Edge                  |
 |  O  |  I  |  I  | Face                  |
 |  I  |  I  |  I  | Face                  |
 |  O  |  O  |  B  | Vertex                |
 |  O  |  B  |  B  | Face                  |
 |  B  |  B  |  B  | Vertex                |
 |  I  |  I  |  B  | Face                  |
 |  I  |  B  |  B  | Edge                  |
 |  O  |  I  |  B  | Edge                  |

 The testing function uses this table to do bookkeeping on the result of
 testing each axis (via mark_inside(), mark_outside(), and mark_boundary()).
 After fully classifying the query point, the nearest point N can be acquired
 by calling p_BN_B().  */
class NearestFeature {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(NearestFeature)

  NearestFeature() = default;

  /** Makes note that the query point lies *inside* the interval along the
   indicated `axis` a negative signed `distance` units beyond the interval
   boundary in the `sign` direction. (A negative distance beyond means a
   positive distance inside.)  */
  void mark_inside(int axis, double sign, double distance) {
    std::cout << "  mark_inside(" << axis << ", " << sign << ", " << distance
              << ")\n";
    value_ += kInside;
    if (std::abs(distance) < nearest_inside_axis_dist_) {
      std::cout << "     updating inside\n";
      nearest_inside_axis_ = axis;
      nearest_inside_axis_dist_ = std::abs(distance);
      selector_(axis) = sign;
    }
  }

  /** Makes note that the query point lies *outside* the interval along the
   indicated `axis` a positive signed `distance`
   units beyond the interval boundary in the `sign` direction..  */
  void mark_outside(int axis, double sign, double distance) {
    std::cout << "  mark_outside(" << axis << ", " << sign << ")\n";
    value_ += kOutside;
    selector_(axis) = sign;
    UpdateOutside(axis, sign, std::abs(distance));
  }

  /** Makes note that the query point lies *on* the interval boundary along the
   indicated `axis` at the `sign` direction end of the interval.  */
  void mark_boundary(int axis, double sign) {
    std::cout << "  mark_boundary(" << axis << ", " << sign << ")\n";
    value_ += kBoundary;
    selector_(axis) = sign;
    UpdateOutside(axis, sign, 0.0);
  }

  /** Given the box half-dimensions `h` and the query point `p_BQ`, uses the
   nearest-feature classification to produce `p_BN_B`.  */
  template <typename T>
  Vector3<T> p_BN_B(const Eigen::Vector3d& h, const Vector3<T>& p_BQ) const {
    std::cout << "p_BN_B( " << h.transpose() << ", " << p_BQ.transpose() << ")\n";
    if (is_vertex()) {
      return selector_.cwiseProduct(h);
    } else if (is_edge()) {
      Vector3<T> p_BN;
      for (int i = 0; i < 3; ++i) {
        if (i != nearest_inside_axis_) p_BN(i) = selector_(i) * h(i);
        else p_BN(i) = p_BQ(i);
      }
      return p_BN;
    } else {
      Vector3<T> p_BN;
      for (int i = 0; i < 3; ++i) {
        std::cout << "  axis " << i << " from selector: " << (i == face_axis()) << "\n";
        if (i == face_axis()) p_BN(i) = selector_(i) * h(i);
        else p_BN(i) = p_BQ(i);
      }
      return p_BN;
    }
  }

  /** @name   Utility functions

   These functions are _not_ truly intended to be part of the public API. It is
   sufficient to mark the various axes and then query for the nearest point.
   These functions are part of those calculations but only made public to
   facilitate testing.  */
  //@{

  /** Reports true if the nearest feature is a vertex.  */
  bool is_vertex() const {
    return value_ == 3 * kOutside || value_ == 2 * kOutside + kBoundary ||
           value_ == kBoundary * 3;
  }

  /** Reports true if the nearest feature is an edge.  */
  bool is_edge() const {
    return value_ == 2 * kOutside + kInside ||
           value_ == kOutside + kInside + kBoundary ||
           value_ == kInside + 2 * kBoundary;
  }

  /** Reports true if the nearest feature is a face.  */
  bool is_face() const { return !(is_edge() || is_vertex()); }

  /** Provides a selector; the nearest point will have *some* (and possibly all)
   elements drawn from the extent of the box. This selector can be used to
   _select_ them via `feature.selector().cwiseProduct(h);`.

   Note: The result is not necessarily *the* nearest point. Other elements will
   need to be drawn from the query point itself in a complementary manner. The
   full interpretation of the selector depends on the feature type. It should
   not generally be accessed.  */
  Eigen::Vector3d selector() const {
    if (is_edge()) {
      Eigen::Vector3d edge_selector = selector_;
      edge_selector(nearest_inside_axis_) = 0;
      return edge_selector;
    } else {
      // Vertex needs the full selector. For face, only one coefficient matters.
      return selector_;
    }
  }

  /** If nearest features is a face, reports the axis the face is perpendicular
   to. The direction _sign_ can be drawn from the selector: i.e.,
   `feature.selector()(feature.face_axis())`.
   @pre is_face() is true.  */
  int face_axis() const {
    if (value_ == kInside * 3) {
      return nearest_inside_axis_;
    } else {
      return farthest_outside_face_;
    }
  }

  /** Returns the axis that reported inside closest to the boundary.  */
  int inside_axis() const { return nearest_inside_axis_; }

 //@}

 private:
  void UpdateOutside(int axis, double sign, double distance) {
    if (distance > farthest_outside_face_dist_) {
      std::cout << "     updating outside\n";
      farthest_outside_face_ = axis;
      farthest_outside_face_dist_ = distance;
      selector_(axis) = sign;
    }
  }

  // Bookkeeping for producing the nearest point N.

  // Captures the relevant signs for the direction of the query point.
  Eigen::Vector3d selector_{0, 0, 0};

  // Track the axis that is *inside* and *nearest* its own boundary.
  double nearest_inside_axis_dist_{std::numeric_limits<double>::max()};
  int nearest_inside_axis_{-1};

  // Track the axis that reports *outside* and *farthest* from its own boundary.
  double farthest_outside_face_dist_{-std::numeric_limits<double>::max()};
  int farthest_outside_face_{-1};

  // We use a tricky encoding to ignore which axes are classified in what way.
  // Given that there are only three axes, each type of classification can only
  // appear 0, 1, 2, or 3 times. We encode those into a single integer by making
  // sure each counter can only change a disjoint set of bits. For simplicity,
  // each classifiers counter is a factor of 10 from the others.
  static constexpr int kInside = 1;
  static constexpr int kOutside = 10;
  static constexpr int kBoundary = 100;
  int value_{0};
};

/** Classifies a point Q relative to a box B as being inside, outside, or on the
 surface (within a tolerance). This classification produces two pieces of
 information:
   - A vector of "location" values which reports in each dimension of the box
     whether the value of Q on the ith axis lies inside, outside, or on the
     boundary of the interval of the box on that axis.
   - A point N such that N is the point in B nearest to Q. Note: if Q is inside
     or on the box, N = Q.

 @param half_size  A vector of the half-size of the box in its canonical frame.
 @param p_BQ       The query point measured and expressed in the box's frame B.
 @param location   The per-axis location data.
 @retval p_BN      The point N (as documented above).  */
NearestFeature FindNearestFeature(const Eigen::Vector3d& half_size,
                                  const Eigen::Vector3d& p_BQ) {
  std::cout << "=============== Find Nearest Feature ===========\n";
  std::cout << "h:    " << half_size.transpose() << "\n";
  std::cout << "p_BP: " << p_BQ.transpose() << "\n";
  NearestFeature feature;
  for (int i = 0; i < 3; ++i) {
    const double tolerance = DistanceToPointRelativeTolerance(half_size(i));
    const double coord = p_BQ(i);
    double dist = std::abs(coord) - half_size(i);
    if (dist >= -tolerance) {
      if (dist > tolerance)
        feature.mark_outside(i, Sign(coord), dist);
      else
        feature.mark_boundary(i, Sign(coord));
    } else {
      feature.mark_inside(i, Sign(coord), dist);
    }
  }
  std::cout << "===============================================\n";
  return feature;
}

/** Witness data for the signed distance from a point Q to a box in frame B.  */
template <typename T>
struct BoxWitness {
  /** The point on the surface of box (N), nearest to Q.  */
  Vector3<T> p_BN_B;
  /** The gradient of B's signed distance evaluated at N.  */
  Vector3<T> grad_B;
  /** The position vector from N to Q.  */
  Vector3<T> p_NQ_B;
};

/** Computes witness data from nearest feature data and query point Q. The
 inputs and outputs are all measured and expressed in the box's frame B.

 @param feature  The nearest feature data computed for this box and the query
                 point Q, by FindNearestFeature().
 @param p_BQ_B   The position vector from B's origin to the query point Q.
 @param h        The half-extents of the box.  */
template <typename T>
BoxWitness<T> ComputeWitnessFromFeature(const NearestFeature& feature,
                                        const Vector3<T>& p_BQ_B,
                                        const Eigen::Vector3d& h) {
  BoxWitness<T> witness;
  witness.p_BN = feature.p_BN_B(h, p_BQ_B);

  if (feature.is_face()) {
    const int face_axis = feature.face_axis();
    witness.p_NQ_B = p_BQ_B - witness.p_BN;
    witness.grad_B =
        Eigen::Vector3d::Unit(face_axis) * feature.selector()(face_axis);
  } else {
    DRAKE_ASSERT(feature.is_vertex() || feature.is_edge());
    // p_NQ_B *can't* point into the box; if Q were inside the box, a face would
    // be the near feature.
    witness.p_NQ_B = p_BQ_B - witness.p_BN;
    // TODO(SeanCurtis-TRI): Don't do this calculation in T; do it in double.
    //  I'd be throwing out the derivatives anyways.
    const double kEps = std::numeric_limits<double>::epsilon();
    if (witness.p_NQ_B.dot(witness.p_NQ_B) > kEps * kEps) {
      witness.grad_B = witness.p_NQ_B.normalized();
    } else {
      // When *on* the feature, the selector has either 3 or 2 ones for vertex
      // or edge, respectively. We'll let the compiler use a literal if it can.
      const double denom = feature.is_vertex() ? std::sqrt(3) : std::sqrt(2);
      witness.grad_B = feature.selector() / denom;
    }
  }
  return witness;
}

/** Specialization for box and sphere. This is formulated such that the box is
 always shape A and the sphere is shape B. However, they are named `box` and
 `sphere` in frames B and S, respectively, leading to poses `X_FB` and `X_FS`.
 This is most important when it comes time to report witness points and
 gradient. The _sphere_ is B.  */
template <typename T>
Witness<T> Compute(
    const fcl::Boxd& box, const math::RigidTransform<T>& X_FB,
    const fcl::Sphered& sphere, const math::RigidTransform<T>& X_FS) {
  const Eigen::Vector3d h = box.side / 2;
  const Vector3<T>& p_FS_F = X_FS.translation();
  const Vector3<T> p_BS_B = X_FB.inverse() * p_FS_F;

  // Identify the nearest feature and supporting data: see NearestFeature.
  NearestFeature feature = FindNearestFeature(h, convert_to_double(p_BS_B));

  BoxWitness<T> box_witness = ComputeWitnessFromFeature(feature, p_BS_B, h);

  // TODO(SeanCurtis-TRI): I have an ugly collision between the struct's symbols
  //  and logical shape symbols (B is A?) I neee to revisit this. S1 and S2,
  //  perhaps?
  // TODO(SeanCurtis-TRI): this suggests distance should just go in the witness.
  T distance = box_witness.grad_B.dot(box_witness.p_NQ_B);
  // The witness point on _object_ A (the box) in A's frame.
  Vector3<T> p_ACa_A = box_witness.p_BN_B;

  // The witness point on _object_ B (the sphere) in B's frame.
  const Vector3<T> p_BCb_A = p_BS_B - sphere.radius * box_witness.grad_B;
  Vector3<T> p_BCb_B = X_FS.inverse() * X_FB * p_BCb_A;
  // The gradient of the signed distance field of _object_ B (the sphere)
  // evaluated at Ca -- it happens to be the negative of grad_B.
  const Vector3<T> grad_B_F = X_FB.inverse() * -box_witness.grad_B;

  // Using moves with the expectation that if T = AutoDiffXd moving the
  // derivatives will be cheaper than copying.
  return Witness<T>{std::move(p_ACa_A), std::move(p_BCb_B),
                    std::move(grad_B_F), std::move(distance)};
}

}  // namespace signed_distance
}  // namespace internal
}  // namespace geometry
}  // namespace drake
