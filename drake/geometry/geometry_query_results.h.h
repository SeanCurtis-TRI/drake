#pragma once

#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {

// TODO(SeanCurtis-TRI): Include the objects involved -- GeometryFrameIds?
//    Determine what the query primitive is: the GeometryFrameId?  Contact
//    element?  Visual element?  Pointer?  Id?  Etc.
//    This applies to all of these return types.

/**
 The data for a single contact between two bodies/elements.

 @tparam T The underlying scalar type. Must be a valid Eigen scalar.
 */
template <typename T>
struct Contact {
  /** The point of contact in the world frame. */
  Eigen::Vector3<T> point_W;

  /** The contact normal in the world frame. */
  Eigen::Vector3<T> normal_W;

  /** The penetration distance. */
  double distance{};
};

/**
 The data for reporting the distance between two bodies/elements.

 @tparam T The underlying scalar type. Must be a valid Eigen scalar.
 */
template <typename T>
struct NearestPair {
  /** The point on A nearest B, in A's frame. */
  Eigen::Vector3<T> p_A_A;

  /** The point on B nearest A, in B's frame. */
  Eigen::Vector3<T> p_B_B;

  /** The distance between p_A_A and p_B_B (measured in a common frame). */
  double distance{};
};

/**
 The data for reporting the body/element nearest a point.

 @tparam T The underlying scalar type. Must be a valid Eigen scalar.
 */
template <typename T>
struct PointProximity {
  /** The point on A nearest the query point, in A's frame. */
  Eigen::Vector3<T>p_A_A;

  /** The distance between p_A_A and p_B_B (measured in a common frame). */
  double distance{};

  // TODO(SeanCurtis-TRI): Should I include normal direction and world-frame
  // value of nearest point?
};

}  // namespace geometry
}  // namespace drake
