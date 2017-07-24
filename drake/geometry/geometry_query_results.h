#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"

namespace drake {
namespace geometry {

// TODO(SeanCurtis-TRI): Include the objects involved -- GeometryFrameIds?
//    Determine what the query primitive is: the GeometryFrameId?  Contact
//    element?  Visual element?  Pointer?  Id?  Etc.
//    This applies to all of these return types.

/** A characterization of the intersection of two penetrating geometries. The
 characterization consists of a pair of points and a normal. The points
 represent a point on each geometry that most deeply penetrates the other
 geometry (in the normal direction). For convenience, the penetration depth
 is provided and is equal to:

 `depth = || (p_WCa - p_WCb) ⋅ nhat_AB_W ||₂`.
 @tparam T The underlying scalar type. Must be a valid Eigen scalar. */
template <typename T>
struct PenetrationAsPointPair {
  /** The id of the first geometry in the contact. */
  GeometryId id_A;
  /** The id of the second geometry in the contact. */
  GeometryId id_B;
  /** The point on A that most deeply penetrates B, measured and expressed in
   the world frame. */
  Vector3<T> p_WCa;
  /** The point on B that most deeply penetrates A, measured and expressed in
   the world frame. */
  Vector3<T> p_WCb;
  /** The unit-length normal which defines the penetration direction, pointing
   from geometry A to geometry B, measured and expressed in the world frame. */
  Vector3<T> nhat_AB_W;
  /** The penetration depth. */
  T depth{};
};

/** The data for reporting the distance between two geometries, A and B.
 @tparam T The underlying scalar type. Must be a valid Eigen scalar. */
template <typename T>
struct NearestPair {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(NearestPair)

  NearestPair() {}

  /** Constructor
   @param a       The id of the first geometry (A).
   @param b       The id of the second geometry (B).
   @param p_A     The point on geometry A's surface nearest B, in A's frame.
   @param p_B     The point on geometry B's surface nearest A, in B's frame.
   @param dist    The distance between p_A and p_B. */
  NearestPair(GeometryId a, GeometryId b, const Vector3<T>& p_A,
              const Vector3<T>& p_B, T dist) : id_A(a), id_B(b),
                                               p_ACa(p_A), p_BCb(p_B),
                                               distance(dist) {}

  /** The id of the first geometry in the pair. */
  GeometryId id_A;
  /** The id of the second geometry in the pair. */
  GeometryId id_B;
  /** The point on geometry A's surface nearest B, in A's frame. */
  Vector3<T> p_ACa;
  /** The point on geometry B's surface nearest A, in B's frame. */
  Vector3<T> p_BCb;
  /** The distance between p_A_A and p_B_B (measured in a common frame). */
  T distance{};
};

/** The data for reporting the geometry nearest a query point. The struct does
 not store the value of the query point, but assumes the user will correlate the
 result with the query previously provided query point.
 @tparam T The underlying scalar type. Must be a valid Eigen scalar. */
template <typename T>
struct PointProximity {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PointProximity)

  PointProximity() {}
  /** Constructor
   @param id              The id of the near geometry (named "A").
   @param local_point     The point on A's surface near the query point (the
                          proximal point), in A's frame.
   @param world_point     The proximal point, in the world frame.
   @param world_dir       A unit-length vector pointing from the proximal
                          point to the query point, in the world frame.
   @param dist            The distance from the query point to the proximal
                          point. */
  PointProximity(GeometryId id, const Vector3<T>& local_point,
                 const Vector3<T>& world_point, const Vector3<T>& world_dir,
                 T dist)
      : id_A(id),
        p_ACa(local_point),
        p_WCa(world_point),
        rhat_CaQ_W(world_dir),
        distance(dist) {}
  /** The id of the near geometry, named "A". */
  GeometryId id_A;
  /** The point on A's surface nearest the query point, in A's frame. */
  Vector3<T> p_ACa;
  /** The point on A's surface nearest the query point, in the world frame. */
  Vector3<T> p_WCa;
  /** An _optional_ unit-length vector indicating the direction from the point
   on A's surface to the query point Q (measured and expressed in the world
   frame). This only contains a vector if the query point does _not_ lie on the
   surface (i.e., |distance| > epsilon). */
  optional<Vector3<T>> rhat_CaQ_W;
  /** The *signed* distance between p_ACa and the query point (negative values
   imply the point lies *inside* the surface). */
  T distance{};
};

}  // namespace geometry
}  // namespace drake
