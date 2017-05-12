#pragma once

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "drake/common/drake_optional.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_index.h"
#include "drake/geometry/geometry_query_inputs.h"
#include "drake/geometry/shapes.h"

namespace drake {
namespace geometry {

// forward declaration
class Geometry;
template <typename T> class GeometryInstance;

namespace internal {

/** Internal version of the NearestPair data which contains geometry engine
 *index* values instead of global geometry identifiers.
 @see drake::geometry::NearestPair
 @tparam T The underlying scalar type. Must be a valid Eigen scalar. */
template <typename T>
struct NearestPair {
  NearestPair() {}
  NearestPair(GeometryIndex a, GeometryIndex b, const Vector3<T>& p_A,
              const Vector3<T>& p_B, T dist) : index_A(a), index_B(b),
                                               p_A_A(p_A), p_B_B(p_B),
                                               distance(dist) {}
  /** The engine index of the first geometry in the pair. */
  GeometryIndex index_A;
  /** The engine index of the second geometry in the pair. */
  GeometryIndex index_B;

  /** The point on A nearest B, in A's frame. */
  Vector3<T> p_A_A;

  /** The point on B nearest A, in B's frame. */
  Vector3<T> p_B_B;

  /** The distance between p_A_A and p_B_B (measured in a common frame). */
  T distance{};
};

/** Internal version of the PointProximity data which contains geometry engine
 *index* value instead of global geometry identifier.
 @see drake::geometry::PointProximity
 @tparam T The underlying scalar type. Must be a valid Eigen scalar. */
template <typename T>
struct PointProximity {
  PointProximity() {}
  PointProximity(GeometryIndex index, Vector3<T> local_point,
                 Vector3<T> world_point, Vector3<T> world_dir, T dist)
      : index_A(index),
        p_ACa(local_point),
        p_WCa(world_point),
        rhat_CaQ_W(world_dir),
        distance(dist) {}
  /** The index of the near geometry, named "A". */
  GeometryIndex index_A;
  /** The point on A's surface nearest the query point, in A's frame. */
  Vector3<T> p_ACa;
  /** The point on A's surface nearest theq uery point, in the world frame. */
  Vector3<T> p_WCa;
  /** A unit-length vector indicating the direction from the point on A's
   surface to the query point Q (measured and expressed in the world frame). */
  Vector3<T> rhat_CaQ_W;
  /** The *signed* distance between p_ACa and the query point (negative values
   imply the point lies *inside* the surface). */
  T distance{std::numeric_limits<double>::infinity()};
};
}  // namespace internal

/**
 A geometry engine is the underlying engine for computing the results of
 geometric queries.

 It owns the geometry instances and, once it has been provided with the poses
 of the geometry, it provides geometric queries on that geometry. This
 serves as the abstract interface for all engines. Specific implementations
 will implement the query evaluations.

 @internal Historically, this replaces the DrakeCollision::Model class.

 @tparam T The underlying scalar type. Must be a valid Eigen scalar. */
template <typename T>
class GeometryEngine {
 public:
  virtual ~GeometryEngine() {}

  std::unique_ptr<GeometryEngine> Clone() const {
    return std::unique_ptr<GeometryEngine>(DoClone());
  }

  /** @name                    Geometry Management
   @{ */

  /** Reports the _minimum_ vector size required for a successful invocation of
   UpdateWorldPoses(). A vector of this size is guaranteed to span all of the
   active geometry indices. */
  virtual int get_update_input_size() const = 0;

  /** Add movable geometry to the engine. The engine takes ownership of the
   instance. The instances transform is a fixed pose relative to an unknown
   parent frame (which will be determined later.
   @param geometry    The geometry instance to add to the engine.
   @return  An index by which the geometry can be referenced later. */
  // TODO(SeanCurtis-TRI): Include the invocation of the geometry.
  virtual GeometryIndex AddDynamicGeometry(std::unique_ptr<Shape> shape) = 0;

  /** Add anchored geometry to the engine. The engine takes ownership of the
   instance. The instance's pose is a fixed pose relative to the _world_ frame
   `W`.
   @param geometry    The geometry instance to add to the engine.
   @return  An index by which the geometry can be referenced later. */
  virtual GeometryIndex AddAnchoredGeometry(
      std::unique_ptr<GeometryInstance<T>> data) = 0;

  /** Removes the geometry associated with the given `index` from the engine.
   To maintain a compact representation, the engine can move one other geometry
   into the just-vacated `index` site. If it does so, the return value will
   contain a GeometryIndex. If it chooses not to (or if it isn't possible --
   there are no remaining geometries -- it will contain nothing. */
  virtual optional<GeometryIndex> RemoveGeometry(GeometryIndex index) = 0;

  /** Provides the poses for all of the geometries in the engine. This vector
   should span the full range of active GeometryIndex values provided by the
   engine. The iᵗʰ entry contains the pose for the frame whose GeometryIndex
   value is `i`.
   @param X_WG     The poses of each geometry `G` measured and expressed in the
                   world frame `W`. */
  // TODO(SeanCurtis-TRI): I could do things here differently a number of ways:
  //  1. I could make this move semantics (or swap semantics).
  //  2. I could simply have a method that returns a mutable reference to such
  //    a vector and the caller sets values there directly.
  virtual void UpdateWorldPoses(const std::vector<Isometry3<T>>& X_WG) = 0;

  //@}

  /** @name                   Proximity Queries

   These queries represent _proximity_ queries -- queries to determine what is
   near by, or what is closest. This is not about overlapping/penetration --
   the proximity of overlapping/penetrating objects should be zero.

   These queries are _not_ affected by collision filtering. */

  //@{

  /** Computes the pair-wise nearest points for all elements in the world.

   The output vector will *not* be cleared. Proximity information will merely be
   added to the vector.

   @param[out]  near_points     A vector containing `O(N²)` pairs, where there
                                are `N` elements in the world.
   @returns True if the operation was successful. */
  virtual bool ComputePairwiseClosestPoints(
      std::vector<internal::NearestPair<T>>* near_points) const = 0;

  // NOTE: This maps to Model::closestPointsAllToAll().
  /** Computes the pair-wise nearest points for all elements in the given set.

   The output vector will *not* be cleared. Contact information will merely be
   added to the vector.

   @param[in]   ids_to_check    A vector of `N` geometry ids for which the
                                pair-wise points are computed.
   @param[out]  near_points     A vector containing O(N²) pairs.
   @returns True if the operation was successful. */
  virtual bool ComputePairwiseClosestPoints(
      const std::vector<GeometryIndex>& ids_to_check,
      std::vector<internal::NearestPair<T>>* near_points) const = 0;

  // NOTE: This maps to Model::closestPointsPairwise().
  /** Computes the pair-wise nearest points for the explicitly indicated pairs of
   bodies.

   The output vector will *not* be cleared. Contact information will merely be
   added to the vector.

   @param[in]  pairs        A vector of `N` body pairs. The closest points for
                            each pair will be computed.
   @param[out] near_points  A vector of `N` NearestPair values will be added
                            to the vector, one for each input pair.
   @returns True if the operation was successful. */
  virtual bool ComputePairwiseClosestPoints(
      const std::vector<internal::GeometryIndexPair>& pairs,
      std::vector<internal::NearestPair<T>>* near_points) const = 0;

  // NOTE: This maps to Model::collisionDetectFromPoints().
  /** Determines the nearest body/element to a point for a set of points. This
   only considers *convex* geometry.

   @param[in]   points        An ordered list of `N` points represented
                              column-wise by a `3 x N` Matrix.
   @param[out]  near_bodies   A vector of `N` PointProximity instances such that
                              the iᵗʰ instance reports the nearest body/element
                              to the iᵗʰ point. The vector is assumed to be
                              empty already.
   @returns True if the operation was successful. */
  virtual bool FindClosestGeometry(
      const Eigen::Matrix3Xd& points,
      std::vector<internal::PointProximity<T>>* near_bodies) const = 0;
  //@}
 protected:
  /*! NVI implementation for cloning GeometryEngine instances.
   @return A _raw_ pointers to the newly cloned GeometryEngine instance.
   */
  virtual GeometryEngine* DoClone() const = 0;

 private:
};
}  // namespace geometry
}  // namespace drake
