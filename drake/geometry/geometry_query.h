#pragma once

#include <vector>

#include "drake/geometry/geometry_query_results.h"

namespace drake {
namespace geometry {

/**
 A class containing the geometric state of the world for a single context.

 This class serves as the output of the GeometryWorld and GeometrySystem.
 A valid GeometryQuery instance will support geometric queries on the geometry
 on the world predicated on a given Context. Queries include:

    - Distance between geometries
    - Collisions between geometry
    - Ray-geometry intersection
 */
class GeometryQuery {
 public:
  // TODO(SeanCurtis-TRI): Determine how to limit the scope of these queries.
  //    e.g., limit it to just those elements belonging to a single input,
  //      limit it to a subset of elements in a single input,
  //      limit it to a subset of elements which span multiple inputs.

  // TODO(SeanCurtis-TRI): Should I return error codes instead of booleans?

  // NOTE: This is just a taxonomy of the *types* of queries; the interfaces
  // should not be considered complete. Each of these fundamental queries may
  // have different variants depending on the scope.

  //----------------------------------------------------------------------------
  /** @name                   Proximity Queries

   These queries represent _proximity_ queries -- queries to determine what is
   near by, or what is closest. This is not about overlapping/penetration --
   the proximity of overlapping/penetrating objects should be zero.

   These queries are _not_ affected by collision filtering.
   */

  //@{

  /**
   Computes the pair-wise nearest points for all elements in the world.

   The output vector will *not* be cleared. Contact information will merely be
   added to the vector.

   @param[out]  near_points     A vector containing `O(N²)` pairs, where there
                                are `N` elements in the world.
   @returns True if the operation was successful.
   */
  bool ComputePairwiseClosestPoints(
      std::vector<NearestPair<T>>* near_points) const;

  // NOTE: This maps to Model::closestPointsAllToAll().
  /**
   Computes the pair-wise nearest points for all elements in the given set.

   The output vector will *not* be cleared. Contact information will merely be
   added to the vector.

   @param[in]   ids_to_check    A vector of `N` geometry ids for which the
                                pair-wise points are computed.
   @param[out]  near_points     A vector containing O(N²) pairs.
   @returns True if the operation was successful.
   */
  bool ComputePairwiseClosestPoints(
      const std::vector<int>& ids_to_check,
      std::vector<NearestPair<T>>* near_points) const;

  // NOTE: This maps to Model::closestPointsPairwise().
  /**
   Computes the pair-wise nearest points for the explicitly indicated pairs of
   bodies.

   The output vector will *not* be cleared. Contact information will merely be
   added to the vector.

   @param[in]  pairs        A vector of `N` body pairs. The closest points for
                            each pair will be computed.
   @param[out] near_points  A vector of `N` NearestPair values will be added
                            to the vector, one for each input pair.
   @returns True if the operation was successful.
   */
  bool ComputePairwiseClosestPoints(
      const std::vector<GeometryFramePair>& pairs,
      std::vector<NearestPair<T>>* near_points) const;

  // NOTE: This maps to Model::collisionDetectFromPoints().
  /**
   Determines the nearest body/element to a point for a set of points.

   @param[in]   points        An ordered list of `N` points represented
                              column-wise by a `3 x N` Matrix.
   @param[out]  near_bodies   A vector of `N` PointProximity instances such that
                              the iᵗʰ instance reports the nearest body/element
                              to the iᵗʰ point.
   @returns True if the operation was successful.
   */
  bool FindClosestBodies(const Eigen::Matrix3Xd& points,
                         std::vector<PointProximity<T>>* near_bodies) const;

  // NOTE: This maps to Model::collidingPoints().
  /**
   Determines the nearest body/element to a point for a set of points up to
   a specified `distance`.

   Given a vector of `points` in the world coordinate frame, returns the
   indices of those `points` that are within the provided `distance` of any
   collision geometry in the model.

   In other words, the index `i` is included in the returned vector of indices
   iff a sphere of radius `distance`, located at `input_points[i]` collides with
   any collision element in the model.

   @param[in]   points        An ordered list of `N` points represented
                              column-wise by a `3 x N` Matrix.
   @param[in]   distance      The maximum distance from a point that is allowed.
   @returns A vector with indices in `points` of all those points that lie
   within `distance` units of any collision geometry.
   */
  // TODO(SeanCurtis-TRI): Should this return a vector, or popualate a passed-in
  // vector?
  std::vector<size_t> FindGeometryProximalPoints(
      const Eigen::Matrix3X<T>& points, double distance) const;

  /**
   Given a vector of `points` in the world coordinate frame, reports if _any_
   of those `points` lie within a specified `distance` of any collision geometry
   in the model.

   In other words, this method tests if any of the spheres of radius
   `distance` located at `input_points[i]` collides with any part of
   the model. This method returns as soon as any of these spheres collides
   with the model. Points are not checked against one another but only against
   the existing model.

   @param[in]   points    The list of points to check for collisions against the
                          model.
   @param[in]   distance  The radius of a control sphere around each point used
                          to check for collisions with the model.
  @return True if any point is closer than `distance` units to collision
          geometry. **/
  bool IsAnyGeometryNear(const Eigen::Matrix3X<T>& points,
                         double distance) const;

  //@}

  //----------------------------------------------------------------------------
  /** @name                Collision Queries

   These queries detect _collisions_ between geometry. Two geometries collide
   if they overlap each other and are not explicitly excluded through
   @ref collision_filter_concepts "collision filtering". These algorithms find
   those colliding cases, characterize them, and report the essential
   characteristics of that collision.
   */

  //@{

  // NOTE: This maps to Model::ComputeMaximumDepthCollisionPoints().
  /**
   Computes the contact across all elements in the world. Only reports results
   for elements in *contact*; if two elements are separated, there will be no
   result for that pair.

   This method is affected by collision filtering; element pairs that have
   been filtered will not produce contacts, even if their collision geometry is
   penetrating.

   The output vector will *not* be cleared. Contact information will merely be
   added to the vector.

   @param[out]  contacts    All contacts will be aggregated in this structure.
   @returns True if the operation was successful.
   */
  bool ComputeContact(std::vector<Contact<T>>* contacts) const;

  //@}

  //----------------------------------------------------------------------------
  /** @name                  Raycast Queries

   These queries perform raycast queries. Raycast queries report what, if
   anything, lies in a particular direction from a query point.
   */

  //@{

  // NOTE: This maps to Model::collisionRaycast().
  /**
   Performs one or more raycasts against the scene geometry.

   @param[in]  origin           A `3 x N` matrix where each column specifies the
                                position of a ray's origin in the world frame.
                                If `origin` is `3 x 1`, the same origin is used
                                for all rays.
   @param[in]  ray_endpoint     A `3 x N` matrix where each column specifies a
                                point *away* from the corresponding origin
                                through which the ray passes.
   @param[out] distances        A `N`-length vector of distance values. The
                                `iᵗʰ` value is the distance along the `iᵗʰ` ray.
                                The value is negative if the ray didn't hit any
                                surface.
   @param[out] normals          A `3 x N` matrix of values, where the `iᵗʰ`
                                column is the normal of the surface where the
                                `iᵗʰ` ray intersected. Values are undefined if
                                the `iᵗʰ` distance is negative.
   @returns True if the operation was successful.
   */
  bool CastRays(const Eigen::Matrix3X<T>& origin,
                const Eigen::Matrix3X<T>& ray_endpoint,
                Eigen::VectorXd* distances, Eigen::Matrix3X<T>* normals) const;

  //@}

  // TODO(SeanCurtis-TRI): The list of Model functions not yet explicitly
  //  accounted for:
  //      potentialCollisionPoints -- really frigging weird; could include
  //        multiple penetrating points, but also non penetrating points.
  //        a) This is not called outside of tests.
  //        b) This seems to be a *very* bullet-specific method.
  //
};
}  // namespace geometry
}  // namespace drake
