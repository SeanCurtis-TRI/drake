#pragma once

#include "drake/geometry/geometry_query_results.h"

namespace drake {
namespace geometry {

/**
 A class containing the geometric state of the world for a single context.

 This class serves as the output of the GeometryWorld and GeometrySystem.
 A valid GeometryQuery instance will support geometric queries on the geometry
 on the world predicated on a given Context.  Queries include:

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
  // should not be considered complete.  Each of these fundamental queries may
  // have different variants depending on the scope.

  //----------------------------------------------------------------------------
  /** @name                   Proximity Queries

   These queries represent _proximity_ queries -- queries to determine what is
   near by, or what is closest.  This is not about overlapping -- the proximity
   of overlapping objects should be zero.
   */

  //@{

  // NOTE: This maps to Model::closestPointsAllToAll().
  /**
   Computes the pair-wise nearest points for all elements in the world.

   The output vector will *not* be cleared.  Contact information will merely be
   added to the vector.

   @param[out]  near_points     All contacts will be aggregated in this
                                structure.
   @returns True if the operation was successful.
   */
  bool ComputePairwiseClosestPoints(
      std::vector<NearestPair>* near_points) const;

  // NOTE: This maps to Model::closestPointsPairwise().
  /**
   Computes the pair-wise nearest points for the explicitly indicated pairs of
   bodies.

   @param[in]  pairs        A "list" of body pairs to find the distance between.
   @param[out] near_points  A similarly sized list of distance structs.
   @returns True if the operation was successful.
   */
  bool ComputePairwiseClosestPoints(
      const std::vector<GeometryFramePair>& pairs,
      std::vector<NearestPair>* near_points) const;

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
  bool NearestToPoints(const Eigen::Matrix3Xd& points,
                       std::vector<PointProximity>* near_bodies) const;

  //@}

  //----------------------------------------------------------------------------
  /** @name                Collision Queries

   These queries detect _collisions_ between geometry.  Two geometries collide
   if they overlap each other and are not explicitly excluded through
   @ref collision_filter_concepts "collision filtering".  These algorithms find
   those colliding cases, characterize them, and report the essential
   characteristics of that collision.
   */

  //@{

  // NOTE: This maps to Model::ComputeMaximumDepthCollisionPoints().
  /**
   Computes the contact across all elements in the world.  Only reports results
   for elements in *contact*; if two elements are separated, there will be no
   result for that pair.

   This method is affected by collision filtering; element pairs that have
   been filtered will not produce contacts, even if their collision geometry is
   penetrating.

   The output vector will *not* be cleared.  Contact information will merely be
   added to the vector.

   @param[out]  contacts    All contacts will be aggregated in this structure.
   @returns True if the operation was successful.
   */
  bool ComputeContact(std::vector<Contact>& contacts) const;

  //@}

  //----------------------------------------------------------------------------
  /** @name                  Raycast Queries

   These queries perform raycast queries.  Raycast queries report what, if
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
  bool CastRays(const Eigen::Matrix3Xd& origin,
                const Eigen::Matrix3Xd& ray_endpoint,
                Eigen::VectorXd* distances, Eigen::Matrix3Xd* normals) const;

  //@}

  // TODO(SeanCurtis-TRI): The list of Model functions not yet explicitly
  //  accounted for:
  //      collisionDetectFromPoints -- distance from points to nearest object
  //      potentialCollisionPoints -- really frigging weird; could include
  //        multiple penetrating points, but also non penetrating points.
  //        a) This is not called outside of tests.
  //        b) This seems to be a *very* bullet-specific method.
  //      collidingPoints -- which points are within distance d of a body
  //      collidingPointsCHeckOnly -- simply query if any point is "near" a body
  //
};
}  // namespace geometry
}  // namespace drake
