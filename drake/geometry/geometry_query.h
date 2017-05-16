#pragma once

#include <unordered_map>
#include <vector>

#include "drake/geometry/internal_geometry.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_query_results.h"

namespace drake {
namespace geometry {

// Forward declarations
template <typename T> class GeometryState;
template <typename T> class GeometryEngine;

/** An ordered pair of geometries -- given by identifier. */
struct GeometryPair {
  GeometryPair(GeometryId idA, GeometryId idB)
      : geometry_a(idA), geometry_b(idB) {}
  GeometryId geometry_a;
  GeometryId geometry_b;
};

/** An ordered pair of frames -- given by identifier. */
struct FramePair {
  FrameId frame_a;
  FrameId frame_b;
};

/** A class containing the geometric state of the world for a single context.

 This class serves as the output of the GeometryWorld and GeometrySystem.
 A valid GeometryQuery instance will support geometric queries on the geometry
 on the world predicated on a given Context. Queries include:

    - Distance between geometries
    - Collisions between geometry
    - Ray-geometry intersection
 */
 template <typename T>
class GeometryQuery {
 public:
  // The query can only be copied from another -- the const references preclude
  // any other behavior.
  GeometryQuery(const GeometryQuery&) = default;
  GeometryQuery& operator=(const GeometryQuery&) = delete;
  GeometryQuery(GeometryQuery&&) = default;
  GeometryQuery& operator=(GeometryQuery&&) = delete;
  // TODO(SeanCurtis-TRI): Determine how to limit the scope of these queries.
  //    e.g., limit it to just those elements belonging to a single input,
  //      limit it to a subset of elements in a single input,
  //      limit it to a subset of elements which span multiple inputs.

  // TODO(SeanCurtis-TRI): Should I return error codes instead of booleans?
  //  Im returning bools because the old Model did, but I have yet to *use* it.

  // NOTE: This is just a taxonomy of the *types* of queries; the interfaces
  // should not be considered complete. Each of these fundamental queries may
  // have different variants depending on the scope.

  //----------------------------------------------------------------------------
  /** @name                   Proximity Queries

   These queries represent _proximity_ queries -- queries to determine what is
   near by, or what is closest. This is not about overlapping/penetration --
   the proximity of overlapping/penetrating objects should be zero.

   These queries are _not_ affected by collision filtering. */

  //@{

  /** Computes the pair-wise nearest points for all geometries in the world.

   The output vector will *not* be cleared. Contact information will merely be
   added to the vector.

   @param[out]  near_points     A vector containing `O(N²)` pairs, where there
                                are `N` geometries in the world.
   @returns True if the operation was successful. */
  bool ComputePairwiseClosestPoints(
      std::vector<NearestPair<T>>* near_points) const;

  // NOTE: This maps to Model::closestPointsAllToAll().
  /** Computes the pair-wise nearest points for all geometries in the given set.

   The output vector will *not* be cleared. Contact information will merely be
   added to the vector.

   @param[in]   ids_to_check    A vector of `N` geometry ids for which the
                                pair-wise points are computed.
   @param[out]  near_points     A vector containing O(N²) pairs.
   @returns True if the operation was successful. */
  bool ComputePairwiseClosestPoints(
      const std::vector<GeometryId>& ids_to_check,
      std::vector<NearestPair<T>>* near_points) const;

  // NOTE: This maps to Model::closestPointsPairwise().
  /** Computes the pair-wise nearest points for the explicitly indicated pairs
   of geomeries.

   The output vector will *not* be cleared. Contact information will merely be
   added to the vector.

   @param[in]  pairs        A vector of `N` geometry pairs. The closest points
                            for each pair will be computed.
   @param[out] near_points  A vector of `N` NearestPair values will be added
                            to the vector, one for each input pair.
   @returns True if the operation was successful. */
  bool ComputePairwiseClosestPoints(
      const std::vector<GeometryPair>& pairs,
      std::vector<NearestPair<T>>* near_points) const;
  // TODO(SeanCurtis-TRI): Add a version that takes *frame* pairs.

  // NOTE: This maps to Model::collisionDetectFromPoints().
  /** Determines the nearest body/element to a point for a set of points.

   @param[in]   points        An ordered list of `N` points represented
                              column-wise by a `3 x N` Matrix.
   @param[out]  near_bodies   A vector of `N` PointProximity instances such that
                              the iᵗʰ instance reports the nearest body/element
                              to the iᵗʰ point.
   @returns True if the operation was successful. */
  bool FindClosestGeometry(const Eigen::Matrix3Xd &points,
                           std::vector<PointProximity<T>> *near_bodies) const;
#if 0
  // NOTE: This maps to Model::collidingPoints().
  /** Determines which of the given list of `points` are no farther than
   `distance` meters from _any_ collision geometry.

   In other words, the index `i` is included in the returned vector of indices
   iff a sphere of radius `distance`, located at `input_points[i]` collides with
   any collision element in the model.

   @param[in]   points        An ordered list of `N` points represented
                              column-wise by a `3 x N` Matrix.
   @param[in]   distance      The maximum distance from a point that is allowed.
   @param[out]  results       A vector of indices into `points`. Each index
                              indicates that the corresponding point is within
                              closer than `distance` meters away from some
                              geometry. The vector will _not_ be cleared and
                              the indexes will be added to the current values.
   @returns True if the operation was successful. */
  bool FindGeometryProximalPoints(const Matrix3X<T>& points, double distance,
                                  std::vector<size_t>* results) const;

  /** Given a vector of `points` in the world coordinate frame, reports if _any_
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
          geometry. */
  bool IsAnyGeometryNear(const Matrix3X<T>& points,
                         double distance) const;

  //@}

  //----------------------------------------------------------------------------
  /** @name                Collision Queries

   These queries detect _collisions_ between geometry. Two geometries collide
   if they overlap each other and are not explicitly excluded through
   @ref collision_filter_concepts "collision filtering". These algorithms find
   those colliding cases, characterize them, and report the essential
   characteristics of that collision.  */

  //@{

  // NOTE: This maps to Model::ComputeMaximumDepthCollisionPoints().
  /** Computes the contact across all elements in the world. Only reports
   results for elements in *contact*; if two elements are separated, there will
   be no result for that pair.

   This method is affected by collision filtering; element pairs that have
   been filtered will not produce contacts, even if their collision geometry is
   penetrating.

   The output vector will *not* be cleared. Contact information will merely be
   added to the vector.

   @param[out]  contacts    All contacts will be aggregated in this structure.
   @returns True if the operation was successful. */
  bool ComputeContact(std::vector<Contact<T>>* contacts) const;

  //@}

  //----------------------------------------------------------------------------
  /** @name                  Ray-casting Queries

   These queries perform ray-cast queries. Ray-cast queries report what, if
   anything, lies in a particular direction from a query point.
   */

  //@{

  // NOTE: This maps to Model::collisionRaycast().
  /** Cast one or more rays against the scene geometry.

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
  bool CastRays(const Matrix3X<T>& origin,
                const Matrix3X<T>& ray_endpoint,
                Eigen::VectorXd* distances, Matrix3X<T>* normals) const;

  //@}

  // TODO(SeanCurtis-TRI): The list of Model functions not yet explicitly
  //  accounted for:
  //      potentialCollisionPoints -- really frigging weird; could include
  //        multiple penetrating points, but also non penetrating points.
  //        a) This is not called outside of tests.
  //        b) This seems to be a *very* bullet-specific method.
  //
#endif
 private:
  friend class GeometryState<T>;

  // This can only be created by a GeometryWorld instance.
  GeometryQuery(
      const GeometryEngine<T>& engine, const std::vector<GeometryId>& ids,
      const std::unordered_map<GeometryId, internal::InternalGeometry>&
          geometries)
      : engine_(engine),
        index_to_id_map_(ids),
        id_to_geometry_map_(geometries) {}

  // The underlying engine that performs all queries.
  const GeometryEngine<T>& engine_;

  // A mapping from GeometryIndex (from the engine) to GeometryIds (in
  // GeometryWorld).
  const std::vector<GeometryId>& index_to_id_map_;

  // A mapping from GeometryId to the underlying geometry.
  const std::unordered_map<GeometryId, internal::InternalGeometry>&
      id_to_geometry_map_;
};
}  // namespace geometry
}  // namespace drake
