#pragma once

#include <memory>
#include <vector>

#include "drake/common/autodiff.h"
#include "drake/common/drake_optional.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_index.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {

template <typename T> class GeometryState;

namespace internal {

// TODO(SeanCurtis-TRI): Swap Isometry3 for the new Transform class.

// TODO(SeanCurtis-TRI): Re-introduce this with the distance queries.
#if 0
/** Specification of a pair of geometry indices. Serves as part of the query
 interface to allow queries on explicitly itemized pairs of geometries. */
struct GeometryIndexPair {
  GeometryIndexPair(GeometryIndex i1, GeometryIndex i2)
      : index1(i1), index2(i2) {}
  GeometryIndex index1;
  GeometryIndex index2;
};
#endif

/** The underlying engine for performing geometric _proximity_ queries.
 It owns the geometry instances and, once it has been provided with the poses
 of the geometry, it provides geometric queries on that geometry.

 Proximity queries span a range of types, including:

   - penetration
   - distance
   - ray-intersection

 @tparam T The scalar type. Must be a valid Eigen scalar.

 Instantiated templates for the following kinds of T's are provided:
 - double
 - AutoDiffXd

 They are already available to link against in the containing library.
 No other values for T are currently supported.

 @internal Historically, this replaces the DrakeCollision::Model class.  */
template <typename T>
class ProximityEngine {
 public:
  ProximityEngine();
  ~ProximityEngine();

  /** Construct a deep copy of the provided `other` engine. */
  ProximityEngine(const ProximityEngine& other);

  /** Set `this` engine to be a deep copy of the `other` engine. */
  ProximityEngine& operator=(const ProximityEngine& other);

  /** Construct an engine by moving the data of a source engine. The source
   engine will be returned to its default-initialized state. */
  ProximityEngine(ProximityEngine&& other) noexcept;

  /** Move assign a source engine to this engine. The source
   engine will be returned to its default-initialized state. */
  ProximityEngine& operator=(ProximityEngine&& other) noexcept;

  /** Returns an independent copy of this engine templated on the AutoDiffXd
   scalar type. If the engine is already an AutoDiffXd engine, it is equivalent
   to using the copy constructor to create a duplicate on the heap. */
  std::unique_ptr<ProximityEngine<AutoDiffXd>> ToAutoDiffXd() const;

  /** @name Topology management */
  //@{

  /** Adds the given `shape` to the engine's dynamic geometry.  */
  GeometryIndex AddDynamicGeometry(const Shape& shape);

  /** Adds the given `shape` to the engine's anchored geometry at the fixed
   pose given by `X_WG` (in the world frame W).  */
  AnchoredGeometryIndex AddAnchoredGeometry(const Shape& shape,
                                            const Isometry3<double>& X_WG);

  /** Reports the _total_ number of geometries in the engine -- dynamic and
   anchored (spanning all sources).  */
  int num_geometries() const;

  /** Reports the number of _dynamic_ geometries (spanning all sources). */
  int num_dynamic() const;

  /** Reports the number of _anchored_ geometries (spanning all sources). */
  int num_anchored() const;

  //@}

  /** Updates the poses for all of the dynamic geometries in the engine. It
   is an invariant that _every_ registered dynamic geometry, across _all_
   geometry sources, has a _unique_ index that lies in the range
   [0, num_dynamic() - 1]. Therefore, `X_WG` should have size equal to
   num_dynamics() and any other length will cause program failure. The iᵗʰ entry
   contains the pose for the geometry whose GeometryIndex value is `i`.
   @param X_WG     The poses of each geometry `G` measured and expressed in the
                   world frame `W`. */
  // TODO(SeanCurtis-TRI): I could do things here differently a number of ways:
  //  1. I could make this move semantics (or swap semantics).
  //  2. I could simply have a method that returns a mutable reference to such
  //    a vector and the caller sets values there directly.
  void UpdateWorldPoses(const std::vector<Isometry3<T>>& X_WG);

#if 0
  /** @name                   Proximity Queries

   These queries represent _proximity_ queries -- queries to determine what is
   near by, or what is closest. This is not about overlapping/penetration --
   the proximity of overlapping/penetrating objects should be zero.

   These queries are _not_ affected by collision filtering. */

  //@{

  /** Computes the pair-wise nearest points for all elements in the world.

   The output vector will *not* be cleared. Proximity information will merely be
   added to the vector.

   @param[in]   ids             A map from geometry _index_ to the corresponding
                                global geometry identifier.
   @param[out]  near_points     A vector containing `O(N²)` pairs, where there
                                are `N` elements in the world.
   @returns True if the operation was successful. */
  virtual bool ComputePairwiseClosestPoints(
      const std::vector<GeometryId>& ids,
      std::vector<NearestPair<T>>* near_points) const = 0;

  // NOTE: This maps to Model::closestPointsAllToAll().
  /** Computes the pair-wise nearest points for all elements in the given set.

   The output vector will *not* be cleared. Contact information will merely be
   added to the vector.

   @param[in]   ids             A map from geometry _index_ to the corresponding
                                global geometry identifier.
   @param[in]   ids_to_check    A vector of `N` geometry ids for which the
                                pair-wise points are computed.
   @param[out]  near_points     A vector containing O(N²) pairs.
   @returns True if the operation was successful. */
  virtual bool ComputePairwiseClosestPoints(
      const std::vector<GeometryId>& ids,
      const std::vector<GeometryIndex>& ids_to_check,
      std::vector<NearestPair<T>>* near_points) const = 0;

  // NOTE: This maps to Model::closestPointsPairwise().
  /** Computes the pair-wise nearest points for the explicitly indicated pairs of
   bodies.

   The output vector will *not* be cleared. Contact information will merely be
   added to the vector.

   @param[in]   ids         A map from geometry _index_ to the corresponding
                            global geometry identifier.
   @param[in]  pairs        A vector of `N` body pairs. The closest points for
                            each pair will be computed.
   @param[out] near_points  A vector of `N` NearestPair values will be added
                            to the vector, one for each input pair.
   @returns True if the operation was successful. */
  virtual bool ComputePairwiseClosestPoints(
      const std::vector<GeometryId>& ids,
      const std::vector<GeometryIndexPair>& pairs,
      std::vector<NearestPair<T>>* near_points) const = 0;

  // NOTE: This maps to Model::collisionDetectFromPoints().
  /** Determines the nearest body/element to a point for a set of points. This
   only considers *convex* geometry.

   @param[in]   ids           A map from geometry _index_ to the corresponding
                              global geometry identifier.
   @param[in]   points        An ordered list of `N` points represented
                              column-wise by a `3 x N` Matrix.
   @param[out]  near_bodies   A vector of `N` PointProximity instances such that
                              the iᵗʰ instance reports the nearest body/element
                              to the iᵗʰ point. The vector is assumed to be
                              empty already.
   @returns True if the operation was successful. */
  virtual bool FindClosestGeometry(
      const std::vector<GeometryId>& ids,
      const Eigen::Matrix3Xd& points,
      std::vector<PointProximity<T>>* near_bodies) const = 0;

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
#endif

  //----------------------------------------------------------------------------
  /** @name                Collision Queries

   These queries detect _collisions_ between geometry. Two geometries collide
   if they overlap each other and are not explicitly excluded through
   @ref collision_filter_concepts "collision filtering". These algorithms find
   those colliding cases, characterize them, and report the essential
   characteristics of that collision.  */

  //@{

  // NOTE: This maps to Model::ComputeMaximumDepthCollisionPoints().
  /** Computes the penetrations across all pairs of geometries in the world.
   Only reports results for _penetrating_ geometries; if two geometries are
   separated, there will be no result for that pair.

   The penetrations are characterized by pairs of points (providing some measure
   of the penetration "depth" of the two objects -- but _not_ the overlapping
   volume.

   @cond
   // TODO(SeanCurtis-TRI): Once collision filtering is supported, pull this
   // *out* of the cond tag.
   This method is affected by collision filtering; geometry pairs that
   have been filtered will not produce contacts, even if their collision
   geometry is penetrating.
   @endcond

   @param[in]   dynamic_map   A map from geometry _index_ to the corresponding
                              global geometry identifier for dynamic geometries.
   @param[in]   anchored_map  A map from geometry _index_ to the corresponding
                              global geometry identifier for anchored
                              geometries.
   @returns A vector populated with all detected penetrations characterized as
            point pairs. */
  std::vector<PenetrationAsPointPair<double>> ComputePointPairPenetration(
      const std::vector<GeometryId>& dynamic_map,
      const std::vector<GeometryId>& anchored_map) const;

  //@}

  // TODO(SeanCurtis-TRI): The list of Model functions not yet explicitly
  //  accounted for:
  //      potentialCollisionPoints -- really frigging weird; could include
  //        multiple penetrating points, but also non penetrating points.
  //        a) This is not called outside of tests.
  //        b) This seems to be a *very* bullet-specific method.
  //

 private:
  ////////////////////////////////////////////////////////////////////////////

  // Testing utilities:
  // These functions facilitate *limited* introspection into the engine state.
  // This enables unit tests to make assertions about pre- and post-operation
  // state.

  // Reports true if other is detectably a deep copy of this engine.
  bool IsDeepCopy(const ProximityEngine<T>& other) const;

  // Reports the translation of the dynamic geometry at the given index. The
  // test assumes transformations are correctly handled and merely uses
  // translation to determine correct *data flow*.
  Vector3<double> GetDynamicTranslation(int index) const;

  // Reports the translation of the anchored geometry at the given index. The
  // test assumes transformations are correctly handled and merely uses
  // translation to determine correct *data flow*.
  Vector3<double> GetAnchoredTranslation(int index) const;

  // Reports the index stored in the reified collision object at the given
  // index. It should always return the input value if the data structure is
  // maintained through geometry removal.
  int GetDynamicGeometryIndex(int index) const;

  // Reports the index stored in the reified collision object at the given
  // index. It should always return the input value if the data structure is
  // maintained through geometry removal.
  int GetAnchoredGeometryIndex(int index) const;

  ////////////////////////////////////////////////////////////////////////////

  // TODO(SeanCurtis-TRI): Pimpl + template implementation has proven
  // problematic. This gets around it but it isn't a reliable long-term
  // solution. Figure out how to make this work with unique_ptr or
  // copyable unique_ptr
  //
  // The implementation details.
  class Impl;
  Impl* impl_{};

  // Private constructor to use for scalar conversion.
  explicit ProximityEngine(Impl* impl);

  // Engine on one scalar can see the members of other engines.
  template <typename> friend class ProximityEngine;

  // Facilitate testing.
  friend class ProximityEngineTester;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
