#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/geometry/geometry_context.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"

namespace drake {
namespace geometry {

template <typename T>
class SceneGraph;

/** The %QueryObject serves as a mechanism to perform geometry queries on the
 world's geometry. The SceneGraph has an abstract-valued port that contains
 a  %QueryObject (i.e., a %QueryObject-valued output port).

 To perform geometry queries on SceneGraph:
   - a LeafSystem must have a %QueryObject-valued input port and connect it to
     the corresponding query output port on SceneGraph,
   - the querying LeafSystem can evaluate the input port, retrieving a `const
     QueryObject&` in return, and, finally,
   - invoke the appropriate method on the %QueryObject.

 The const reference returned by the input port is considered "live" - it is
 linked to the context, system, and cache (making full use of all of those
 mechanisms). This const reference should _never_ be persisted; doing so can
 lead to erroneous query results. It is simpler and more advisable to acquire it
 for evaluation in a limited scope (e.g., CalcTimeDerivatives()) and then
 discard it. If a %QueryObject is needed for many separate functions in a
 LeafSystem, each should re-evaluate the input port. The underlying caching
 mechanism should make the cost of this negligible.

 In addition to not persisting the reference from the output port, the
 %QueryObject shouldn't be copied. Strictly speaking, it is an allowed
 operation, but the result is not live, and any geometry query performed on the
 copy will throw an exception.

 A %QueryObject _cannot_ be converted to a different scalar type. A %QueryObject
 of scalar type S can only be acquired from the output port of a SceneGraph
 of type S evaluated on a corresponding GeometryContext, also of type S.

 @tparam T The scalar type. Must be a valid Eigen scalar.

 Instantiated templates for the following kinds of T's are provided:
 - double
 - AutoDiffXd

 They are already available to link against in the containing library.
 No other values for T are currently supported.  */
template <typename T>
class QueryObject {
 public:
  // NOTE: The copy semantics are provided to be compatible with AbstractValue.
  // The result will always be a "default" QueryObject (i.e., all pointers are
  // null). There is no public constructor, the assumption is that the only way
  // to acquire a reference/instance of QueryObject is through the
  // SceneGraph output port. The SceneGraph is responsible for
  // guaranteeing the returned QueryObject is "live" (via CalcQueryObject()).
  QueryObject(const QueryObject& other);
  QueryObject& operator=(const QueryObject&);
  // NOTE: The move semantics are implicitly deleted by the copy semantics.
  // There is no sense in "moving" a query object.

  // Note to developers on adding queries:
  //  All queries should call ThrowIfDefault() before taking any action.
  //  Furthermore, an invocation of that query method should be included in
  //  query_object_test.cc in the DefaultQueryThrows test to confirm that the
  //  query *is* calling ThrowIfDefault().

  //----------------------------------------------------------------------------
  /** @name                State queries */
  //@{

  /** Reports the name for the given source id.
   @throws  std::runtime_error if the %QueryObject is in default configuration.
   @throws  std::logic_error if the identifier is invalid. */
  const std::string& GetSourceName(SourceId id) const;

  /** Reports the id of the frame to which the given geometry id is registered.
   @throws  std::runtime_error if the %QueryObject is in default configuration.
   @throws  std::logic_error if the geometry id is invalid. */
  FrameId GetFrameId(GeometryId geometry_id) const;

  //@}

  //----------------------------------------------------------------------------
  /** @name                Collision Queries

   These queries detect _collisions_ between geometry. Two geometries collide
   if they overlap each other and are not explicitly excluded through
   @ref collision_filter_concepts "collision filtering". These algorithms find
   those colliding cases, characterize them, and report the essential
   characteristics of that collision.  */
  //@{

  /** Computes the penetrations across all pairs of geometries in the world.
   Only reports results for _penetrating_ geometries; if two geometries are
   separated, there will be no result for that pair. Pairs of _anchored_
   geometry are also not reported. The penetration between two geometries is
   characterized as a point pair (see PenetrationAsPointPair).

   <!--
   This method is affected by collision filtering; element pairs that
   have been filtered will not produce contacts, even if their collision
   geometry is penetrating.
   TODO(SeanCurtis-TRI): This isn't true yet.

   NOTE: This is currently declared as double because we haven't exposed FCL's
   templated functionality yet. When that happens, double -> T.
   -->

   @returns A vector populated with all detected penetrations characterized as
            point pairs. */
  std::vector<PenetrationAsPointPair<double>> ComputePointPairPenetration()
      const;

  //@}

#if 0
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

   @param context         The geometry context to query against.
   @param near_points     A vector containing `O(N²)` pairs, where there are `N`
                          geometries in the world.
   @returns True if the operation was successful. */
  bool ComputePairwiseClosestPoints(
      const GeometryContext<T>& context,
      std::vector<NearestPair<T>>* near_points) const;

  // NOTE: This maps to Model::closestPointsAllToAll().
  /** Computes the pair-wise nearest points for all geometries in the given set.

   The output vector will *not* be cleared. Contact information will merely be
   added to the vector.

   @param context         The geometry context to query against.
   @param ids_to_check    A vector of `N` geometry ids for which the pair-wise
                          points are computed.
   @param near_points     A vector containing O(N²) pairs.
   @returns True if the operation was successful. */
  bool ComputePairwiseClosestPoints(
      const GeometryContext<T>& context,
      const std::vector<GeometryId>& ids_to_check,
      std::vector<NearestPair<T>>* near_points) const;

  // NOTE: This maps to Model::closestPointsPairwise().
  /** Computes the pair-wise nearest points for the explicitly indicated pairs
   of geometries.

   The output vector will *not* be cleared. Contact information will merely be
   added to the vector.

   @param context      The geometry context to query against.
   @param pairs        A vector of `N` geometry pairs. The closest points for
                       each pair will be computed.
   @param near_points  A vector of `N` NearestPair values will be added to the
                       vector, one for each input pair.
   @returns True if the operation was successful. */
  bool ComputePairwiseClosestPoints(
      const GeometryContext<T>& context,
      const std::vector<GeometryPair>& pairs,
      std::vector<NearestPair<T>>* near_points) const;
  // TODO(SeanCurtis-TRI): Add a version that takes *frame* pairs.

  // NOTE: This maps to Model::collisionDetectFromPoints().
  /** Determines the nearest body/element to a point for a set of points.

   @param context      The geometry context to query against.
   @param points       An ordered list of `N` points represented column-wise by
                       a `3 x N` Matrix.
   @param near_bodies  A vector of `N` PointProximity instances such that the
                       iᵗʰ instance reports the nearest body/element to the iᵗʰ
                       point.
   @returns True if the operation was successful. */
  bool FindClosestGeometry(
      const GeometryContext<T>& context,
      const Eigen::Matrix3Xd &points,
      std::vector<PointProximity<T>> *near_bodies) const;

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
   @returns True if the operation was successful.  */
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
   @returns True if the operation was successful.   */
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
  // SceneGraph is the only class that can instantiate QueryObjects.
  friend class SceneGraph<T>;
  // Convenience class for testing.
  friend class QueryObjectTester;

  // Only the SceneGraph<T> can instantiate this class - it gets
  // instantiated into a *copyable* default instance (to facilitate allocation
  // in contexts).
  QueryObject() = default;

  void ThrowIfDefault() const {
    if (!(context_ && scene_graph_)) {
      throw std::runtime_error(
          "Attempting to perform query on invalid QueryObject. "
          "Did you copy the QueryObject?");
    }
  }

  // TODO(SeanCurtis-TRI): Consider an alternate formulation. This stores
  // pointers to context and systems which raise some red flags in order to
  // enable other systems to evaluate queries without having a copy of the
  // geometry system or its context.
  //
  // Alternatively, this could store the *index* of the system in its parent
  // diagram. The context shares the same index in the parent diagram context.
  // Then the LeafSystem desiring to perform a query would pass itself and its
  // own context in (along with the query parameters). The QueryObject would
  // use those and the index to get the SceneGraph and GeometryContext.
  //
  // Several issues:
  //  1. Leads to a clunky API (passing self and context into *every* query).
  //  2. The index value would be insufficient if the SceneGraph were buried
  //     in a diagram with its query object port exported in the diagram.
  // This is documented for future consideration, and should not necessarily be
  // interpreted as a guaranteed task.

  // The contents of the "live" query object. It has pointers to the system and
  // context from which it spawned. It uses these to compute geometry queries
  // on the current context (fully-dependent on context). These pointers must
  // be null for "baked" contexts (e.g., the result of copying a "live"
  // context).
  const GeometryContext<T>* context_{nullptr};
  const SceneGraph<T>* scene_graph_{nullptr};
};

}  // namespace geometry
}  // namespace drake
