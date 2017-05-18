#pragma once

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "drake/common/drake_optional.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_index.h"
#include "drake/geometry/geometry_query_results.h"
#include "drake/geometry/geometry_query_inputs.h"
#include "drake/geometry/shapes.h"

namespace drake {
namespace geometry {

// These structs are for internal use only and are *not* part of the public
// API.
namespace internal {
struct GeometryIndexPair {
  GeometryIndexPair(GeometryIndex i1, GeometryIndex i2)
      : index1(i1), index2(i2) {}
  GeometryIndex index1;
  GeometryIndex index2;
};
}  // namespace internal

// forward declaration
class Geometry;
template <typename T> class GeometryInstance;


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
      const std::vector<internal::GeometryIndexPair>& pairs,
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
#endif
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
