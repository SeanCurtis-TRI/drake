#pragma once

#include <memory>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/geometry/geometry_engine.h"
#include "drake/geometry/geometry_instance.h"

namespace drake {
namespace geometry {

/** A stub geometry engine that operates only on spheres. This will be my short-
 term solution for getting the GeometryWorld _infrastructure_ up and running
 independent of the underlying engine details.

 @tparam T The underlying scalar type. Must be a valid Eigen scalar. */
template <typename T>
class GeometryEngineStub : public GeometryEngine<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GeometryEngineStub)

  GeometryEngineStub();

  // Geometry management methods

  int get_update_input_size() const override {
    return static_cast<int>(geometries_.size());
  }

  GeometryIndex AddDynamicGeometry(std::unique_ptr<Shape> shape) override;

  GeometryIndex AddAnchoredGeometry(
      std::unique_ptr<GeometryInstance<T>> geometry) override;

  optional<GeometryIndex> RemoveGeometry(GeometryIndex index) override;

  void UpdateWorldPoses(const std::vector<Isometry3<T>>& X_WP) override;

  // Proximity query methods
  bool ComputePairwiseClosestPoints(
      const std::vector<GeometryId>& ids,
      std::vector<NearestPair<T>>* near_points) const override;
  bool ComputePairwiseClosestPoints(
      const std::vector<GeometryId>& ids,
      const std::vector<GeometryIndex>& ids_to_check,
      std::vector<NearestPair<T>>* near_points) const override;
  bool ComputePairwiseClosestPoints(
      const std::vector<GeometryId>& ids,
      const std::vector<internal::GeometryIndexPair>& pairs,
      std::vector<NearestPair<T>>* near_points) const override;

  bool FindClosestGeometry(
      const std::vector<GeometryId>& ids,
      const Eigen::Matrix3Xd& points,
      std::vector<PointProximity<T>>* near_bodies) const override;

 protected:
  // NVI implementation for cloning GeometryEngine instances.
  // @return A _raw_ pointers to the newly cloned GeometryEngine instance.
  GeometryEngineStub* DoClone() const override {
    return new GeometryEngineStub(*this);
  }

 private:
  // The underlying method for executing
  template <class PairSet>
  bool ComputePairwiseClosestPointsHelper(
      const std::vector<GeometryId>& ids,
      const PairSet& pair_set,
      std::vector<NearestPair<T>>* near_points) const;

  // The geometries owned by this geometry engine.
  std::vector<copyable_unique_ptr<Shape>> geometries_;
  // The world poses for the geometries. It should be an invariant that
  // geometries_.size() == X_WG_.size().
  std::vector<Isometry3<T>> X_WG_;
};
}  // namespace geometry
}  // namespace drake
