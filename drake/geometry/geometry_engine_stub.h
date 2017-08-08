#pragma once

#include <memory>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_engine.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace stub_shapes {

// The index type for getting the "owned" index of an engine shape. This is the
// index of the shape in the stub engine's owned_geometries_ vector.
using OwnedIndex = TypeSafeIndex<class StubOwnedTag>;

/** Base class for shapes used by the stub engine. */
template <typename T>
class EngineShape {
 public:
  // NOTE: This is a simple hack for the stub engine and is *not* intended to
  // reflect the design for a real geometry engine.
  /** Specification of shape type. */
  enum Type {
    kUnknown = 0,
    kSphere,
    kHalfSpace,
  };

  explicit EngineShape(Type type, OwnedIndex index)
      : type_(type), index_(index) {}

  virtual ~EngineShape() {}

  /** Updates the geometry to reflect the current pose in the _world_ frame. */
  virtual void Update(const Isometry3<T>& X_WC) = 0;

  Type get_type() const { return type_; }
  OwnedIndex get_index() const { return index_; }
  void set_index(OwnedIndex index) { index_ = index; }

  std::unique_ptr<EngineShape> Clone() const {
    return std::unique_ptr<EngineShape>(DoClone());
  }

 protected:
  virtual EngineShape* DoClone() const = 0;

 private:
  Type type_{kUnknown};
  // The index into the owned array for this shape.
  OwnedIndex index_;
};

}  // namespace stub_shapes

/** A stub geometry engine that operates only on spheres. This will be my short-
 term solution for getting the GeometryWorld _infrastructure_ up and running
 independent of the underlying engine details.

 @tparam T The underlying scalar type. Must be a valid Eigen scalar. */
template <typename T>
class GeometryEngineStub : public GeometryEngine<T>, public ShapeReifier {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GeometryEngineStub)

  GeometryEngineStub();

  // Geometry management methods

  int get_update_input_size() const override {
    return static_cast<int>(geometries_.size());
  }

  GeometryIndex AddDynamicGeometry(const Shape& shape) override;

  AnchoredGeometryIndex AddAnchoredGeometry(const Shape& shape,
                                            Isometry3<T> X_WG) override;

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
      const std::vector<GeometryIndexPair>& pairs,
      std::vector<NearestPair<T>>* near_points) const override;

  bool FindClosestGeometry(
      const std::vector<GeometryId>& ids,
      const Eigen::Matrix3Xd& points,
      std::vector<PointProximity<T>>* near_bodies) const override;

  std::vector<PenetrationAsPointPair<T>> ComputePenetration(
      const std::vector<GeometryId>& dynamic_map,
      const std::vector<GeometryId>& anchored_map) const override;

  // ShapeReifier interface implementation.
  void implementGeometry(const Sphere& sphere) override;
  void implementGeometry(const HalfSpace& half_space) override;

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

  // The set of all owned geometries. It should be an invariant that
  // geometries_.size() + anchored_geometries_.size() =
  //                                                   owned_geometries_.size().
  std::vector<copyable_unique_ptr<stub_shapes::EngineShape<T>>>
      owned_geometries_;
  // The subset of owned_geometries_ which represent dynamic geometries.
  std::vector<stub_shapes::EngineShape<T>*> geometries_;
  // The subset of owned_geometries_ which represent anchored geometries.
  std::vector<stub_shapes::EngineShape<T>*> anchored_geometries_;
  // The world poses for the geometries. It should be an invariant that
  // geometries_.size() == X_WG_.size().
  std::vector<Isometry3<T>> X_WG_;
};
}  // namespace geometry
}  // namespace drake
