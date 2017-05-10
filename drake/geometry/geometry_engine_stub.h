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

  int get_update_input_size() const override {
    return static_cast<int>(geometries_.size());
  }

  GeometryIndex AddDynamicGeometry(std::unique_ptr<Shape> shape) override;

  GeometryIndex AddAnchoredGeometry(
      std::unique_ptr<GeometryInstance<T>> geometry) override;

  void RemoveGeometry(GeometryIndex index) override;

  void UpdateWorldPoses(const std::vector<Isometry3<T>>& X_WP) override;

 protected:
  /** NVI implementation for cloning GeometryEngine instances.
   @return A _raw_ pointers to the newly cloned GeometryEngine instance.
   */
  GeometryEngineStub* DoClone() const override {
    return new GeometryEngineStub(*this);
  }
  /** The geometries owned by this geometry engine. */
  std::vector<copyable_unique_ptr<Shape>> geometries_;
};
}  // namespace geometry
}  // namespace drake
