#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {

/**
 A geometry instance combines a geometry definition (i.e., a shape of some
 sort), a pose (relative to a parent frame), material information, and an
 opaque collection of metadata.

 @tparam T The underlying scalar type. Must be a valid Eigen scalar. */
template <typename T>
class GeometryInstance {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GeometryInstance)

  /** Constructor.
   @param X_PG   The pose of this geometry (`G`) in its parent's frame (`P`). */
  explicit GeometryInstance(const Isometry3<T>& X_PG);

  /** Returns the pose of the instance relative to its parent _frame_. */
  const Isometry3<T>& get_pose() const { return X_FG_; }

  /** Sets the pose relative to the parent _frame_ `X_PG` for this instance. */
  void set_pose(const Isometry3<T>& X_PG) { X_FG_ = X_PG; }

 private:
  // The pose of the geometry relative to the source frame it ultimately hangs
  // from.
  Isometry3<T> X_FG_;
};
}  // namespace geometry
}  // namespace drake
