#pragma once

#include <memory>
#include <utility>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_material.h"
#include "drake/geometry/shapes.h"

namespace drake {
namespace geometry {

/**
 A geometry instance combines a geometry definition (i.e., a shape of some
 sort), a pose (relative to a parent frame), material information, and an
 opaque collection of metadata. */
// TODO(SeanCurtis-TRI): Determine what the right templatization of this is.
// The pose in the parent frame is a constant. However, the materials it is
// given may not be. So, move the template to the methods that use/get
// material types?
class GeometryInstance {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GeometryInstance)

  /** Constructor.
   @param X_PG   The pose of this geometry (`G`) in its parent's frame (`P`).
   @param shape  The underlying shape for this geometry instance. */
  GeometryInstance(const Isometry3<double>& X_PG, std::unique_ptr<Shape> shape);

  /** Constructor.
   @param X_PG   The pose of this geometry (`G`) in its parent's frame (`P`).
   @param shape  The underlying shape for this geometry instance.
   @param vis_material The visual material to apply to this geometry. */
  GeometryInstance(const Isometry3<double>& X_PG, std::unique_ptr<Shape> shape,
                   const VisualMaterial& vis_material);

  /** Returns the pose of the instance relative to its parent _frame_. */
  const Isometry3<double>& get_pose() const { return X_FG_; }

  /** Sets the pose relative to the parent _frame_ `X_PG` for this instance. */
  void set_pose(const Isometry3<double>& X_PG) { X_FG_ = X_PG; }

  void set_visual_material(const VisualMaterial& material) {
    visual_material_ = material;
  }
  const VisualMaterial& get_visual_material() const { return visual_material_; }

  /** Releases the shape from the instance. */
  std::unique_ptr<Shape> release_shape() { return std::move(shape_); }

 private:
  // The pose of the geometry relative to the source frame it ultimately hangs
  // from.
  Isometry3<double> X_FG_;

  // The shape associated with this instance.
  copyable_unique_ptr<Shape> shape_;

  // The "rendering" material -- e.g., OpenGl contexts and the like.
  VisualMaterial visual_material_;
};
}  // namespace geometry
}  // namespace drake
