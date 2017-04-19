#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {

/**
 A geometry instance combines a geometry definition (i.e., a shape of some
 sort), a pose (relative to a parent frame), material information, and an
 opaque collection of metadata.

 @tparam T The underlying scalar type. Must be a valid Eigen scalar.
 */
template <typename T>
class GeometryInstance {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GeometryInstance)

  /** Constructor.

   @param X_PG   The pose of this geometry (`G`) in its parent's frame (`P`).
   */
  GeometryInstance(const Isometry3<T>& X_PG) : X_FG_(X_PG) {}

 private:
  // TODO(SeanCurtis-TRI): This represents the choice of having instances not
  // know about arbitrary hierarchies of rigid geometries. Determine if this is
  // the right choice.

  // The pose of the geometry relative to the source frame it ultimately hangs
  // from.
  Isometry3<T> X_FG_;

};
}  // namespace geometry
}  // namespace drake
