#pragma once

#include "drake/common/drake_copyable.h"
namespace drake {
namespace geometry {

/**
 A geometry instance combines a geometry definition (i.e., a shape of some
 sort), a pose (relative to a parent frame), material information, and an
 opaque collection of metadata.
 */
class GeometryInstance {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GeometryInstance)

  GeometryInstance() = default;
};
}  // namespace geometry
}  // namespace drake
