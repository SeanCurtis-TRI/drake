#pragma once

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_index.h"

namespace drake {
namespace geometry {

// These structs are for internal use only and are *not* part of the public
// API.
namespace internal {
struct GeometryIndexPair {
  GeometryIndexPair(GeometryIndex i1, GeometryIndex i2) : index1(i1), index2(i2) {}
  GeometryIndex index1;
  GeometryIndex index2;
};
}  // namespace internal

}  // namespace geometry
}  // namespace drake
