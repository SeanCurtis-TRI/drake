#pragma once

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_index.h"

namespace drake {
namespace geometry {

/** An ordered pair of geometries -- given by identifier. */
struct GeometryPair {
  GeometryPair(GeometryId idA, GeometryId idB)
      : geometry_a(idA), geometry_b(idB) {}
  GeometryId geometry_a;
  GeometryId geometry_b;
};

/** An ordered pair of frames -- given by identifier. */
struct FramePair {
  FrameId frame_a;
  FrameId frame_b;
};

}  // namespace geometry
}  // namespace drake
