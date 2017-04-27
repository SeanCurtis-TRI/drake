#pragma once

#include "drake/common/type_safe_index.h"

namespace drake {
namespace geometry {

/// Type used to locate the index of a geometry in the geometry engine.
using GeometryIndex = TypeSafeIndex<class GeometryTag>;

/// Type used to locate the index of a frame pose in the geometry engine.
using PoseIndex = TypeSafeIndex<class GeometryPoseTag>;

}  // namespace geometry
}  // namespace drake
