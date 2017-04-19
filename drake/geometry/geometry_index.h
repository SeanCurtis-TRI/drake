#pragma once

#include "drake/common/type_safe_index.h"

namespace drake {
namespace geometry {

/// Type used to locate the index of a geometry in the geometry engine.
using GeometryIndex = TypeSafeIndex<class GeometryTag>;

}  // namespace geometry
}  // namespace drake
