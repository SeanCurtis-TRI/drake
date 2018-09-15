#pragma once

#include "drake/common/type_safe_index.h"

namespace drake {
namespace geometry {

/** Type used to locate any geometry in the render engine. */
using RenderIndex = TypeSafeIndex<class RenderTag>;

/** Index used to identify a geometry in the proximity engine. The same index
 type applies to both anchored and dynamic geometries. They are distinguished
 by which method they are passed to.  */
using ProximityIndex = TypeSafeIndex<class ProximityTag>;

/** Index into the global set of internal entities -- spanning all roles. The
 index can refer to either a dynamic geometry, an anchored geometry, or a frame
 -- the context determines which is valid.  In other words, the first
 InternalGeometry, the first InternalAnchoredGeometry, and the first
 InternalFrame will all have the same InternalIndex.  */
using InternalIndex = TypeSafeIndex<class InternalGeometryTag>;

}  // namespace geometry
}  // namespace drake
