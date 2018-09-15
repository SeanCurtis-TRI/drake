#pragma once

#include "drake/common/type_safe_index.h"

namespace drake {
namespace geometry {

/** Type used to locate any geometry in the render engine. */
using RenderIndex = TypeSafeIndex<class RenderTag>;

/** Index into the dynamic geometries with a proximity role.  */
using DynamicProximityIndex = TypeSafeIndex<class DynamicProximityTag>;

/** Index into the anchored geometries with a proximity role.  */
using AnchoredProximityIndex = TypeSafeIndex<class DynamicAnchoredTag>;
/** Index into the global set of internal entities -- spanning all roles. The
 index can refer to either a dynamic geometry, an anchored geometry, or a frame
 -- the context determines which is valid.  In other words, the first
 InternalGeometry, the first InternalAnchoredGeometry, and the first
 InternalFrame will all have the same InternalIndex.  */
using InternalIndex = TypeSafeIndex<class InternalGeometryTag>;

}  // namespace geometry
}  // namespace drake
