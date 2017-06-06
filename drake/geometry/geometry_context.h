#pragma once

#include "drake/geometry/geometry_state.h"
#include "drake/systems/framework/leaf_context.h"

namespace drake {
namespace geometry {

/** The custom leaf context type for GeometrySystem and GeometryWorld. */
template <typename T>
class GeometryContext : public drake::systems::LeafContext<T> {
 public:
  /** Returns a mutable reference of the underlying geometry state. */
  GeometryState<T>& get_mutable_geometry_state();

  /** Returns a const reference of the underlying geometry state. */
  const GeometryState<T>& get_geometry_state() const;
};

}  // namespace geometry
}  // namespace drake
