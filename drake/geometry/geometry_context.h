#pragma once

#include "drake/systems/framework/leaf_context.h"
namespace drake {
namespace geometry {

template <typename T>
class GeometryContext : public drake::systems::LeafContext<T> {};

}  // namespace geometry
}  // namespace drake
