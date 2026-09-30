#pragma once

#include <memory>

#include <coal/collision_object.h>

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

// Creates an coal::CollisionObject for a given Drake Shape, stamping the
// geometry id into the Coal object's user data (as required by
// the various Callback types) and setting the Coal object's pose to the given
// pose.
std::unique_ptr<coal::CollisionObject> MakeCoalObject(
    const Shape& shape, GeometryId id, bool is_dynamic,
    const math::RigidTransformd& X_WG = math::RigidTransformd::Identity());

}  // namespace internal
}  // namespace geometry
}  // namespace drake
