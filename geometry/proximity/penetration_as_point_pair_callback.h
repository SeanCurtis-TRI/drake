#pragma once

#include <optional>
#include <unordered_map>
#include <vector>

#include <coal/collision_data.h>
#include <coal/collision_object.h>

#include "drake/common/drake_export.h"
#include "drake/geometry/proximity/collision_filter.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {
namespace penetration_as_point_pair DRAKE_NO_EXPORT {

/* Supporting data for the detecting collision between geometries and reporting
 them as a pair of points (see PenetrationAsPointPair). It includes:

    - A collision filter instance. Aliased.
    - A Coal collision request. Aliased.
    - The poses. Aliased.
    - A vector of point pairs -- one instance of PenetrationAsPointPair for
      every supported, unfiltered penetrating pair. Aliased. */
template <typename T>
struct CallbackData {
  CallbackData(
      const CollisionFilter* collision_filter_in,
      const std::unordered_map<GeometryId, math::RigidTransform<T>>* X_WGs_in,
      std::vector<PenetrationAsPointPair<T>>* point_pairs_in)
      : collision_filter(*collision_filter_in),
        X_WGs(*X_WGs_in),
        point_pairs(*point_pairs_in) {
    DRAKE_DEMAND(collision_filter_in != nullptr);
    DRAKE_DEMAND(X_WGs_in != nullptr);
    DRAKE_DEMAND(point_pairs_in != nullptr);
    request.num_max_contacts = 1;
    request.enable_contact = true;
    // This is the tolerance Drake has historically asked of the GJK solver.
    // Note that we deliberately leave `epa_tolerance` at Coal's default; see
    // the note on penetration depth in CalcDistanceFallback().
    request.gjk_tolerance = 2e-12;
    // Coal defaults this to 1e-12, which would report pairs that are
    // separated by a positive distance as colliding. Drake wants strict
    // penetration, so we ask for it explicitly.
    request.collision_distance_threshold = 0;
  }

  /* The collision filter system.  */
  const CollisionFilter& collision_filter;

  /* The parameters for the Coal object-object collision function.  */
  coal::CollisionRequest request;

  /** The pose of each geometry in the scene. */
  const std::unordered_map<GeometryId, math::RigidTransform<T>>& X_WGs;

  /* The results of the collision query.  */
  std::vector<PenetrationAsPointPair<T>>& point_pairs;
};

/* Callback function for Coal's collide() function for retrieving a *single*
 contact. As documented by QueryObject::ComputePointPairPenetration(), the
 result added to the output data is the same, regardless of the order of
 the two Coal objects.  */
template <typename T>
bool Callback(coal::CollisionObject* object_A_ptr,
              coal::CollisionObject* object_B_ptr, void* callback_data);

/* Given two objects that are candidates for a collision, returns the
 point-pair contact result. If the penetration depth turns out to be negative
 (no collision), returns nullopt. */
template <typename T>
std::optional<PenetrationAsPointPair<T>> MaybeMakePointPair(
    coal::CollisionObject* object_A_ptr, coal::CollisionObject* object_B_ptr,
    const CallbackData<T>& data);

// clang-format off
}  // namespace penetration_as_point_pair
// clang-format on
}  // namespace internal
}  // namespace geometry
}  // namespace drake
