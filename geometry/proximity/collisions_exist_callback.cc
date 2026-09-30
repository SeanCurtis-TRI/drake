#include "drake/geometry/proximity/collisions_exist_callback.h"

#include <coal/collision.h>

#include "drake/geometry/proximity/proximity_utilities.h"

namespace drake {
namespace geometry {
namespace internal {
namespace has_collisions {

CallbackData::CallbackData(const CollisionFilter* collision_filter_in)
    : collision_filter(*collision_filter_in) {
  DRAKE_DEMAND(collision_filter_in != nullptr);
  request.num_max_contacts = 1;
  request.enable_contact = false;
  // This is the tolerance Drake has historically asked of the GJK solver.
  // Note that we deliberately leave `epa_tolerance` at Coal's default.
  request.gjk_tolerance = 2e-12;
  // Coal defaults this to 1e-12, which would report pairs that are separated
  // by a positive distance as colliding. Drake wants strict penetration, so we
  // ask for it explicitly.
  request.collision_distance_threshold = 0;
}

bool Callback(coal::CollisionObject* object_A_ptr,
              coal::CollisionObject* object_B_ptr, void* callback_data) {
  auto& data = *static_cast<CallbackData*>(callback_data);

  const EncodedData encoding_a(*object_A_ptr);
  const EncodedData encoding_b(*object_B_ptr);

  const bool can_collide =
      data.collision_filter.CanCollideWith(encoding_a.id(), encoding_b.id());
  if (!can_collide) return false;

  // Unpack the callback data.
  const coal::CollisionRequest& request = data.request;

  // This callback only works for a single contact, this confirms a request
  // hasn't been made for more contacts.
  DRAKE_ASSERT(request.num_max_contacts == 1);
  coal::CollisionResult result;

  // Perform nearphase collision detection.
  coal::collide(object_A_ptr, object_B_ptr, request, result);

  data.collisions_exist = result.isCollision();
  return data.collisions_exist;
}

}  // namespace has_collisions
}  // namespace internal
}  // namespace geometry
}  // namespace drake
