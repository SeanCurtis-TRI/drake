#include "drake/geometry/geometry_world.h"

#include "drake/geometry/geometry_channel.h"
#include "drake/geometry/geometry_state.h"

namespace drake {
namespace geometry {

using drake::systems::AbstractValue;
using drake::systems::Context;
using drake::systems::Value;
using std::make_unique;
using std::unique_ptr;
using std::vector;

template <typename T>
unique_ptr<GeometryChannel<T>> GeometryWorld<T>::RequestChannel(
    Context<T>* context) {
  auto& state = context->get_mutable_state()
                    ->template get_mutable_abstract_state<GeometryState<T>>(0);
  ChannelId id = state.RequestChannelId();
  // Can't use make_unique because the constructor is private, friended to
  // GeometryWorld.
  return unique_ptr<GeometryChannel<T>>(new GeometryChannel<T>(id));
}

template <typename T>
GeometryId GeometryWorld<T>::AddAnchoredGeometry(Context<T>* context,
                               unique_ptr<GeometryInstance> geometry,
                               const Isometry3<T>& X_WG) {
  GeometryId id = GeometryId::get_new_id();

  // TODO(SeanCurtis-TRI): Actually do this work.
  return id;
}

template <typename T>
vector<unique_ptr<AbstractValue>> GeometryWorld<T>::AllocateAbstractValues() {
  vector<unique_ptr<AbstractValue>> values;
  values.push_back(make_unique<Value<GeometryState<T>>>());
  return values;
}

template <typename T>
void GeometryWorld<T>::UpdateFrames(Context<T>* context,
                  const FrameKinematicsSet<T>& frame_kinematics) {

}

// Explicitly instantiates on the most common scalar types.
template class GeometryWorld<double>;

}  // namespace geometry
}  // namespace drake
