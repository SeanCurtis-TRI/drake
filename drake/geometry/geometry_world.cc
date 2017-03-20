#include "drake/geometry/geometry_world.h"

#include

namespace drake {
namespace geometry {

using drake::systems::AbstractValue;
using drake::systems::Context;
using drake::systems::Value;
using std::make_unique;
using std::unique_ptr;
using std::vector;

template <typename T>
unique_ptr<GeometryChannel<T>> GeometryWorld<T>::RequestChannel(Context* context) {
  // TODO(SeanCurtis-TRI): Actually draw this from the context, and update the
  // context to know about the new channel
  static int NEXT_CHANNEL = 0;
  return make_unique<GeometryChannel<T>>(NEXT_CHANNEL++);
}

template <typename T>
GeometryId GeometryWorld<T>::AddAnchoredGeometry(Context<T>* context,
                               unique_ptr<GeometryInstance> geometry,
                               const Isometry3<T>& X_WG) {
  // TODO(SeanCurtis-TRI): Actually do this work.
  static int NEXT_GEOMETRY = 1;
  return GeometryId(NEXT_GEOMETRY++);
}

template <typename T>
vector<unique_ptr<AbstractValue>> GeometryWorld<T>::AllocateAbstractValues() {
  vector<unique_ptr<AbstractValue>> values;
  values.push_back(make_unique<Value<GeometryState>>());
  return values;
}

template <typename T>
void GeometryWorld<T>::UpdateFrames(Context<T>* context,
                  const FrameKinematicsSet<T>& frame_kinematics) {

}

}  // namespace geometry
}  // namespace drake
