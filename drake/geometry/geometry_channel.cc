#include "drake/geometry/geometry_channel.h"

#include "drake/common/drake_assert.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_state.h"

namespace drake {
namespace geometry {

using drake::systems::Context;
using std::unique_ptr;

template <typename T>
GeometryChannel<T>::GeometryChannel(ChannelId index) : id_(index),
                                                       is_open_(true) {
}

template <typename T>
GeometryChannel<T>::~GeometryChannel() {
  DRAKE_DEMAND(!is_open_);
}

template <typename T>
void GeometryChannel<T>::Close(Context<T>* context) {
  auto& state = context->get_mutable_state()
      ->template get_mutable_abstract_state<GeometryState<T>>(0);
  state.CloseChannel(id_);
  // TODO(SeanCurtis-TRI): Modify the context to remove this geometry's data.
  is_open_ = false;
}

template <typename T>
FrameId GeometryChannel<T>::DeclareFrame(Context<T>* context) {
  auto& state = context->get_mutable_state()
                    ->template get_mutable_abstract_state<GeometryState<T>>(0);
  FrameId id = state.RequestFrameIdForChannel(id_);
  return id;
}

template <typename T>
GeometryId GeometryChannel<T>::DeclareGeometry(Context<T>* context,
                                            FrameId frame_id,
                                            unique_ptr<GeometryInstance> geometry,
                                            const Isometry3<T>& X_FG) {
  auto& state = context->get_mutable_state()
      ->template get_mutable_abstract_state<GeometryState<T>>(0);
  GeometryId id = state.RequestGeometryIdForFrame(id_, frame_id);
  // TODO(SeanCurtis-TRI): Do the work!
  return id;
}

template <typename T>
GeometryId GeometryChannel<T>::DeclareGeometry(Context<T>* context,
                                            GeometryId geometry_id,
                                            unique_ptr<GeometryInstance> geometry,
                                            const Isometry3<T>& X_FG) {
  auto& state = context->get_mutable_state()
      ->template get_mutable_abstract_state<GeometryState<T>>(0);
  auto frame_id = state.GetFrameId(geometry_id);
  return DeclareGeometry(context, frame_id, std::move(geometry), X_FG);
}

template <typename T>
FrameKinematicsSet<T> GeometryChannel<T>::GetFrameKinematicsSet(
    const Context<T>& context) {
  FrameKinematicsSet<T> set(id_);
  // TODO(SeanCurtis-TRI): Populate this from the reference data.
  return set;
}

template <typename T>
void GeometryChannel<T>::UpdateFrameKinematicsSet(
    const Context<T>& context, FrameKinematicsSet<T>* frame_set) {
  // TODO(SeanCurtis-TRI): Do this work.
}

// Explicitly instantiates on the most common scalar types.
template class GeometryChannel<double>;

}  // namespace geometry
}  // namespace drake
