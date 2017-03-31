#include "drake/geometry/geometry_world.h"

#include "drake/geometry/geometry_context.h"
#include "drake/geometry/geometry_instance.h"
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
SourceId GeometryWorld<T>::RegisterNewSource(GeometryContext<T>* context) {
  auto& state = context->get_mutable_state()
                    ->template get_mutable_abstract_state<GeometryState<T>>(0);
  return state.RequestSourceId();
}

template <typename T>
void GeometryWorld<T>::RemoveSource(SourceId source_id, GeometryContext<T>* context) {
  auto& state = context->get_mutable_state()
      ->template get_mutable_abstract_state<GeometryState<T>>(0);
  return state.RemoveSource(source_id);
}

template <typename T>
FrameId GeometryWorld<T>::RegisterFrame(GeometryContext<T>* context,
                                        SourceId source_id) {
  auto& state = context->get_mutable_state()
      ->template get_mutable_abstract_state<GeometryState<T>>(0);
  FrameId id = state.RequestFrameIdForSource(source_id);
  // TODO(SeanCurtis-TRI): Instantiate frame in data and handle eventual
  // payload.
  return id;
}

template <typename T>
GeometryId GeometryWorld<T>::RegisterGeometry(GeometryContext<T>* context,
                                              SourceId source_id,
                                              FrameId frame_id,
                                              unique_ptr<GeometryInstance> geometry,
                                              const Isometry3<T>& X_FG) {
  auto& state = context->get_mutable_state()
      ->template get_mutable_abstract_state<GeometryState<T>>(0);
  GeometryId id = state.RequestGeometryIdForFrame(source_id, frame_id);
  // TODO(SeanCurtis-TRI): Do the work!
  return id;
}

template <typename T>
GeometryId GeometryWorld<T>::RegisterGeometry(GeometryContext<T>* context,
                                              SourceId source_id,
                                              GeometryId geometry_id,
                                              unique_ptr<GeometryInstance> geometry,
                                              const Isometry3<T>& X_FG) {
  auto& state = context->get_mutable_state()
      ->template get_mutable_abstract_state<GeometryState<T>>(0);
  auto frame_id = state.GetFrameId(geometry_id);
  // TODO(SeanCurtis-TRI): The actual code should hang it on the frame
  // associated with the geometry_id.
  return RegisterGeometry(context, source_id, frame_id, std::move(geometry), X_FG);
}

template <typename T>
GeometryId GeometryWorld<T>::RegisterAnchoredGeometry(GeometryContext<T>* context,
                                                      SourceId source_id,
                               unique_ptr<GeometryInstance> geometry,
                               const Isometry3<T>& X_WG) {
  GeometryId id = GeometryId::get_new_id();
  // TODO(SeanCurtis-TRI): Actually do this work.
  return id;
}

template <typename T>
FrameKinematicsSet<T> GeometryWorld<T>::GetFrameKinematicsSet(
    const GeometryContext<T>& context, SourceId source_id) {
  DRAKE_DEMAND(context.get_state().template get_abstract_state<GeometryState<T>>(0).SourceIsActive(source_id));
  FrameKinematicsSet<T> set(source_id);
  return set;
}

template <typename T>
void GeometryWorld<T>::SetFrameKinematics(
    GeometryContext<T>* context, const FrameKinematicsSet<T>& frame_kinematics) {
  // TODO(SeanCurtis-TRI): Do this work.
}

template <typename T>
vector<unique_ptr<AbstractValue>> GeometryWorld<T>::AllocateAbstractValues() {
  vector<unique_ptr<AbstractValue>> values;
  values.push_back(make_unique<Value<GeometryState<T>>>());
  return values;
}

// Explicitly instantiates on the most common scalar types.
template class GeometryWorld<double>;

}  // namespace geometry
}  // namespace drake
