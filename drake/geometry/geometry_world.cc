#include "drake/geometry/geometry_world.h"

#include <string>
#include <utility>
#include <vector>

#include "drake/geometry/geometry_context.h"
#include "drake/geometry/geometry_frame.h"
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
SourceId GeometryWorld<T>::RegisterNewSource(GeometryContext<T>* context,
                                             const std::string& name) {
  using std::to_string;
  SourceId id = SourceId::get_new_id();
  auto& state = context->get_mutable_state()
      ->template get_mutable_abstract_state<GeometryState<T>>(0);
  state.RegisterNewSource(id);
  std::string source_name = name == "" ? "Source_" + to_string(id) : name;
  for (auto pair : sources_) {
    if (pair.second == source_name) {
      throw std::logic_error(
          "Registering new source with duplicate name: " + source_name + ".");
    }
  }
  sources_[id] = source_name;
  return id;
}

template <typename T>
bool GeometryWorld<T>::SourceIsRegistered(SourceId id) const {
  return sources_.find(id) != sources_.end();
}

template <typename T>
FrameId GeometryWorld<T>::RegisterFrame(GeometryContext<T>* context,
                                        SourceId source_id,
                                        const GeometryFrame<T>& frame) {
  auto state = get_mutable_state(context);
  FrameId id = state->RegisterFrame(source_id, frame);
  // TODO(SeanCurtis-TRI): Instantiate frame in data and handle eventual
  // payload.
  return id;
}

template <typename T>
GeometryId GeometryWorld<T>::RegisterGeometry(
    GeometryContext<T>* context, SourceId source_id, FrameId frame_id,
    unique_ptr<GeometryInstance<T>> geometry) {
  auto state = get_mutable_state(context);
  return state->RegisterGeometry(source_id, frame_id, move(geometry));
}

template <typename T>
GeometryId GeometryWorld<T>::RegisterGeometry(
    GeometryContext<T>* context, SourceId source_id, GeometryId geometry_id,
    unique_ptr<GeometryInstance<T>> geometry) {
  auto state = get_mutable_state(context);
  return state->RegisterGeometryWithParent(source_id, geometry_id,
                                          std::move(geometry));
}

template <typename T>
GeometryId GeometryWorld<T>::RegisterAnchoredGeometry(
    GeometryContext<T>* context, SourceId source_id,
    unique_ptr<GeometryInstance<T>> geometry) {
  auto state = get_mutable_state(context);
  return state->RegisterAnchoredGeometry(source_id, std::move(geometry));
}

template <typename T>
void GeometryWorld<T>::ClearSource(GeometryContext<T>* context,
                                   SourceId source_id) {
  auto state = get_mutable_state(context);
  state->ClearSource(source_id);
}

template <typename T>
FrameKinematicsSet<T> GeometryWorld<T>::GetFrameKinematicsSet(
    SourceId source_id) {
  DRAKE_ASSERT_VOID(AssertValidSource(source_id));
  FrameKinematicsSet<T> set(source_id);
  return set;
}

template <typename T>
void GeometryWorld<T>::SetFrameKinematics(
    GeometryContext<T>* context,
    const FrameKinematicsSet<T>& frame_kinematics) {
  auto state = get_state(*context);
  state.SetFrameKinematics(frame_kinematics);
}

template <typename T>
vector<unique_ptr<AbstractValue>> GeometryWorld<T>::AllocateAbstractValues() {
  vector<unique_ptr<AbstractValue>> values;
  values.push_back(make_unique<Value<GeometryState<T>>>());
  return values;
}

template <typename T>
GeometryState<T>* GeometryWorld<T>::get_mutable_state(
    GeometryContext<T>* context) {
  return &context->get_mutable_state()
              ->template get_mutable_abstract_state<GeometryState<T>>(0);
}

template <typename T>
const GeometryState<T>& GeometryWorld<T>::get_state(
    const GeometryContext<T>& context) {
  return context.get_state().template get_abstract_state<GeometryState<T>>(0);
}

template <typename T>
void GeometryWorld<T>::AssertValidSource(SourceId source_id) const {
  using std::to_string;
  if (!SourceIsRegistered(source_id)) {
    throw std::logic_error("Invalid source id: " + to_string(source_id) + ".");
  }
}

// Explicitly instantiates on the most common scalar types.
template class GeometryWorld<double>;

}  // namespace geometry
}  // namespace drake
