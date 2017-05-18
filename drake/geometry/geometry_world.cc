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
SourceId GeometryWorld<T>::RegisterNewSource(GeometryState<T>* state,
                                             const std::string& name) {
  using std::to_string;
  SourceId id = SourceId::get_new_id();
  std::string source_name = name == "" ? "Source_" + to_string(id) : name;
  state->RegisterNewSource(id, source_name);
  return id;
}

template <typename T>
bool GeometryWorld<T>::SourceIsRegistered(const GeometryState<T>& state,
                                          SourceId id) const {
  return state.source_is_active(id);
}

template <typename T>
FrameId GeometryWorld<T>::RegisterFrame(GeometryState<T>* state,
                                        SourceId source_id,
                                        const GeometryFrame<T>& frame) {
  FrameId id = state->RegisterFrame(source_id, frame);
  return id;
}

template <typename T>
GeometryId GeometryWorld<T>::RegisterGeometry(
    GeometryState<T>* state, SourceId source_id, FrameId frame_id,
    unique_ptr<GeometryInstance<T>> geometry) {
  return state->RegisterGeometry(source_id, frame_id, move(geometry));
}

template <typename T>
GeometryId GeometryWorld<T>::RegisterGeometry(
    GeometryState<T>* state, SourceId source_id, GeometryId geometry_id,
    unique_ptr<GeometryInstance<T>> geometry) {
  return state->RegisterGeometryWithParent(source_id, geometry_id,
                                          std::move(geometry));
}

template <typename T>
GeometryId GeometryWorld<T>::RegisterAnchoredGeometry(
    GeometryState<T>* state, SourceId source_id,
    unique_ptr<GeometryInstance<T>> geometry) {
  return state->RegisterAnchoredGeometry(source_id, std::move(geometry));
}

template <typename T>
void GeometryWorld<T>::ClearSource(GeometryState<T>* state,
                                   SourceId source_id) {
  state->ClearSource(source_id);
}

template <typename T>
FrameKinematicsSet<T> GeometryWorld<T>::GetFrameKinematicsSet(
    const GeometryState<T>& state, SourceId source_id) {
  DRAKE_ASSERT_VOID(AssertValidSource(state, source_id));
  FrameKinematicsSet<T> set(source_id);
  return set;
}

template <typename T>
void GeometryWorld<T>::SetFrameKinematics(
    GeometryState<T>* state,
    const FrameKinematicsSet<T>& frame_kinematics) {
  state->SetFrameKinematics(frame_kinematics);
}

template <typename T>
unique_ptr<GeometryState<T>> GeometryWorld<T>::CreateState() {
  return make_unique<GeometryState<T>>();
}

template <typename T>
void GeometryWorld<T>::AssertValidSource(const GeometryState<T>& state,
                                         SourceId source_id) const {
  using std::to_string;
  if (!state.source_is_active(source_id)) {
    throw std::logic_error("Invalid source id: " + to_string(source_id) + ".");
  }
}

// Explicitly instantiates on the most common scalar types.
template class GeometryWorld<double>;

}  // namespace geometry
}  // namespace drake
