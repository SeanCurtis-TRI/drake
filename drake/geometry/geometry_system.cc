#include "drake/geometry/geometry_system.h"

#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/geometry/geometry_context.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_state.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_context.h"

namespace drake {
namespace geometry {

using systems::AbstractValue;
using systems::Context;
using systems::InputPortDescriptor;
using systems::LeafSystem;
using systems::SystemOutput;
using std::vector;

template <typename T>
GeometrySystem<T>::GeometrySystem() : LeafSystem<T>() {
  // Only GeometryWorld can create a GeometryState; we create one, copy it into
  // the value and then delete the original (as execution moves out of scope).
  std::unique_ptr<GeometryState<T>> state = geometry_world_.CreateState();
  auto state_value = AbstractValue::Make<GeometryState<T>>(*state.get());
  initial_state_ = &state_value->template GetMutableValue<GeometryState<T>>();
  this->DeclareAbstractState(std::move(state_value));
}

template <typename T>
GeometrySystem<T>::~GeometrySystem() {}

template <typename T>
const InputPortDescriptor<T>& GeometrySystem<T>::AddSourceInput(
    const std::string& name) {
  if (initial_state_ != nullptr) {
    DRAKE_ASSERT(static_cast<int>(input_source_ids_.size()) ==
                 this->get_num_input_ports());
    SourceId source_id =
        geometry_world_.RegisterNewSource(initial_state_, name);
    const auto& input_port = this->DeclareAbstractInputPort();
    input_source_ids_.push_back(source_id);
    return input_port;
  } else {
    throw std::logic_error(
        "A context has been created for this system. Adding "
        "new geometry sources is no longer possible.");
  }
}

template <typename T>
SourceId GeometrySystem<T>::get_port_source_id(
    const InputPortDescriptor<T>& port_descriptor) const {
  return get_port_source_id(port_descriptor.get_index());
}

template <typename T>
SourceId GeometrySystem<T>::get_port_source_id(int port_index) const {
  if (port_index >= 0 &&
      port_index < static_cast<int>(input_source_ids_.size())) {
    return input_source_ids_[port_index];
  }
  throw std::logic_error("Requesting source id for an input port with an "
                         "invalid index: " + std::to_string(port_index)
                         + ".");
}

template <typename T>
void GeometrySystem<T>::DoCalcOutput(const Context<T>& context,
                  SystemOutput<T>* output) const {
  // Calculate the the lcm data (names and pose data)
}

template <typename T>
FrameKinematicsSet<T> GeometrySystem<T>::MakeDefaultFrameKinematicsSet(
    SourceId source_id) {
  ThrowIfContextAllocated();
  return geometry_world_.GetFrameKinematicsSet(*initial_state_, source_id);
}

template <typename T>
FrameId GeometrySystem<T>::RegisterFrame(SourceId source_id,
                                         const GeometryFrame<T>& frame) {
  ThrowIfContextAllocated();
  return geometry_world_.RegisterFrame(initial_state_, source_id, frame);
}

template <typename T>
FrameId GeometrySystem<T>::RegisterFrame(Context<T>* sibling_context,
                                         SourceId source_id,
                                         const GeometryFrame<T>& frame) {
  GeometryState<T>* state =
      ExtractMutableStateViaSiblingContext(sibling_context);
  return geometry_world_.RegisterFrame(state, source_id, frame);
}

template <typename T>
FrameId GeometrySystem<T>::RegisterFrame(SourceId source_id, FrameId parent_id,
                                         const GeometryFrame<T>& frame) {
  ThrowIfContextAllocated();
  return geometry_world_.RegisterFrame(initial_state_, source_id, parent_id,
                                       frame);
}

template <typename T>
FrameId GeometrySystem<T>::RegisterFrame(Context<T>* sibling_context,
                                         SourceId source_id, FrameId parent_id,
                                         const GeometryFrame<T>& frame) {
  GeometryState<T>* state =
      ExtractMutableStateViaSiblingContext(sibling_context);
  return geometry_world_.RegisterFrame(state, source_id, parent_id,
                                       frame);
}

template <typename T>
GeometryId GeometrySystem<T>::RegisterGeometry(
    SourceId source_id, FrameId frame_id,
    std::unique_ptr<GeometryInstance<T>> geometry) {
  ThrowIfContextAllocated();
  return geometry_world_.RegisterGeometry(initial_state_, source_id, frame_id,
                                          std::move(geometry));
}

template <typename T>
GeometryId GeometrySystem<T>::RegisterGeometry(
    Context<T>* sibling_context, SourceId source_id, FrameId frame_id,
    std::unique_ptr<GeometryInstance<T>> geometry) {
  GeometryState<T>* state =
      ExtractMutableStateViaSiblingContext(sibling_context);
  return geometry_world_.RegisterGeometry(state, source_id, frame_id,
                                          std::move(geometry));
}

template <typename T>
GeometryId GeometrySystem<T>::RegisterGeometry(
    SourceId source_id, GeometryId geometry_id,
    std::unique_ptr<GeometryInstance<T>> geometry) {
  ThrowIfContextAllocated();
  return geometry_world_.RegisterGeometry(initial_state_, source_id,
                                          geometry_id, std::move(geometry));
}

template <typename T>
GeometryId GeometrySystem<T>::RegisterGeometry(
    Context<T>* sibling_context, SourceId source_id, GeometryId geometry_id,
    std::unique_ptr<GeometryInstance<T>> geometry) {
  GeometryState<T>* state =
      ExtractMutableStateViaSiblingContext(sibling_context);
  return geometry_world_.RegisterGeometry(state, source_id,
                                          geometry_id, std::move(geometry));
}

template <typename T>
GeometryId GeometrySystem<T>::RegisterAnchoredGeometry(
    SourceId source_id,
    std::unique_ptr<GeometryInstance<T>> geometry) {
  ThrowIfContextAllocated();
  return geometry_world_.RegisterAnchoredGeometry(initial_state_, source_id,
                                                  std::move(geometry));
}

template <typename T>
GeometryId GeometrySystem<T>::RegisterAnchoredGeometry(
    Context<T>* sibling_context, SourceId source_id,
    std::unique_ptr<GeometryInstance<T>> geometry) {
  GeometryState<T>* state =
      ExtractMutableStateViaSiblingContext(sibling_context);
  return geometry_world_.RegisterAnchoredGeometry(state, source_id,
                                                  std::move(geometry));
}

template <typename T>
void GeometrySystem<T>::ClearSource(SourceId source_id) {
  ThrowIfContextAllocated();
  geometry_world_.ClearSource(initial_state_, source_id);
}

template <typename T>
void GeometrySystem<T>::ClearSource(Context<T>* sibling_context,
                                    SourceId source_id) {
  GeometryState<T>* state =
      ExtractMutableStateViaSiblingContext(sibling_context);
  geometry_world_.ClearSource(state, source_id);
}

template <typename T>
void GeometrySystem<T>::RemoveFrame(SourceId source_id, FrameId frame_id) {
  ThrowIfContextAllocated();
  geometry_world_.RemoveFrame(initial_state_, source_id, frame_id);
}

template <typename T>
void GeometrySystem<T>::RemoveFrame(Context<T>* sibling_context,
                                    SourceId source_id, FrameId frame_id) {
  GeometryState<T>* state =
      ExtractMutableStateViaSiblingContext(sibling_context);
  geometry_world_.RemoveFrame(state, source_id, frame_id);
}

template <typename T>
void GeometrySystem<T>::RemoveGeometry(SourceId source_id,
                                       GeometryId geometry_id) {
  ThrowIfContextAllocated();
  geometry_world_.RemoveGeometry(initial_state_, source_id, geometry_id);
}

template <typename T>
void GeometrySystem<T>::RemoveGeometry(Context<T>* sibling_context,
                                       SourceId source_id,
                                       GeometryId geometry_id) {
  GeometryState<T>* state =
      ExtractMutableStateViaSiblingContext(sibling_context);
  geometry_world_.RemoveGeometry(state, source_id, geometry_id);
}

template <typename T>
const std::string& GeometrySystem<T>::get_source_name(
    const Context<T>& sibling_context, SourceId id) const {
  const GeometryState<T>& state =
      ExtractStateViaSiblingContext(sibling_context);
  return geometry_world_.get_source_name(state, id);
}

template <typename T>
bool GeometrySystem<T>::SourceIsRegistered(const Context<T>& sibling_context,
                                           SourceId id) const {
  const GeometryState<T>& state =
      ExtractStateViaSiblingContext(sibling_context);
  return geometry_world_.SourceIsRegistered(state, id);
}

template <typename T>
FrameKinematicsSet<T> GeometrySystem<T>::GetFrameKinematicsSet(
    const Context<T>& sibling_context, SourceId source_id) const {
  const GeometryState<T>& state =
      ExtractStateViaSiblingContext(sibling_context);
  return geometry_world_.GetFrameKinematicsSet(state, source_id);
}

template <typename T>
FrameId GeometrySystem<T>::GetFrameId(const systems::Context<T> &context,
                                      GeometryId geometry_id) const {
  const GeometryState<T>& state = ExtractStateViaSiblingContext(context);
  return state.GetFrameId(geometry_id);
}

template <typename T>
bool GeometrySystem<T>::ComputeContact(const systems::Context<T> &context,
                                       vector<Contact<T>> *contacts) const {
  const GeometryState<T>& state = UpdateFromInputs(context);
  return geometry_world_.ComputeContact(state, contacts);
}

template <typename T>
const GeometryState<T>& GeometrySystem<T>::UpdateFromInputs(
    const Context<T>& sibling_context) const {
  // TODO(SeanCurtis-TRI): This needs to exploit a cache to avoid doing this
  // work redundantly.
  // This is the horrible, hacky terrible thing where I'm implicitly treating
  // my own context's const state to be mutable so I can make sure the geometry
  // world state is up to date (relative to its inputs).
  // This needs to be done in a better way that will do this only *once*
  // regardless of how many queries are performed.
  const GeometryContext<T>& g_context = get_context(sibling_context);
  const GeometryState<T>& state = get_state(g_context);
  GeometryState<T>& mutable_state = const_cast<GeometryState<T>&>(state);
  for (int i = 0; i < this->get_num_input_ports(); ++i) {
    mutable_state.SetFrameKinematics(
        this->template EvalAbstractInput(g_context, i)
            ->template GetValue<FrameKinematicsSet<T>>());
  }
  mutable_state.SignalUpdateComplete();
  return state;
}

template <typename T>
std::unique_ptr<Context<T>> GeometrySystem<T>::MakeContext() const {
  // Disallow further geometry source additions.
  if (initial_state_ != nullptr) initial_state_ = nullptr;
  return std::unique_ptr<Context<T>>(new GeometryContext<T>());
}

template <typename T>
GeometryState<T>* GeometrySystem<T>::ExtractMutableStateViaSiblingContext(
    Context<T>* sibling_context) {
  GeometryContext<T>* g_context = get_mutable_context(sibling_context);
  return get_mutable_state(g_context);
}

template <typename T>
const GeometryState<T>& GeometrySystem<T>::ExtractStateViaSiblingContext(
    const Context<T>& sibling_context) const {
  const GeometryContext<T>& g_context = get_context(sibling_context);
  return get_state(g_context);
}

template <typename T>
GeometryContext<T>* GeometrySystem<T>::get_mutable_context(
    const Context<T>* sibling_context) {
  const systems::DiagramContext<T>* parent_context =
      dynamic_cast<const systems::DiagramContext<T>*>(
          sibling_context->get_parent());
  if (parent_context != nullptr) {
    int index = this->get_subsystem_index();
    const Context<T> *child_context =
        parent_context->GetSubsystemContext(index);
    const GeometryContext<T> *geometry_context =
        dynamic_cast<const GeometryContext<T> *>(child_context);
    if (geometry_context != nullptr) {
      return const_cast<GeometryContext<T> *>(geometry_context);
    }
  }
  throw std::logic_error("The context given cannot be used to acquire this "
                         "geometry system's context.");
}

template <typename T>
const GeometryContext<T>& GeometrySystem<T>::get_context(
    const Context<T>& sibling_context) const {
  return *const_cast<GeometrySystem<T>*>(this)->get_mutable_context(
      &sibling_context);
}

template <typename T>
GeometryState<T>* GeometrySystem<T>::get_mutable_state(
    GeometryContext<T>* context) {
  return &context->get_mutable_state()
      ->template get_mutable_abstract_state<GeometryState<T>>(0);
}

template <typename T>
const GeometryState<T>& GeometrySystem<T>::get_state(
    const GeometryContext<T>& context) const {
  return context.get_state().template get_abstract_state<GeometryState<T>>(0);
}

template <typename T>
void GeometrySystem<T>::ThrowIfContextAllocated() const {
  if (initial_state_ == nullptr)
    throw std::logic_error("Operation invalid; a context has already been "
                           "allocated.");
}

// Explicitly instantiates on the most common scalar types.
template class GeometrySystem<double>;

}  // namespace geometry
}  // namespace drake
