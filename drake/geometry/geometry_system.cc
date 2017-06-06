#include "drake/geometry/geometry_system.h"

#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/geometry/geometry_context.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_state.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_context.h"
#include "drake/systems/rendering/pose_bundle.h"

namespace drake {
namespace geometry {

using systems::AbstractValue;
using systems::Context;
using systems::InputPortDescriptor;
using systems::LeafSystem;
using systems::OutputPortDescriptor;
using systems::SystemOutput;
using systems::rendering::PoseBundle;
using std::vector;

template <typename T>
GeometrySystem<T>::GeometrySystem() : LeafSystem<T>() {
  // Only GeometryWorld can create a GeometryState; we create one, copy it into
  // the value and then delete the original (as execution moves out of scope).
  std::unique_ptr<GeometryState<T>> state = geometry_world_.CreateState();
  auto state_value = AbstractValue::Make<GeometryState<T>>(*state.get());
  initial_state_ = &state_value->template GetMutableValue<GeometryState<T>>();
  this->DeclareAbstractState(std::move(state_value));

  // Place holder for the pose bundle output abstract port.
  this->DeclareAbstractOutputPort();
}

template <typename T>
GeometrySystem<T>::~GeometrySystem() {}

template <typename T>
SourceId GeometrySystem<T>::RegisterSource(const std::string &name) {
  if (!context_allocated_) {
    return initial_state_->RegisterNewSource(name);
  } else {
    throw std::logic_error(
        "A context has been created for this system. Adding "
        "new geometry sources is no longer possible.");
  }
}

template <typename T>
const systems::InputPortDescriptor<T>&
GeometrySystem<T>::get_port_for_source_id(SourceId id) {
  using std::to_string;
  auto itr = input_source_ids_.find(id);
  if (itr != input_source_ids_.end()) {
    return this->get_input_port(itr->second);
  } else {
    if (!context_allocated_) {
      if (initial_state_->source_is_active(id)) {
        DRAKE_ASSERT(static_cast<int>(input_source_ids_.size()) ==
            this->get_num_input_ports());
        const auto& input_port = this->DeclareAbstractInputPort();
        input_source_ids_[id] = input_port.get_index();
        return input_port;
      } else {
        throw std::logic_error("Can't create input port for unknown source id: "
                               + to_string(id) + ".");
      }
    } else {
      throw std::logic_error(
          "Can't create new input ports after context has been allocated.");
    }
  }
}

template <typename T>
void GeometrySystem<T>::DoCalcOutput(const Context<T>& context,
                  SystemOutput<T>* output) const {
  // This should *always* be invoked with a GeometryContext.
  const GeometryState<T>& state =
      static_cast<const GeometryContext<T>&>(context).get_geometry_state();
  PoseBundle<T>& bundle =
      output->GetMutableData(0)->template GetMutableValue<PoseBundle<T>>();
  int i = 0;
  // This would be more efficient if I simply iterated through engine order.
  //  To achieve that, I'd need one more map from
  for (FrameId f_id : initial_state_->get_frame_ids()) {
    bundle.set_pose(i, state.get_pose_in_parent(f_id));
    // TODO(SeanCurtis-TRI): Handle velocity.
    ++i;
  }
}

template <typename T>
  std::unique_ptr<AbstractValue> GeometrySystem<T>::AllocateOutputAbstract(
  const OutputPortDescriptor<T>&) const {
  using std::to_string;
  // NOTE: Using initial_state_ *feels* a bit fragile. This takes place *after*
  // the context has been allocated. Tests against an allocated context
  // guarantee that no change can be made to the model geometry state to which
  // this variable points and now. It is carved in stone. So, it will be safe.
  auto value = AbstractValue::Make(PoseBundle<T>(
      initial_state_->get_num_frames()));
  // This is the work that should be done whenever the pose bundle is resize;
  //  the configuration
  auto& bundle = value->template GetMutableValue<PoseBundle<T>>();
  int i = 0;
  for (FrameId f_id : initial_state_->get_frame_ids()) {
    int frame_group = initial_state_->get_frame_group(f_id);
    bundle.set_model_instance_id(i, frame_group);

    SourceId s_id = initial_state_->get_source_id(f_id);
    const std::string& src_name = initial_state_->get_source_name(s_id);
    const std::string& frm_name = initial_state_->get_frame_name(f_id);
    // TODO(SeanCurtis-TRI): Consider replacing geometry id with geometry name.
    std::string name = src_name + "::" + frm_name;
    bundle.set_name(i, name);
    ++i;
  }
  return value;
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
  return initial_state_->RegisterFrame(source_id, frame);
}

template <typename T>
FrameId GeometrySystem<T>::RegisterFrame(Context<T>* sibling_context,
                                         SourceId source_id,
                                         const GeometryFrame<T>& frame) {
  GeometryContext<T>& context =
      ExtractMutableContextViaSiblingContext(*sibling_context);
  return geometry_world_.RegisterFrame(&context, source_id, frame);
}

template <typename T>
FrameId GeometrySystem<T>::RegisterFrame(SourceId source_id, FrameId parent_id,
                                         const GeometryFrame<T>& frame) {
  ThrowIfContextAllocated();
  return initial_state_->RegisterFrame(source_id, parent_id, frame);
}

template <typename T>
FrameId GeometrySystem<T>::RegisterFrame(Context<T>* sibling_context,
                                         SourceId source_id, FrameId parent_id,
                                         const GeometryFrame<T>& frame) {
  GeometryContext<T>& context =
      ExtractMutableContextViaSiblingContext(*sibling_context);
  return geometry_world_.RegisterFrame(&context, source_id, parent_id,
                                       frame);
}

template <typename T>
GeometryId GeometrySystem<T>::RegisterGeometry(
    SourceId source_id, FrameId frame_id,
    std::unique_ptr<GeometryInstance<T>> geometry) {
  ThrowIfContextAllocated();
  return initial_state_->RegisterGeometry(source_id, frame_id,
                                          std::move(geometry));
}

template <typename T>
GeometryId GeometrySystem<T>::RegisterGeometry(
    Context<T>* sibling_context, SourceId source_id, FrameId frame_id,
    std::unique_ptr<GeometryInstance<T>> geometry) {
  // TODO(SeanCurtis-TRI): Resize the output pose bundle.
  GeometryContext<T>& context =
      ExtractMutableContextViaSiblingContext(*sibling_context);
  return geometry_world_.RegisterGeometry(&context, source_id, frame_id,
                                          std::move(geometry));
}

template <typename T>
GeometryId GeometrySystem<T>::RegisterGeometry(
    SourceId source_id, GeometryId geometry_id,
    std::unique_ptr<GeometryInstance<T>> geometry) {
  ThrowIfContextAllocated();
  return initial_state_->RegisterGeometryWithParent(source_id, geometry_id,
                                                    std::move(geometry));
}

template <typename T>
GeometryId GeometrySystem<T>::RegisterGeometry(
    Context<T>* sibling_context, SourceId source_id, GeometryId geometry_id,
    std::unique_ptr<GeometryInstance<T>> geometry) {
  // TODO(SeanCurtis-TRI): Resize the output pose bundle.
  GeometryContext<T>& context =
      ExtractMutableContextViaSiblingContext(*sibling_context);
  return geometry_world_.RegisterGeometry(&context, source_id,
                                          geometry_id, std::move(geometry));
}

template <typename T>
GeometryId GeometrySystem<T>::RegisterAnchoredGeometry(
    SourceId source_id,
    std::unique_ptr<GeometryInstance<T>> geometry) {
  ThrowIfContextAllocated();
  return initial_state_->RegisterAnchoredGeometry(source_id,
                                                  std::move(geometry));
}

template <typename T>
GeometryId GeometrySystem<T>::RegisterAnchoredGeometry(
    Context<T>* sibling_context, SourceId source_id,
    std::unique_ptr<GeometryInstance<T>> geometry) {
  // TODO(SeanCurtis-TRI): Resize the output pose bundle.
  GeometryContext<T>& context =
      ExtractMutableContextViaSiblingContext(*sibling_context);
  return geometry_world_.RegisterAnchoredGeometry(&context, source_id,
                                                  std::move(geometry));
}

template <typename T>
void GeometrySystem<T>::ClearSource(SourceId source_id) {
  ThrowIfContextAllocated();
  initial_state_->ClearSource(source_id);
}

template <typename T>
void GeometrySystem<T>::ClearSource(Context<T>* sibling_context,
                                    SourceId source_id) {
  // TODO(SeanCurtis-TRI): Resize the output pose bundle.
  GeometryContext<T>& context =
      ExtractMutableContextViaSiblingContext(*sibling_context);
  geometry_world_.ClearSource(&context, source_id);
}

template <typename T>
void GeometrySystem<T>::RemoveFrame(SourceId source_id, FrameId frame_id) {
  ThrowIfContextAllocated();
  initial_state_->RemoveFrame(source_id, frame_id);
}

template <typename T>
void GeometrySystem<T>::RemoveFrame(Context<T>* sibling_context,
                                    SourceId source_id, FrameId frame_id) {
  // TODO(SeanCurtis-TRI): Resize the output pose bundle.
  GeometryContext<T>& context =
      ExtractMutableContextViaSiblingContext(*sibling_context);
  geometry_world_.RemoveFrame(&context, source_id, frame_id);
}

template <typename T>
void GeometrySystem<T>::RemoveGeometry(SourceId source_id,
                                       GeometryId geometry_id) {
  ThrowIfContextAllocated();
  initial_state_->RemoveGeometry(source_id, geometry_id);
}

template <typename T>
void GeometrySystem<T>::RemoveGeometry(Context<T>* sibling_context,
                                       SourceId source_id,
                                       GeometryId geometry_id) {
  // TODO(SeanCurtis-TRI): Resize the output pose bundle.
  GeometryContext<T>& context =
      ExtractMutableContextViaSiblingContext(*sibling_context);
  geometry_world_.RemoveGeometry(&context, source_id, geometry_id);
}

template <typename T>
const std::string& GeometrySystem<T>::get_source_name(
    const Context<T>& sibling_context, SourceId id) const {
  const GeometryContext<T>& context =
      ExtractContextViaSiblingContext(sibling_context);
  return geometry_world_.get_source_name(context, id);
}

template <typename T>
bool GeometrySystem<T>::SourceIsRegistered(const Context<T>& sibling_context,
                                           SourceId id) const {
  const GeometryContext<T>& context =
      ExtractContextViaSiblingContext(sibling_context);
  return geometry_world_.SourceIsRegistered(context, id);
}

template <typename T>
FrameKinematicsSet<T> GeometrySystem<T>::GetFrameKinematicsSet(
    const Context<T>& sibling_context, SourceId source_id) const {
  const GeometryContext<T>& context =
      ExtractContextViaSiblingContext(sibling_context);
  return geometry_world_.GetFrameKinematicsSet(context, source_id);
}

template <typename T>
FrameId GeometrySystem<T>::GetFrameId(
    const systems::Context<T>& sibling_context, GeometryId geometry_id) const {
  const GeometryContext<T>& context =
      ExtractContextViaSiblingContext(sibling_context);
  return context.get_geometry_state().GetFrameId(geometry_id);
}

template <typename T>
bool GeometrySystem<T>::ComputeContact(const systems::Context<T> &context,
                                       vector<Contact<T>>* contacts) const {
  const GeometryContext<T>& g_context = UpdateFromInputs(context);
  return geometry_world_.ComputeContact(g_context, contacts);
}

template <typename T>
const GeometryContext<T>& GeometrySystem<T>::UpdateFromInputs(
    const Context<T>& sibling_context) const {
  // TODO(SeanCurtis-TRI): This needs to exploit a cache to avoid doing this
  // work redundantly.
  // This is the horrible, hacky terrible thing where I'm implicitly treating
  // my own context's const state to be mutable so I can make sure the geometry
  // world state is up to date (relative to its inputs).
  // This needs to be done in a better way that will do this only *once*
  // regardless of how many queries are performed.
  const GeometryContext<T>& g_context =
      ExtractContextViaSiblingContext(sibling_context);
  const GeometryState<T>& state = g_context.get_geometry_state();
  GeometryState<T>& mutable_state = const_cast<GeometryState<T>&>(state);
  for (int i = 0; i < this->get_num_input_ports(); ++i) {
    mutable_state.SetFrameKinematics(
        this->template EvalAbstractInput(g_context, i)
            ->template GetValue<FrameKinematicsSet<T>>());
  }
  // TODO(SeanCurtis-TRI): Change this method name to: FinalizeKinematicsUpdate.
  mutable_state.FinalizeKinematicsUpdate();
  return g_context;
}

template <typename T>
std::unique_ptr<Context<T>> GeometrySystem<T>::MakeContext() const {
  // Disallow further geometry source additions.
  context_allocated_ = true;
  return std::unique_ptr<Context<T>>(new GeometryContext<T>());
}

template <typename T>
GeometryContext<T>& GeometrySystem<T>::ExtractMutableContextViaSiblingContext(
    const Context<T>& sibling_context) {
    const systems::DiagramContext<T>* parent_context =
        dynamic_cast<const systems::DiagramContext<T>*>(
            sibling_context.get_parent());
    if (parent_context != nullptr) {
      int index = this->get_subsystem_index();
      const Context<T> *child_context =
          parent_context->GetSubsystemContext(index);
      const GeometryContext<T> *geometry_context =
          dynamic_cast<const GeometryContext<T>*>(child_context);
      if (geometry_context != nullptr) {
        return *const_cast<GeometryContext<T>*>(geometry_context);
      }
    }
    throw std::logic_error("The context given cannot be used to acquire this "
                               "geometry system's context.");
}

template <typename T>
const GeometryContext<T>& GeometrySystem<T>::ExtractContextViaSiblingContext(
    const Context<T>& sibling_context) const {
  return const_cast<GeometrySystem<T>*>(this)
              ->ExtractMutableContextViaSiblingContext(sibling_context);
}

template <typename T>
void GeometrySystem<T>::ThrowIfContextAllocated() const {
  if (context_allocated_)
    throw std::logic_error("Operation invalid; a context has already been "
                           "allocated.");
}

// Explicitly instantiates on the most common scalar types.
template class GeometrySystem<double>;

}  // namespace geometry
}  // namespace drake
