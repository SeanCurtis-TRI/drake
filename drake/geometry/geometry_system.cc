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
using systems::LeafContext;
using systems::LeafSystem;
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

  this->DeclareAbstractOutputPort(&GeometrySystem::MakePoseBundle,
                                  &GeometrySystem::CalcPoseBundle);
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
const InputPortDescriptor<T>&
GeometrySystem<T>::get_port_for_source_id(
    SourceId id, GeometrySystem<T>::PortType port_type) {
  using std::to_string;
  SourcePorts* source_ports;

  // Access port data based on the source id -- catching the possibility of an
  // invalid id.
  auto itr = input_source_ids_.find(id);
  if (itr != input_source_ids_.end()) {
    source_ports = &(itr->second);
  } else {
    if (!context_allocated_) {
      if (initial_state_->source_is_active(id)) {
        source_ports = &input_source_ids_[id];
      } else {
        throw std::logic_error("Can't create input port for unknown source id: "
                               + to_string(id) + ".");
      }
    } else {
      throw std::logic_error(
          "Can't create new input ports after context has been allocated.");
    }
  }

  // Helper method to return the input port (creating it as necessary).
  auto get_port = [this](int* port_id) -> const InputPortDescriptor<T>& {
    if (*port_id != -1) {
      return this->get_input_port(*port_id);
    } else {
      const auto &input_port = this->DeclareAbstractInputPort();
      *port_id = input_port.get_index();
      return input_port;
    }
  };

  // Get the port based on requested type.
  switch (port_type) {
    case ID: {
      return get_port(&source_ports->id_port);
    }
    case POSE: {
      return get_port(&source_ports->pose_port);
    }
    case VELOCITY: {
      return get_port(&source_ports->velocity_port);
    }
  }
}

template <typename T>
const systems::InputPortDescriptor<T>&
GeometrySystem<T>::get_source_frame_id_port(SourceId id) {
  return get_port_for_source_id(id, ID);
}

template <typename T>
const systems::InputPortDescriptor<T>&
GeometrySystem<T>::get_source_pose_port(SourceId id) {
  return get_port_for_source_id(id, POSE);
}

template <typename T>
const systems::InputPortDescriptor<T>&
GeometrySystem<T>::get_source_velocity_port(SourceId id) {
  return get_port_for_source_id(id, VELOCITY);
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
void GeometrySystem<T>::CalcPoseBundle(const Context<T>& context,
                                       PoseBundle<T>* output) const {
  // TODO(SeanCurtis-TRI): Adding/removing frames during discrete updates will
  // change the size/composition of the pose bundle. This output port will *not*
  // be updated to reflect that. I must test the output port to confirm that it
  // is up to date w.r.t. the current state of the world.
  //  Add serial number to GeometryState and PoseBundle. If serial numbers match
  //  everything is good. Otherwise, I need to modify the pose bundle.
  //  This *also* requires modification of PoseBundle to make it mutable.
  int i = 0;
  const auto& g_context = static_cast<const GeometryContext<T>&>(context);
  const auto& g_state = g_context.get_geometry_state();
  for (FrameId f_id : initial_state_->get_frame_ids()) {
    output->set_pose(i, g_state.get_pose_in_parent(f_id));
    // TODO(SeanCurtis-TRI): Handle velocity.
    ++i;
  }
}

template <typename T>
PoseBundle<T> GeometrySystem<T>::MakePoseBundle(
    const Context<T>& context) const {
  const auto& g_context = static_cast<const GeometryContext<T>&>(context);
  const auto& g_state = g_context.get_geometry_state();
  PoseBundle<T> bundle(g_context.get_geometry_state().get_num_frames());
  int i = 0;
  for (FrameId f_id : g_state.get_frame_ids()) {
    int frame_group = g_state.get_frame_group(f_id);
    bundle.set_model_instance_id(i, frame_group);

    SourceId s_id = g_state.get_source_id(f_id);
    const std::string& src_name = g_state.get_source_name(s_id);
    const std::string& frm_name = g_state.get_frame_name(f_id);
    std::string name = src_name + "::" + frm_name;
    bundle.set_name(i, name);
    ++i;
  }
  return bundle;
}

template <typename T>
const GeometryContext<T>& GeometrySystem<T>::UpdateFromInputs(
    const Context<T>& sibling_context) const {
  using std::to_string;
  // TODO(SeanCurtis-TRI): This needs to exploit a cache to avoid doing this
  // work redundantly (and to even allow *changing* geometry engine state.
  // This is the horrible, hacky terrible thing where I'm implicitly treating
  // my own context's const state to be mutable so I can make sure the geometry
  // world state is up to date (relative to its inputs).
  const GeometryContext<T>& g_context =
      ExtractContextViaSiblingContext(sibling_context);
  const GeometryState<T>& state = g_context.get_geometry_state();
  GeometryState<T>& mutable_state = const_cast<GeometryState<T>&>(state);

  for (const auto& pair : state.source_frame_id_map_) {
    if (pair.second.size() > 0) {
      SourceId source_id = pair.first;
      const auto itr = input_source_ids_.find(source_id);
      if (itr != input_source_ids_.end()) {
        const int id_port = itr->second.id_port;
        if (id_port >= 0) {
          const FrameIdVector& ids =
              this->template EvalAbstractInput(g_context, id_port)
                  ->template GetValue<FrameIdVector>();
          state.ValidateFrameIds(ids);
          const int pose_port = itr->second.pose_port;
          if (pose_port >= 0) {
            const FramePoseSet<T>& poses =
                this->template EvalAbstractInput(g_context, pose_port)
                    ->template GetValue<FramePoseSet<T>>();
            mutable_state.SetFramePoses(ids, poses);
          } else {
            throw std::logic_error(
                "Source " + to_string(source_id) + " has registered frames "
                "but does not provide pose values on the input port.");
          }
        } else {
          throw std::logic_error(
              "Source " + to_string(source_id) + " has registered frames "
              "but does not provide id values on the input port.");
        }
      } else {
        throw std::logic_error(
            "Source " + to_string(source_id) + " has registered frames "
                "but does not provide values on any input port.");
      }
    }
  }

  // TODO(SeanCurtis-TRI): This should be part of responding to dirty pose
  // inputs.
  mutable_state.FinalizePoseUpdate();
  // TODO(SeanCurtis-TRI): Add velocity as appropriate.
  return g_context;
}

template <typename T>
std::unique_ptr<LeafContext<T>> GeometrySystem<T>::DoMakeContext() const {
  // Disallow further geometry source additions.
  context_allocated_ = true;
  return std::unique_ptr<LeafContext<T>>(new GeometryContext<T>());
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
