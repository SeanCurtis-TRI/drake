#include "drake/geometry/geometry_system.h"

#include <string>
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
using systems::SparsityMatrix;
using systems::SystemOutput;
using systems::rendering::PoseBundle;
using std::vector;

#define THROW_IF_CONTEXT_ALLOCATED ThrowIfContextAllocated(__FUNCTION__);

template <typename T>
GeometrySystem<T>::GeometrySystem() : LeafSystem<T>() {
  // Only GeometryWorld can create a GeometryState; we create one, copy it into
  // the value and then delete the original (as execution moves out of scope).
  std::unique_ptr<GeometryState<T>> state = geometry_world_.CreateState();
  auto state_value = AbstractValue::Make<GeometryState<T>>(*state.get());
  initial_state_ = &state_value->template GetMutableValue<GeometryState<T>>();
  this->DeclareAbstractState(std::move(state_value));

  bundle_port_index_ =
      this->DeclareAbstractOutputPort(&GeometrySystem::MakePoseBundle,
                                      &GeometrySystem::CalcPoseBundle)
          .get_index();

  query_port_index_ =
      this->DeclareAbstractOutputPort(&GeometrySystem::MakeQueryHandle,
                                      &GeometrySystem::CalcQueryHandle)
          .get_index();
}

template <typename T>
SourceId GeometrySystem<T>::RegisterSource(const std::string &name) {
  THROW_IF_CONTEXT_ALLOCATED
  SourceId source_id = initial_state_->RegisterNewSource(name);
  // Instantiates a default-initialized SourcePorts instance for the new
  // source id.
  input_source_ids_[source_id];
  return source_id;
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
  THROW_IF_CONTEXT_ALLOCATED
  return initial_state_->RegisterFrame(source_id, frame);
}

template <typename T>
FrameId GeometrySystem<T>::RegisterFrame(SourceId source_id, FrameId parent_id,
                                         const GeometryFrame<T>& frame) {
  THROW_IF_CONTEXT_ALLOCATED
  return initial_state_->RegisterFrame(source_id, parent_id, frame);
}

template <typename T>
GeometryId GeometrySystem<T>::RegisterGeometry(
    SourceId source_id, FrameId frame_id,
    std::unique_ptr<GeometryInstance<T>> geometry) {
  THROW_IF_CONTEXT_ALLOCATED
  return initial_state_->RegisterGeometry(source_id, frame_id,
                                          std::move(geometry));
}

template <typename T>
GeometryId GeometrySystem<T>::RegisterGeometry(
    SourceId source_id, GeometryId geometry_id,
    std::unique_ptr<GeometryInstance<T>> geometry) {
  THROW_IF_CONTEXT_ALLOCATED
  return initial_state_->RegisterGeometryWithParent(source_id, geometry_id,
                                                    std::move(geometry));
}

template <typename T>
GeometryId GeometrySystem<T>::RegisterAnchoredGeometry(
    SourceId source_id,
    std::unique_ptr<GeometryInstance<T>> geometry) {
  THROW_IF_CONTEXT_ALLOCATED
  return initial_state_->RegisterAnchoredGeometry(source_id,
                                                  std::move(geometry));
}

template <typename T>
void GeometrySystem<T>::ClearSource(SourceId source_id) {
  THROW_IF_CONTEXT_ALLOCATED
  initial_state_->ClearSource(source_id);
}

template <typename T>
void GeometrySystem<T>::RemoveFrame(SourceId source_id, FrameId frame_id) {
  THROW_IF_CONTEXT_ALLOCATED
  initial_state_->RemoveFrame(source_id, frame_id);
}

template <typename T>
void GeometrySystem<T>::RemoveGeometry(SourceId source_id,
                                       GeometryId geometry_id) {
  THROW_IF_CONTEXT_ALLOCATED
  initial_state_->RemoveGeometry(source_id, geometry_id);
}

template <typename T>
const std::string& GeometrySystem<T>::get_source_name(
    const QueryHandle<T>& handle, SourceId id) const {
  DRAKE_DEMAND(handle.context_);
  const GeometryContext<T>& context = *handle.context_;
  return geometry_world_.get_source_name(context, id);
}

template <typename T>
bool GeometrySystem<T>::SourceIsRegistered(const QueryHandle<T>& handle,
                                           SourceId id) const {
  DRAKE_DEMAND(handle.context_);
  const GeometryContext<T>& context = *handle.context_;
  return geometry_world_.SourceIsRegistered(context, id);
}

template <typename T>
FrameId GeometrySystem<T>::GetFrameId(
    const QueryHandle<T>& handle, GeometryId geometry_id) const {
  DRAKE_DEMAND(handle.context_);
  const GeometryContext<T>& context = *handle.context_;
  return context.get_geometry_state().GetFrameId(geometry_id);
}

template <typename T>
bool GeometrySystem<T>::ComputeContact(const QueryHandle<T>& handle,
                                       vector<Contact<T>>* contacts) const {
  DRAKE_DEMAND(handle.context_);
  const GeometryContext<T>& g_context = FullPoseUpdate(handle);
  return geometry_world_.ComputeContact(g_context, contacts);
}

template <typename T>
bool GeometrySystem<T>::DoHasDirectFeedthrough(const SparsityMatrix*,
                                               int input_port,
                                               int output_port) const {
  DRAKE_ASSERT(input_port >= 0);
  DRAKE_ASSERT(input_port < this->get_num_input_ports());
  DRAKE_ASSERT(output_port >= 0);
  DRAKE_ASSERT(output_port < this->get_num_output_ports());
  // Only has direct feedthrough to the pose bundle output.
  return output_port == bundle_port_index_;
}

template <typename T>
QueryHandle<T> GeometrySystem<T>::MakeQueryHandle(
    const systems::Context<T>& context) const {
  const GeometryContext<T>* geom_context =
      dynamic_cast<const GeometryContext<T>*>(&context);
  DRAKE_DEMAND(geom_context);
  return QueryHandle<T>(geom_context);
}

template <typename T>
void GeometrySystem<T>::CalcQueryHandle(const Context<T>& context,
                                        QueryHandle<T>* output) const {
  const GeometryContext<T>* geom_context =
      dynamic_cast<const GeometryContext<T>*>(&context);
  DRAKE_DEMAND(geom_context);
  output->context_ = geom_context;
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
const GeometryContext<T>& GeometrySystem<T>::FullPoseUpdate(
    const QueryHandle<T>& handle) const {
  // TODO(SeanCurtis-TRI): Update this when the cache is available.
  // This method is const, the handle is const and the context that is contained
  // in the handle is const. Ultimately, this will pull cached entities to do
  // the query work. For now, we have to const cast the thing so that we can
  // update the geometry engine contained.

  using std::to_string;

  const GeometryContext<T>& g_context = *handle.context_;
  const GeometryState<T>& state = g_context.get_geometry_state();
  GeometryState<T>& mutable_state = const_cast<GeometryState<T>&>(state);

  for (const auto& pair : state.source_frame_id_map_) {
    if (pair.second.size() > 0) {
      SourceId source_id = pair.first;
      const auto itr = input_source_ids_.find(source_id);
      DRAKE_ASSERT(itr != input_source_ids_.end());
      const int id_port = itr->second.id_port;
      if (id_port >= 0) {
        const FrameIdVector& ids =
            this->template EvalAbstractInput(g_context, id_port)
                ->template GetValue<FrameIdVector>();
        // TODO(SeanCurtis-TRI): Consider only doing this in debug builds.
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
void GeometrySystem<T>::ThrowIfContextAllocated(
    const char* source_method) const {
  if (context_allocated_)
    throw std::logic_error(
        "The call to " + std::string(source_method) + " is invalid; a "
        "context has already been allocated.");
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
    throw std::logic_error("Can't acquire input port for unknown source id: "
                               + to_string(id) + ".");
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
    default:
      // This is here because gcc fails to recognize that all enumerations have
      // been covered.
      throw std::runtime_error(
          "All enum values have been listed; this should not be reached!");
  }
}

// Explicitly instantiates on the most common scalar types.
template class GeometrySystem<double>;

}  // namespace geometry
}  // namespace drake
