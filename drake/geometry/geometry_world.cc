#include "drake/geometry/geometry_world.h"

#include <utility>
#include <vector>

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
SourceId GeometryWorld<T>::RegisterNewSource() {
  SourceId id = SourceId::get_new_id();
  sources_.insert(id);
  return id;
}

template <typename T>
bool GeometryWorld<T>::SourceIsRegistered(SourceId id) const {
  return sources_.find(id) != sources_.end();
}

template <typename T>
FrameId GeometryWorld<T>::RegisterFrame(GeometryContext<T>* context,
                                        SourceId source_id) {
  DRAKE_DEMAND(SourceIsRegistered(source_id));
  auto& state = context->get_mutable_state()
      ->template get_mutable_abstract_state<GeometryState<T>>(0);
  FrameId id = state.RequestFrameIdForSource(source_id);
  // TODO(SeanCurtis-TRI): Instantiate frame in data and handle eventual
  // payload.
  return id;
}

template <typename T>
GeometryId GeometryWorld<T>::RegisterGeometry(
    GeometryContext<T>* context, SourceId source_id, FrameId frame_id,
    unique_ptr<GeometryInstance> geometry, const Isometry3<T>& X_FG) {
  DRAKE_DEMAND(SourceIsRegistered(source_id));
  auto& state = context->get_mutable_state()
                    ->template get_mutable_abstract_state<GeometryState<T>>(0);
  GeometryId id = state.RequestGeometryIdForFrame(source_id, frame_id);
  // TODO(SeanCurtis-TRI): Do the work!
  return id;
}

template <typename T>
GeometryId GeometryWorld<T>::RegisterGeometry(
    GeometryContext<T>* context, SourceId source_id, GeometryId geometry_id,
    unique_ptr<GeometryInstance> geometry, const Isometry3<T>& X_FG) {
  DRAKE_DEMAND(SourceIsRegistered(source_id));
  auto& state = context->get_mutable_state()
                    ->template get_mutable_abstract_state<GeometryState<T>>(0);
  auto frame_id = state.GetFrameId(geometry_id);
  // TODO(SeanCurtis-TRI): X_FG must be modified.
  return RegisterGeometry(context, source_id, frame_id, std::move(geometry),
                          X_FG);
}

template <typename T>
GeometryId GeometryWorld<T>::RegisterAnchoredGeometry(
    GeometryContext<T>* context, SourceId source_id,
    unique_ptr<GeometryInstance> geometry, const Isometry3<T>& X_WG) {
  DRAKE_DEMAND(SourceIsRegistered(source_id));
  GeometryId id = GeometryId::get_new_id();
  // TODO(SeanCurtis-TRI): Actually do this work.
  return id;
}

template <typename T>
void GeometryWorld<T>::RemoveSource(GeometryContext<T>* context,
                                    SourceId source_id) {
  DRAKE_DEMAND(SourceIsRegistered(source_id));
  auto& state = context->get_mutable_state()
      ->template get_mutable_abstract_state<GeometryState<T>>(0);
  state.RemoveSource(source_id);
  sources_.erase(source_id);
}

template <typename T>
void GeometryWorld<T>::ClearSource(GeometryContext<T>* context,
                                   SourceId source_id) {
  DRAKE_DEMAND(SourceIsRegistered(source_id));
  auto& state = context->get_mutable_state()
      ->template get_mutable_abstract_state<GeometryState<T>>(0);
  state.ClearSource(source_id);
}

template <typename T>
FrameKinematicsSet<T> GeometryWorld<T>::GetFrameKinematicsSet(SourceId source_id) {
  DRAKE_DEMAND(SourceIsRegistered(source_id));
  FrameKinematicsSet<T> set(source_id);
  return set;
}

template <typename T>
void GeometryWorld<T>::SetFrameKinematics(
    GeometryContext<T>* context,
    const FrameKinematicsSet<T>& frame_kinematics) {
  ValidateKinematicsSet(context, frame_kinematics);
  // TODO(SeanCurtis-TRI): Push data into collision engine.
  // Step 2. Push the kinematics data into the collision engine(s).
}

template <typename T>
vector<unique_ptr<AbstractValue>> GeometryWorld<T>::AllocateAbstractValues() {
  vector<unique_ptr<AbstractValue>> values;
  values.push_back(make_unique<Value<GeometryState<T>>>());
  return values;
}

template <typename T>
void GeometryWorld<T>::ValidateKinematicsSet(
    GeometryContext<T>* context,
    const FrameKinematicsSet<T>& frame_kinematics) {
  using std::to_string;
  auto& state =
      context->get_state().template get_abstract_state<GeometryState<T>>(0);
  SourceId source_id = frame_kinematics.get_source_id();
  auto& frames = state.GetFramesForSource(source_id);
  const int ref_frame_count = static_cast<int>(frames.size());
  if (ref_frame_count != frame_kinematics.get_frame_count()) {
    // TODO(SeanCurtis-TRI): Determine if more specific information is necesary.
    // e.g., which frames are missing/added.
    throw std::logic_error(
        "Disagreement in expected number of frames (" +
        std::to_string(frames.size()) + ") and the given number of frames (" +
        std::to_string(frame_kinematics.get_frame_count()) + ").");
  } else {
    for (auto id : frame_kinematics.get_frame_ids()) {
      if (frames.find(id) == frames.end()) {
        throw std::logic_error(
            "Frame id provided in kinematics data (" + to_string(id) + ") "
            "that does not belong to the source (" + to_string(source_id) + ")."
            " At least one required frame id is also missing.");
      }
    }
  }
}

// Explicitly instantiates on the most common scalar types.
template class GeometryWorld<double>;

}  // namespace geometry
}  // namespace drake
