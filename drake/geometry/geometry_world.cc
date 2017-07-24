#include "drake/geometry/geometry_world.h"

#include <string>
#include <utility>
#include <vector>

#include "drake/geometry/geometry_context.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_query_inputs.h"
#include "drake/geometry/geometry_query_results.h"
#include "drake/geometry/geometry_state.h"

namespace drake {
namespace geometry {

using drake::systems::AbstractValue;
using drake::systems::AbstractValues;
using drake::systems::Context;
using drake::systems::Value;
using std::make_unique;
using std::unique_ptr;
using std::vector;

template <typename T>
unique_ptr<GeometryContext<T>> GeometryWorld<T>::MakeContext() const {
  auto context = make_unique<GeometryContext<T>>();
  vector<unique_ptr<AbstractValue>> values;
  values.emplace_back(
      make_unique<Value<GeometryState<T>>>(move(CreateState())));
  context->set_abstract_state(make_unique<AbstractValues>(move(values)));
  return context;
}

template <typename T>
int GeometryWorld<T>::get_num_frames(const GeometryContext<T>& context) const {
  return context.get_geometry_state().get_num_frames();
}

template <typename T>
int GeometryWorld<T>::get_num_moving_geometries(
    const GeometryContext<T>& context) const {
  return context.get_geometry_state().get_num_geometries();
}

template <typename T>
SourceId GeometryWorld<T>::RegisterNewSource(GeometryContext<T>* context,
                                             const std::string& name) {
  return context->get_mutable_geometry_state().RegisterNewSource(name);
}

template <typename T>
bool GeometryWorld<T>::SourceIsRegistered(const GeometryContext<T>& context,
                                          SourceId id) const {
  return context.get_geometry_state().source_is_active(id);
}

template <typename T>
FrameId GeometryWorld<T>::RegisterFrame(GeometryContext<T>* context,
                                        SourceId source_id,
                                        const GeometryFrame<T>& frame) {
  return context->get_mutable_geometry_state().RegisterFrame(source_id, frame);
}

template <typename T>
FrameId GeometryWorld<T>::RegisterFrame(GeometryContext<T>* context,
                                        SourceId source_id, FrameId parent_id,
                                        const GeometryFrame<T>& frame) {
  return context->get_mutable_geometry_state().RegisterFrame(source_id,
                                                             parent_id, frame);
}

template <typename T>
GeometryId GeometryWorld<T>::RegisterGeometry(
    GeometryContext<T>* context, SourceId source_id, FrameId frame_id,
    unique_ptr<GeometryInstance<T>> geometry) {
  return context->get_mutable_geometry_state().RegisterGeometry(
      source_id, frame_id, move(geometry));
}

template <typename T>
GeometryId GeometryWorld<T>::RegisterGeometry(
    GeometryContext<T>* context, SourceId source_id, GeometryId geometry_id,
    unique_ptr<GeometryInstance<T>> geometry) {
  return context->get_mutable_geometry_state().RegisterGeometryWithParent(
      source_id, geometry_id, std::move(geometry));
}

template <typename T>
GeometryId GeometryWorld<T>::RegisterAnchoredGeometry(
    GeometryContext<T>* context, SourceId source_id,
    unique_ptr<GeometryInstance<T>> geometry) {
  return context->get_mutable_geometry_state().RegisterAnchoredGeometry(
      source_id, std::move(geometry));
}

template <typename T>
void GeometryWorld<T>::ClearSource(GeometryContext<T>* context,
                                   SourceId source_id) {
  context->get_mutable_geometry_state().ClearSource(source_id);
}

template <typename T>
void GeometryWorld<T>::RemoveFrame(GeometryContext<T>* context,
                                   SourceId source_id, FrameId frame_id) {
  context->get_mutable_geometry_state().RemoveFrame(source_id, frame_id);
}

template <typename T>
void GeometryWorld<T>::RemoveGeometry(GeometryContext<T>* context,
                                      SourceId source_id,
                                      GeometryId geometry_id) {
  context->get_mutable_geometry_state().RemoveGeometry(source_id, geometry_id);
}

template <typename T>
void GeometryWorld<T>::SetFramePoses(GeometryContext<T>* context,
                                     const FrameIdVector& ids,
                                     const FramePoseSet<T>& poses) {
  context->get_mutable_geometry_state().SetFramePoses(ids, poses);
}

template <typename T>
void GeometryWorld<T>::SetFrameVelocities(
    GeometryContext<T>* context, const FrameIdVector& ids,
    const FrameVelocitySet<T>& velocities) {
  context->get_mutable_geometry_state().SetFrameVelocities(ids, velocities);
}

template <typename T>
unique_ptr<GeometryState<T>> GeometryWorld<T>::CreateState() const {
  return make_unique<GeometryState<T>>();
}

template <typename T>
bool GeometryWorld<T>::ComputePairwiseClosestPoints(
    const GeometryContext<T>& context,
    std::vector<NearestPair<T>>* near_points) const {
  const GeometryState<T>& state = context.get_geometry_state();
  return state.geometry_engine_->ComputePairwiseClosestPoints(
      state.geometry_index_id_map_, near_points);
}

template <typename T>
bool GeometryWorld<T>::ComputePairwiseClosestPoints(
    const GeometryContext<T>& context,
    const std::vector<GeometryId>& ids_to_check,
    std::vector<NearestPair<T>>* near_points) const {
  const GeometryState<T>& state = context.get_geometry_state();
  std::vector<GeometryIndex> indices;
  indices.reserve(ids_to_check.size());
  for (const auto id : ids_to_check) {
    indices.push_back(state.geometries_.at(id).get_engine_index());
  }
  return state.geometry_engine_->ComputePairwiseClosestPoints(
      state.geometry_index_id_map_, indices, near_points);
}

template <typename T>
bool GeometryWorld<T>::ComputePairwiseClosestPoints(
    const GeometryContext<T>& context,
    const std::vector<GeometryPair> &pairs,
    std::vector<NearestPair<T>> *near_points) const {
  const GeometryState<T>& state = context.get_geometry_state();
  std::vector<internal::GeometryIndexPair> index_pairs;
  index_pairs.reserve(pairs.size());
  for (const auto& pair : pairs) {
    index_pairs.emplace_back(
        state.geometries_.at(pair.geometry_a).get_engine_index(),
        state.geometries_.at(pair.geometry_b).get_engine_index());
  }
  return state.geometry_engine_->ComputePairwiseClosestPoints(
      state.geometry_index_id_map_, index_pairs, near_points);
}

template <typename T>
bool GeometryWorld<T>::FindClosestGeometry(
    const GeometryContext<T>& context,
    const Eigen::Matrix3Xd& points,
    std::vector<PointProximity<T>>* near_bodies) const {
  const GeometryState<T>& state = context.get_geometry_state();
  return state.geometry_engine_->FindClosestGeometry(
      state.geometry_index_id_map_, points, near_bodies);
}

template <typename T>
bool GeometryWorld<T>::ComputeContact(const GeometryContext<T>& context,
                    std::vector<PenetrationAsPointPair<T>>* contacts) const {
  const GeometryState<T>& state = context.get_geometry_state();
  return state.geometry_engine_->ComputeContact(
      state.geometry_index_id_map_, state.anchored_geometry_index_id_map_,
      contacts);
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
