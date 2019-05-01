#include "drake/geometry/query_object.h"

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/geometry/scene_graph.h"

namespace drake {
namespace geometry {

template <typename T>
QueryObject<T>::QueryObject(const QueryObject&)
    : context_{nullptr}, scene_graph_{nullptr} {}

template <typename T>
QueryObject<T>& QueryObject<T>::operator=(const QueryObject<T>&) {
  context_ = nullptr;
  scene_graph_ = nullptr;
  return *this;
}

template <typename T>
std::vector<PenetrationAsPointPair<double>>
QueryObject<T>::ComputePointPairPenetration() const {
  ThrowIfDefault();

  // TODO(SeanCurtis-TRI): Modify this when the cache system is in place.
  scene_graph_->FullPoseUpdate(*context_);
  const GeometryState<T>& state = geometry_state();
  return state.ComputePointPairPenetration();
}

template <typename T>
std::vector<SignedDistancePair<T>>
QueryObject<T>::ComputeSignedDistancePairwiseClosestPoints() const {
  ThrowIfDefault();

  // TODO(SeanCurtis-TRI): Modify this when the cache system is in place.
  scene_graph_->FullPoseUpdate(*context_);
  const GeometryState<T>& state = context_->get_geometry_state();
  return state.ComputeSignedDistancePairwiseClosestPoints();
}

template <typename T>
std::vector<SignedDistanceToPoint<T>>
QueryObject<T>::ComputeSignedDistanceToPoint(
    const Vector3<T>& p_WQ,
    const double threshold) const {
  ThrowIfDefault();

  scene_graph_->FullPoseUpdate(*context_);
  const GeometryState<T>& state = context_->get_geometry_state();
  return state.ComputeSignedDistanceToPoint(p_WQ, threshold);
}

template <typename T>
const GeometryState<T>& QueryObject<T>::geometry_state() const {
  // TODO(SeanCurtis-TRI): Handle the "baked" query object case.
  DRAKE_DEMAND(scene_graph_ != nullptr);
  DRAKE_DEMAND(context_ != nullptr);
  return context_->get_geometry_state();
}

}  // namespace geometry
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::QueryObject)
