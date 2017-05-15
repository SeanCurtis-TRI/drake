#include "drake/geometry/geometry_query.h"
#include "geometry_engine.h"

namespace drake {
namespace geometry {

template <typename T>
bool GeometryQuery<T>::ComputePairwiseClosestPoints(
    std::vector<NearestPair<T>>* near_points) const {
  return engine_.ComputePairwiseClosestPoints(index_to_id_map_, near_points);
}

template <typename T>
bool GeometryQuery<T>::ComputePairwiseClosestPoints(
    const std::vector<GeometryId>& ids_to_check,
    std::vector<NearestPair<T>>* near_points) const {
  std::vector<GeometryIndex> indices;
  indices.reserve(ids_to_check.size());
  for (const auto id : ids_to_check) {
    indices.push_back(id_to_geometry_map_.at(id).get_engine_index());
  }
  return engine_.ComputePairwiseClosestPoints(
      index_to_id_map_, indices, near_points);
}

template <typename T>
bool GeometryQuery<T>::ComputePairwiseClosestPoints(
    const std::vector<GeometryPair> &pairs,
    std::vector<NearestPair<T>> *near_points) const {
  std::vector<internal::GeometryIndexPair> index_pairs;
  index_pairs.reserve(pairs.size());
  for (const auto pair : pairs) {
    index_pairs.emplace_back(
        id_to_geometry_map_.at(pair.geometry_a).get_engine_index(),
        id_to_geometry_map_.at(pair.geometry_b).get_engine_index()
    );
  }
  return engine_.ComputePairwiseClosestPoints(
      index_to_id_map_, index_pairs, near_points);
}

template <typename T>
bool GeometryQuery<T>::FindClosestGeometry(
    const Eigen::Matrix3Xd& points,
    std::vector<PointProximity<T>>* near_bodies) const {
  return engine_.FindClosestGeometry(index_to_id_map_, points, near_bodies);
}

// Explicitly instantiates on the most common scalar types.
template class GeometryQuery<double>;

}  // namespace geometry
}  // namespace drake
