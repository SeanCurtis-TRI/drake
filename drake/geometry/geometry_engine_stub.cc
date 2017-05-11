#include "drake/geometry/geometry_engine_stub.h"

#include <memory>
#include <utility>

#include "drake/geometry/geometry_query_inputs.h"

namespace drake {
namespace geometry {

using internal::GeometryIndexPair;
using std::move;
using std::unique_ptr;
using std::vector;

template <typename T>
GeometryEngineStub<T>::GeometryEngineStub() : GeometryEngine<T>() {}

template <typename T>
GeometryIndex GeometryEngineStub<T>::AddDynamicGeometry(unique_ptr<Shape> shape) {
  DRAKE_DEMAND(shape->get_type() == Shape::SPHERE);
  GeometryIndex index(geometries_.size());
  geometries_.emplace_back(move(shape));
  X_WG_.emplace_back();
  DRAKE_ASSERT(X_WG_.size() == geometries_.size());
  return index;
}

template <typename T>
GeometryIndex GeometryEngineStub<T>::AddAnchoredGeometry(
    std::unique_ptr<GeometryInstance<T>> geometry) {
  throw std::runtime_error("Not implemented yet.");
}

template <typename T>
void GeometryEngineStub<T>::RemoveGeometry(GeometryIndex index) {
  // TODO(SeanCurtis-TRI): This should probably have a mechanism for remapping
  // indices to maintain compact representation.
  geometries_[index].reset();
}

template <typename T>
void GeometryEngineStub<T>::UpdateWorldPoses(const vector<Isometry3<T>>& X_WG) {
  X_WG_.clear();
  X_WG_.reserve(X_WG.size());
  X_WG_.insert(X_WG_.begin(), X_WG.begin(), X_WG.end());
}

// Helpers for queries ------------------------------------------------------

// Base class for iterators that have to create the O(N²) combinations of
// geometry indices to operate on. There is a logical set of indices (e.g.,
// {0, 1, 2, 3, ... N-1} and the iterator steps through them as: (0, 1), (0, 2),
// (0, N-1), (1, 2), ...(N-2, N-1). Derivations will change how the input set
// of indices is interpreted.
//
// Sub-classes must define two methods:
//  Pair make_pair() const;
//    - Given the state of the iterator's internal indices, produces a Pair of
//      indices from the input set.
//  int get_set_size() const;
//    - Reports the size of the index set.
template <class Derived>
class pair_iterator {
 public:
  GeometryIndexPair operator*() const {
    return static_cast<const Derived*>(this)->make_pair();
  }
  const Derived& operator++() {
    ++j_;
    if (j_ >= static_cast<const Derived*>(this)->get_set_size()) {
      ++i_;
      j_ = i_ + 1;
    }
    return static_cast<Derived&>(*this);
  }
  bool operator!=(const Derived& other) {
    return i_ != other.i_ && j_ != other.j_;
  }

 protected:
  pair_iterator(int curr_val) : i_(curr_val), j_(curr_val + 1) {}
  int i_{0};   // The index of the first index in the pair.
  int j_{0};   // The index of the second index in the pair.
};

// The RangedPairSet defines the input indices as {0, 1, 2, 3, ... N-1} and
// produces all pairs from (0, 0), to (N-2, N-1).
class RangedPairSet {
 public:
  class iterator : public pair_iterator<iterator> {
    friend class RangedPairSet;

   protected:
    iterator(int curr_val, int limit)
        : pair_iterator(curr_val), set_size_(limit) {}
   public:
    GeometryIndexPair make_pair() const {
      return GeometryIndexPair(GeometryIndex(i_), GeometryIndex(j_));
    }
    int get_set_size() const { return set_size_; }

   private:
    int set_size_{0};
  };

  // Constructor -- given set size N, uses the indices {0, ..., N-1}.
  RangedPairSet(int set_size) : N_(set_size) {}
  iterator begin() const { return iterator(0, N_); }
  iterator end() const { return iterator(N_ - 1, N_); }
  size_t size() const { return (N_ * (N_ - 1)) / 2; }

 private:
  int N_{0};
};

// The ExplicitPairSet generates pairs from an explicitly defined set:
// {a, b, c, d, ..., z} and produces the pairs {a, b}, {a, c}, ..., {y, z}.
class ExplicitPairSet {
 public:
  class iterator : public pair_iterator<iterator> {
    friend class ExplicitPairSet;

   protected:
    iterator(int curr_val, const std::vector<GeometryIndex>& values)
        : pair_iterator(curr_val), values_(values) {}
   public:
    GeometryIndexPair make_pair() const {
      return GeometryIndexPair(values_[i_], values_[j_]);
    }
    int get_set_size() const { return static_cast<int>(values_.size()); }

   private:
    const std::vector<GeometryIndex>& values_;
  };

  ExplicitPairSet(const std::vector<GeometryIndex>& values) : values_(values) {}
  iterator begin() const { return iterator(0, values_); }
  iterator end() const {
    return iterator(static_cast<int>(values_.size()) - 1, values_);
  }
  size_t size() const {
    size_t sz = values_.size();
    return (sz * (sz - 1)) / 2;
  }

 private:
  const std::vector<GeometryIndex>& values_;
};

template <typename T>
template <class PairSet>
bool GeometryEngineStub<T>::ComputePairwiseClosestPointsHelper(
    const PairSet& pair_set,
    std::vector<internal::NearestPair<T>>* near_points) const {
  size_t input = near_points->size();
  near_points->resize(near_points->size() + pair_set.size());
  for (const auto& pair : pair_set) {
    const Sphere* sphere_A =
        static_cast<const Sphere*>(geometries_[pair.index1].get());
    const auto p_WA = X_WG_[pair.index1].translation();
    const double radius_A = sphere_A->get_radius();
    const Sphere* sphere_B =
        static_cast<const Sphere*>(geometries_[pair.index2].get());
    const auto p_WB = X_WG_[pair.index2].translation();
    const double radius_B = sphere_B->get_radius();

    // Compute
    //  r_ACa: the point on A closest to B in A's frame.
    //  r_BCb: The point on B closest to A in B's frame.
    //  rhat_CaCb_W: The unit vector pointing from Ca to Cb.
    //    rhat_CaCb_W is the same as rhat_AB (the unit vector pointing from A's
    //    center to B's center because we are limited to spheres.
    auto r_AB_W = p_WB - p_WA;
    T dist = r_AB_W.norm();
    auto rhat_CaCb_W = r_AB_W / dist;
    auto r_ACa_W = rhat_CaCb_W * radius_A;
    auto r_ACa_A = X_WG_[pair.index1].linear().transpose() * r_ACa_W;
    auto r_BCb_W = rhat_CaCb_W * -radius_B;
    auto r_BCb_B = X_WG_[pair.index2].linear().transpose() * r_BCb_W;
    (*near_points)[input] = internal::NearestPair<T>(
        pair.index1, pair.index2, r_ACa_A, r_BCb_B, dist - radius_A - radius_B);

    ++input;
  }
  return true;
};

// Proximity Queries --------------------------------------------------------

template <typename T>
bool GeometryEngineStub<T>::ComputePairwiseClosestPoints(
    std::vector<internal::NearestPair<T>>* near_points) const {
  return ComputePairwiseClosestPointsHelper(
      RangedPairSet(get_update_input_size()), near_points);
}

template <typename T>
bool GeometryEngineStub<T>::ComputePairwiseClosestPoints(
    const std::vector<GeometryIndex>& ids_to_check,
    std::vector<internal::NearestPair<T>>* near_points) const {
  return ComputePairwiseClosestPointsHelper(ExplicitPairSet(ids_to_check),
                                            near_points);
}

template <typename T>
bool GeometryEngineStub<T>::ComputePairwiseClosestPoints(
    const std::vector<internal::GeometryIndexPair>& pairs,
    std::vector<internal::NearestPair<T>>* near_points) const {
  return ComputePairwiseClosestPointsHelper(pairs, near_points);
}

template <typename T>
bool GeometryEngineStub<T>::FindClosestGeometry(
    const Eigen::Matrix3Xd& points,
    std::vector<internal::PointProximity<T>>* near_bodies) const {
  std::vector<internal::PointProximity<T>>& data = *near_bodies;
  using std::abs;
  DRAKE_ASSERT(near_bodies->size() == 0);
  near_bodies->resize(points.cols());
  for (int i = 0; i < points.cols(); ++i) {
    for (int g = 0; g < get_update_input_size(); ++g) {
      const Sphere& sphere = static_cast<const Sphere&>(*geometries_[g].get());
      const auto& p_WA = X_WG_[g].translation();
      auto r_AQ = points.block<3, 1>(0, i) - p_WA;
      T distance = r_AQ.norm() - sphere.get_radius();
      if (abs(distance) < abs(data[i].distance)) {
        data[i].index_A = g;
        data[i].distance = distance;
        data[i].rhat_CaQ_W = r_AQ;  // defer division until its needed.
      }
    }
  }
  // Nearest bodies have been identified, now collect up the data.
  for (int i = 0; i < points.cols(); ++i) {
    // We know this is the displacement vector and distance we want; normalize.
    if (abs(data[i].distance) < Eigen::NumTraits<T>::dummy_precision()) {
      // TODO(SeanCurtis-TRI): What would be the appropriate response here?
    }
    const Sphere& sphere = static_cast<const Sphere&>(*geometries_[i].get());
    data[i].rhat_CaQ_W /= data[i].distance;
    const auto& p_WA = X_WG_[i].translation();
    auto offset = data[i].rhat_CaQ_W * sphere.get_radius();
    data[i].p_WCa = p_WA + offset;
    data[i].p_ACa = X_WG_[i].linear().transpose() * offset;
  }
  return true;
}

// Explicitly instantiates on the most common scalar types.
template class GeometryEngineStub<double>;

}  // namespace geometry
}  // namespace drake
