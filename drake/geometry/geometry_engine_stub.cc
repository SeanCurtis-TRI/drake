#include "drake/geometry/geometry_engine_stub.h"

#include <limits>
#include <memory>

#include "drake/common/extract_double.h"
#include "drake/geometry/geometry_query_inputs.h"

namespace drake {
namespace geometry {
namespace {

using std::make_unique;
using stub_shapes::EngineShape;
using stub_shapes::EngineSphere;
using stub_shapes::EngineHalfSpace;

// Helper method to compute the contact between two spheres.
template <typename T>
optional<PenetrationAsPointPair<T>> CollideSpheres(
    const EngineSphere<T>& sphere_A, const Vector3<T>& p_WA,
    const EngineSphere<T>& sphere_B, const Vector3<T>& p_WB) {
  // TODO(SeanCurtis-TRI): Rather than pass in positions, update the sphere.
  auto r_AB = p_WB - p_WA;
  T dist_sqd = r_AB.squaredNorm();
  const double separating_dist = sphere_A.radius() + sphere_B.radius();
  const double separating_dist_sqd = separating_dist * separating_dist;
  if (dist_sqd < separating_dist_sqd) {
    // Distance between *centers*!
    T distance = sqrt(dist_sqd);
    if (distance > Eigen::NumTraits<T>::dummy_precision()) {
      PenetrationAsPointPair<T> contact;
      contact.depth = separating_dist - distance;
      contact.nhat_AB_W = r_AB / distance;
      contact.p_WCa = p_WA + contact.nhat_AB_W * sphere_A.radius();
      contact.p_WCb = p_WB - contact.nhat_AB_W * sphere_B.radius();
      return contact;
    }
  }
  return {};
}

template <typename T>
optional<PenetrationAsPointPair<T>> CollideHalfSpace(
    const EngineSphere<T>& sphere, const Vector3<T>& p_WA,
    const EngineHalfSpace<T>& plane) {
  using std::abs;
  T signed_dist = plane.calc_signed_distance(p_WA) - sphere.radius();
  if (signed_dist < 0) {
    PenetrationAsPointPair<T> contact;
    contact.depth = -signed_dist;
    // Penetration direction is *opposite* the plane normal, because the sphere
    // is always considered to be "A" in the (A, B) pair.
    contact.nhat_AB_W = -plane.normal();
    contact.p_WCa = p_WA + contact.nhat_AB_W * sphere.radius();
    contact.p_WCb =
        p_WA - plane.normal() * (sphere.radius() + signed_dist);
    return contact;
  }
  return {};
}

}  // namespace

using std::make_unique;
using std::move;
using std::unique_ptr;
using std::vector;

template <typename T>
GeometryEngineStub<T>::GeometryEngineStub(const GeometryEngineStub<T>& other)
    : GeometryEngine<T>(),
      owned_geometries_(other.owned_geometries_),
      geometries_(),
      anchored_geometries_(),
      X_WG_(other.X_WG_) {
  // Make sure mappings in geometries_ and anchored_geometries_ reflect the
  // newly copied shapes in owned_geometries_.
  geometries_.reserve(other.geometries_.size());
  for (const auto& geometry : other.geometries_) {
    geometries_.push_back(owned_geometries_[geometry->get_index()].get_mutable());
  }

  anchored_geometries_.reserve(other.anchored_geometries_.size());
  for (const auto& geometry : other.anchored_geometries_) {
    anchored_geometries_.push_back(
        owned_geometries_[geometry->get_index()].get_mutable());
  }
}

template <typename T>
GeometryEngineStub<T>& GeometryEngineStub<T>::operator=(
    const GeometryEngineStub<T>& other) {
  owned_geometries_ = other.owned_geometries_;
  X_WG_ = other.X_WG_;

  // Make sure mappings in geometries_ and anchored_geometries_ reflect the
  // newly copied shapes in owned_geometries_.
  geometries_.clear();
  geometries_.reserve(other.geometries_.size());
  for (const auto& geometry : other.geometries_) {
    geometries_.push_back(owned_geometries_[geometry->get_index()].get_mutable());
  }

  anchored_geometries_.clear();
  anchored_geometries_.reserve(other.anchored_geometries_.size());
  for (const auto& geometry : other.anchored_geometries_) {
    anchored_geometries_.push_back(
        owned_geometries_[geometry->get_index()].get_mutable());
  }

  return *this;
}

template <typename T>
GeometryIndex GeometryEngineStub<T>::AddDynamicGeometry(
    const Shape& shape) {
  GeometryIndex index(geometries_.size());
  shape.Reify(this);
  X_WG_.emplace_back();
  auto& last = owned_geometries_.back();
  geometries_.push_back(static_cast<EngineShape<T>*>(last.get_mutable()));
  DRAKE_ASSERT(X_WG_.size() == geometries_.size());
  DRAKE_ASSERT(geometries_.size() + anchored_geometries_.size() ==
               owned_geometries_.size());
  return index;
}

template <typename T>
AnchoredGeometryIndex GeometryEngineStub<T>::AddAnchoredGeometry(
    const Shape& shape, Isometry3<T> X_WG) {
  AnchoredGeometryIndex index(anchored_geometries_.size());
  shape.Reify(this);
  anchored_geometries_.push_back(owned_geometries_.back().get_mutable());
  anchored_geometries_.back()->Update(X_WG);
  DRAKE_ASSERT(geometries_.size() + anchored_geometries_.size() ==
      owned_geometries_.size());
  return index;
}

template <typename T>
optional<GeometryIndex> GeometryEngineStub<T>::RemoveGeometry(
    GeometryIndex index) {
  return RemoveGeometryHelper<GeometryIndex>(index, &geometries_);
}

template <typename T>
optional<AnchoredGeometryIndex> GeometryEngineStub<T>::RemoveAnchoredGeometry(
    AnchoredGeometryIndex index) {
  return RemoveGeometryHelper<AnchoredGeometryIndex>(index,
                                                     &anchored_geometries_);
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
// of indices are interpreted.
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
  explicit pair_iterator(int curr_val) : i_(curr_val), j_(curr_val + 1) {}
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
  explicit RangedPairSet(int set_size) : N_(set_size) {}
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

  explicit ExplicitPairSet(const std::vector<GeometryIndex>& values)
      : values_(values) {}
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
    const std::vector<GeometryId>& ids,
    const PairSet& pair_set,
    std::vector<NearestPair<T>>* near_points) const {
  size_t input = near_points->size();
  near_points->resize(near_points->size() + pair_set.size());
  for (const auto& pair : pair_set) {
    const EngineSphere<T>* sphere_A =
        static_cast<const EngineSphere<T>*>(geometries_[pair.index1]);
    const Vector3<T> p_WA = X_WG_[pair.index1].translation();
    const double radius_A = sphere_A->radius();
    const EngineSphere<T>* sphere_B =
        static_cast<const EngineSphere<T>*>(geometries_[pair.index2]);
    const Vector3<T> p_WB = X_WG_[pair.index2].translation();
    const double radius_B = sphere_B->radius();

    // Compute
    //  r_ACa: the point on A closest to B in A's frame.
    //  r_BCb: The point on B closest to A in B's frame.
    //  rhat_CaCb_W: The unit vector pointing from Ca to Cb.
    //    rhat_CaCb_W is the same as rhat_AB (the unit vector pointing from A's
    //    center to B's center because we are limited to spheres.
    Vector3<T> r_AB_W = p_WB - p_WA;
    T dist = r_AB_W.norm();
    auto rhat_CaCb_W = r_AB_W / dist;
    Vector3<T> r_ACa_W = rhat_CaCb_W * radius_A;
    auto r_ACa_A = X_WG_[pair.index1].linear().transpose() * r_ACa_W;
    auto r_BCb_W = rhat_CaCb_W * -radius_B;
    auto r_BCb_B = X_WG_[pair.index2].linear().transpose() * r_BCb_W;
    (*near_points)[input] = NearestPair<T>(
        ids.at(pair.index1), ids.at(pair.index2), r_ACa_A, r_BCb_B,
        dist - radius_A - radius_B);

    ++input;
  }
  return true;
}

// Proximity Queries --------------------------------------------------------

template <typename T>
bool GeometryEngineStub<T>::ComputePairwiseClosestPoints(
    const std::vector<GeometryId>& ids,
    std::vector<NearestPair<T>>* near_points) const {
  return ComputePairwiseClosestPointsHelper(
      ids, RangedPairSet(get_update_input_size()), near_points);
}

template <typename T>
bool GeometryEngineStub<T>::ComputePairwiseClosestPoints(
    const std::vector<GeometryId>& ids,
    const std::vector<GeometryIndex>& ids_to_check,
    std::vector<NearestPair<T>>* near_points) const {
  return ComputePairwiseClosestPointsHelper(
      ids, ExplicitPairSet(ids_to_check), near_points);
}

template <typename T>
bool GeometryEngineStub<T>::ComputePairwiseClosestPoints(
    const std::vector<GeometryId>& ids,
    const std::vector<GeometryIndexPair>& pairs,
    std::vector<NearestPair<T>>* near_points) const {
  return ComputePairwiseClosestPointsHelper(ids, pairs, near_points);
}

template <typename T>
bool GeometryEngineStub<T>::FindClosestGeometry(
    const std::vector<GeometryId>& ids,
    const Eigen::Matrix3Xd& points,
    std::vector<PointProximity<T>>* near_bodies) const {
  std::vector<PointProximity<T>>& data = *near_bodies;
  std::vector<GeometryIndex> near_geometry(points.cols());
  using std::abs;
  DRAKE_ASSERT(near_bodies->size() == 0);
  near_bodies->resize(points.cols());
  // Implementation note:
  //  This uses a two pass approach. In the first pass, we do the *minimum*
  //  amount of work to find the closest geometry. In the second pass we do the
  //  full work to compute all of the return values.
  for (int i = 0; i < points.cols(); ++i) {
    // TODO(SeanCurtis-TRI): Does this work for autodiff?
    data[i].distance = std::numeric_limits<double>::infinity();
    for (int g = 0; g < get_update_input_size(); ++g) {
      const EngineSphere<T>& sphere =
          static_cast<const EngineSphere<T>&>(*geometries_[g]);
      const auto& p_WA = X_WG_[g].translation();
      Vector3<T> r_AQ = points.block<3, 1>(0, i) - p_WA;
      double radius_sqd = sphere.radius() * sphere.radius();
      T distance = r_AQ.squaredNorm() - radius_sqd;
      if (abs(distance) < abs(data[i].distance)) {
        near_geometry[i] = g;
        data[i].distance = distance;  // defer square root until it's needed.
        data[i].rhat_CaQ_W = r_AQ;  // defer division until it's needed.
      }
    }
  }
  // Nearest bodies have been identified, now do the calculations for the
  // reported data.
  for (int i = 0; i < points.cols(); ++i) {
    data[i].id_A = ids[near_geometry[i]];
    const EngineSphere<T>& sphere =
        static_cast<const EngineSphere<T>&>(*geometries_[i]);
    const auto& p_WA = X_WG_[near_geometry[i]].translation();
    const double radius_sqd = sphere.radius() * sphere.radius();
    data[i].distance =
        sqrt(data[i].distance - radius_sqd) - sphere.radius();

    Vector3<T> offset = Vector3<T>::Zero();
    if (abs(data[i].distance) > Eigen::NumTraits<T>::dummy_precision()) {
      *data[i].rhat_CaQ_W /= data[i].distance;
      offset = *data[i].rhat_CaQ_W * sphere.radius();
    } else {
      data[i].rhat_CaQ_W = {};
    }
    data[i].p_WQa = p_WA + offset;
    data[i].p_AQa = X_WG_[i].linear().transpose() * offset;
  }
  return true;
}

template <typename T>
std::vector<PenetrationAsPointPair<T>> GeometryEngineStub<
    T>::ComputePenetration(const std::vector<GeometryId>& dynamic_map,
                           const std::vector<GeometryId>& anchored_map) const {
  std::vector<PenetrationAsPointPair<T>> contacts;
  // A simple O(N²) algorithm.
  for (int i = 0; i < get_update_input_size(); ++i) {
    const EngineSphere<T>& sphere_A =
        static_cast<const EngineSphere<T>&>(*geometries_[i]);
    // dynamic-anchored collisions.
    for (int a = 0; a < static_cast<int>(anchored_geometries_.size()); ++a) {
      optional<PenetrationAsPointPair<T>> contact;
      switch (anchored_geometries_[a]->get_type()) {
        case EngineShape<T>::kSphere:
          {
            // This implicitly disallows anchored spheres from a penetration
            // calculation perspective.
            DRAKE_DEMAND(false && "This isn't implemented yet");
            const EngineSphere<T>& sphere_B =
                static_cast<const EngineSphere<T>&>(*anchored_geometries_[a]);
            // TODO(SeanCurtis-TRI): I need to get anchored poses. This is a
            // BUG!! I have no guarantees that `a` is a valid index into X_WG_.
            // The simple solution is to put the center back into the sphere and
            // simply grab the center directly.
            contact = CollideSpheres<T>(sphere_A, X_WG_[i].translation(),
                                        sphere_B, X_WG_[a].translation());
          }
          break;
        case EngineShape<T>::kHalfSpace:
          {
          const EngineHalfSpace<T>& half_space =
              static_cast<const EngineHalfSpace<T>&>(*anchored_geometries_[a]);
          contact =
              CollideHalfSpace<T>(sphere_A, X_WG_[i].translation(), half_space);
          }
          break;
        case EngineShape<T>::kUnknown:
          throw std::logic_error("Anchored geometry has unknown type");
      }
      if (contact) {
        (*contact).id_A = dynamic_map[i];
        (*contact).id_B = anchored_map[a];
        contacts.push_back(*contact);
      }
    }
    // dynamic-dynamic collisions.
    // TODO(SeanCurtis-TRI): This should fail badly if there is a halfspace
    // registered. It also won't work with cylinders.
    for (int j = i + 1; j < get_update_input_size(); ++j) {
      const EngineSphere<T>& sphere_B =
          static_cast<const EngineSphere<T>&>(*geometries_[j]);
      auto contact = CollideSpheres<T>(sphere_A, X_WG_[i].translation(),
                                       sphere_B, X_WG_[j].translation());
      if (contact) {
        (*contact).id_A = dynamic_map[i];
        (*contact).id_B = dynamic_map[j];
        contacts.push_back(*contact);
      }
    }
  }
  return contacts;
}

template <typename T>
std::unique_ptr<GeometryEngine<AutoDiffXd>>
GeometryEngineStub<T>::ToAutoDiff() const {
  auto engine_clone = make_unique<GeometryEngineStub<AutoDiffXd>>();
  engine_clone->owned_geometries_.reserve(owned_geometries_.size());
  engine_clone->geometries_.reserve(geometries_.size());
  engine_clone->anchored_geometries_.reserve(anchored_geometries_.size());
  size_t geometry_index = 0;
  size_t anchored_index = 0;
  for (size_t owned_index = 0; owned_index < owned_geometries_.size(); ++owned_index) {
    const auto* source_shape = owned_geometries_[owned_index].get();
    std::unique_ptr<EngineShape<AutoDiffXd>> clone_shape = source_shape->ToAutoDiff();
    engine_clone->owned_geometries_.emplace_back(std::move(clone_shape));
    EngineShape<AutoDiffXd>* shape_ad =
        engine_clone->owned_geometries_.back().get_mutable();
    if (geometry_index < geometries_.size() &&
        source_shape == geometries_[geometry_index++]) {
      engine_clone->geometries_.push_back(shape_ad);
    } else if (anchored_index < anchored_geometries_.size() &&
        source_shape == anchored_geometries_[anchored_index++]) {
      engine_clone->anchored_geometries_.push_back(shape_ad);
    } else {
      DRAKE_ABORT_MSG("Attempting to convert a shape that is neither dynamic "
                      "nor anchored.");
    }
  }

  // Copy geometry poses
  engine_clone->X_WG_.resize(X_WG_.size());
  for (size_t i = 0; i < X_WG_.size(); ++i) {
    // NOTE: Can't assign Isometry3<double> to Isometry3<AutoDiff>. But we *can*
    // assign Matrix<double> to Matrix<AutoDiff>, so that's what we're doing.
    engine_clone->X_WG_[i].matrix() = X_WG_[i].matrix();
  }

  return engine_clone;
}

template <typename T>
void GeometryEngineStub<T>::ImplementGeometry(const Sphere& sphere) {
  stub_shapes::OwnedIndex index(static_cast<int>(owned_geometries_.size()));
  owned_geometries_.emplace_back(std::unique_ptr<EngineSphere<T>>(
      new EngineSphere<T>(index, sphere.get_radius())));
}

template <typename T>
void GeometryEngineStub<T>::ImplementGeometry(const Cylinder&) {
  // TODO(SeanCurtis-TRI): Allow collisions with cylinders by introducing
  // EngineCylinder -- better yet, defer it until I steal it from FCL.
}

template <typename T>
void GeometryEngineStub<T>::ImplementGeometry(const HalfSpace&) {
  stub_shapes::OwnedIndex index(static_cast<int>(owned_geometries_.size()));
  owned_geometries_.emplace_back(make_unique<EngineHalfSpace<T>>(
      index, Vector3<T>{0, 0, 1}, Vector3<T>::Zero()));
}


// Explicitly instantiates on the most common scalar types.
template class GeometryEngineStub<double>;
template class GeometryEngineStub<AutoDiffXd>;

}  // namespace geometry
}  // namespace drake
