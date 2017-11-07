#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/autodiff.h"
#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_engine.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace stub_shapes {

// The index type for getting the "owned" index of an engine shape. This is the
// index of the shape in the stub engine's owned_geometries_ vector.
using OwnedIndex = TypeSafeIndex<class StubOwnedTag>;

/** Base class for shapes used by the stub engine. */
template <typename T>
class EngineShape {
 public:
  // NOTE: This is a simple hack for the stub engine and is *not* intended to
  // reflect the design for a real geometry engine.
  /** Specification of shape type. */
  enum Type {
    kUnknown = 0,
    kSphere,
    kHalfSpace,
  };

  EngineShape(Type type, OwnedIndex index)
      : type_(type), index_(index) {}

  virtual ~EngineShape() {}

  /** Updates the geometry to reflect the current pose in the _world_ frame. */
  virtual void Update(const Isometry3<T>& X_WC) = 0;

  Type get_type() const { return type_; }
  OwnedIndex get_index() const { return index_; }
  void set_index(OwnedIndex index) { index_ = index; }

  std::unique_ptr<EngineShape> Clone() const { return DoClone(); }

  std::unique_ptr<EngineShape<AutoDiffXd>> ToAutoDiff() const {
    return DoToAutoDiff();
  }

 protected:

  virtual std::unique_ptr<EngineShape<T>> DoClone() const = 0;
  virtual std::unique_ptr<EngineShape<AutoDiffXd>> DoToAutoDiff() const = 0;

 private:
  Type type_{kUnknown};
  // The index into the owned array for this shape.
  OwnedIndex index_;
};

/** The engine's representation of a half space. It is defined in its canonical
 frame C lying on the origin with its normal in the +z-axis direction. */
template <typename T>
class EngineHalfSpace final : public EngineShape<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(EngineHalfSpace)

  /** Constructor.
   @param normal    A vector normal to the plane. It points in the "outside"
                    direction. It is assumed to be unit length. It is
                    measured and expressed in the frame F.
   @param r_FC      The position vector from the origin of frame F to the origin
                    of the canonical frame C. */
  EngineHalfSpace(stub_shapes::OwnedIndex index, const Vector3<T>& normal,
                  const Vector3<T>& r_FC)
      : EngineShape<T>(EngineShape<T>::kHalfSpace,
                       index) {
    Update(normal, r_FC);
  }

  /** Constructor. Defines the plane from the given transform. The plane normal
   is the z-axis of isometry's z-axis and the origin, translated by the isometry
   is a point on the plane.
   @param X_FC      The transform from the plane's canonical frame (normal is
                    +z-axis, lying on th origin) to the frame F.
   */
  EngineHalfSpace(stub_shapes::OwnedIndex index, const Isometry3<T>& X_FC)
      : EngineShape<T>(EngineShape<T>::kHalfSpace,
                       index) {
    Update(X_FC);
  }

  /** Constructor for initializing a half space from another half space of
   different type.
   @param source    The source half space to copy from.
   @tparam U  The scalar type of the source. */
  template <typename U>
  EngineHalfSpace(const EngineHalfSpace<U> source)
      : EngineShape<T>(EngineShape<T>::kHalfSpace, source.get_index()),
        normal_(source.normal_.template cast<T>()),
        d_(source.d_) {}

  void Update(const Isometry3<T>& X_WC) override {
    Update(X_WC.linear().col(2), X_WC.translation());
  }

  /** Sets the plane parameters from the given normal and point on the plane.
   @param normal    A vector normal to the plane. It points in the "outside"
                    direction. It is assumed to be unit length. It is
                    measured and expressed in the frame F.
   @param r_FC      The position vector from the origin of frame F to the origin
                    of the canonical frame C. */
  void Update(const Vector3<T>& normal, const Vector3<T>& r_FC) {
    normal_ = normal;
    d_ = -normal_.dot(r_FC);
  }

  /** Reports the signed distance of a point (measured from frame F's origin as
   `r_FP`) to the half-space's plane boundary. Positive values indicate
   *outside* the half-space. It is assumed that the plane has already been
   "posed" in frame F via a call to Update(). */
  T calc_signed_distance(const Vector3<T>& r_FP) const {
    return normal_.dot(r_FP) + d_;
  }

  const Vector3<T>& normal() const { return normal_; }
  T d() const { return d_; };

 protected:
  template <typename>
  friend class EngineHalfSpace;

  std::unique_ptr<EngineShape<T>> DoClone() const override {
    return std::make_unique<EngineHalfSpace>(*this);
  }

  std::unique_ptr<EngineShape<AutoDiffXd>> DoToAutoDiff() const override {
    return std::make_unique<EngineHalfSpace<AutoDiffXd>>(*this);
  }

 private:
  // Defines the implicit equation of the plane: P(x) = dot(N, x) + d
  Vector3<T> normal_;
  T d_{0.0};
};

/** The engine's representation of a sphere. It is defined in its canonical
 frame C centered on the origin with the given circle. */
template <typename T>
class EngineSphere final : public EngineShape<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(EngineSphere)

  EngineSphere(stub_shapes::OwnedIndex index, double radius)
      : EngineShape<T>(EngineShape<T>::kSphere, index), radius_(radius) {}

  /** Constructor for initializing a half space from another half space of
   different type.
   @param source    The source half space to copy from.
   @tparam U  The scalar type of the source. */
  template <typename U>
  EngineSphere(const EngineSphere<U> source)
      : EngineShape<T>(EngineShape<T>::kSphere, source.get_index()),
        radius_(source.radius_) {}

  double radius() const { return radius_; }

  void Update(const Isometry3<T>& X_WC) override {}

 protected:
  // Friends with shapes of all scalar types.
  template <typename>
  friend class EngineSphere;

  std::unique_ptr<EngineShape<T>> DoClone() const override {
    return std::make_unique<EngineSphere>(*this);
  }

  std::unique_ptr<EngineShape<AutoDiffXd>> DoToAutoDiff() const override {
    return std::make_unique<EngineSphere<AutoDiffXd>>(*this);
  }

 private:
  double radius_;
};

}  // namespace stub_shapes

// Forward declaration
template <typename T> class GeometryEngineStubTester;

/** A stub geometry engine that operates only on spheres. This will be my short-
 term solution for getting the GeometryWorld _infrastructure_ up and running
 independent of the underlying engine details.

 @tparam T The scalar type. Must be a valid Eigen scalar.

 Instantiated templates for the following kinds of T's are provided:
 - double
 - AutoDiffXd

 They are already available to link against in the containing library.
 No other values for T are currently supported. */
template <typename T>
class GeometryEngineStub : public GeometryEngine<T>, public ShapeReifier {
 public:
  /** @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
   MoveAssignable */
  //@{

  GeometryEngineStub(const GeometryEngineStub& other);
  GeometryEngineStub& operator=(const GeometryEngineStub& other);
  GeometryEngineStub(GeometryEngineStub&&) = default;
  GeometryEngineStub& operator=(GeometryEngineStub&&) = default;

  //@}

  GeometryEngineStub() = default;

  // Geometry management methods

  int get_update_input_size() const override {
    return static_cast<int>(geometries_.size());
  }

  GeometryIndex AddDynamicGeometry(const Shape& shape) override;

  AnchoredGeometryIndex AddAnchoredGeometry(const Shape& shape,
                                            Isometry3<T> X_WG) override;

  optional<GeometryIndex> RemoveGeometry(GeometryIndex index) override;

  optional<AnchoredGeometryIndex> RemoveAnchoredGeometry(
      AnchoredGeometryIndex index) override;

  void UpdateWorldPoses(const std::vector<Isometry3<T>>& X_WP) override;

  // Proximity query methods
  bool ComputePairwiseClosestPoints(
      const std::vector<GeometryId>& ids,
      std::vector<NearestPair<T>>* near_points) const override;
  bool ComputePairwiseClosestPoints(
      const std::vector<GeometryId>& ids,
      const std::vector<GeometryIndex>& ids_to_check,
      std::vector<NearestPair<T>>* near_points) const override;
  bool ComputePairwiseClosestPoints(
      const std::vector<GeometryId>& ids,
      const std::vector<GeometryIndexPair>& pairs,
      std::vector<NearestPair<T>>* near_points) const override;

  bool FindClosestGeometry(
      const std::vector<GeometryId>& ids,
      const Eigen::Matrix3Xd& points,
      std::vector<PointProximity<T>>* near_bodies) const override;

  std::vector<PenetrationAsPointPair<T>> ComputePenetration(
      const std::vector<GeometryId>& dynamic_map,
      const std::vector<GeometryId>& anchored_map) const override;

  // Cloning and transmogrification
  std::unique_ptr<GeometryEngine<AutoDiffXd>> ToAutoDiff() const override;

  // ShapeReifier interface implementation.
  void ImplementGeometry(const Sphere& sphere) override;
  void ImplementGeometry(const Cylinder& cylinder) override;
  void ImplementGeometry(const HalfSpace& half_space) override;

  std::vector<Isometry3<T>>& get_mutable_poses() { return X_WG_; }

 protected:
  // Friends with shapes of all scalar types.
  template <typename>
  friend class GeometryEngineStub;

  friend class GeometryEngineStubTester<T>;

  // NVI implementation for cloning GeometryEngine instances.
  // @return A _raw_ pointers to the newly cloned GeometryEngine instance.
  GeometryEngineStub* DoClone() const override {
    return new GeometryEngineStub(*this);
  }

 private:
  // The underlying method for executing
  template <class PairSet>
  bool ComputePairwiseClosestPointsHelper(
      const std::vector<GeometryId>& ids,
      const PairSet& pair_set,
      std::vector<NearestPair<T>>* near_points) const;

  template <typename IndexType>
  optional<IndexType> RemoveGeometryHelper(
      IndexType index,
      std::vector<stub_shapes::EngineShape<T>*>* geometries_p) {
    using std::swap;
    std::vector<stub_shapes::EngineShape<T>*>& geometries = *geometries_p;
    IndexType last(static_cast<int>(geometries.size()) - 1);
    if (last != index) {
      swap(geometries[index], geometries[last]);
    }
    stub_shapes::OwnedIndex removed = geometries.back()->get_index();
    swap(owned_geometries_[removed], owned_geometries_.back());
    owned_geometries_[removed]->set_index(removed);
    geometries.pop_back();
    owned_geometries_.pop_back();
    if (last != index) {
      return last;
    } else {
      return {};
    }
  }

  // The set of all owned geometries. It should be an invariant that
  // geometries_.size() + anchored_geometries_.size() =
  //                                                   owned_geometries_.size().
  std::vector<copyable_unique_ptr<stub_shapes::EngineShape<T>>>
      owned_geometries_;
  // The subset of owned_geometries_ which represent dynamic geometries.
  std::vector<stub_shapes::EngineShape<T>*> geometries_;
  // The subset of owned_geometries_ which represent anchored geometries.
  std::vector<stub_shapes::EngineShape<T>*> anchored_geometries_;
  // The world poses for the geometries. It should be an invariant that
  // geometries_.size() == X_WG_.size().
  std::vector<Isometry3<T>> X_WG_;
};
}  // namespace geometry
}  // namespace drake
