#pragma once

#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/multibody/multibody_tree/math/spatial_velocity.h"

namespace drake {
namespace geometry {

/** A class representing an ordered collection of kinematics values. The values
 can ultimately be of different types (e.g., pose, velocity, etc.). This single
 class allows all of them to use the same functionality, but still be declared
 on the appropriately typed class to represent the kinematic quantity. This
 won't be invoked directly by other code. Instead, the aliased instantiations
 of FramePoseVector and FrameVelocityVector.
 @tparam KinematicsValue     The representation of the kinematics value. */
template <class KinematicsValue>
class FrameKinematicsVector {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FrameKinematicsVector)

  typedef typename std::vector<KinematicsValue>::const_iterator Iterator;

  /** Empty constructor.
   @param source_id   The id for the geometry source reporting frame kinematics.
   */
  explicit FrameKinematicsVector(SourceId source_id) : source_id_(source_id) {}

  /** Constructor which initializes the values by _copying_ the given data.
   @param source_id   The id for the geometry source reporting frame kinematics.
   @param data        The vector of values which are copied into this vector. */
  FrameKinematicsVector(SourceId source_id,
                        const std::vector<KinematicsValue>& data)
      : source_id_(source_id), data_(data) {}

  /** Constructor which initializes the values by _moving_ them from the
   given set.
   @param source_id   The id for the geometry source reporting frame kinematics.
   @param data        The vector of values which are moved into this vector. */
  FrameKinematicsVector(SourceId source_id,
                        std::vector<KinematicsValue>&& data)
  : source_id_(source_id), data_(std::move(data)) {}

  /** Reports the source id for this data. */
  SourceId get_source_id() const { return source_id_; }

  /** Report the number of values stored in the set. */
  int size() const { return static_cast<int>(data_.size()); }

  /** Returns the iᵗʰ value */
  const KinematicsValue& get_value(int i) const { return data_.at(i); }

  /** Appends the given kinematics value to the set.
   * @param value    The kinematics value to add.
   * @return The total number of values in the set. */
  int AddValue(const KinematicsValue& value) {
    data_.push_back(value);
    return this->size();
  }

  /** Appends the given set of kinematics values to the set.
   @param values  The ordered set of values to append to the current set. */
  int AddValues(const std::vector<KinematicsValue>& values) {
    data_.insert(data_.begin(), values.begin(), values.end());
    return this->size();
  }

  // TODO(SeanCurtis-TRI): I believe I need "set" methods; minimize copying/
  // writing of pose values. This may ultimately depend on how things integrate
  // with the cache.

  /** Simply removes the value at position 'index'. */
  void RemoveByIndex(int index) {
    data_.erase(data_.begin() + index);
  }

  /** @name  Support for range-based loop iteration */
  //@{

  Iterator begin() const { return data_.cbegin(); }
  Iterator end() const { return data_.cend(); }

  // @}

 private:
  // The id of the reporting geometry source.
  SourceId source_id_;
  // The ordered set of moving frame kinematics values.
  std::vector<KinematicsValue> data_;
};

template <typename T>
using FramePoseSet = FrameKinematicsVector<Isometry3<T>>;

template <typename T>
using FrameVelocitySet =
    FrameKinematicsVector<drake::multibody::SpatialVelocity<T>>;

}  // namespace geometry
}  // namespace drake
