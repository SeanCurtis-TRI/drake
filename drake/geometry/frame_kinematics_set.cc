#include "drake/geometry/frame_kinematics_set.h"

#include <string>
#include <unordered_set>

#include "drake/common/drake_assert.h"

namespace drake {
namespace geometry {

using drake::multibody::SpatialVelocity;
using std::unordered_set;
using std::vector;
using drake::multibody::SpatialVelocity;

template <typename T>
FrameKinematicsSet<T>::FrameKinematicsSet(SourceId id) : id_(id) {}

template <typename T>
void FrameKinematicsSet<T>::Clear() {
  frame_ids_.clear();
  poses_.clear();
  velocities_.clear();
  accelerations_.clear();
  configuration_ = DataConfig::INITIALIZED;
}

template <typename T>
int FrameKinematicsSet<T>::ReportPose(FrameId frame_id,
                                      const SpatialPose<T> &X_WF) {
  // Test invariants:
  //  1. No duplicate frames.
  //  2. All previous data has only pose data.
  DRAKE_ASSERT_VOID(ThrowIfFound(frame_id));
  ValidateConfiguration(DataConfig::POSE);
  return ReportPoseNoCheck(frame_id, X_WF);
}

template <typename T>
int FrameKinematicsSet<T>::ReportPoses(
    const vector<FrameId>& frame_ids,
    const vector<SpatialPose<T>>& X_WF) {
  // Test invariants:
  //  1. No duplicate frames.
  //  2. All previous data has only pose data.
  //  3. Input data has matching sizes.
  DRAKE_ASSERT_VOID(ThrowIfFound(frame_ids));
  if (frame_ids.size() != X_WF.size()) {
    throw std::logic_error("Mismatch in frame id and pose counts: " +
    std::to_string(frame_ids.size()) + " frames vs. " +
    std::to_string(X_WF.size()) + " poses.");
  }
  ValidateConfiguration(DataConfig::POSE);
  return ReportPosesNoCheck(frame_ids, X_WF);
}

template <typename T>
int FrameKinematicsSet<T>::ReportPoseVelocity(FrameId frame_id,
                                              const SpatialPose<T> &X_WF,
                                              const SpatialVelocity<T> &V_WF) {
  // Test invariants:
  //  1. No duplicate frames.
  //  2. All previous data has only pose and velocity data.
  DRAKE_ASSERT_VOID(ThrowIfFound(frame_id));
  ValidateConfiguration(DataConfig::POSE_VELOCITY);
  return ReportFrameVelocityNoCheck(frame_id, X_WF, V_WF);
}

template <typename T>
int FrameKinematicsSet<T>::ReportPosesVelocities(
    const vector<FrameId>& frame_ids,
    const vector<SpatialPose<T>>& X_WF,
    const vector<SpatialVelocity<T>>& V_WF) {
  // Test invariants:
  //  1. No duplicate frames.
  //  2. All previous data has only pose and velocity data.
  //  3. Input data has matching sizes.
  DRAKE_ASSERT_VOID(ThrowIfFound(frame_ids));
  if (frame_ids.size() != X_WF.size() ||
      frame_ids.size() != V_WF.size()) {
    throw std::logic_error("Mismatch in input counts: " +
        std::to_string(frame_ids.size()) + " frames, " +
        std::to_string(X_WF.size()) + " poses, and " +
        std::to_string(V_WF.size()) + " velocities.");
  }
  ValidateConfiguration(DataConfig::POSE_VELOCITY);
  return ReportPosesVelocitiesNoCheck(frame_ids, X_WF, V_WF);
}

template <typename T>
int FrameKinematicsSet<T>::ReportFullKinematics(
    FrameId frame_id, const SpatialPose<T> &X_WF,
    const SpatialVelocity<T> &V_WF, const SpatialAcceleration<T> &A_WF) {
  // Test invariants:
  //  1. No duplicate frames.
  //  2. All previous data has all data.
  DRAKE_ASSERT_VOID(ThrowIfFound(frame_id));
  ValidateConfiguration(DataConfig::POSE_VELOCITY_ACCELERATION);
  return ReportFullKinematicsNoCheck(frame_id, X_WF, V_WF, A_WF);
}

template <typename T>
int FrameKinematicsSet<T>::ReportFullKinematics(
    const vector<FrameId>& frame_ids,
    const vector<SpatialPose<T>>& X_WF,
    const vector<SpatialVelocity<T>>& V_WF,
    const vector<SpatialAcceleration<T>>& A_WF) {
  // Test invariants:
  //  1. No duplicate frames.
  //  2. I must have the same number of accelerations as frames.
  //  3. Input data has matching sizes.
  DRAKE_ASSERT_VOID(ThrowIfFound(frame_ids));
  if (frame_ids.size() != X_WF.size() ||
      frame_ids.size() != V_WF.size() ||
      frame_ids.size() != A_WF.size()) {
    throw std::logic_error("Mismatch in input counts: " +
        std::to_string(frame_ids.size()) + " frames, " +
        std::to_string(X_WF.size()) + " poses " +
        std::to_string(V_WF.size()) + " velocities, and " +
        std::to_string(A_WF.size()) + " accelerations.");
  }
  ValidateConfiguration(DataConfig::POSE_VELOCITY_ACCELERATION);
  return ReportFullKinematicsNoCheck(frame_ids, X_WF, V_WF, A_WF);
}

template <typename T>
const SpatialPose<T>& FrameKinematicsSet<T>::GetPose(FrameId frame_id) const {
  using std::to_string;
  // TODO(SeanCurtis-TRI): This needs to be *faster*. This is called *every*
  // time the context changes *per body*.
  for (int i = 0; i < get_frame_count(); ++i) {
    if (frame_ids_[i] == frame_id) return poses_[i];
  }
  throw std::logic_error(
      "Requesting pose for frame id that is not contained in the kinematics "
      "set: " + to_string(frame_id) + ".");
}

template <typename T>
void FrameKinematicsSet<T>::ThrowIfFound(FrameId id) const {
  for (auto old_id : frame_ids_) {
    if (old_id == id) {
      ThrowResetFrameError(id);
    }
  }
}

template <typename T>
void FrameKinematicsSet<T>::ThrowIfFound(const vector<FrameId>& ids) const {
  unordered_set<FrameId> unique_ids(ids.begin(), ids.end());
  if (unique_ids.size() != ids.size()) {
    // TODO(SeanCurtis-TRI): Provide which id(s) are duplicated.
    throw std::logic_error(
        "Provided set of frame identifiers has "
        "duplicate identifiers.");
  }

  // Iterate through the smaller set to test for membership in the larger set.
  if (ids.size() < frame_ids_.size()) {
    unordered_set<FrameId> existing_ids(frame_ids_.begin(), frame_ids_.end());
    for (auto id : unique_ids) {
      if (existing_ids.find(id) != existing_ids.end()) {
        // TODO(SeanCurtis-TRI): Report *all* frames that have already been set,
        // not just the first.
        ThrowResetFrameError(id);
      }
    }
  } else {
    for (auto id : frame_ids_) {
      if (unique_ids.find(id) != unique_ids.end()) {
        // TODO(SeanCurtis-TRI): Report *all* frames that have already been set,
        // not just the first.
        ThrowResetFrameError(id);
      }
    }
  }
}

template <typename T>
void FrameKinematicsSet<T>::ThrowResetFrameError(FrameId id) {
  throw std::logic_error(
      "Attempting to set the values of a frame that has already been "
          "set: " + std::to_string(id.get_value()));
}

template <typename T>
void FrameKinematicsSet<T>::ValidateConfiguration(DataConfig test_config) {
  if (configuration_ != test_config) {
    // They don't match -- potential error.
    if (configuration_ == DataConfig::INITIALIZED) {
      // Not yet initialized. Initialize and move on.
      configuration_ = test_config;
    } else {
      // Inconsistent data configuration!
      auto config_string = [](DataConfig config) {
        switch (config) {
          case DataConfig::INITIALIZED:
            DRAKE_DEMAND(false &&
                         "This should never be evaluated with INITIALIZED!");
          case DataConfig::POSE:
            return std::string("pose");
          case DataConfig::POSE_VELOCITY:
            return std::string("pose and velocity");
          case DataConfig::POSE_VELOCITY_ACCELERATION:
            return std::string("pose, velocity, and acceleration");
          case DataConfig::READ:
            DRAKE_DEMAND(false &&
                         "The frame kinematics set has already been passed "
                         "to geometry world and cannot be modified without "
                         " being cleared first.");
        }
        throw std::runtime_error("This code should *not* be reachable!");
      };
      throw std::logic_error(
          "Inconsistent kinematics data. Attempting to set " +
          config_string(test_config) + " when previous frames were given " +
          config_string(configuration_) + ".");
    }
  }
}

template <typename T>
int FrameKinematicsSet<T>::ReportPoseNoCheck(FrameId frame_id,
                                             const SpatialPose<T>& X_WF) {
  // Insert values
  frame_ids_.push_back(frame_id);
  poses_.push_back(X_WF);
  return static_cast<int>(frame_ids_.size());
}

template <typename T>
int FrameKinematicsSet<T>::ReportPosesNoCheck(
    const std::vector<FrameId>& frame_ids,
    const std::vector<SpatialPose<T>>& X_WF) {
  const int new_count = static_cast<int>(frame_ids_.size() + frame_ids.size());
  frame_ids_.reserve(new_count);
  frame_ids_.insert(frame_ids_.end(), frame_ids.begin(), frame_ids.end());
  poses_.reserve(new_count);
  poses_.insert(poses_.end(), X_WF.begin(), X_WF.end());
  return static_cast<int>(new_count);
}

template <typename T>
int FrameKinematicsSet<T>::ReportFrameVelocityNoCheck(
    FrameId frame_id, const SpatialPose<T>& X_WF,
    const drake::multibody::SpatialVelocity<T>& V_WF) {
  int new_count = ReportPoseNoCheck(frame_id, X_WF);
  velocities_.push_back(V_WF);
  return new_count;
}

template <typename T>
int FrameKinematicsSet<T>::ReportPosesVelocitiesNoCheck(
    const std::vector<FrameId>& frame_ids,
    const std::vector<SpatialPose<T>>& X_WF,
    const std::vector<drake::multibody::SpatialVelocity<T>>& V_WF) {
  int new_count = ReportPosesNoCheck(frame_ids, X_WF);
  velocities_.reserve(new_count);
  velocities_.insert(velocities_.end(), V_WF.begin(), V_WF.end());
  return new_count;
}

template <typename T>
int FrameKinematicsSet<T>::ReportFullKinematicsNoCheck(
    FrameId frame_id, const SpatialPose<T>& X_WF,
    const drake::multibody::SpatialVelocity<T>& V_WF,
    const SpatialAcceleration<T>& A_WF) {
  int new_count = ReportFrameVelocityNoCheck(frame_id, X_WF, V_WF);
  accelerations_.push_back(A_WF);
  return new_count;
}

template <typename T>
int FrameKinematicsSet<T>::ReportFullKinematicsNoCheck(
    const std::vector<FrameId>& frame_ids,
    const std::vector<SpatialPose<T>>& X_WF,
    const std::vector<drake::multibody::SpatialVelocity<T>>& V_WF,
    const std::vector<SpatialAcceleration<T>>& A_WF) {
  int new_count = ReportPosesVelocitiesNoCheck(frame_ids, X_WF, V_WF);
  accelerations_.reserve(new_count);
  accelerations_.insert(accelerations_.end(), A_WF.begin(), A_WF.end());
  return new_count;
}

// Explicitly instantiates on the most common scalar types.
template class FrameKinematicsSet<double>;

}  // namespace geometry
}  // namespace drake
