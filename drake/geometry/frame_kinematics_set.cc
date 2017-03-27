#include "drake/geometry/frame_kinematics_set.h"

#include <sstream>

namespace drake {
namespace geometry {

using drake::multibody::SpatialVelocity;
using std::unordered_map;

template <typename T>
FrameKinematicsSet<T>::FrameKinematicsSet(ChannelId id) : id_(id) {}

template <typename T>
void FrameKinematicsSet<T>::Clear() {
  frame_indices_.clear();
  frame_write_state_.clear();
  poses_.clear();
  velocities_.clear();
  accelerations_.clear();
}

template <typename T>
void FrameKinematicsSet<T>::SetFramePose(FrameId frame_id,
                                         const SpatialPose<T>& X_WF) {
  size_t index = GetIndexOrThrowIfInvalid(frame_id);
  poses_[index] = X_WF;
  frame_write_state_[index] = KinematicsWriteState::POSE;
}

template <typename T>
void FrameKinematicsSet<T>::SetFrameVelocity(FrameId frame_id,
                                             const SpatialPose<T>& X_WF,
                                             const SpatialVelocity<T>& V_WF) {
  size_t index = GetIndexOrThrowIfInvalid(frame_id);
  poses_[index] = X_WF;
  velocities_[index] = V_WF;
  frame_write_state_[index] = KinematicsWriteState::POSE_VELOCITY;
}

template <typename T>
void FrameKinematicsSet<T>::SetFrameFullKinematics(
    FrameId frame_id, const SpatialPose<T>& X_WF,
    const SpatialVelocity<T>& V_WF, const SpatialAcceleration<T>& A_WF) {
  size_t index = GetIndexOrThrowIfInvalid(frame_id);
  poses_[index] = X_WF;
  velocities_[index] = V_WF;
  accelerations_[index] = A_WF;
  frame_write_state_[index] = KinematicsWriteState::ALL;
}

template <typename T>
size_t FrameKinematicsSet<T>::GetIndexOrThrowIfInvalid(FrameId frame_id) const {
  unordered_map<FrameId, size_t>::const_iterator itr =
      frame_indices_.find(frame_id);
  if (itr != frame_indices_.end()) {
    return itr->second;
  }
  std::stringstream ss;
  ss << "Attempting to access a frame (" << frame_id << ") that is not in the"
      " FrameKinematicsSet for channel " << id_ << ".";
  throw std::runtime_error(ss.str());
}

// Explicitly instantiates on the most common scalar types.
template class FrameKinematicsSet<double>;

}  // namespace geometry
}  // namespace drake
