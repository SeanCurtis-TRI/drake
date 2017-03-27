#pragma once

#include <unordered_map>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/multibody/multibody_tree/math/spatial_velocity.h"

namespace drake {
namespace geometry {

// NOTE: These classes don't exist yet in the multibody namespace yet. These are
// dummy placeholders to simply allow the declarations below to be "valid".
template <typename T>
class SpatialPose {

};

template <typename T>
class SpatialAcceleration {

};

// Forward declarations.
template <typename T> class GeometryChannel;

/**
 Represents the kinematics data for a set of declared geometry frames. It serves
 as the value object that transports frame kinematics from an upstream owner to
 GeometryWorld. The owner of the frames uses the FrameId values provided during
 frame declaration to set the kinematics values for each frame.

 Attempts to set a frame with an invalid id, will be considered an error.
 Frames that are *not* explicitly set will maintain current position with
 zero velocity and zero acceleration.

 @tparam T The underlying scalar type. Must be a valid Eigen scalar.
 */
template <typename T>
class FrameKinematicsSet {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FrameKinematicsSet)

  // TODO(SeanCurtis-TRI): Is this method strictly necessary? Should it be private?
  /** Clears all of the kinematics data. */
  void Clear();

  /** Sets the pose kinematics data for the indicated frame, `F`.
   If the frame identifier does not map to a known frame in this set, an
   exception will be thrown.
   @param frame_id    The identifier for the target frame `F`.
   @param X_WF        The pose fo the frame `F` in the world frame `W`.
   */
  void SetFramePose(FrameId frame_id, const SpatialPose<T>& X_WF);

  /** Sets the pose and velocity kinematics data for the indicated frame, `F`.
   If the frame identifier does not map to a known frame in this set, an
   exception will be thrown.
   @param frame_id    The identifier for the target frame `F`.
   @param X_WF        The pose fo the frame `F` in the world frame `W`.
   @param V_WF        The velocity of frame `F` measured and expressed in `W`.
   */
  void SetFrameVelocity(FrameId frame_id, const SpatialPose<T>& X_WF,
                        const drake::multibody::SpatialVelocity<T>& V_WF);

  /** Sets the pose and velocity kinematics data for the indicated frame, `F`.
   If the frame identifier does not map to a known frame in this set, an
   exception will be thrown.
   @param frame_id    The identifier for the target frame `F`.
   @param X_WF        The pose fo the frame `F` in the world frame `W`.
   @param V_WF        The velocity of frame `F` measured and expressed in `W`.
   @param A_WF        The acceleration of frame `F` measured and expressed in
                      `W`.
   */
  void SetFrameFullKinematics(FrameId frame_id, const SpatialPose<T>& X_WF,
                              const drake::multibody::SpatialVelocity<T>& V_WF,
                              const SpatialAcceleration<T>& A_WF);

  /** Reports the identifier for the channel from which this kinematics data
   comes from. */
  ChannelId get_channel() const { return id_; }

  // Only allow GeometryChannel to construct the set.
  friend class GeometryChannel<T>;

 private:

  // Private constructor disables arbitrary construction; only a GeometryChannel
  // can create one -- tying the set to the originating channel.
  FrameKinematicsSet(ChannelId id);

  // Sets the channel id to which this data set belongs.
  void set_channel(ChannelId id) { id_ = id; }

  // Attempts to get the index for the given frame id, throwing an exception if
  // the frame id given does *not* belong in this set.
  size_t GetIndexOrThrowIfInvalid(FrameId id) const;

  // Specifies the state of any particular state's
  enum class KinematicsWriteState {
    UNWRITTEN = 0,
    POSE = 1,
    POSE_VELOCITY = 3,
    ALL = 7
  };

  // The channel id of the channel to which this set belongs.
  ChannelId id_;

  // A map from frame id to its position in the corresponding vectors. For 'N'
  // declared frames, there should be `N` entries.  Every FrameId returned by
  // GeometryChannel::DeclareFrame() should be stored here. It is a bijection
  // from that set of FrameIds to the sequence [0, N-1].
  std::unordered_map<FrameId, size_t> frame_indices_;

  // The write state of each frame. It is set to UNWRITTEN in Clear() and set to
  // the appropriate value for each of the three Set...() methods.
  std::vector<KinematicsWriteState> frame_write_state_;

  // The poses for the declared frames.  'N' poses for 'N' declared frames.
  std::vector<SpatialPose<T>> poses_;

  // The velocities for the declared frames.  'N' poses for 'N' declared frames.
  std::vector<drake::multibody::SpatialVelocity<T>> velocities_;

  // The accelerations for the declared frames.  'N' poses for 'N' declared
  // frames.
  std::vector<SpatialAcceleration<T>> accelerations_;

  // A version value to facilitate maintaining synchronization between a
  // persisted set and the context. Structural changes to the membership of a
  // set lead to advances in version number.
  int64_t version_{0};
};

}  // namespace geometry
}  // namespace drake
