#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/multibody/multibody_tree/math/spatial_velocity.h"

namespace drake {
namespace geometry {

// NOTE: These classes don't exist yet in the multibody namespace yet. These are
// dummy placeholders to simply allow the declarations below to be "valid".
class SpatialPose {

};

class SpatialAcceleration {

};

/**
 Represents the kinematics data for a set of declared geometry frames. It serves
 as the value object that transports frame kinematics from an upstream owner to
 GeometryWorld. The owner of the frames uses the FrameId values provided during
 frame declaration to set the kinematics values for each frame.

 Attempts to set a frame with an invalid id, will be considered an error.
 Frames that are *not* explicitly set will maintain current position with
 zero velocity and zero acceleration.
 */
class FrameKinematicsSet {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FrameKinematicsSet)

  /** Clears all of the kinematics data. */
  void Clear();

  /** Sets the pose kinematics data for the indicated frame, `F`.
   If the frame identifier does not map to a known frame in this set, an
   exception will be thrown.
   @param frame_id    The identifier for the target frame `F`.
   @param X_WF        The pose fo the frame `F` in the world frame `W`.
   */
  void SetFramePose(FrameId frame_id, const SpatialPose& X_WF);

  /** Sets the pose and velocity kinematics data for the indicated frame, `F`.
   If the frame identifier does not map to a known frame in this set, an
   exception will be thrown.
   @param frame_id    The identifier for the target frame `F`.
   @param X_WF        The pose fo the frame `F` in the world frame `W`.
   @param V_WF        The velocity of frame `F` measured and expressed in `W`.
   */
  void SetFrameVelocity(FrameId frame_id, const SpatialPose& X_WF,
                        const SpatialVelocity& V_WF);

  /** Sets the pose and velocity kinematics data for the indicated frame, `F`.
   If the frame identifier does not map to a known frame in this set, an
   exception will be thrown.
   @param frame_id    The identifier for the target frame `F`.
   @param X_WF        The pose fo the frame `F` in the world frame `W`.
   @param V_WF        The velocity of frame `F` measured and expressed in `W`.
   @param A_WF        The acceleration of frame `F` measured and expressed in
                      `W`.
   */
  void SetFrameFullKinematics(FrameId frame_id, const SpatialPose& X_WF,
                              const drake::multibody::SpatialVelocity& V_WF,
                              const SpatialAcceleration& A_WF);

  /** Reports the identifier for the channel from which this kinematics data
   comes from. */
  ChannelId get_channel() const { return id_; }

  // Only allow GeometryChannel to construct the set.
  friend class GeometryChannel;

 private:

  // Private constructor disables arbitrary construction; only a GeometryChannel
  // can create one -- tying the set to the originating channel.
  FrameKinematicsSet(ChannelId id);

  // Sets the channel id to which this data set belongs.
  void set_channel(ChannelId id) { id_ = id; }

  // Specifies the state of any particular state's
  enum class KinematicsWriteState {
    UNWRITTEN,
    POSE,
    POSE_VELOCITY,
    ALL
  };

  // A version value to facilitate maintaining synchronization between a
  // persisted set and the context. Structural changes to the membership of a
  // set lead to advances in version number.
  int64_t version_{0};

  // A map from frame id to its position in the corresponding vectors. For 'N'
  // declared frames, there should be `N` entries.  Every FrameId returned by
  // GeometryChannel::DeclareFrame() should be stored here. It is a bijection
  // from that set of FrameIds to the sequence [0, N-1].
  std::unordered_map<FrameId, int> frame_indices_;

  // The write state of each frame. It is set to UNWRITTEN in Clear() and set to
  // the appropriate value for each of the three Set...() methods.
  std::vector<KinematicsWriteState> frame_write_state_;

  // The poses for the declared frames.  'N' poses for 'N' declared frames.
  std::vector<SpatialPose> poses_;

  // The velocities for the declared frames.  'N' poses for 'N' declared frames.
  std::vector<drake::multibody::SpatialVelocity> velocities_;

  // The accelerations for the declared frames.  'N' poses for 'N' declared
  // frames.
  std::vector<SpatialAcceleration> accelerations_;
};

}  // namespace geometry
}  // namespace drake
