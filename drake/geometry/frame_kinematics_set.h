#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/multibody/multibody_tree/math/spatial_velocity.h"
#include "drake/geometry/spatial_pose.h"

namespace drake {
namespace geometry {

// NOTE: These classes don't exist yet in the multibody namespace yet. These are
// dummy place-holders to simply allow the declarations below to be "valid".

template <typename T>
class SpatialAcceleration {};

// forward declaration
template <typename T> class GeometryWorld;

/**
 Represents the kinematics data for a set of registered geometry frames. It
 serves as the value object that transports frame kinematics values from an
 upstream owner to GeometryWorld. The owner of the frames uses the FrameId
 values provided during frame registration to set the kinematics values for each
 frame. Generally, the kinematics values consist of pose, velocity, and
 acceleration.

 __Usage__

 _Allocating a SystemOutput_: The source system can request the creation of a
 %FrameKinematicsSet for its output port. It does so by providing its SourceId
 value (see GeometryWorld::GetFrameKinematicsSet()).

 _Calculating the output value_: In LeafSystem::DoCalcOutput(), acquire a
 reference to the %FrameKinematicsSet in the SystemOutput. Immediately clear it
 by calling FrameKinematicsSet::Clear(). Subsequently, define the kinematics
 values for all registered frames.

 The following are considered mis-use and will cause errors to be thrown:
   1. Defining kinematics values for frame identifiers which were _not_
   registered on this set's SourceId. This won't cause an exception in the
   invocation of the reporting method, but downstream in GeometryWorld.
   2. Attempting to set the kinematics values for the same frame id twice. The
   reporting functions will throw an exception (but only in Debug build).
   3. For the report interface that take vectors of data, if the size of the
   vectors disagree, an exception is thrown upon method invocation.
   4. Pose is _always_ required, but velocity and acceleration are optional. For
   any frame, the user can report pose, pose and velocity, or pose, velocity,
   and acceleration. (Pose and acceleration _only_ is not a valid option.)
   However, the _same_ types of data must be reported for all frames in the set.
   Failure to do so will produce an error upon invocation.

 @tparam T The underlying scalar type. Must be a valid Eigen scalar.
 */
template <typename T>
class FrameKinematicsSet {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FrameKinematicsSet)

  /** Clears all of the kinematics data in preparation for setting data for
   _all_ of the set's frames. */
  void Clear();

  /** Reports the pose kinematics data for a single frame, `F`, indicated by its
   frame id.
   @param frame_id    The identifier for the target frame `F`.
   @param X_WF        The pose of frame `F` relative to the world frame `W`.
   @returns  The total number of frames defined in the set (after adding this
             frame.
   @throws std::logic_error
       1. values have already been provided for the given `frame_id` -- only in
       Debug build, or
       2. previously reported frames (since the last invocation of Clear()) have
       velocity or acceleration values.
   */
  int ReportPose(FrameId frame_id, const SpatialPose<T> &X_WF);

  /** Reports the pose values for a set of frames, {`Fᵢ`}.  It is assumed that
   the iᵗʰ pose belongs to the iᵗʰ frame identifier.
   @param frame_ids   The identifier for the target frames `Fᵢ`.
   @param X_WF        The poses of frames `Fᵢ` relative to the world frame `W`.
   @returns  The total number of frames defined in the set (after adding these
             frames.
   @throws std::logic_error
       1. values have already been provided for any of the given frame ids --
       only in Debug build,
       2. previously reported frames (since the last invocation of Clear()) have
       velocity or acceleration values, or
       3. there are not an equal number of frame ids as pose values.
   */

  int ReportPoses(const std::vector<FrameId>& frame_ids,
                  const std::vector<SpatialPose<T>>& X_WF);

  /** Reports the pose and velocity values for the indicated frame, `F`.
   @param frame_id    The identifier for the target frame `F`.
   @param X_WF        The pose of the frame `F` in the world frame `W`.
   @param V_WF        The velocity of frame `F` measured and expressed in `W`.
   @returns  The total number of frames defined in the set (after adding this
             frame.
   @throws std::logic_error
       1. values have already been provided for any of the given `frame_id` --
       only in Debug build, or
       2. previously reported frames (since the last invocation of Clear()) have
       have a different set of values (i.e., pose only, or pose, velocity, and
       acceleration).
   */
  int ReportPoseVelocity(FrameId frame_id, const SpatialPose<T> &X_WF,
                         const drake::multibody::SpatialVelocity<T> &V_WF);

  /** Reports the pose and velocity values for a set of frames, {`Fᵢ`}. It is
   assumed that the iᵗʰ pose and velocity belong with the iᵗʰ frame identifier.
   @param frame_ids   The identifier for the target frames `Fᵢ`.
   @param X_WF        The poses of frames `Fᵢ` relative to the world frame `W`.
   @param V_WF        The velocities of frames `Fᵢ` measured and expressed in
                      `W`.
   @returns  The total number of frames defined in the set (after adding these
             frames.
   @throws std::logic_error
       1. values have already been provided for any of the given `frame_id` --
       only in Debug build, or
       2. previously reported frames (since the last invocation of Clear()) have
       have a different set of values (i.e., pose only, or pose, velocity, and
       acceleration), or
       3. there are not an equal number of frame ids as pose or velocity values.
   */
  int ReportPosesVelocities(
      const std::vector<FrameId> &frame_ids,
      const std::vector<SpatialPose<T>> &X_WF,
      const std::vector<drake::multibody::SpatialVelocity<T>> &V_WF);

  /** Reports the pose, velocity, and acceleration values for the indicated
   frame, `F`.
   @param frame_id    The identifier for the target frame `F`.
   @param X_WF        The pose of the frame `F` in the world frame `W`.
   @param V_WF        The velocity of frame `F` measured and expressed in `W`.
   @param A_WF        The acceleration of frame `F` measured and expressed in
                      `W`.
   @returns  The total number of frames defined in the set (after adding this
             frame.
   @throws std::logic_error
       1. values have already been provided for any of the given `frame_id` --
       only in Debug build, or
       2. previously reported frames (since the last invocation of Clear()) have
       have a different set of values (i.e., pose only, or pose and velocity).
   */
  int ReportFullKinematics(FrameId frame_id, const SpatialPose<T> &X_WF,
                           const drake::multibody::SpatialVelocity<T> &V_WF,
                           const SpatialAcceleration<T> &A_WF);

  /** Reports the pose, velocity, and acceleration values for a set of frames,
   {`Fᵢ`}. It is assumed that the iᵗʰ pose, velocity, and acceleration belong
   with the iᵗʰ frame identifier.
   @param frame_ids   The identifier for the target frames `Fᵢ`.
   @param X_WF        The poses of frames `Fᵢ` relative to the world frame `W`.
   @param V_WF        The velocities of frames `Fᵢ` measured and expressed in
                      `W`.
   @param A_WF        The accelerations of frames `Fᵢ` measured and expressed in
                      `W`.
   @returns  The total number of frames defined in the set (after adding these
             frames.
   @throws std::logic_error
       1. values have already been provided for any of the given `frame_id` --
       only in Debug build, or
       2. previously reported frames (since the last invocation of Clear()) have
       have a different set of values (i.e., pose only, or pose and velocity),
       or
       3. there are not an equal number of frame ids as pose, velocity or
       acceleration values.
   */
  int ReportFullKinematics(
      const std::vector<FrameId>& frame_ids,
      const std::vector<SpatialPose<T>>& X_WF,
      const std::vector<drake::multibody::SpatialVelocity<T>>& V_WF,
      const std::vector<SpatialAcceleration<T>>& A_WF);

  /** Reports the identifier for the geometry source to which this kinematics
   data applies. */
  SourceId get_source_id() const { return id_; }

  /** Reports the number of frames that have reported values. */
  int get_frame_count() const { return static_cast<int>(frame_ids_.size()); }

  // TODO(SeanCurtis-TRI): This would be an *expensive* way to get each of the
  // frame's kinematics data. Consider improving the look up/bundling kinematics
  // data, etc.
  /** Returns the pose for the identified frame. Throws std::logic_error if
   the identified frame is not in the set. */
  const SpatialPose<T>& GetPose(FrameId frame_id) const;

  // TODO(SeanCurtis-TRI): Determine if this is a good interface.
  /** Return the frames included in the data set. */
  const std::vector<FrameId>& get_frame_ids() const { return frame_ids_; }
//
//  const std::vector<SpatialPose<T>>& get_poses() const { return poses_; }
//  const std::vector<drake::multibody::SpatialVelocity<T>> get_velocities()
//      const {
//    return velocities_;
//  }
//  const std::vector<SpatialAcceleration<T>>& get_accelerations() const {
//    return accelerations_;
//  }

 private:
  // The inferred configuration of what data has been provided per frame. It
  // is reset in clear, and defined by the first report invocation. This
  // provides the mechanism through which we determine that all frames are
  // provided the same amount of data.
  enum class DataConfig {
    INITIALIZED,                  // Awaiting report to infer configuration.
    POSE,                         // Only pose data.
    POSE_VELOCITY,                // Only pose and velocity data.
    POSE_VELOCITY_ACCELERATION,   // All data.
    READ                          // Data has been consumed; no changes without
                                  // invoking clear.
  };

  // Only allow GeometryWorld to construct the set.
  friend class GeometryWorld<T>;

  // Private constructor disables arbitrary construction; only a GeometryWorld
  // can create one. This guarantees that they can only be instantiated for
  // valid source ids.
  explicit FrameKinematicsSet(SourceId id);

  // Throws an exception if the given id is in the defined identifiers already.
  // Helps enforce the idea that the data for a frame can only be set once
  // per "session".
  void ThrowIfFound(FrameId id) const;

  // Throws an exception if any of the ids are in the defined identifiers
  // already, or if frames are duplicated in the vector.
  void ThrowIfFound(const std::vector<FrameId>& ids) const;

  // Throws the exception to report kinematics data has been reported twice for
  // the given frame identifier.
  static void ThrowResetFrameError(FrameId id);

  // Responsible for detecting inconsistency in which values are provided to the
  // frame kinematics set. Throws an exception if not validated.
  void ValidateConfiguration(DataConfig test_config);

  // These methods do the work of saving given data *without* doing any checks.
  // It facilitates eliminating code duplication without performing redundant
  // checks at runtime. There is a *NoCheck version for each public Report*
  // method.
  int ReportPoseNoCheck(FrameId frame_id, const SpatialPose<T> &X_WF);
  int ReportPosesNoCheck(const std::vector<FrameId>& frame_ids,
                              const std::vector<SpatialPose<T>>& X_WF);
  int ReportFrameVelocityNoCheck(
      FrameId frame_id, const SpatialPose<T>& X_WF,
      const drake::multibody::SpatialVelocity<T>& V_WF);
  int ReportPosesVelocitiesNoCheck(
      const std::vector<FrameId>& frame_ids,
      const std::vector<SpatialPose<T>>& X_WF,
      const std::vector<drake::multibody::SpatialVelocity<T>>& V_WF);
  int ReportFullKinematicsNoCheck(
      FrameId frame_id, const SpatialPose<T>& X_WF,
      const drake::multibody::SpatialVelocity<T>& V_WF,
      const SpatialAcceleration<T>& A_WF);
  int ReportFullKinematicsNoCheck(
      const std::vector<FrameId>& frame_ids,
      const std::vector<SpatialPose<T>>& X_WF,
      const std::vector<drake::multibody::SpatialVelocity<T>>& V_WF,
      const std::vector<SpatialAcceleration<T>>& A_WF);

  // The channel id of the channel to which this set belongs.
  SourceId id_;

  // The configuration of the reported kinematics data. Used to enforce
  // consistency.
  DataConfig configuration_{DataConfig::INITIALIZED};

  // These four vectors represent a struct of arrays. Four pieces of data
  // implicitly coordinated by their index in the vectors. The data includes:
  //  1. Frame identifier
  //  2. Pose
  //  3. Velocity
  //  4. Acceleration
  // As an invariant, there should always be an equal number of frame
  // identifiers as pose values, called `N`. There should either be 0 or `N`
  // velocity values and. The count of acceleration values should be `N` or
  // the number of velocity values. The point is that velocity and acceleration
  // are optional, but for any value to be provided, all lower-order values must
  // also be provided.

  std::vector<FrameId> frame_ids_;

  std::vector<SpatialPose<T>> poses_;

  std::vector<drake::multibody::SpatialVelocity<T>> velocities_;

  std::vector<SpatialAcceleration<T>> accelerations_;
};

}  // namespace geometry
}  // namespace drake
