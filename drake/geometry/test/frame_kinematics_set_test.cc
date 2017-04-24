#include "drake/geometry/frame_kinematics_set.h"

#include <gtest/gtest.h>

#include "drake/geometry/test/expect_error_message.h"
#include "drake/geometry/test/geometry_world_stub.h"

namespace drake {
namespace geometry {
namespace {

using FKSet = FrameKinematicsSet<double>;
using Pose = SpatialPose<double>;
using Velocity = drake::multibody::SpatialVelocity<double>;
using Acceleration = SpatialAcceleration<double>;
using std::vector;
using GWorld = GeometryWorld<double>;

// Core infrastructure for performing tests on the FrameKinematicsSet.
class FrameKinematicsSetTest : public ::testing::Test {
 protected:
  void SetUp() {
    source_id_ = SourceId::get_new_id();
    for (int i = 0; i < kFrameCount; ++i) {
      frames_.push_back(FrameId::get_new_id());
    }
  }

  SourceId source_id_;
  vector<FrameId> frames_;
  constexpr static int kFrameCount = 5;
};

// Confirms that the FrameKinematicsSet is created in the correct configuration.
// No data and with the correct source identifier.
TEST_F(FrameKinematicsSetTest, Constructor) {
  FKSet fks = GWorld::MakeFKS(source_id_);
  EXPECT_EQ(fks.get_source_id(), source_id_);
  EXPECT_EQ(fks.get_frame_count(), 0);
}

// Confirms that pose(s) can be successfully set.
TEST_F(FrameKinematicsSetTest, ReportPose) {
  FKSet fks = GWorld::MakeFKS(source_id_);

  // Add a single pose.
  ASSERT_EQ(fks.get_frame_count(), 0);
  int count = fks.ReportPose(frames_[0], Pose());
  EXPECT_EQ(count, 1);
  EXPECT_EQ(fks.get_frame_count(), 1);

  // Add a set of poses.
  vector<FrameId> ids(++frames_.begin(), frames_.end());
  vector<Pose> poses;
  for (size_t i = 0; i < ids.size(); ++i) {
    poses.push_back(Pose());
  }
  count = fks.ReportPoses(ids, poses);
  EXPECT_EQ(count, static_cast<int>(frames_.size()));
}

// Confirms that pose(s) and velocity(ies) can be set.
TEST_F(FrameKinematicsSetTest, ReportPoseVelocity) {
  FKSet fks = GWorld::MakeFKS(source_id_);

  // Add a single pose.
  ASSERT_EQ(fks.get_frame_count(), 0);
  int count = fks.ReportPoseVelocity(frames_[0], Pose(), Velocity());
  EXPECT_EQ(count, 1);
  EXPECT_EQ(fks.get_frame_count(), 1);

  // Add a set of poses.
  vector<FrameId> ids(++frames_.begin(), frames_.end());
  vector<Pose> poses;
  vector<Velocity> velocities;
  for (size_t i = 0; i < ids.size(); ++i) {
    poses.push_back(Pose());
    velocities.push_back(Velocity());
  }
  count = fks.ReportPosesVelocities(ids, poses, velocities);
  EXPECT_EQ(count, static_cast<int>(frames_.size()));
}

// Confirms that pose(s), velocity(ies), and acceleration(s) can be set.
TEST_F(FrameKinematicsSetTest, ReportAll) {
  FKSet fks = GWorld::MakeFKS(source_id_);

  // Add a single pose.
  ASSERT_EQ(fks.get_frame_count(), 0);
  int count =
      fks.ReportFullKinematics(frames_[0], Pose(), Velocity(), Acceleration());
  EXPECT_EQ(count, 1);
  EXPECT_EQ(fks.get_frame_count(), 1);

  // Add a set of poses.
  vector<FrameId> ids(++frames_.begin(), frames_.end());
  vector<Pose> poses;
  vector<Velocity> velocities;
  vector<Acceleration> accelerations;
  for (size_t i = 0; i < ids.size(); ++i) {
    poses.push_back(Pose());
    velocities.push_back(Velocity());
    accelerations.push_back(Acceleration());
  }
  count = fks.ReportFullKinematics(ids, poses, velocities, accelerations);
  EXPECT_EQ(count, static_cast<int>(frames_.size()));
}

// Confirms that clear removes values and enables new configuration of reported
// data.
TEST_F(FrameKinematicsSetTest, Clear) {
  FKSet fks = GWorld::MakeFKS(source_id_);
  ASSERT_EQ(fks.get_frame_count(), 0);
  int count = fks.ReportPose(frames_[0], Pose());
  ASSERT_EQ(count, 1);
  ASSERT_EQ(fks.get_frame_count(), 1);

  fks.Clear();
  // This attempts to set a different *set* of data on a different frame. Only
  // works if the previous data was successfully cleared.
  ASSERT_EQ(fks.get_frame_count(), 0);
  count = fks.ReportPoseVelocity(frames_[1], Pose(), Velocity());
  ASSERT_EQ(count, 1);
  ASSERT_EQ(fks.get_frame_count(), 1);

  fks.Clear();
  // This attempts to set a different *set* of data on a different frame. Only
  // works if the previous data was successfully cleared.
  ASSERT_EQ(fks.get_frame_count(), 0);
  count =
      fks.ReportFullKinematics(frames_[2], Pose(), Velocity(), Acceleration());
  ASSERT_EQ(count, 1);
  ASSERT_EQ(fks.get_frame_count(), 1);
}

// Confirms that an exception with meaningful message is provided if the user
// tries to provide mismatched kinematics data within a single set.
TEST_F(FrameKinematicsSetTest, MisMatchConfiguration) {
  FKSet fks = GWorld::MakeFKS(source_id_);

  // Case 1: Initialize to pose (p), then try velocity (p & v) and full
  // (p, v, & a).
  int count = fks.ReportPose(frames_[0], Pose());
  EXPECT_EQ(count, 1);
  EXPECT_ERROR_MESSAGE(fks.ReportPoseVelocity(frames_[1], Pose(), Velocity()),
                       std::logic_error,
                       "Inconsistent.+ pose and velocity.+ pose.*");
  EXPECT_ERROR_MESSAGE(
      fks.ReportFullKinematics(frames_[1], Pose(), Velocity(), Acceleration()),
      std::logic_error,
      "Inconsistent.+ pose, velocity, and acceleration.+ pose.*");

  fks.Clear();

  // Case 2: Initialize to p & v, then try to set, p, and pva.
  count = fks.ReportPoseVelocity(frames_[0], Pose(), Velocity());
  EXPECT_EQ(count, 1);
  EXPECT_ERROR_MESSAGE(fks.ReportPose(frames_[1], Pose()), std::logic_error,
                       "Inconsistent.+ pose.+ pose and velocity.*");
  EXPECT_ERROR_MESSAGE(
      fks.ReportFullKinematics(frames_[1], Pose(), Velocity(), Acceleration()),
      std::logic_error,
      "Inconsistent.+ pose, velocity, and acceleration.+ pose and velocity.*");

  fks.Clear();

  // Case 2: Initialize to p, v, &a, then try to set, p, and p & v.
  count =
      fks.ReportFullKinematics(frames_[0], Pose(), Velocity(), Acceleration());
  EXPECT_EQ(count, 1);
  EXPECT_ERROR_MESSAGE(
      fks.ReportPose(frames_[1], Pose()), std::logic_error,
      "Inconsistent.+ pose.+pose, velocity, and acceleration.*");
  EXPECT_ERROR_MESSAGE(
      fks.ReportPoseVelocity(frames_[1], Pose(), Velocity()), std::logic_error,
      "Inconsistent.+pose and velocity.+pose, velocity, and acceleration.*");
}

// Confirms the expected exception and messages when inputs don't have matched
// sizes.
TEST_F(FrameKinematicsSetTest, MisMatchInputSizes) {
  FKSet fks = GWorld::MakeFKS(source_id_);

  vector<Pose> poses;
  vector<Velocity> velocities;
  vector<Acceleration> accelerations;
  // Initialize all data sets to be *1* less than the number of frames.
  for (size_t i = 0; i < frames_.size() - 1; ++i) {
    poses.emplace_back();
    velocities.emplace_back();
    accelerations.emplace_back();
  }

  // NOTE: Do not reorder. The sizes of the data vectors depend on correct
  // execution order.

  // Case 1: |frames| != |poses|.
  EXPECT_ERROR_MESSAGE(
      fks.ReportPoses(frames_, poses),
      std::logic_error,
      "Mismatch.+\\d frames.*\\d poses\\.");

  // Case 2: |frames| == |poses|, != |velocities|.
  poses.emplace_back();
  EXPECT_EQ(frames_.size(), poses.size());
  EXPECT_NE(frames_.size(), velocities.size());
  EXPECT_ERROR_MESSAGE(
      fks.ReportPosesVelocities(frames_, poses, velocities),
      std::logic_error,
      "Mismatch.+\\d frames.*\\d poses.*\\d velocities\\.");

  // Case 3: |frames| != |poses|, == |velocities|.
  poses.pop_back();
  velocities.emplace_back();
  EXPECT_NE(frames_.size(), poses.size());
  EXPECT_EQ(frames_.size(), velocities.size());
  EXPECT_ERROR_MESSAGE(
      fks.ReportPosesVelocities(frames_, poses, velocities),
      std::logic_error,
      "Mismatch.+\\d frames.*\\d poses.*\\d velocities\\.");

  // Case 4: |frames| == |poses|, == |velocities|, != |accelerations|.
  poses.emplace_back();
  EXPECT_EQ(frames_.size(), poses.size());
  EXPECT_EQ(frames_.size(), velocities.size());
  EXPECT_NE(frames_.size(), accelerations.size());
  EXPECT_ERROR_MESSAGE(
      fks.ReportFullKinematics(frames_, poses, velocities, accelerations),
      std::logic_error,
      "Mismatch.+\\d frames.*\\d poses.*\\d velocities.*\\d accelerations\\.");

  // Case 5: |frames| == |poses|, != |velocities|, == |accelerations|.
  velocities.pop_back();
  accelerations.emplace_back();
  EXPECT_EQ(frames_.size(), poses.size());
  EXPECT_NE(frames_.size(), velocities.size());
  EXPECT_EQ(frames_.size(), accelerations.size());
  EXPECT_ERROR_MESSAGE(
      fks.ReportFullKinematics(frames_, poses, velocities, accelerations),
      std::logic_error,
      "Mismatch.+\\d frames.*\\d poses.*\\d velocities.*\\d accelerations\\.");

  // Case 6: |frames| != |poses|, == |velocities|, == |accelerations|.
  poses.pop_back();
  velocities.emplace_back();
  EXPECT_NE(frames_.size(), poses.size());
  EXPECT_EQ(frames_.size(), velocities.size());
  EXPECT_EQ(frames_.size(), accelerations.size());
  EXPECT_ERROR_MESSAGE(
      fks.ReportFullKinematics(frames_, poses, velocities, accelerations),
      std::logic_error,
      "Mismatch.+\\d frames.*\\d poses.*\\d velocities.*\\d accelerations\\.");
}

#ifndef DRAKE_ASSERT_IS_DISARMED
// These tests are only relevant if DRAKE_ASSERT is armed. So, we only build
// and run the tests in that context.

// Confirms that attempting to duplicate a previous frame in ReportPose throws
// the expected exception.
TEST_F(FrameKinematicsSetTest, SinglePoseDuplicatesFrame) {
  FKSet fks = GWorld::MakeFKS(source_id_);
  int count = fks.ReportPose(frames_[0], Pose());
  EXPECT_EQ(count, 1);
  EXPECT_ERROR_MESSAGE(
      {fks.ReportPose(frames_[0], Pose());},
      std::logic_error,
      "Attempting to set the values of a frame that has already been set:.+");
}

// Confirms that attempting to duplicate a previous frame in ReportPoses throws
// the expected exception.
TEST_F(FrameKinematicsSetTest, MultiPoseDuplicatesFrame) {
  FKSet fks = GWorld::MakeFKS(source_id_);
  int count = fks.ReportPose(frames_[0], Pose());
  EXPECT_EQ(count, 1);
  vector<Pose> poses;
  for (size_t i = 0; i < frames_.size(); ++i) {
    poses.emplace_back();
  }
  EXPECT_ERROR_MESSAGE(
      {fks.ReportPoses(frames_, poses);},
      std::logic_error,
      "Attempting to set the values of a frame that has already been set:.+");
}

// Confirms that attempting to duplicate a previous frame in ReportPoseVelocity
// throws the expected exception.
TEST_F(FrameKinematicsSetTest, SingleVelocityDuplicatesFrame) {
  FKSet fks = GWorld::MakeFKS(source_id_);
  int count = fks.ReportPoseVelocity(frames_[0], Pose(), Velocity());
  EXPECT_EQ(count, 1);
  EXPECT_ERROR_MESSAGE(
      {fks.ReportPoseVelocity(frames_[0], Pose(), Velocity());},
      std::logic_error,
      "Attempting to set the values of a frame that has already been set:.+");
}

// Confirms that attempting to duplicate a previous frame in
// ReportPosesVelocities throws the expected exception.
TEST_F(FrameKinematicsSetTest, MultiVelocityDuplicatesFrame) {
  FKSet fks = GWorld::MakeFKS(source_id_);
  int count = fks.ReportPoseVelocity(frames_[0], Pose(), Velocity());
  EXPECT_EQ(count, 1);
  vector<Pose> poses;
  vector<Velocity> velocities;
  for (size_t i = 0; i < frames_.size(); ++i) {
    poses.emplace_back();
    velocities.emplace_back();
  }
  EXPECT_ERROR_MESSAGE(
      {fks.ReportPosesVelocities(frames_, poses, velocities);},
      std::logic_error,
      "Attempting to set the values of a frame that has already been set:.+");
}

// Confirms that attempting to duplicate a previous frame in ReportPoseVelocity
// throws the expected exception.
TEST_F(FrameKinematicsSetTest, SingleFullDuplicatesFrame) {
  FKSet fks = GWorld::MakeFKS(source_id_);
  int count = fks.ReportFullKinematics(frames_[0], Pose(), Velocity(),
                                       Acceleration());
  EXPECT_EQ(count, 1);
  EXPECT_ERROR_MESSAGE(
      {fks.ReportFullKinematics(frames_[0], Pose(), Velocity(),
                                Acceleration());},
      std::logic_error,
      "Attempting to set the values of a frame that has already been set:.+");
}

// Confirms that attempting to duplicate a previous frame in
// ReportPosesVelocities throws the expected exception.
TEST_F(FrameKinematicsSetTest, MultiFullDuplicatesFrame) {
  FKSet fks = GWorld::MakeFKS(source_id_);
  int count = fks.ReportFullKinematics(frames_[0], Pose(), Velocity(),
                                       Acceleration());
  EXPECT_EQ(count, 1);
  vector<Pose> poses;
  vector<Velocity> velocities;
  vector<Acceleration> accelerations;
  for (size_t i = 0; i < frames_.size(); ++i) {
    poses.emplace_back();
    velocities.emplace_back();
    accelerations.emplace_back();
  }
  EXPECT_ERROR_MESSAGE(
      {fks.ReportFullKinematics(frames_, poses, velocities, accelerations);},
      std::logic_error,
      "Attempting to set the values of a frame that has already been set:.+");
}
#endif

}  // namespace
}  // namespace geometry
}  // namespace drake
