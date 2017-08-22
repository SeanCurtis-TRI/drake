#include "drake/geometry/frame_kinematics_vector.h"

#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {
namespace test {

using std::move;
using std::vector;

// Because FrameKinematicsVector inherits from std::vector, we assume that class
// has been propertly tested. We limit the testing to the public constructors
// and singel additional field.

GTEST_TEST(FrameKinematicsVector, DefaultConstructor) {
  SourceId source_id = SourceId::get_new_id();

  FramePoseVector<double> pose_set(source_id);

  EXPECT_EQ(pose_set.get_source_id(), source_id);
  EXPECT_EQ(pose_set.size(), 0);
  EXPECT_EQ(pose_set.begin(), pose_set.end());
}

GTEST_TEST(FrameKinematicsVector, CopyConstructor) {
  SourceId source_id = SourceId::get_new_id();
  std::vector<Isometry3<double>> poses;
  int kPoseCount = 5;
  for (int i = 0; i < kPoseCount; ++i) {
    Isometry3<double> pose = Isometry3<double>::Identity();
    pose.translation() << i, i, i;
    poses.push_back(pose);
  }

  FramePoseVector<double> pose_set(source_id, poses);

  EXPECT_EQ(pose_set.get_source_id(), source_id);
  EXPECT_EQ(poses.size(), kPoseCount);
  EXPECT_EQ(pose_set.size(), kPoseCount);
  for (int i = 0; i < kPoseCount; ++i) {
    Vector3<double> pose{i, i, i};
    EXPECT_EQ(pose_set[i].translation(), pose);
  }
}

GTEST_TEST(FrameKinematicsVector, MoveConstructor) {
  SourceId source_id = SourceId::get_new_id();
  std::vector<Isometry3<double>> poses;
  int kPoseCount = 5;
  for (int i = 0; i < kPoseCount; ++i) {
    Isometry3<double> pose = Isometry3<double>::Identity();
    pose.translation() << i, i, i;
    poses.push_back(pose);
  }

  FramePoseVector<double> pose_set(source_id, move(poses));

  EXPECT_EQ(pose_set.get_source_id(), source_id);
  EXPECT_EQ(poses.size(), 0);
  EXPECT_EQ(pose_set.size(), kPoseCount);
  for (int i = 0; i < kPoseCount; ++i) {
    Vector3<double> pose{i, i, i};
    EXPECT_EQ(pose_set[i].translation(), pose);
  }
}

}  // namespace test
}  // namespace geometry
}  // namespace drake
