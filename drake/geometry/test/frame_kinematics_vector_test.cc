#include "drake/geometry/frame_kinematics_vector.h"

#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"

namespace drake {
namespace geometry {
namespace test {

using std::move;
using std::vector;

// Tests the functionality for the FrameKinematicsVector through the
// FramePoseSet alias. This functionality doesn't explicitly depend on the
// data type, so the choice is irrelevant.

// Simply tests successful construction. Implicitly tests size and get_value.
GTEST_TEST(FrameKinematicsVector, ConstructorSuccess) {
  SourceId source_id = SourceId::get_new_id();

  // Case: Empty set.
  FramePoseSet<double> pose_set1(source_id);
  EXPECT_EQ(pose_set1.get_source_id(), source_id);
  EXPECT_EQ(pose_set1.size(), 0);

  // Case: Copy from vector.
  vector<Isometry3<double>> poses{Isometry3<double>::Identity(),
                                  Isometry3<double>::Identity(),
                                  Isometry3<double>::Identity(),
                                  Isometry3<double>::Identity()};
  const int pose_count = static_cast<int>(poses.size());
  FramePoseSet<double> pose_set2(source_id, poses);
  EXPECT_EQ(pose_set2.get_source_id(), source_id);
  EXPECT_EQ(pose_set2.size(), pose_count);
  // Confirms successful copy.
  for (int i = 0; i < pose_count; ++i) {
    EXPECT_TRUE(CompareMatrices(
        pose_set2.get_value(i).matrix().block<3, 4>(0, 0),
        poses[i].matrix().block<3, 4>(0, 0)));
  }

  // Case: Move from vector.
  for (int i = 0; i < pose_count; ++i) {
    poses[i].translation()(0) = i;
  }
  FramePoseSet<double> pose_set3(source_id, move(poses));
  EXPECT_EQ(pose_set3.get_source_id(), source_id);
  EXPECT_EQ(pose_set3.size(), pose_count);
  EXPECT_EQ(poses.size(), 0);
  for (int i = 0; i < pose_count; ++i) {
    EXPECT_EQ(pose_set3.get_value(i).translation()(0), i);
  }
}

// Tests the range iterators.
GTEST_TEST(FrameKinematicsVector, RangeIteration) {
  vector<Isometry3<double>> poses{Isometry3<double>::Identity(),
                                  Isometry3<double>::Identity(),
                                  Isometry3<double>::Identity(),
                                  Isometry3<double>::Identity()};
  FramePoseSet<double> pose_set(SourceId::get_new_id(), poses);
  int i = 0;
  for (auto pose : pose_set) {
    EXPECT_TRUE(CompareMatrices(pose.matrix().block<3, 4>(0, 0),
                                poses[i++].matrix().block<3, 4>(0, 0)));
  }
}

// Tests the functionality for adding single values to the set.
GTEST_TEST(FrameKinematicsVector, AddingValueSingle) {
  FramePoseSet<double> pose_set(SourceId::get_new_id());
  // Do *not* re-order these tests; the logic depends on the sequence.
  // Case: Add single to empty.
  Isometry3<double> pose0 = Isometry3<double>::Identity();
  pose0.translation() << 1, 2, 3;
  int report_count = -1;
  EXPECT_NO_THROW(report_count = pose_set.AddValue(pose0));
  EXPECT_EQ(report_count, 1);
  EXPECT_EQ(pose_set.size(), 1);
  EXPECT_TRUE(CompareMatrices(pose_set.get_value(0).matrix().block<3, 4>(0, 0),
                              pose0.matrix().block<3, 4>(0, 0)));

  // Case: Add single to non-empty (unique).
  Isometry3<double> pose1 = Isometry3<double>::Identity();
  pose1.translation() << 10, 20, 30;
  EXPECT_NO_THROW(report_count = pose_set.AddValue(pose1));
  EXPECT_EQ(report_count, 2);
  EXPECT_EQ(pose_set.size(), 2);
  EXPECT_TRUE(CompareMatrices(pose_set.get_value(1).matrix().block<3, 4>(0, 0),
                              pose1.matrix().block<3, 4>(0, 0)));
}

// Tests the functionality for adding multiple values to the set.
GTEST_TEST(FrameKinematicsVector, AddingValuesMultiple) {
  vector<Isometry3<double>> poses1{Isometry3<double>::Identity(),
                                   Isometry3<double>::Identity(),
                                   Isometry3<double>::Identity(),
                                   Isometry3<double>::Identity()};
  vector<Isometry3<double>> poses2{Isometry3<double>::Identity(),
                                   Isometry3<double>::Identity(),
                                   Isometry3<double>::Identity(),
                                   Isometry3<double>::Identity()};
  // Do *not* re-order these tests; the logic depends on the sequence.

  FramePoseSet<double> pose_set(SourceId::get_new_id());

  // Case: Add multiple to empty (all unique).
  int report_count = -1;
  EXPECT_NO_THROW(report_count = pose_set.AddValues(poses1));
  EXPECT_EQ(report_count, static_cast<int>(poses1.size()));
  EXPECT_EQ(pose_set.size(), report_count);
  for (int i = 0; i < report_count; ++i) {
    EXPECT_TRUE(CompareMatrices(
        pose_set.get_value(i).matrix().block<3, 4>(0, 0),
        poses1[i].matrix().block<3, 4>(0, 0)));
  }

  // Case: Add multiple to non-empty (unique result).
  EXPECT_NO_THROW(report_count = pose_set.AddValues(poses2));
  EXPECT_EQ(report_count, static_cast<int>(poses1.size() + poses2.size()));
  EXPECT_EQ(pose_set.size(), report_count);
  int i = 0;
  for (; i < static_cast<int>(poses1.size()); ++i) {
    EXPECT_TRUE(CompareMatrices(
        pose_set.get_value(i).matrix().block<3, 4>(0, 0),
        poses1[i].matrix().block<3, 4>(0, 0)));
  }
  for (int j = 0; i < report_count; ++i, ++j) {
    EXPECT_TRUE(CompareMatrices(
        pose_set.get_value(i).matrix().block<3, 4>(0, 0),
        poses2[j].matrix().block<3, 4>(0, 0)));
  }
}

// Tests value removal from the set.
GTEST_TEST(FrameKinematicsVector, RemoveValues) {
  vector<Isometry3<double>> poses{Isometry3<double>::Identity(),
                                  Isometry3<double>::Identity(),
                                  Isometry3<double>::Identity(),
                                  Isometry3<double>::Identity()};
  for (size_t i = 0; i < poses.size(); ++i) {
    poses[i].translation()(0) = i;
  }
  FramePoseSet<double> pose_set(SourceId::get_new_id(), move(poses));
  pose_set.RemoveByIndex(0);
  for (size_t i = 0; i < poses.size(); ++i) {
    EXPECT_EQ(pose_set.get_value(i).translation()(0),
              static_cast<double>(i + 1));
  }
}

}  // namespace test
}  // namespace geometry
}  // namespace drake
