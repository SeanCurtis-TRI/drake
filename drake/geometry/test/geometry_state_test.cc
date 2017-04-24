#include "drake/geometry/geometry_state.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/frame_kinematics_set.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/test/expect_error_message.h"

namespace drake {
namespace geometry {

// This serves as a _mock_ GeometryWorld. GeometryWorld serves as a factory of
// the FrameKinematicsSet class. It is the _only_ class that can generate them.
// The GeometryState class is responsible for validating the state of a
// FrameKinematicsSet. To test that function, I need access to an instance of
// the FrameKinematicsSet. This mock allows me to create such an instance
// in a very lightweight manner by exploiting a declared friend relationship.
template <typename T>
class GeometryWorld {
 public:
  static FrameKinematicsSet<T> MakeFKS(SourceId s_id) {
    return FrameKinematicsSet<T>(s_id);
  }
};

namespace  {

using GState = GeometryState<double>;
using std::make_unique;
using std::move;
using std::unique_ptr;

class GeometryStateTest : public ::testing::Test {
 protected:
  void SetUp() {
    instance_pose_.translation() << 10, 20, 30;
    instance_ = make_unique<GeometryInstance<double>>(instance_pose_);
  }

  // Utility method for adding a source to the state.
  SourceId NewSource() {
    SourceId s_id = SourceId::get_new_id();
    geometry_state_.RegisterNewSource(s_id);
    return s_id;
  }

  // This method sets up a dummy tree to facilitate testing. It creates a single
  // source with a fixed number of frames and geometries per frame.
  SourceId SetUpSingleSourceTree() {
    SourceId s_id = NewSource();

    // Creates k frames with n geometries each.
    Isometry3<double> pose;
    pose = Isometry3<double>::Identity();
    for (int f = 0; f < kFrameCount; ++f) {
      frames_.push_back(geometry_state_.RegisterFrame(s_id));
      int geometry_position = f * kGeometryCount;
      for (int g = 0; g < kGeometryCount; ++g) {
        // The gᵗʰ geometry has position [ g+1 0 0 ]ᵀ.
        pose.translation() << g + 1, 0, 0;
        geometries_[geometry_position++] = geometry_state_.RegisterGeometry(
            s_id, frames_[f],
            make_unique<GeometryInstance<double>>(pose));
      }
    }
    // Confirms that the same source is reachable from all geometries.
    for (int i = 0; i < kFrameCount * kGeometryCount; ++i) {
      EXPECT_EQ(geometry_state_.GetSourceId(geometries_[i]), s_id);
    }
    return s_id;
  }

  // Reports characteristics of the dummy tree.
  int single_tree_frame_count() const { return kFrameCount; }
  int single_tree_geometry_count() const {
    return kFrameCount * kGeometryCount;
  }

  // This method confirms that the stored dummy identifiers don't map to any
  // active source identifier.
  void AssertSingleTreeCleared() {
    // confirm frames have been closed
    for (int f = 0; f < kFrameCount; ++f) {
      EXPECT_ERROR_MESSAGE(geometry_state_.GetSourceId(frames_[f]),
                           std::logic_error,
                           "Referenced frame \\d+ has not been registered.");
    }
    // confirm geometries have been closed
    for (int g = 0; g < kFrameCount * kGeometryCount; ++g) {
      EXPECT_ERROR_MESSAGE(geometry_state_.GetSourceId(geometries_[g]),
                           std::logic_error,
                           "Referenced geometry \\d+ does not belong to a known"
                           " frame.");
    }
  }

  // Members owned by the test class.
  unique_ptr<GeometryInstance<double>> instance_;
  Isometry3<double> instance_pose_{Isometry3<double>::Identity()};
  GeometryState<double> geometry_state_;

  // Values for setting up and testing the dummy tree.
  enum Counts {
    kFrameCount = 2,
    kGeometryCount = 3
  };
  // The frame ids created in the dummy tree instantiation.
  std::vector<FrameId> frames_;
  // The geometry ids created in the dummy tree instantiation.
  GeometryId geometries_[kFrameCount * kGeometryCount];
};

// Confirms that a new GeometryState has no data.
TEST_F(GeometryStateTest, Constructor) {
  EXPECT_EQ(geometry_state_.get_num_sources(), 0);
  EXPECT_EQ(geometry_state_.GetNumFrames(), 0);
  EXPECT_EQ(geometry_state_.get_num_geometries(), 0);
}

// Tests the geometry statistics values. It uses the single-source tree to
// create a state with interesting metrics. Also confirms the "is active"-ness
// of known valid sources and known invalid sources.
TEST_F(GeometryStateTest, GeometryStatistics) {
  SourceId dummy_source = SetUpSingleSourceTree();
  EXPECT_TRUE(geometry_state_.source_is_active(dummy_source));
  EXPECT_EQ(geometry_state_.get_num_sources(), 1);
  EXPECT_EQ(geometry_state_.GetNumFrames(), single_tree_frame_count());
  EXPECT_EQ(geometry_state_.get_num_geometries(), single_tree_geometry_count());
  SourceId false_id = SourceId::get_new_id();
  EXPECT_FALSE(geometry_state_.source_is_active(false_id));
}

// Tests that an attempt to add a frame to an invalid source throws an exception
// with meaningful message.
TEST_F(GeometryStateTest, AddFrameToInvalidSource) {
  SourceId s_id = SourceId::get_new_id();  // This is not an active source.
  ASSERT_ERROR_MESSAGE(geometry_state_.RegisterFrame(s_id),
                       std::logic_error,
                       "Referenced geometry source \\d+ is not active.");
}

// Tests that a frame added to a valid source can be used to acquire that source
// and appears in the source's frames.
TEST_F(GeometryStateTest, AddFirstFrameToValidSource) {
  SourceId s_id = NewSource();
  FrameId fid = geometry_state_.RegisterFrame(s_id);
  EXPECT_EQ(geometry_state_.GetSourceId(fid), s_id);
  const auto &frame_set = geometry_state_.GetFramesForSource(s_id);
  EXPECT_NE(frame_set.find(fid), frame_set.end());
  EXPECT_EQ(frame_set.size(), 1);
}

// Tests that a frame added to a valid source which already has frames is
// correctly appended.
TEST_F(GeometryStateTest, AddFrameToSourceWithFrames) {
  SourceId s_id = SetUpSingleSourceTree();
  FrameId fid = geometry_state_.RegisterFrame(s_id);
  EXPECT_EQ(geometry_state_.GetSourceId(fid), s_id);
  const auto &frame_set = geometry_state_.GetFramesForSource(s_id);
  EXPECT_NE(frame_set.find(fid), frame_set.end());
  EXPECT_EQ(frame_set.size(), kFrameCount + 1);
}

// Tests that a frame added to a new source doesn't modify previously existing
// sources.
TEST_F(GeometryStateTest, AddFrameToNewSourceWithFrames) {
  SourceId s_id = SetUpSingleSourceTree();
  SourceId new_s_id = SourceId::get_new_id();
  geometry_state_.RegisterNewSource(new_s_id);
  FrameId fid = geometry_state_.RegisterFrame(new_s_id);
  // Confirm addition.
  EXPECT_EQ(geometry_state_.GetSourceId(fid), new_s_id);
  {
    const auto &frame_set = geometry_state_.GetFramesForSource(new_s_id);
    EXPECT_NE(frame_set.find(fid), frame_set.end());
    EXPECT_EQ(frame_set.size(), 1);
  }
  // Confirm original source is unchanged.
  {
    const auto &frame_set = geometry_state_.GetFramesForSource(s_id);
    EXPECT_EQ(frame_set.find(fid), frame_set.end());
    EXPECT_EQ(frame_set.size(), kFrameCount);
  }
}

// Tests the valid removal of an existing frame (and its attached geometry).
TEST_F(GeometryStateTest, RemoveFrame) {
  SourceId s_id = SetUpSingleSourceTree();
  EXPECT_EQ(geometry_state_.GetNumFrames(), kFrameCount);
  EXPECT_EQ(geometry_state_.get_num_geometries(), kFrameCount * kGeometryCount);
  geometry_state_.RemoveFrame(s_id, frames_[0]);
  EXPECT_EQ(geometry_state_.GetNumFrames(), kFrameCount - 1);
  EXPECT_EQ(geometry_state_.get_num_geometries(),
            (kFrameCount -1)* kGeometryCount);
  EXPECT_ERROR_MESSAGE(geometry_state_.GetSourceId(frames_[0]),
                       std::logic_error,
                       "Referenced frame \\d+ has not been registered.")
}

// Tests the response to invalid invocations of RemoveFrame.
TEST_F(GeometryStateTest, RemoveFrameInvalid) {
  SourceId s_id = SetUpSingleSourceTree();

  // Case: Valid source, invalid frame.
  EXPECT_ERROR_MESSAGE(geometry_state_.RemoveFrame(s_id, FrameId::get_new_id()),
                       std::logic_error,
                       "Referenced frame \\d+ has not been registered.");

  // Case: Invalid source, valid frame.
  EXPECT_ERROR_MESSAGE(
      geometry_state_.RemoveFrame(SourceId::get_new_id(), frames_[0]),
      std::logic_error,
      "Trying to remove frame \\d+ from source \\d+.+the frame doesn't "
      "belong.+");

  // Case: Valid source and frame, but frame does _not_ belong to source.
  SourceId s_id2 = SourceId::get_new_id();
  geometry_state_.RegisterNewSource(s_id2);
  FrameId frame_id = geometry_state_.RegisterFrame(s_id2);
  EXPECT_EQ(geometry_state_.GetNumFrames(), kFrameCount + 1);
  EXPECT_ERROR_MESSAGE(
      geometry_state_.RemoveFrame(s_id, frame_id),
      std::logic_error,
      "Trying to remove frame \\d+ from source \\d+.+the frame doesn't "
      "belong.+");
}

// Tests registration of geometry on valid source and frame. This relies on the
// correctness of GeometryState::GetSourceId(GeometryId) and
// GeometryState::GetFrameId(GeometryId) and, therefore, implicitly tests them.
TEST_F(GeometryStateTest, RegisterGeometryGoodSource) {
  SourceId s_id = NewSource();
  FrameId f_id = geometry_state_.RegisterFrame(s_id);
  GeometryId g_id = geometry_state_.RegisterGeometry(s_id, f_id,
                                                     move(instance_));
  EXPECT_EQ(geometry_state_.GetFrameId(g_id), f_id);
  EXPECT_EQ(geometry_state_.GetSourceId(g_id), s_id);
  Isometry3<double> X_FG = geometry_state_.GetPoseInFrame(g_id);
  CompareMatrices(X_FG.matrix(), instance_pose_.matrix());
}

// Tests registration of geometry on valid source and frame. This relies on the
// correctness of GeometryState::GetSourceId(GeometryId) and
// GeometryState::GetFrameId(GeometryId) and, therefore, implicitly tests them.
TEST_F(GeometryStateTest, RegisterGeometryMissingSource) {
  SourceId s_id = SourceId::get_new_id();
  FrameId f_id = FrameId::get_new_id();
  EXPECT_ERROR_MESSAGE(geometry_state_.RegisterGeometry(s_id, f_id,
                                                        move(instance_)),
                       std::logic_error,
                       "Referenced geometry source \\d+ is not active.");
}

// Tests registration of geometry on valid source and non-existant frame.
TEST_F(GeometryStateTest, RegisterGeometryMissingFrame) {
  SourceId s_id = NewSource();

  FrameId f_id = FrameId::get_new_id();
  EXPECT_ERROR_MESSAGE(geometry_state_.RegisterGeometry(s_id, f_id,
                                                        move(instance_)),
                       std::logic_error,
                       "Referenced frame \\d+ for source \\d+\\."
                           " But the frame doesn't belong to the source.");
}

// Tests error resulting from passing a null GeometryInstance.
TEST_F(GeometryStateTest, RegisterNullGeometry) {
  SourceId s_id = NewSource();
  FrameId f_id = geometry_state_.RegisterFrame(s_id);
  unique_ptr<GeometryInstance<double>> null_geometry;
  EXPECT_ERROR_MESSAGE(geometry_state_.RegisterGeometry(s_id, f_id,
                                                     move(null_geometry)),
                       std::logic_error,
                       "Registering null geometry to frame \\d+, on source "
                       "\\d+.");
}

// Tests the logic for hanging a geometry on another geometry. This confirms
// topology and pose values.
TEST_F(GeometryStateTest, RegisterGeometryonValidGeometry) {
  SourceId s_id = SetUpSingleSourceTree();
  Isometry3<double> pose = Isometry3<double>::Identity();
  Isometry3<double> expected_pose = Isometry3<double>::Identity();
  double x = 3;
  double y = 2;
  double z = 1;
  pose.translation() << x, y, z;
  const int parent_index = 0;
  auto instance = make_unique<GeometryInstance<double>>(pose);
  GeometryId g_id =
      geometry_state_.RegisterGeometryWithParent(s_id,
                                                 geometries_[parent_index],
                                                 move(instance));
  // This relies on the gᵗʰ geometry has position [ g+1 0 0 ]ᵀ.
  expected_pose.translation() << (parent_index + 1) + x, y, z;
  EXPECT_EQ(geometry_state_.GetFrameId(geometries_[parent_index]),
            geometry_state_.GetFrameId(g_id));
  // TODO(SeanCurtis-TRI): Test that the pose of the registered geometry,
  // collapsed down to the common frame, *is* the expected pose.
  Isometry3<double> X_FG = geometry_state_.GetPoseInFrame(g_id);
  EXPECT_TRUE(CompareMatrices(X_FG.matrix(), expected_pose.matrix(),
                  1e-14, MatrixCompareType::absolute));
  Isometry3<double> X_PG = geometry_state_.GetPoseInParent(g_id);
  EXPECT_TRUE(CompareMatrices(X_PG.matrix(), pose.matrix(),
                  1e-14, MatrixCompareType::absolute));
}

// Tests the response to the erroneous action of trying to hang a new geometry
// on a non-existant geometry id.
TEST_F(GeometryStateTest, RegisterGeometryonInvalidGeometry) {
  SourceId s_id = SetUpSingleSourceTree();
  Isometry3<double> pose = Isometry3<double>::Identity();
  auto instance = make_unique<GeometryInstance<double>>(pose);
  GeometryId junk_id = GeometryId::get_new_id();
  EXPECT_ERROR_MESSAGE(
      geometry_state_.RegisterGeometryWithParent(s_id, junk_id, move(instance)),
      std::logic_error,
      "Referenced geometry \\d+ does not belong to a known frame.");
}

// Tests the response to passing a null pointer as a GeometryInstance.
TEST_F(GeometryStateTest, RegisterNullGeometryonGeometry) {
  SourceId s_id = SetUpSingleSourceTree();
  unique_ptr<GeometryInstance<double>> instance;
  EXPECT_ERROR_MESSAGE(
      geometry_state_.RegisterGeometryWithParent(s_id, geometries_[0],
                                                 move(instance)),
      std::logic_error,
      "Registering null geometry to geometry \\d+, on source \\d+.");
}

// Tests the RemoveGeometry functionality.
TEST_F(GeometryStateTest, RemoveGeometry) {
  SourceId s_id = SetUpSingleSourceTree();
  // Confirm that the first geometry belongs to the first frame.
  ASSERT_EQ(geometry_state_.GetFrameId(geometries_[0]), frames_[0]);
  EXPECT_EQ(geometry_state_.get_num_geometries(), kFrameCount * kGeometryCount);
  geometry_state_.RemoveGeometry(s_id, geometries_[0]);
  EXPECT_EQ(geometry_state_.get_num_geometries(),
            kFrameCount * kGeometryCount - 1);
  EXPECT_ERROR_MESSAGE(
      geometry_state_.GetFrameId(geometries_[0]),
      std::logic_error,
      "Referenced geometry \\d+ does not belong to a known frame.");
}

// Tests the RemoveGeometry functionality in which the geometry removed has
// children.
TEST_F(GeometryStateTest, RemoveGeometryRecursiveParent) {
  SourceId s_id = SetUpSingleSourceTree();
  // Confirm that the first geometry belongs to the first frame.
  ASSERT_EQ(geometry_state_.GetFrameId(geometries_[0]), frames_[0]);
  // Hang geometry from the first geometry.
  GeometryId g_id = geometry_state_.RegisterGeometryWithParent(
      s_id, geometries_[0],
      make_unique<GeometryInstance<double>>(Isometry3<double>::Identity()));
  EXPECT_EQ(geometry_state_.get_num_geometries(),
            kFrameCount * kGeometryCount + 1);
  EXPECT_EQ(geometry_state_.GetFrameId(g_id), frames_[0]);

  geometry_state_.RemoveGeometry(s_id, geometries_[0]);
  EXPECT_EQ(geometry_state_.get_num_geometries(),
            kFrameCount * kGeometryCount - 1);
  EXPECT_ERROR_MESSAGE(
      geometry_state_.GetFrameId(geometries_[0]),
      std::logic_error,
      "Referenced geometry \\d+ does not belong to a known frame.");
  EXPECT_ERROR_MESSAGE(
      geometry_state_.GetFrameId(g_id),
      std::logic_error,
      "Referenced geometry \\d+ does not belong to a known frame.");
}

// Tests the RemoveGeometry functionality in which the geometry is a child of
// another geometry.
TEST_F(GeometryStateTest, RemoveGeometryRecursiveChild) {
  SourceId s_id = SetUpSingleSourceTree();
  // Confirm that the first geometry belongs to the first frame.
  ASSERT_EQ(geometry_state_.GetFrameId(geometries_[0]), frames_[0]);
  // Hang geometry from the first geometry.
  GeometryId g_id = geometry_state_.RegisterGeometryWithParent(
      s_id, geometries_[0],
      make_unique<GeometryInstance<double>>(Isometry3<double>::Identity()));
  EXPECT_EQ(geometry_state_.get_num_geometries(),
            kFrameCount * kGeometryCount + 1);
  EXPECT_EQ(geometry_state_.GetFrameId(g_id), frames_[0]);

  geometry_state_.RemoveGeometry(s_id, g_id);
  EXPECT_EQ(geometry_state_.get_num_geometries(),
            kFrameCount * kGeometryCount);
  EXPECT_EQ(geometry_state_.GetFrameId(geometries_[0]), frames_[0]);
  EXPECT_ERROR_MESSAGE(
      geometry_state_.GetFrameId(g_id),
      std::logic_error,
      "Referenced geometry \\d+ does not belong to a known frame.");
}

// Tests the response to invalid misuse of RemoveGeometry.
TEST_F(GeometryStateTest, RemoveGeometryInvalid) {
  SourceId s_id = SetUpSingleSourceTree();

  // Case: Invalid source id, valid geometry id.
  EXPECT_ERROR_MESSAGE(
      geometry_state_.RemoveGeometry(SourceId::get_new_id(),
                                     geometries_[0]),
      std::logic_error,
      "Trying to remove geometry \\d+ from source \\d+.+geometry doesn't "
      "belong.+");

  // Case: Invalid geometry id, valid source id.
  EXPECT_ERROR_MESSAGE(
      geometry_state_.RemoveGeometry(s_id, GeometryId::get_new_id()),
      std::logic_error,
      "Referenced geometry \\d+ does not belong to a known frame.");

  // Case: Valid geometry and source, but geometry belongs to different source.
  SourceId s_id2 = SourceId::get_new_id();
  geometry_state_.RegisterNewSource(s_id2);
  FrameId frame_id = geometry_state_.RegisterFrame(s_id2);
  EXPECT_EQ(geometry_state_.GetNumFrames(), kFrameCount + 1);
  GeometryId g_id = geometry_state_.RegisterGeometry(
      s_id2, frame_id,
      make_unique<GeometryInstance<double>>(Isometry3<double>::Identity()));
  EXPECT_EQ(geometry_state_.get_num_geometries(),
            kFrameCount * kGeometryCount + 1);
  EXPECT_ERROR_MESSAGE(
      geometry_state_.RemoveGeometry(s_id, g_id),
      std::logic_error,
      "Trying to remove geometry \\d+ from source \\d+.+geometry doesn't "
          "belong.+");
}

// Tests the registration of anchored geometry.
TEST_F(GeometryStateTest, RegisterAnchoredGeometry) {
  Isometry3<double> pose = Isometry3<double>::Identity();
  auto instance = make_unique<GeometryInstance<double>>(pose);
  EXPECT_ERROR_MESSAGE(
      geometry_state_.RegisterAnchoredGeometry(SourceId::get_new_id(),
                                               move(instance)),
      std::runtime_error,
      "Not implemented yet!");
}

// Tests the response of attempting to register a null pointer GeometryInstance
// as anchored geometry.
TEST_F(GeometryStateTest, RegisterAnchoredNullGeometry) {
  unique_ptr<GeometryInstance<double>> instance;
  EXPECT_ERROR_MESSAGE(
      geometry_state_.RegisterAnchoredGeometry(SourceId::get_new_id(),
                                               move(instance)),
      std::runtime_error,
      "Not implemented yet!");
}

// Confirms the behavior for requesting geometry poses with a bad geometry
// identifier. The basic behavior is tested implicitly in other tests because
// they rely on them to validate state.
TEST_F(GeometryStateTest, GetPoseForBadGeometryId) {
  EXPECT_ERROR_MESSAGE(
      geometry_state_.GetPoseInFrame(GeometryId::get_new_id()),
      std::logic_error,
      "Requesting a geometry pose with an invalid geometry identifier.*");
  EXPECT_ERROR_MESSAGE(
      geometry_state_.GetPoseInParent(GeometryId::get_new_id()),
      std::logic_error,
      "Requesting a geometry pose with an invalid geometry identifier.*");
}

// This confirms the failure state of calling GeometryState::GetSourceId with a
// bad frame/geometry identifier.
TEST_F(GeometryStateTest, GetSourceIdFromBadId) {
  EXPECT_ERROR_MESSAGE(geometry_state_.GetSourceId(FrameId::get_new_id()),
                       std::logic_error,
                       "Referenced frame \\d+ has not been registered.");
  EXPECT_ERROR_MESSAGE(geometry_state_.GetSourceId(GeometryId::get_new_id()),
                       std::logic_error,
                       "Referenced geometry \\d+ does not belong to a known "
                       "frame.");
}

// This confirms the failure state of calling GeometryState::GetFrameId with a
// bad geometry identifier.
TEST_F(GeometryStateTest, GetFrameIdFromBadId) {
  EXPECT_ERROR_MESSAGE(geometry_state_.GetFrameId(GeometryId::get_new_id()),
                       std::logic_error,
                       "Referenced geometry \\d+ does not belong to a known "
                       "frame.");
}

// This tests that clearing a source eliminates all of its geometry and frames,
// leaving the source active.
TEST_F(GeometryStateTest, ClearSourceData) {
  EXPECT_ERROR_MESSAGE(geometry_state_.ClearSource(SourceId::get_new_id()),
                       std::logic_error,
                       "Referenced geometry source \\d+ is not active.");

  SourceId s_id = SetUpSingleSourceTree();
  geometry_state_.ClearSource(s_id);
  EXPECT_TRUE(geometry_state_.source_is_active(s_id));
  AssertSingleTreeCleared();
}

// Tests the functionality for acquiring the parent geoemtry for a given
// geometry.
TEST_F(GeometryStateTest, GetParentGeometry) {
  SourceId s_id = SetUpSingleSourceTree();

  // Case: Attempt to query non-existant geometry id.
  EXPECT_ERROR_MESSAGE(
      geometry_state_.FindParentGeometry(GeometryId::get_new_id()),
      std::logic_error,
      "Referenced geometry \\d+ does not belong to a known frame.");

  // Case: Query geometry id directly registered to the frame. The optional must
  // be false.
  auto frame_result = geometry_state_.FindParentGeometry(geometries_[0]);
  EXPECT_FALSE(frame_result);

  // Case: Query geometry registered to another geometry.
  auto instance = make_unique<GeometryInstance<double>>(
      Isometry3<double>::Identity());
  GeometryId g_id = geometry_state_.RegisterGeometryWithParent(s_id,
                                                               geometries_[0],
                                                               move(instance));
  auto geo_result = geometry_state_.FindParentGeometry(g_id);
  EXPECT_TRUE(geo_result);
  EXPECT_EQ(*geo_result, geometries_[0]);
}

// Tests the validation of frame kinematics data provided.
TEST_F(GeometryStateTest, ValidateKinematicsData) {
  SourceId s_id = SetUpSingleSourceTree();
  FrameKinematicsSet<double> fks = GeometryWorld<double>::MakeFKS(s_id);
  // Create one pose per frame.
  std::vector<SpatialPose<double>> poses;
  for (size_t i = 0; i < frames_.size(); ++i) {
    poses.emplace_back();
  }
  // Case: Valid data.
  fks.ReportPoses(frames_, poses);
  EXPECT_NO_THROW(geometry_state_.ValidateKinematicsSet(fks));

  // Case: Strictly missing required frames.
  fks.Clear();
  fks.ReportPose(frames_[0], poses[0]);
  EXPECT_ERROR_MESSAGE(geometry_state_.ValidateKinematicsSet(fks),
                       std::logic_error,
                       "Disagreement in expected number of frames \\(\\d+\\) "
                           "and the given number of frames \\(\\d+\\).");

  // Case: Strictly adding frames that don't belong.
  fks.Clear();
  fks.ReportPoses(frames_, poses);
  fks.ReportPose(FrameId::get_new_id(), SpatialPose<double>());
  EXPECT_ERROR_MESSAGE(geometry_state_.ValidateKinematicsSet(fks),
                       std::logic_error,
                       "Disagreement in expected number of frames \\(\\d+\\) "
                           "and the given number of frames \\(\\d+\\).");

  // Case: Correct number; required frame swapped with invalid frame.
  fks.Clear();
  std::vector<FrameId> frames_subset(++frames_.begin(), frames_.end());
  std::vector<SpatialPose<double>> pose_subset(++poses.begin(), poses.end());
  fks.ReportPoses(frames_subset, pose_subset);
  fks.ReportPose(FrameId::get_new_id(), SpatialPose<double>());
  EXPECT_ERROR_MESSAGE(geometry_state_.ValidateKinematicsSet(fks),
                       std::logic_error,
                       "Frame id provided in kinematics data \\(\\d+\\) "
                           "does not belong to the source \\(\\d+\\)."
                           " At least one required frame id is also missing.");
}

}  // namespace
}  // namespace geometry
}  // namespace drake
