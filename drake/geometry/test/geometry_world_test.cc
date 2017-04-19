#include "drake/geometry/geometry_world.h"

#include <memory>
#include <regex>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_context.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_state.h"

// This test indirectly tests the GeometryState functionality as well. The
// methods of GeometryState that *modify* the state are private, intended only
// to be exercised by a GeometryWorld instance. These tests modify the state and
// verify the changes.

namespace drake {
namespace geometry {
namespace {

using drake::Isometry3;
using drake::systems::AbstractValues;
using std::make_unique;
using std::move;
using std::regex;
using std::regex_match;
using std::unique_ptr;

// Helper method; verifies exception messages against regular expressions.
// This provides test failure messages that are easiser to grok.
void ExpectErrorMessage(const char* err_msg, const char* reg_exp) {
  auto matcher = [](const char* s, const char* re) {
    return regex_match(s, regex(re)); };
  EXPECT_PRED2(matcher, err_msg, reg_exp);
}

class GeometryWorldTest : public ::testing::Test {
 protected:
  void SetUp() {
    world_ = make_unique<GeometryWorld<double>>();
    context_ = make_unique<GeometryContext<double>>();
    auto abstract_values = world_->AllocateAbstractValues();
    context_->set_abstract_state(
        make_unique<AbstractValues>(move(abstract_values)));
    geometry_state_ =
        &context_->get_mutable_abstract_state<GeometryState<double>>(0);
  }

  // This method sets up a dummy tree to facilitate testing, returning the
  // identifier of the source that owns the assets.
  SourceId SetUpDummyTree() {
    SourceId s_id = world_->RegisterNewSource();

    // Creates k frames with n geometries each.
    for (int f = 0; f < kFrameCount; ++f) {
      frames_.push_back(world_->RegisterFrame(context_.get(), s_id));
      int geometry_position = f * kGeometryCount;
      for (int g = 0; g < kGeometryCount; ++g) {
        geometries_[geometry_position++] =
            world_->RegisterGeometry(context_.get(), s_id, frames_[f],
                                     make_unique<GeometryInstance>(), pose_);
      }
    }
    // Confirms that the same source is reachable from all geometries.
    for (int i = 0; i < kFrameCount * kGeometryCount; ++i) {
      EXPECT_EQ(geometry_state_->GetSourceId(geometries_[i]), s_id);
    }
    return s_id;
  }

  // This method confirms that the stored dummy identifiers don't map to any
  // active source identifier.
  void AssertDummyTreeCleared() {
    // confirm frames have been closed
    for (int f = 0; f < kFrameCount; ++f) {
      try {
        geometry_state_->GetSourceId(frames_[f]);
        GTEST_FAIL();
      } catch (const std::runtime_error& err) {
        ExpectErrorMessage(err.what(),
                           "Referenced frame \\d+ has not been registered.");
      }
    }
    // confirm geometries have been closed
    for (int g = 0; g < kFrameCount * kGeometryCount; ++g) {
      try {
        geometry_state_->GetSourceId(geometries_[g]);
        GTEST_FAIL();
      } catch (const std::runtime_error& err) {
        ExpectErrorMessage(err.what(),
                           "Referenced geometry \\d+ does not belong to a known "
                               "frame.");
      }
    }
  }

  // Members owned by the test class.
  unique_ptr<GeometryWorld<double>> world_;
  unique_ptr<GeometryContext<double>> context_;
  // The GeometryState instance is actually owned by the context; this is simply
  // a convenience handle.
  GeometryState<double>* geometry_state_;
  unique_ptr<GeometryInstance> instance_;
  Isometry3<double> pose_;

  // Values for setting up and testing the dummy tree.
  constexpr static int kFrameCount = 2;
  constexpr static int kGeometryCount = 3;
  // The frame ids created in the dummy tree instantiation.
  std::vector<FrameId> frames_;
  // The geometry ids created in the dummy tree instantiation.
  GeometryId geometries_[kFrameCount * kGeometryCount];
};

// Tests the lifespan of a geometry sources. This implicitly tests the
// constructor, test for "openness", and the Close method.
TEST_F(GeometryWorldTest, ChannelLifeSpan) {
  SourceId id = world_->RegisterNewSource();
  EXPECT_TRUE(world_->SourceIsRegistered(id));
  SourceId false_id = SourceId::get_new_id();
  EXPECT_FALSE(world_->SourceIsRegistered(false_id));
}

// This tests the functionality where a source is added to the GeometryState
// implicitly by attempting to add a frame to it. This relies on the correctness
// of GeometryState::GetSourceId(FrameId) and, therefore, implicitly tests it.
TEST_F(GeometryWorldTest, AddSourceFromFrame) {
  SourceId id = world_->RegisterNewSource();
  FrameId fid = world_->RegisterFrame(context_.get(), id);
  SourceId parent_id = geometry_state_->GetSourceId(fid);
  EXPECT_EQ(parent_id, id);
}

// This confirms that if the a source id already exists in the GeometryState,
// subsequent frames are added to it. This relies on the correctness
// of GeometryState::GetSourceId(FrameId) and, therefore, implicitly tests it.
TEST_F(GeometryWorldTest, AddToExistingSource) {
  SourceId id = world_->RegisterNewSource();
  FrameId fid_1 = world_->RegisterFrame(context_.get(), id);
  FrameId fid_2 = world_->RegisterFrame(context_.get(), id);

  // Confirm that the second frame didn't in any way supplant the first.
  SourceId parent_id_1 = geometry_state_->GetSourceId(fid_1);
  SourceId parent_id_2 = geometry_state_->GetSourceId(fid_2);
  EXPECT_EQ(parent_id_1, id);
  EXPECT_EQ(parent_id_2, id);
}

// Tests registration of geometry on valid source and frame. This relies on the
// correctness of GeometryState::GetSourceId(GeometryId) and
// GeometryState::GetFrameId(GeometryId) and, therefore, implicitly tests them.
TEST_F(GeometryWorldTest, RegisterGeometryGoodSource) {
  SourceId s_id = world_->RegisterNewSource();
  FrameId f_id = world_->RegisterFrame(context_.get(), s_id);
  GeometryId g_id = world_->RegisterGeometry(context_.get(), s_id, f_id,
                                             move(instance_), pose_);
  EXPECT_EQ(geometry_state_->GetFrameId(g_id), f_id);
  EXPECT_EQ(geometry_state_->GetSourceId(g_id), s_id);
}

// Tests registration of geometry on valid source and non-existant frame.
TEST_F(GeometryWorldTest, RegisterGeometryMissingFrame) {
  SourceId s_id = world_->RegisterNewSource();
  // This is necessary to make sure the source id gets into the context.
  world_->RegisterFrame(context_.get(), s_id);

  FrameId f_id = FrameId::get_new_id();
  try {
    world_->RegisterGeometry(context_.get(), s_id, f_id,
                             move(instance_), pose_);
  } catch (const std::runtime_error& err) {
    ExpectErrorMessage(err.what(), "Referenced frame \\d+ for source \\d+\\."
        " But the frame doesn't belong to the source.");
  }
}

// This confirms the failure state of calling GeometryState::GetSourceId with a
// bad frame/geometry identifier.
TEST_F(GeometryWorldTest, GetSourceIdFromBadId) {
  try {
    geometry_state_->GetSourceId(FrameId::get_new_id());
  } catch (const std::runtime_error& err) {
    ExpectErrorMessage(err.what(),
                       "Referenced frame \\d+ has not been registered.");
  }
  try {
    geometry_state_->GetSourceId(GeometryId::get_new_id());
  } catch (const std::runtime_error& err) {
    ExpectErrorMessage(err.what(),
                       "Referenced geometry \\d+ does not belong to a known "
                           "frame.");
  }
}

// This confirms the failure state of calling GeometryState::GetFrameId with a
// bad geometry identifier.
TEST_F(GeometryWorldTest, GetFrameIdFromBadId) {
  try {
    geometry_state_->GetFrameId(GeometryId::get_new_id());
  } catch (const std::runtime_error& err) {
    ExpectErrorMessage(err.what(),
                       "Referenced geometry \\d+ does not belong to a known "
                           "frame.");
  }
}

// This tests that clearing a source eliminates all of its geometry and frames,
// leaving the source active.
TEST_F(GeometryWorldTest, ClearSourceData) {
  SourceId s_id = SetUpDummyTree();
  world_->ClearSource(context_.get(), s_id);
  EXPECT_TRUE(world_->SourceIsRegistered(s_id));
  AssertDummyTreeCleared();
}

// This tests that clearing a source eliminates all of its geometry and frames,
// leaving the source active.
TEST_F(GeometryWorldTest, RemoveSourceData) {
  SourceId s_id = SetUpDummyTree();
  world_->RemoveSource(context_.get(), s_id);
  EXPECT_FALSE(world_->SourceIsRegistered(s_id));
  AssertDummyTreeCleared();
}

// Tests the validation of frame kinematics data provided.
TEST_F(GeometryWorldTest, ValidateKinematicsData) {
  SourceId s_id = SetUpDummyTree();
  FrameKinematicsSet<double> fks = world_->GetFrameKinematicsSet(s_id);
  // Create one pose per frame.
  std::vector<SpatialPose<double>> poses;
  for (size_t i = 0; i < frames_.size(); ++i) {
    poses.emplace_back();
  }
  // Case: Valid data.
  fks.ReportPoses(frames_, poses);
  EXPECT_NO_THROW(world_->SetFrameKinematics(context_.get(), fks));

  // Case: Strictly missing required frames.
  fks.Clear();
  fks.ReportPose(frames_[0], poses[0]);
  try {
    world_->SetFrameKinematics(context_.get(), fks);
    GTEST_FAIL();
  } catch (const std::logic_error& err) {
    ExpectErrorMessage(err.what(),
                       "Disagreement in expected number of frames \\(\\d+\\) "
                       "and the given number of frames \\(\\d+\\).");
  }

  // Case: Strictly adding frames that don't belong.
  fks.Clear();
  fks.ReportPoses(frames_, poses);
  fks.ReportPose(FrameId::get_new_id(), SpatialPose<double>());
  try {
    world_->SetFrameKinematics(context_.get(), fks);
    GTEST_FAIL();
  } catch (const std::logic_error& err) {
    ExpectErrorMessage(err.what(),
                       "Disagreement in expected number of frames \\(\\d+\\) "
                           "and the given number of frames \\(\\d+\\).");
  }

  // Case: Correct number; required frame swapped with invalid frame.
  fks.Clear();
  std::vector<FrameId> frames_subset(++frames_.begin(), frames_.end());
  std::vector<SpatialPose<double>> pose_subset(++poses.begin(), poses.end());
  fks.ReportPoses(frames_subset, pose_subset);
  fks.ReportPose(FrameId::get_new_id(), SpatialPose<double>());
  try {
    world_->SetFrameKinematics(context_.get(), fks);
    GTEST_FAIL();
  } catch (const std::logic_error& err) {
    ExpectErrorMessage(err.what(),
                       "Frame id provided in kinematics data \\(\\d+\\) "
                       "that does not belong to the source \\(\\d+\\)."
                       " At least one required frame id is also missing.");
  }
}

//-------------------- DEATH TESTS ------------------------------

class GeometryWorldDeathTest : public GeometryWorldTest {};

// Confirms that attempting to add a frame to an unregistered source causes
// abort.
TEST_F(GeometryWorldDeathTest, AddToFakeSource) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  SourceId s_id = SourceId::get_new_id();
  ASSERT_DEATH(
      {world_->RegisterFrame(context_.get(), s_id);},
      "abort: failure at .*geometry_world.cc:.+ in RegisterFrame.+"
          "assertion 'SourceIsRegistered\\(source_id\\)' failed");
}

// This confirms that attempting to register geometry on a bad source causes
// abort.
TEST_F(GeometryWorldDeathTest, RegisterGeometryBadSource) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  SourceId s_id = SourceId::get_new_id();
  FrameId f_id = FrameId::get_new_id();
  ASSERT_DEATH(
      {world_->RegisterGeometry(context_.get(), s_id, f_id,
                                move(instance_), pose_);},
      "abort: failure at .*geometry_world.cc:.+ in RegisterGeometry.+"
          "assertion 'SourceIsRegistered\\(source_id\\)' failed");
}

}  // namespace
}  // namespace geometry
}  // namespace drake
