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

  // Members owned by the test class.
  unique_ptr<GeometryWorld<double>> world_;
  unique_ptr<GeometryContext<double>> context_;
  // The GeometryState instance is actually owned by the context; this is simply
  // a convenience handle.
  GeometryState<double>* geometry_state_;
  unique_ptr<GeometryInstance> instance_;
  Isometry3<double> pose_;
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
  SourceId s_id = world_->RegisterNewSource();

  // Creates k frames with n geometries each.
  const int frame_count = 2;
  const int geometry_count = 3;
  FrameId frames[frame_count];
  GeometryId geometries[frame_count * geometry_count];
  for (int f = 0; f < frame_count; ++f) {
    frames[f] = world_->RegisterFrame(context_.get(), s_id);
    int geometry_position = f * geometry_count;
    for (int g = 0; g < geometry_count; ++g) {
      geometries[geometry_position++] =
          world_->RegisterGeometry(context_.get(), s_id, frames[f],
          make_unique<GeometryInstance>(), pose_);
    }
  }
  // Confirms that the same source is reachable from all geometries.
  for (int i = 0; i < frame_count * geometry_count; ++i) {
    EXPECT_EQ(geometry_state_->GetSourceId(geometries[i]), s_id);
  }

  world_->ClearSource(context_.get(), s_id);
  EXPECT_TRUE(world_->SourceIsRegistered(s_id));

  // confirm frames have been closed
  for (int f = 0; f < frame_count; ++f) {
    try {
      geometry_state_->GetSourceId(frames[f]);
      GTEST_FAIL();
    } catch (const std::runtime_error& err) {
      ExpectErrorMessage(err.what(),
                         "Referenced frame \\d+ has not been registered.");
    }
  }
  // confirm geometries have been closed
  for (int g = 0; g < frame_count * geometry_count; ++g) {
    try {
      geometry_state_->GetSourceId(geometries[g]);
      GTEST_FAIL();
    } catch (const std::runtime_error& err) {
      ExpectErrorMessage(err.what(),
                         "Referenced geometry \\d+ does not belong to a known "
                             "frame.");
    }
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
