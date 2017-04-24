#include "drake/geometry/geometry_world.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/geometry/geometry_context.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_state.h"
#include "drake/geometry/test/expect_error_message.h"

namespace drake {
namespace geometry {
namespace {

using drake::Isometry3;
using drake::systems::AbstractValues;
using std::make_unique;
using std::move;
using std::unique_ptr;

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
    instance_ = make_unique<GeometryInstance<double>>(Isometry3<double>());
  }

  // This method sets up a dummy tree to facilitate testing, returning the
  // identifier of the source that owns the assets.
  SourceId SetUpDummyTree() {
    SourceId s_id = world_->RegisterNewSource(context_.get());

    // Creates k frames with n geometries each.
    Isometry3<double> pose;
    for (int f = 0; f < kFrameCount; ++f) {
      frames_.push_back(world_->RegisterFrame(context_.get(), s_id));
      int geometry_position = f * kGeometryCount;
      for (int g = 0; g < kGeometryCount; ++g) {
        geometries_[geometry_position++] = world_->RegisterGeometry(
            context_.get(), s_id, frames_[f],
            make_unique<GeometryInstance<double>>(pose));
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
      EXPECT_ERROR_MESSAGE(geometry_state_->GetSourceId(frames_[f]),
                           std::logic_error,
                           "Referenced frame \\d+ has not been registered.");
    }
    // confirm geometries have been closed
    for (int g = 0; g < kFrameCount * kGeometryCount; ++g) {
      EXPECT_ERROR_MESSAGE(geometry_state_->GetSourceId(geometries_[g]),
                           std::logic_error,
                           "Referenced geometry \\d+ does not belong to a known"
                           " frame.");
    }
  }

  // Members owned by the test class.
  unique_ptr<GeometryWorld<double>> world_;
  unique_ptr<GeometryContext<double>> context_;
  // The GeometryState instance is actually owned by the context; this is simply
  // a convenience handle.
  GeometryState<double>* geometry_state_;
  unique_ptr<GeometryInstance<double>> instance_;
  Isometry3<double> pose_;

  // Values for setting up and testing the dummy tree.
  constexpr static int kFrameCount = 2;
  constexpr static int kGeometryCount = 3;
  // The frame ids created in the dummy tree instantiation.
  std::vector<FrameId> frames_;
  // The geometry ids created in the dummy tree instantiation.
  GeometryId geometries_[kFrameCount * kGeometryCount];
};

// Confirm that the state is extracted from the context without any copying.
TEST_F(GeometryWorldTest, PullStateFromContext) {
  auto state_ptr = world_->get_mutable_state(context_.get());
  EXPECT_EQ(state_ptr, geometry_state_);
  auto& state = world_->get_state(*context_);
  EXPECT_EQ(&state, geometry_state_);
}

// Confirms the behavior of creating a FrameKinematicsSet from a bad source id.
// In release build, this is allowed (it'll throw an exception later when the
// frame kinematics set is used. In Debug build, an exception is thrown at the
// invocation. This tests confirms *both* behaviors.
TEST_F(GeometryWorldTest, CreateFrameKinematicsSetFromBadSource) {
  SourceId s_id = SourceId::get_new_id();
#ifdef DRAKE_ASSERT_IS_DISARMED
  // In release mode, this would be considered valid. No exception thrown.
  auto fks = world_->GetFrameKinematicsSet(s_id);
#else
  EXPECT_ERROR_MESSAGE(world_->GetFrameKinematicsSet(s_id),
                       std::logic_error,
                       "Invalid source id: \\d+.");
#endif
}

}  // namespace
}  // namespace geometry
}  // namespace drake
