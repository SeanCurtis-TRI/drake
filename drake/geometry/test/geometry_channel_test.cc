#include "drake/geometry/geometry_channel.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/geometry/geometry_state.h"
#include "drake/geometry/geometry_world.h"
#include "drake/systems/framework/leaf_context.h"

namespace drake {
namespace geometry {
namespace {

using drake::systems::AbstractValues;
using drake::systems::LeafContext;
using std::make_unique;
using std::move;
using std::unique_ptr;

class GeometryChannelTest : public ::testing::Test {
 protected:
  void SetUp() {
    world_ = make_unique<GeometryWorld<double>>();
    context_ = make_unique<LeafContext<double>>();
    auto abstract_values = world_->AllocateAbstractValues();
    context_->set_abstract_state(
        make_unique<AbstractValues>(move(abstract_values)));
    geometry_state_ =
        &context_->get_mutable_abstract_state<GeometryState<double>>(0);
  }

  // Members owned by the test class.
  unique_ptr<GeometryWorld<double>> world_;
  unique_ptr<LeafContext<double>> context_;
  // The GeometryState instance is actually owned by the context; this is simply
  // a convenience handle.
  GeometryState<double>* geometry_state_;
};

// Tests the lifespan of a GeometryChannel instance. This implicitly tests the
// constructor, test for "openness", and the Close method.
TEST_F(GeometryChannelTest, ChannelLifeSpan) {
  auto channel = world_->RequestChannel(context_.get());
  EXPECT_TRUE(geometry_state_->ChannelIsOpen(channel->get_id()));
  // Closes the channel to avert an assertion error in Debug builds.
  channel->Close(context_.get());
  EXPECT_FALSE(geometry_state_->ChannelIsOpen(channel->get_id()));
}

class GeometryChannelDeathTest : public GeometryChannelTest {};

// Deleting the channel without first having closed it is considered an error.
TEST_F(GeometryChannelDeathTest, DeletionWithoutClosing) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  auto channel = world_->RequestChannel(context_.get());
  GeometryChannel<double>* raw_channel = channel.release();
  EXPECT_TRUE(geometry_state_->ChannelIsOpen(raw_channel->get_id()));
  ASSERT_DEATH({ delete raw_channel; },
               "abort: failure at .*geometry_channel.cc:.+ in "
               "~GeometryChannel\\(\\).+ assertion '!is_open_' failed.");
}
}  // namespace
}  // namespace geometry
}  // namespace drake
