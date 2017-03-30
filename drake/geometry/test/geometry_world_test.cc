#include "drake/geometry/geometry_world.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/geometry/geometry_context.h"
#include "drake/geometry/geometry_state.h"
#include "drake/systems/framework/leaf_context.h"

namespace drake {
namespace geometry {
namespace {

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
  }

  // Members owned by the test class.
  unique_ptr<GeometryWorld<double>> world_;
  unique_ptr<GeometryContext<double>> context_;
  // The GeometryState instance is actually owned by the context; this is simply
  // a convenience handle.
  GeometryState<double>* geometry_state_;
};

// Tests the lifespan of a geometry sources. This implicitly tests the
// constructor, test for "openness", and the Close method.
TEST_F(GeometryWorldTest, ChannelLifeSpan) {
  SourceId id = world_->RegisterNewSource(context_.get());
  EXPECT_TRUE(geometry_state_->SourceIsActive(id));
  // Closes the channel to avert an assertion error in Debug builds.
  world_->RemoveSource(id, context_.get());
  EXPECT_FALSE(geometry_state_->SourceIsActive(id));
}

class GeometryWorldDeathTest : public GeometryWorldTest {};

}  // namespace
}  // namespace geometry
}  // namespace drake
