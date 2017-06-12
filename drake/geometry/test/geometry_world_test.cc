#include "drake/geometry/geometry_world.h"

#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "drake/geometry/geometry_context.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_state.h"
#include "drake/geometry/test/expect_error_message.h"
#include "drake/geometry/test/query_test_utility.h"

namespace drake {
namespace geometry {
namespace {

using drake::Isometry3;
using std::make_unique;
using std::move;
using std::unique_ptr;
using std::vector;

class GeometryWorldTest : public ::testing::Test {
 protected:
  void SetUp() {
    world_ = make_unique<GeometryWorld<double>>();
    context_ = world_->MakeContext();
    geometry_state_ = &context_->get_mutable_geometry_state();
  }

  // Members owned by the test class.
  unique_ptr<GeometryWorld<double>> world_;
  unique_ptr<GeometryContext<double>> context_;
  // The state is owned by the context; this provides easy access.
  GeometryState<double>* geometry_state_;
};

// TODO(SeanCurtis-TRI): Move this into the GeometrySystem test.
// // Confirm that the state is extracted from the context without any copying.
// TEST_F(GeometryWorldTest, PullStateFromContext) {
//  auto state_ptr = world_->get_mutable_state(context_.get());
//  EXPECT_EQ(state_ptr, geometry_state_);
//  auto& state = world_->get_state(*context_);
//  EXPECT_EQ(&state, geometry_state_);
//}

// Confirms the registration of a new source with and without "default" names.
// This does not test error conditions as that is handled in
// geometry_state_test.
TEST_F(GeometryWorldTest, TestSourceNames) {
  using std::to_string;

  GeometryContext<double>* context = context_.get();

  // Case: user-specified source name.
  std::string name = "my_source";
  SourceId named_id = world_->RegisterNewSource(context, name);
  EXPECT_EQ(world_->get_source_name(*context, named_id), name);

  // Case: default name.
  SourceId anon_id = world_->RegisterNewSource(context);
  EXPECT_EQ(world_->get_source_name(*context, anon_id),
            "Source_" + to_string(anon_id));
}

// ---------------------  Geometry Queries --------------------------------

// Geometry query tests. The majority of the work of geometry queries is done
// by the engine. GeometryWorld serves as a public interface to the engine
// (translating global identifiers to engine indices). This merely tests that
// behavior.

class GeometryWorldQueryTest : public test::GeometryQueryTest {};

// Tests that the query correctly maps geometry ids into geometry indexes to get
// the correct results.
TEST_F(GeometryWorldQueryTest, ComputePairwiseClosestPoints_SelectIndices) {
  GeometryWorld<double> world;
  SetUpAxisSpheres();
  const std::vector<GeometryId>& sphere_ids =
      state_tester_.get_index_to_id_map();
  vector<GeometryId> query_ids = {sphere_ids[0],
                                  sphere_ids[2],
                                  sphere_ids[3]};
  std::vector<NearestPair<double>> pairs;
  EXPECT_TRUE(world.ComputePairwiseClosestPoints(*context_,
                                                 query_ids, &pairs));
  vector<test::IdPair> computed_pairs = {{sphere_ids[0], sphere_ids[2]},
                                         {sphere_ids[0], sphere_ids[3]},
                                         {sphere_ids[2], sphere_ids[3]}};
  ExpectNearestPairs(pairs, computed_pairs);
}

// Tests the interface where only explicitly enumerated pairs are included.
TEST_F(GeometryWorldQueryTest, ComputePairwiseClosestPoints_SelectPairs) {
  GeometryWorld<double> world;
  SetUpAxisSpheres();
  vector<GeometryPair> query_pairs;
  const std::vector<GeometryId>& sphere_ids =
      state_tester_.get_index_to_id_map();
  query_pairs.emplace_back(sphere_ids[0], sphere_ids[1]);
  query_pairs.emplace_back(sphere_ids[2], sphere_ids[3]);
  std::vector<NearestPair<double>> results;
  EXPECT_TRUE(world.ComputePairwiseClosestPoints(*context_,
                                                 query_pairs, &results));
  vector<test::IdPair> computed_pairs = {{sphere_ids[0], sphere_ids[1]},
                                         {sphere_ids[2], sphere_ids[3]}};
  ExpectNearestPairs(results, computed_pairs);
}

}  // namespace
}  // namespace geometry
}  // namespace drake
