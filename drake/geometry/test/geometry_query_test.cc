#include "drake/geometry/geometry_query.h"

#include <unordered_map>

#include <gtest/gtest.h>

#include "drake/geometry/test/query_test_utility.h"
#include "drake/geometry/test/geometry_world_stub.h"

namespace drake {
namespace geometry {
namespace {

using std::vector;

class GeometryQueryTest : public test::GeometryQueryTest {
 protected:
  // Does the work of the parent class to set up the sphere demo
  // (see test::GeometryQueryTest::SetUpAxisSpheres), but also initializes a
  // geometry query.
  GeometryQuery<double> SetUpAxisSpheres() {
    test::GeometryQueryTest::SetUpAxisSpheres();

    // TODO(SeanCurtis-TRI): For later tests, move this geometries map into the
    // base class and define meaningful frames.
    FrameId frame_id = FrameId::get_new_id();
    for (size_t i = 0; i < sphere_ids_.size(); ++i) {
      GeometryId g_id = sphere_ids_[i];
      GeometryIndex g_index = axis_indices_[i];
      geometries_[g_id] = internal::InternalGeometry(
          frame_id, g_id, "name", g_index);
    }
    return GeometryWorld<double>::MakeQuery(engine_, sphere_ids_, geometries_);
  }

  std::unordered_map<GeometryId, internal::InternalGeometry> geometries_;
};

// Tests that the query correctly maps geometry ids into geometry indexes to get
// the correct results.
TEST_F(GeometryQueryTest, ComputePairwiseClosestPoints_SelectIndices) {
  auto query = SetUpAxisSpheres();
  vector<GeometryId> query_ids = {sphere_ids_[0],
                                  sphere_ids_[2],
                                  sphere_ids_[3]};
  std::vector<NearestPair<double>> pairs;
  EXPECT_TRUE(query.ComputePairwiseClosestPoints(query_ids, &pairs));
  vector<test::IdPair> computed_pairs = {{sphere_ids_[0], sphere_ids_[2]},
                                         {sphere_ids_[0], sphere_ids_[3]},
                                         {sphere_ids_[2], sphere_ids_[3]}};
  ExpectCorrectProximity(pairs, computed_pairs);
}

// Tests the interface where only explicitly enumerated pairs are included.
TEST_F(GeometryQueryTest, ComputePairwiseClosestPoints_SelectPairs) {
  auto query = SetUpAxisSpheres();
  vector<GeometryPair> query_pairs;
  query_pairs.emplace_back(sphere_ids_[0], sphere_ids_[1]);
  query_pairs.emplace_back(sphere_ids_[2], sphere_ids_[3]);
  std::vector<NearestPair<double>> results;
  EXPECT_TRUE(query.ComputePairwiseClosestPoints(query_pairs, &results));
  vector<test::IdPair> computed_pairs = {{sphere_ids_[0], sphere_ids_[1]},
                                         {sphere_ids_[2], sphere_ids_[3]}};
  ExpectCorrectProximity(results, computed_pairs);
}

}  // namespace
}  // namespace geometry
}  // namespace drake
