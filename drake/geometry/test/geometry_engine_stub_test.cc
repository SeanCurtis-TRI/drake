#include "drake/geometry/geometry_engine_stub.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/geometry/test/query_test_utility.h"

namespace drake {
namespace geometry {
namespace {

using std::vector;

using GEngine = GeometryEngineStub<double>;

class GeometryEngineStubTest : public test::GeometryQueryTest {};

TEST_F(GeometryEngineStubTest, Constructor) {
  GEngine engine;
  EXPECT_EQ(engine.get_update_input_size(), 0);
}

// Tests the interface in which all geometries in the world are included.
TEST_F(GeometryEngineStubTest, ComputePairwiseClosestPoints_All) {
  SetUpAxisSpheres();
  std::vector<NearestPair<double>> results;
  EXPECT_TRUE(engine_.ComputePairwiseClosestPoints(sphere_ids_, &results));
  vector<test::IdPair> computed_pairs = {{sphere_ids_[0], sphere_ids_[1]},
                                   {sphere_ids_[0], sphere_ids_[2]},
                                   {sphere_ids_[0], sphere_ids_[3]},
                                   {sphere_ids_[1], sphere_ids_[2]},
                                   {sphere_ids_[1], sphere_ids_[3]},
                                   {sphere_ids_[2], sphere_ids_[3]}};
  ExpectCorrectProximity(results, computed_pairs);
}

// Tests the interface where the indices included are explicitly provided.
TEST_F(GeometryEngineStubTest, ComputePairwiseClosestPoints_SelectIndices) {
  SetUpAxisSpheres();
  vector<GeometryIndex> query_indices = {axis_indices_[0],
                                         axis_indices_[2],
                                         axis_indices_[3]};
  std::vector<NearestPair<double>> pairs;
  EXPECT_TRUE(engine_.ComputePairwiseClosestPoints(
      sphere_ids_, query_indices, &pairs));
  vector<test::IdPair> computed_pairs = {{sphere_ids_[0], sphere_ids_[2]},
                                   {sphere_ids_[0], sphere_ids_[3]},
                                   {sphere_ids_[2], sphere_ids_[3]}};
  ExpectCorrectProximity(pairs, computed_pairs);
}

// Tests the interface where only explicitly enumerated pairs are included.
TEST_F(GeometryEngineStubTest, ComputePairwiseClosestPoints_SelectPairs) {
  SetUpAxisSpheres();
  vector<internal::GeometryIndexPair> query_pairs;
  query_pairs.emplace_back(axis_indices_[0], axis_indices_[1]);
  query_pairs.emplace_back(axis_indices_[2], axis_indices_[3]);
  std::vector<NearestPair<double>> results;
  EXPECT_TRUE(engine_.ComputePairwiseClosestPoints(
      sphere_ids_, query_pairs, &results));
  vector<test::IdPair> computed_pairs = {{sphere_ids_[0], sphere_ids_[1]},
                                   {sphere_ids_[2], sphere_ids_[3]}};
  ExpectCorrectProximity(results, computed_pairs);
}

// Tests the query determining the closest geometry
TEST_F(GeometryEngineStubTest, FindClosestGeometry) {
  const int kPointCount = 3;
  Matrix3X<double> points;
  points.resize(3, kPointCount);
  SetUpAxisSpheres();

  vector<PointProximity<double>> results;
  vector<PointProximity<double>> expected;

  // Point 0: Directly below sphere 0 at the origin. Because of 0's rotation
  // the world and local nearest points are different.
  points.block<3, 1>(0, 0) << 0, 0, -1;
  expected.push_back(PointProximity<double>(sphere_ids_[0],
                       Vector3<double>(0, -kRadius, 0),
                       Vector3<double>(0, 0, -kRadius),
                       Vector3<double>(0, 0, -1),
                       0.5));

  // Point 1: To the right of sphere 1 (the sphere at <1, 0, 0>).
  points.block<3, 1>(0, 1) << 2, 0, 0;
  expected.push_back(PointProximity<double>(sphere_ids_[1],
                        Vector3<double>(kRadius, 0, 0),
                        Vector3<double>(kRadius + 1, 0, 0),
                        Vector3<double>(1, 0 ,0),
                        0.5));

  // Point 2: Lies *slightly* inside sphere 2 (the sphere at <0, 1, 0>).
  const double kInset = 0.1;
  points.block<3, 1>(0, 2) << 0, 1 + kRadius - kInset, 0;
  expected.push_back(PointProximity<double>(sphere_ids_[2],
                        Vector3<double>(0, kRadius, 0),
                        Vector3<double>(0, 1 + kRadius, 0),
                        Vector3<double>(0, -1, 0),
                        -kInset));

  EXPECT_TRUE(engine_.FindClosestGeometry(sphere_ids_, points, &results));
  EXPECT_EQ(results.size(), expected.size());
  for (int i = 0; i < kPointCount; ++i) {
    EXPECT_EQ(results[i].id_A, expected[i].id_A);
  }
}
}  // namespace
}  // namespace geometry
}  // namespace drake
