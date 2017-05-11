#include "drake/geometry/geometry_engine_stub.h"

#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_index.h"

namespace drake {
namespace geometry {
namespace {

using std::make_pair;
using std::make_unique;
using std::move;
using std::unique_ptr;
using std::unordered_map;
using std::vector;

using GEngine = GeometryEngineStub<double>;

// Convenience struct for associating data with pairs of indices. It can serve
// as a key in a set or map as long as the set/map is declared with this as
// the class as its own Hash class. E.g.,
//  unordered_map<IndexPair, float, IndexPair> pair_to_scalar_;
struct IndexPair {
  GeometryIndex idx1;
  GeometryIndex idx2;
  IndexPair() {}
  IndexPair(GeometryIndex i1, GeometryIndex i2) : idx1(i1), idx2(i2) {}
  bool operator==(const IndexPair& other) const {
    return idx1 == other.idx1 && idx2 == other.idx2;
  }
  // The pair serves is its own hash function.
  size_t operator()(const IndexPair& p) const noexcept {
    size_t key = p.idx1;
    return key << 32 | p.idx2;
  }
};

class GeometryEngineStubTest : public ::testing::Test {
 protected:
  static unique_ptr<Shape> make_sphere(double radius) {
    return unique_ptr<Shape>(new Sphere(radius));
  }

  void SetUpAxisSpheres() {
    // Add set of known spheres to the engine; encode known query relationships
    // between them.
    for (int i = 0; i < kSphereCount; ++i) {
      axis_indices_.push_back(engine_.AddDynamicGeometry(make_sphere(kRadius)));
    }

    // Position them at origin, [1, 0, 0], [0, 1, 0], and [0, 0, 1],
    // respectively. The first sphere is rotated 90 degrees around the x-axis.
    Isometry3<double> pose;
    pose.linear() = Matrix3<double>::Identity();
    vector<Isometry3<double>> poses;
    pose.translation() << 0, 0, 0;
    pose.linear() << 1, 0, 0,
                     0, 0, -1,
                     0, 1, 0;
    poses.push_back(pose);
    pose.linear() = Matrix3<double>::Identity();
    pose.translation() << 1, 0, 0;
    poses.push_back(pose);
    pose.translation() << 0, 1, 0;
    poses.push_back(pose);
    pose.translation() << 0, 0, 1;
    poses.push_back(pose);
    engine_.UpdateWorldPoses(poses);

    const double sqrt2 = sqrt(2);
    near_points_[IndexPair(axis_indices_[0], axis_indices_[1])] =
        make_pair(Vector3<double>(0.5, 0, 0), Vector3<double>(-0.5, 0, 0));
    near_points_[IndexPair(axis_indices_[0], axis_indices_[2])] =
        make_pair(Vector3<double>(0, 0, -0.5), Vector3<double>(0, -0.5, 0));
    near_points_[IndexPair(axis_indices_[0], axis_indices_[3])] =
        make_pair(Vector3<double>(0, 0.5, 0), Vector3<double>(0, 0, -0.5));
    near_points_[IndexPair(axis_indices_[1], axis_indices_[2])] =
        make_pair(Vector3<double>(-sqrt2 / 2, sqrt2 / 2, 0) * kRadius,
                  Vector3<double>(sqrt2 / 2, -sqrt2 / 2, 0) * kRadius);
    near_points_[IndexPair(axis_indices_[1], axis_indices_[3])] =
        make_pair(Vector3<double>(-sqrt2 / 2, 0, sqrt2 / 2) * kRadius,
                  Vector3<double>(sqrt2 / 2, 0, -sqrt2 / 2) * kRadius);
    near_points_[IndexPair(axis_indices_[2], axis_indices_[3])] =
        make_pair(Vector3<double>(0, -sqrt2 / 2, sqrt2 / 2) * kRadius,
                  Vector3<double>(0, sqrt2 / 2, -sqrt2 / 2) * kRadius);

    distances_[IndexPair(axis_indices_[0], axis_indices_[1])] = 0;
    distances_[IndexPair(axis_indices_[0], axis_indices_[2])] = 0;
    distances_[IndexPair(axis_indices_[0], axis_indices_[3])] = 0;
    distances_[IndexPair(axis_indices_[1], axis_indices_[2])] = sqrt2 - 1;
    distances_[IndexPair(axis_indices_[1], axis_indices_[3])] = sqrt2 - 1;
    distances_[IndexPair(axis_indices_[2], axis_indices_[3])] = sqrt2 - 1;
  }

  // A collection of methods to assert correctness of results. Confirms that the
  // nearest pair data is consistent when considering the first *n* spheres.
  void ExpectCorrectProximity(
      const vector<internal::NearestPair<double>>& results,
      const vector<IndexPair>& pairs) {
    EXPECT_EQ(results.size(), pairs.size());

    const double kTolerance = 1e-14;
    for (int i = 0; i < static_cast<int>(pairs.size()); ++i) {
      const auto& pair = pairs[i];
      const auto& data = results[i];
      EXPECT_EQ(data.index_A, pair.idx1);
      EXPECT_EQ(data.index_B, pair.idx2);
      EXPECT_EQ(data.distance, distances_[pair]);
      EXPECT_TRUE(
          CompareMatrices(data.p_A_A, near_points_[pair].first, kTolerance));
      EXPECT_TRUE(
          CompareMatrices(data.p_B_B, near_points_[pair].second, kTolerance));
    }
  }

  GEngine engine_;
  // Geometry indices for the axis-sphere configuration.
  static const float kRadius;
  static const int kSphereCount;
  // The indices of the spheres added to the engine.
  vector<GeometryIndex> axis_indices_;
  // The position of the near point on sphere i, for the pair (i, j), using
  // GetKeyFromIndices to map (i, j) into the key. The point on j, nearest i,
  // is just the negative value.
  unordered_map<IndexPair, std::pair<Vector3<double>, Vector3<double>>, IndexPair> near_points_;
  unordered_map<IndexPair, double, IndexPair> distances_;
};

const float GeometryEngineStubTest::kRadius = 0.5f;
const int GeometryEngineStubTest::kSphereCount = 4;

TEST_F(GeometryEngineStubTest, Constructor) {
  GEngine engine;
  EXPECT_EQ(engine.get_update_input_size(), 0);
}

TEST_F(GeometryEngineStubTest, ComputePairwiseClosestPoints_All) {
  SetUpAxisSpheres();
  std::vector<internal::NearestPair<double>> results;
  EXPECT_TRUE(engine_.ComputePairwiseClosestPoints(&results));
  vector<IndexPair> computed_pairs = { {axis_indices_[0], axis_indices_[1]},
                                       {axis_indices_[0], axis_indices_[2]},
                                       {axis_indices_[0], axis_indices_[3]},
                                       {axis_indices_[1], axis_indices_[2]},
                                       {axis_indices_[1], axis_indices_[3]},
                                       {axis_indices_[2], axis_indices_[3]}};
  ExpectCorrectProximity(results, computed_pairs);
}

TEST_F(GeometryEngineStubTest, ComputePairwiseClosestPoints_SelectIndices) {
  SetUpAxisSpheres();
  vector<GeometryIndex> query_indices = {axis_indices_[0],
                                         axis_indices_[2],
                                         axis_indices_[3]};
  std::vector<internal::NearestPair<double>> pairs;
  EXPECT_TRUE(engine_.ComputePairwiseClosestPoints(query_indices, &pairs));
  vector<IndexPair> computed_pairs = { {axis_indices_[0], axis_indices_[2]},
                                       {axis_indices_[0], axis_indices_[3]},
                                       {axis_indices_[2], axis_indices_[3]}};
  ExpectCorrectProximity(pairs, computed_pairs);
}

TEST_F(GeometryEngineStubTest, ComputePairwiseClosestPoints_SelectPairs) {
  SetUpAxisSpheres();
  vector<internal::GeometryIndexPair> query_pairs;
  query_pairs.emplace_back(axis_indices_[0], axis_indices_[1]);
  query_pairs.emplace_back(axis_indices_[2], axis_indices_[3]);
  std::vector<internal::NearestPair<double>> results;
  EXPECT_TRUE(engine_.ComputePairwiseClosestPoints(query_pairs, &results));
  vector<IndexPair> computed_pairs = { {axis_indices_[0], axis_indices_[1]},
                                       {axis_indices_[2], axis_indices_[3]}};
  ExpectCorrectProximity(results, computed_pairs);
}

// Tests the query determining the closest geometry
TEST_F(GeometryEngineStubTest, FindClosestGeometry) {
  const int kPointCount = 3;
  Matrix3X<double> points;
  points.resize(3, kPointCount);
  SetUpAxisSpheres();

  vector<internal::PointProximity<double>> results;
  vector<internal::PointProximity<double>> expected;

  // Point 0: Directly below sphere 0 at the origin. Because of 0's rotation
  // the world and local nearest points are different.
  points.block<3, 1>(0, 0) << 0, 0, -1;
  expected.emplace_back(axis_indices_[0],
                       Vector3<double>(0, -kRadius, 0),
                       Vector3<double>(0, 0, -kRadius),
                       Vector3<double>(0, 0, -1),
                       0.5);

  // Point 1: To the right of sphere 1 (the sphere at <1, 0, 0>).
  points.block<3, 1>(0, 1) << 2, 0, 0;
  expected.emplace_back(axis_indices_[1],
                        Vector3<double>(kRadius, 0, 0),
                        Vector3<double>(kRadius + 1, 0, 0),
                        Vector3<double>(1, 0 ,0),
                        0.5);

  // Point 2: Lies *slightly* inside sphere 2 (the sphere at <0, 1, 0>).
  const double kInset = 0.1;
  points.block<3, 1>(0, 2) << 0, 1 + kRadius - kInset, 0;
  expected.emplace_back(axis_indices_[2],
                        Vector3<double>(0, kRadius, 0),
                        Vector3<double>(0, 1 + kRadius, 0),
                        Vector3<double>(0, -1, 0),
                        -kInset);

  EXPECT_TRUE(engine_.FindClosestGeometry(points, &results));
  EXPECT_EQ(results.size(), expected.size());
  for (int i = 0; i < kPointCount; ++i) {
    EXPECT_EQ(results[i].index_A, expected[i].index_A);
  }
}

}  // namespace
}  // namespace geometry
}  // namespace drake
