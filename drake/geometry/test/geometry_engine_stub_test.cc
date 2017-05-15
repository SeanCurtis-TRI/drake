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
struct IdPair {
  GeometryId id1;
  GeometryId id2;
  IdPair() {}
  IdPair(GeometryId i1, GeometryId i2) : id1(i1), id2(i2) {}
  bool operator==(const IdPair& other) const {
    return id1 == other.id1 && id2 == other.id2;
  }
  // The pair serves is its own hash function.
  size_t operator()(const IdPair& p) const noexcept {
    size_t key = static_cast<size_t>(p.id1.get_value());
    return key << 32 | static_cast<size_t>(p.id2.get_value());
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
      sphere_ids_.push_back(GeometryId::get_new_id());
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
    near_points_[IdPair(sphere_ids_[0], sphere_ids_[1])] =
        make_pair(Vector3<double>(0.5, 0, 0), Vector3<double>(-0.5, 0, 0));
    near_points_[IdPair(sphere_ids_[0], sphere_ids_[2])] =
        make_pair(Vector3<double>(0, 0, -0.5), Vector3<double>(0, -0.5, 0));
    near_points_[IdPair(sphere_ids_[0], sphere_ids_[3])] =
        make_pair(Vector3<double>(0, 0.5, 0), Vector3<double>(0, 0, -0.5));
    near_points_[IdPair(sphere_ids_[1], sphere_ids_[2])] =
        make_pair(Vector3<double>(-sqrt2 / 2, sqrt2 / 2, 0) * kRadius,
                  Vector3<double>(sqrt2 / 2, -sqrt2 / 2, 0) * kRadius);
    near_points_[IdPair(sphere_ids_[1], sphere_ids_[3])] =
        make_pair(Vector3<double>(-sqrt2 / 2, 0, sqrt2 / 2) * kRadius,
                  Vector3<double>(sqrt2 / 2, 0, -sqrt2 / 2) * kRadius);
    near_points_[IdPair(sphere_ids_[2], sphere_ids_[3])] =
        make_pair(Vector3<double>(0, -sqrt2 / 2, sqrt2 / 2) * kRadius,
                  Vector3<double>(0, sqrt2 / 2, -sqrt2 / 2) * kRadius);

    distances_[IdPair(sphere_ids_[0], sphere_ids_[1])] = 0;
    distances_[IdPair(sphere_ids_[0], sphere_ids_[2])] = 0;
    distances_[IdPair(sphere_ids_[0], sphere_ids_[3])] = 0;
    distances_[IdPair(sphere_ids_[1], sphere_ids_[2])] = sqrt2 - 1;
    distances_[IdPair(sphere_ids_[1], sphere_ids_[3])] = sqrt2 - 1;
    distances_[IdPair(sphere_ids_[2], sphere_ids_[3])] = sqrt2 - 1;
  }

  // A collection of methods to assert correctness of results. Confirms that the
  // nearest pair data is consistent when considering the first *n* spheres.
  void ExpectCorrectProximity(
      const vector<NearestPair<double>>& results,
      const vector<IdPair>& pairs) {
    EXPECT_EQ(results.size(), pairs.size());

    const double kTolerance = 1e-14;
    for (int i = 0; i < static_cast<int>(pairs.size()); ++i) {
      const auto& pair = pairs[i];
      const auto& data = results[i];
      EXPECT_EQ(data.id_A, pair.id1);
      EXPECT_EQ(data.id_B, pair.id2);
      EXPECT_EQ(data.distance, distances_[pair]);
      EXPECT_TRUE(
          CompareMatrices(data.p_ACa, near_points_[pair].first, kTolerance));
      EXPECT_TRUE(
          CompareMatrices(data.p_BCb, near_points_[pair].second, kTolerance));
    }
  }

  GEngine engine_;
  // Geometry indices for the axis-sphere configuration.
  static const double kRadius;
  static const int kSphereCount;
  // The indices of the spheres added to the engine.
  vector<GeometryIndex> axis_indices_;
  // The ith entry contains the id that corresponds to the ith geometry in
  // axis_indices_;
  vector<GeometryId> sphere_ids_;
  // The position of the near point on sphere i, for the pair (i, j), using
  // GetKeyFromIndices to map (i, j) into the key. The point on j, nearest i,
  // is just the negative value.
  unordered_map<IdPair, std::pair<Vector3<double>, Vector3<double>>, IdPair>
      near_points_;
  unordered_map<IdPair, double, IdPair> distances_;
};

const double GeometryEngineStubTest::kRadius = 0.5;
const int GeometryEngineStubTest::kSphereCount = 4;

TEST_F(GeometryEngineStubTest, Constructor) {
  GEngine engine;
  EXPECT_EQ(engine.get_update_input_size(), 0);
}

TEST_F(GeometryEngineStubTest, ComputePairwiseClosestPoints_All) {
  SetUpAxisSpheres();
  std::vector<NearestPair<double>> results;
  EXPECT_TRUE(engine_.ComputePairwiseClosestPoints(sphere_ids_, &results));
  vector<IdPair> computed_pairs = {{sphere_ids_[0], sphere_ids_[1]},
                                   {sphere_ids_[0], sphere_ids_[2]},
                                   {sphere_ids_[0], sphere_ids_[3]},
                                   {sphere_ids_[1], sphere_ids_[2]},
                                   {sphere_ids_[1], sphere_ids_[3]},
                                   {sphere_ids_[2], sphere_ids_[3]}};
  ExpectCorrectProximity(results, computed_pairs);
}

TEST_F(GeometryEngineStubTest, ComputePairwiseClosestPoints_SelectIndices) {
  SetUpAxisSpheres();
  vector<GeometryIndex> query_indices = {axis_indices_[0],
                                         axis_indices_[2],
                                         axis_indices_[3]};
  std::vector<NearestPair<double>> pairs;
  EXPECT_TRUE(engine_.ComputePairwiseClosestPoints(
      sphere_ids_, query_indices, &pairs));
  vector<IdPair> computed_pairs = {{sphere_ids_[0], sphere_ids_[2]},
                                   {sphere_ids_[0], sphere_ids_[3]},
                                   {sphere_ids_[2], sphere_ids_[3]}};
  ExpectCorrectProximity(pairs, computed_pairs);
}

TEST_F(GeometryEngineStubTest, ComputePairwiseClosestPoints_SelectPairs) {
  SetUpAxisSpheres();
  vector<internal::GeometryIndexPair> query_pairs;
  query_pairs.emplace_back(axis_indices_[0], axis_indices_[1]);
  query_pairs.emplace_back(axis_indices_[2], axis_indices_[3]);
  std::vector<NearestPair<double>> results;
  EXPECT_TRUE(engine_.ComputePairwiseClosestPoints(
      sphere_ids_, query_pairs, &results));
  vector<IdPair> computed_pairs = {{sphere_ids_[0], sphere_ids_[1]},
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
