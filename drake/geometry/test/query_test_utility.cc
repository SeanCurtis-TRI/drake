#include "drake/geometry/test/query_test_utility.h"

#include "drake/common/eigen_matrix_compare.h"

namespace drake {
namespace geometry {
namespace test {

using std::make_pair;
using std::vector;

const double GeometryQueryTest::kRadius = 0.5;
const int GeometryQueryTest::kSphereCount = 4;

void GeometryQueryTest::SetUpAxisSpheres() {
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
void GeometryQueryTest::ExpectCorrectProximity(
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

}  // namespace test
}  // namespace geometry
}  // namespace drake
