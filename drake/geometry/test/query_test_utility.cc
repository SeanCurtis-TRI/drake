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
  FrameId frame_id = FrameId::get_new_id();
  for (int i = 0; i < kSphereCount; ++i) {
    GeometryIndex g_index = engine_.AddDynamicGeometry(make_sphere(kRadius));
    sphere_indices_.push_back(g_index);
    GeometryId g_id = GeometryId::get_new_id();
    sphere_ids_.push_back(g_id);
    geometries_[g_id] =
        internal::InternalGeometry(frame_id, g_id, "name", g_index);
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
  expected_nearest_[IdPair(sphere_ids_[0], sphere_ids_[1])] =
    NearestPair<double>(sphere_ids_[0], sphere_ids_[1],
                        Vector3<double>(0.5, 0, 0),
                        Vector3<double>(-0.5, 0, 0),
                        0.0);
  expected_nearest_[IdPair(sphere_ids_[0], sphere_ids_[2])] =
      NearestPair<double>(sphere_ids_[0], sphere_ids_[2],
                          Vector3<double>(0, 0, -0.5),
                          Vector3<double>(0, -0.5, 0),
                          0.0);
  expected_nearest_[IdPair(sphere_ids_[0], sphere_ids_[3])] =
      NearestPair<double>(sphere_ids_[0], sphere_ids_[3],
                          Vector3<double>(0, 0.5, 0),
                          Vector3<double>(0, 0, -0.5),
                          0.0);
  expected_nearest_[IdPair(sphere_ids_[1], sphere_ids_[2])] =
      NearestPair<double>(sphere_ids_[1], sphere_ids_[2],
                          Vector3<double>(-sqrt2 / 2, sqrt2 / 2, 0) * kRadius,
                          Vector3<double>(sqrt2 / 2, -sqrt2 / 2, 0) * kRadius,
                          sqrt2 - 1);
  expected_nearest_[IdPair(sphere_ids_[1], sphere_ids_[3])] =
      NearestPair<double>(sphere_ids_[1], sphere_ids_[3],
                          Vector3<double>(-sqrt2 / 2, 0, sqrt2 / 2) * kRadius,
                          Vector3<double>(sqrt2 / 2, 0, -sqrt2 / 2) * kRadius,
                          sqrt2 - 1);
  expected_nearest_[IdPair(sphere_ids_[2], sphere_ids_[3])] =
      NearestPair<double>(sphere_ids_[2], sphere_ids_[3],
                          Vector3<double>(0, -sqrt2 / 2, sqrt2 / 2) * kRadius,
                          Vector3<double>(0, sqrt2 / 2, -sqrt2 / 2) * kRadius,
                          sqrt2 - 1);
}

// A collection of methods to assert correctness of results. Confirms that the
// nearest pair data is consistent when considering the first *n* spheres.
void GeometryQueryTest::ExpectNearestPairs(
    const vector<NearestPair<double>> &results,
    const vector<IdPair> &pairs) {
  EXPECT_EQ(results.size(), pairs.size());

  const double kTolerance = 1e-14;
  for (int i = 0; i < static_cast<int>(pairs.size()); ++i) {
    const auto& expected_data = expected_nearest_[pairs[i]];
    const auto& data = results[i];
    EXPECT_EQ(data.id_A, expected_data.id_A);
    EXPECT_EQ(data.id_B, expected_data.id_B);
    EXPECT_EQ(data.distance, expected_data.distance);
    EXPECT_TRUE(
        CompareMatrices(data.p_ACa, expected_data.p_ACa, kTolerance));
    EXPECT_TRUE(
        CompareMatrices(data.p_BCb, expected_data.p_BCb, kTolerance));
  }
}

}  // namespace test
}  // namespace geometry
}  // namespace drake
