#include "drake/geometry/test/query_test_utility.h"

#include <utility>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/shapes.h"

namespace drake {
namespace geometry {
namespace test {

using std::make_pair;
using std::make_unique;
using std::vector;

const double GeometryQueryTest::kRadius = 0.5;
const int GeometryQueryTest::kSphereCount = 4;

void GeometryQueryTest::SetUpAxisSpheres() {
  using std::to_string;

  // Add set of known spheres to the engine; encode known query relationships
  // between them.
  source_id_ = SourceId::get_new_id();
  state_.RegisterNewSource(source_id_, "axis-aligned spheres");

  for (int i = 0; i < kSphereCount; ++i) {
    FrameId frame_id = state_.RegisterFrame(
        source_id_, GeometryFrame<double>("frame_" + to_string(i),
                                          Isometry3<double>::Identity()));
    state_.RegisterGeometry(
        source_id_, frame_id,
        make_unique<GeometryInstance<double>>(Isometry3<double>::Identity(),
                                              make_sphere(kRadius)));
  }

  // Position them at origin, [1, 0, 0], [0, 1, 0], and [0, 0, 1],
  // respectively. The first sphere is rotated 90 degrees around the x-axis.
  Isometry3<double> pose;
  pose.linear() = Matrix3<double>::Identity();
  pose.translation() << 0, 0, 0;
  pose.linear() << 1, 0, 0,
      0, 0, -1,
      0, 1, 0;
  poses_.push_back(pose);
  pose.linear() = Matrix3<double>::Identity();
  pose.translation() << 1, 0, 0;
  poses_.push_back(pose);
  pose.translation() << 0, 1, 0;
  poses_.push_back(pose);
  pose.translation() << 0, 0, 1;
  poses_.push_back(pose);
  // bypass frame kinematics sets by setting the values directly.
  state_tester_.get_engine()->UpdateWorldPoses(poses_);

  const std::vector<GeometryId>& sphere_ids =
      state_tester_.get_index_to_id_map();

  const double sqrt2 = sqrt(2);
  expected_nearest_[IdPair(sphere_ids[0], sphere_ids[1])] =
    NearestPair<double>(sphere_ids[0], sphere_ids[1],
                        Vector3<double>(0.5, 0, 0),
                        Vector3<double>(-0.5, 0, 0),
                        0.0);
  expected_nearest_[IdPair(sphere_ids[0], sphere_ids[2])] =
      NearestPair<double>(sphere_ids[0], sphere_ids[2],
                          Vector3<double>(0, 0, -0.5),
                          Vector3<double>(0, -0.5, 0),
                          0.0);
  expected_nearest_[IdPair(sphere_ids[0], sphere_ids[3])] =
      NearestPair<double>(sphere_ids[0], sphere_ids[3],
                          Vector3<double>(0, 0.5, 0),
                          Vector3<double>(0, 0, -0.5),
                          0.0);
  expected_nearest_[IdPair(sphere_ids[1], sphere_ids[2])] =
      NearestPair<double>(sphere_ids[1], sphere_ids[2],
                          Vector3<double>(-sqrt2 / 2, sqrt2 / 2, 0) * kRadius,
                          Vector3<double>(sqrt2 / 2, -sqrt2 / 2, 0) * kRadius,
                          sqrt2 - 1);
  expected_nearest_[IdPair(sphere_ids[1], sphere_ids[3])] =
      NearestPair<double>(sphere_ids[1], sphere_ids[3],
                          Vector3<double>(-sqrt2 / 2, 0, sqrt2 / 2) * kRadius,
                          Vector3<double>(sqrt2 / 2, 0, -sqrt2 / 2) * kRadius,
                          sqrt2 - 1);
  expected_nearest_[IdPair(sphere_ids[2], sphere_ids[3])] =
      NearestPair<double>(sphere_ids[2], sphere_ids[3],
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
