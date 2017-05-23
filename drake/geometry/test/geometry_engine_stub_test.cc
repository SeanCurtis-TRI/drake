#include "drake/geometry/geometry_engine_stub.h"

#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/test/query_test_utility.h"

namespace drake {
namespace geometry {
namespace {

using std::make_unique;
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
  const std::vector<GeometryId>& sphere_ids =
      state_tester_.get_index_to_id_map();
  EXPECT_TRUE(state_tester_.get_engine()->ComputePairwiseClosestPoints(
      sphere_ids, &results));
  vector<test::IdPair> computed_pairs = {{sphere_ids[0], sphere_ids[1]},
                                   {sphere_ids[0], sphere_ids[2]},
                                   {sphere_ids[0], sphere_ids[3]},
                                   {sphere_ids[1], sphere_ids[2]},
                                   {sphere_ids[1], sphere_ids[3]},
                                   {sphere_ids[2], sphere_ids[3]}};
  ExpectNearestPairs(results, computed_pairs);
}

// Tests the interface where the indices included are explicitly provided.
TEST_F(GeometryEngineStubTest, ComputePairwiseClosestPoints_SelectIndices) {
  SetUpAxisSpheres();
  const std::vector<GeometryId>& sphere_ids =
      state_tester_.get_index_to_id_map();
  // As in SetUpAxisSpheres(), this assumes that the indices are 0 - 3 for the
  // spheres.
  vector<GeometryIndex> query_indices = {GeometryIndex(0),
                                         GeometryIndex(2),
                                         GeometryIndex(3)};
  std::vector<NearestPair<double>> pairs;
  EXPECT_TRUE(state_tester_.get_engine()->ComputePairwiseClosestPoints(
      sphere_ids, query_indices, &pairs));
  vector<test::IdPair> computed_pairs = {{sphere_ids[0], sphere_ids[2]},
                                   {sphere_ids[0], sphere_ids[3]},
                                   {sphere_ids[2], sphere_ids[3]}};
  ExpectNearestPairs(pairs, computed_pairs);
}

// Tests the interface where only explicitly enumerated pairs are included.
TEST_F(GeometryEngineStubTest, ComputePairwiseClosestPoints_SelectPairs) {
  SetUpAxisSpheres();
  const std::vector<GeometryId>& dynamic_ids =
      state_tester_.get_index_to_id_map();
  vector<internal::GeometryIndexPair> query_pairs;
  // As in SetUpAxisSpheres(), this assumes that the indices are 0 - 3 for the
  // spheres.
  query_pairs.emplace_back(GeometryIndex(0), GeometryIndex(1));
  query_pairs.emplace_back(GeometryIndex(2), GeometryIndex(3));
  std::vector<NearestPair<double>> results;
  EXPECT_TRUE(state_tester_.get_engine()->ComputePairwiseClosestPoints(
      dynamic_ids, query_pairs, &results));
  vector<test::IdPair> computed_pairs = {{dynamic_ids[0], dynamic_ids[1]},
                                   {dynamic_ids[2], dynamic_ids[3]}};
  ExpectNearestPairs(results, computed_pairs);
}

// Tests the query determining the closest geometry
TEST_F(GeometryEngineStubTest, FindClosestGeometry) {
  const int kPointCount = 3;
  Matrix3X<double> points;
  points.resize(3, kPointCount);
  SetUpAxisSpheres();
  const std::vector<GeometryId>& sphere_ids =
      state_tester_.get_index_to_id_map();

  vector<PointProximity<double>> results;
  vector<PointProximity<double>> expected;

  // Point 0: Directly below sphere 0 at the origin. Because of 0's rotation
  // the world and local nearest points are different.
  points.block<3, 1>(0, 0) << 0, 0, -1;
  expected.push_back(PointProximity<double>(sphere_ids[0],
                       Vector3<double>(0, -kRadius, 0),
                       Vector3<double>(0, 0, -kRadius),
                       Vector3<double>(0, 0, -1),
                       0.5));

  // Point 1: To the right of sphere 1 (the sphere at <1, 0, 0>).
  points.block<3, 1>(0, 1) << 2, 0, 0;
  expected.push_back(PointProximity<double>(sphere_ids[1],
                        Vector3<double>(kRadius, 0, 0),
                        Vector3<double>(kRadius + 1, 0, 0),
                        Vector3<double>(1, 0, 0),
                        0.5));

  // Point 2: Lies *slightly* inside sphere 2 (the sphere at <0, 1, 0>).
  const double kInset = 0.1;
  points.block<3, 1>(0, 2) << 0, 1 + kRadius - kInset, 0;
  expected.push_back(PointProximity<double>(sphere_ids[2],
                        Vector3<double>(0, kRadius, 0),
                        Vector3<double>(0, 1 + kRadius, 0),
                        Vector3<double>(0, -1, 0),
                        -kInset));

  EXPECT_TRUE(state_tester_.get_engine()->FindClosestGeometry(
      sphere_ids, points, &results));
  EXPECT_EQ(results.size(), expected.size());
  for (int i = 0; i < kPointCount; ++i) {
    EXPECT_EQ(results[i].id_A, expected[i].id_A);
  }
}

// Confirms that the basic condition produces *no* collisions; there is
// separation between all axis spheres.
TEST_F(GeometryEngineStubTest, CollisionsFree) {
  SetUpAxisSpheres();
  std::vector<Contact<double>> contacts;
  std::vector<GeometryId> anchored_ids;
  const std::vector<GeometryId>& dynamic_ids =
      state_tester_.get_index_to_id_map();
  EXPECT_TRUE(state_tester_.get_engine()->ComputeContact(
      dynamic_ids, anchored_ids, &contacts));
  EXPECT_EQ(contacts.size(), 0);
}

// Introduces a new sphere at <x', 0, 0> where x' is midway between the origin
// sphere and the x-axis sphere.  It's radius is such it intersects with both
// spheres. The contact should report two collisions.
TEST_F(GeometryEngineStubTest, CollisionsIntersectingSphere) {
  SetUpAxisSpheres();

  // Create and pose collider sphere.
  Isometry3<double> pose = Isometry3<double>::Identity();
  Vector3<double> p_WO = poses_[0].translation();
  Vector3<double> p_WX = poses_[1].translation();
  // Place at the mid-point.
  pose.translation() = (p_WO + p_WX) / 2;
  poses_.push_back(pose);
  const double collide_depth = kRadius * 0.1;
  const double collider_radius =
      (p_WX - p_WO).norm() + collide_depth - kRadius * 2;

  FrameId frame_id = state_.RegisterFrame(
      source_id_, GeometryFrame<double>("collider",
                                        Isometry3<double>::Identity()));
  GeometryId collider_id = state_.RegisterGeometry(
      source_id_, frame_id,
      make_unique<GeometryInstance<double>>(Isometry3<double>::Identity(),
                                            make_sphere(collider_radius)));
  state_tester_.get_engine()->UpdateWorldPoses(poses_);
  // Perform collision
  std::vector<Contact<double>> contacts;
  std::vector<GeometryId> anchored_ids;
  const std::vector<GeometryId>& dynamic_ids =
      state_tester_.get_index_to_id_map();
  EXPECT_TRUE(state_tester_.get_engine()->ComputeContact(
      dynamic_ids, anchored_ids, &contacts));
  EXPECT_EQ(contacts.size(), 2);
  // Contact between sphere at origin with collider.
  EXPECT_EQ(contacts[0].id_A, state_tester_.get_index_to_id_map()[0]);
  EXPECT_EQ(contacts[0].id_B, collider_id);
  EXPECT_FLOAT_EQ(contacts[0].depth, collide_depth);
  EXPECT_TRUE(CompareMatrices(contacts[0].p_WCa,
                              p_WO + Vector3<double>(kRadius, 0, 0)));
  EXPECT_TRUE(CompareMatrices(
      contacts[0].p_WCb,
      pose.translation() - Vector3<double>(collider_radius, 0, 0)));
  EXPECT_TRUE(CompareMatrices(contacts[0].nhat_AcBc_W,
                              (pose.translation() - p_WO).normalized()));
  // Contact between sphere at <1, 0, 0> with collider.
  EXPECT_EQ(contacts[1].id_A, state_tester_.get_index_to_id_map()[1]);
  EXPECT_EQ(contacts[1].id_B, collider_id);
  EXPECT_FLOAT_EQ(contacts[1].depth, collide_depth);
  EXPECT_TRUE(CompareMatrices(contacts[0].p_WCa,
                              p_WX - Vector3<double>(kRadius, 0, 0)));
  EXPECT_TRUE(CompareMatrices(
      contacts[0].p_WCb,
      pose.translation() - Vector3<double>(collider_radius, 0, 0)));
  EXPECT_TRUE(CompareMatrices(contacts[1].nhat_AcBc_W,
                              (pose.translation() - p_WX).normalized()));
}

// Confirms that half space cannot be added as a dynamic geometry.
TEST_F(GeometryEngineStubTest, AddHalfspaceAsDynamic) {
  SetUpAxisSpheres();
  FrameId frame_id = state_.RegisterFrame(
      source_id_, GeometryFrame<double>("space",
                                        Isometry3<double>::Identity()));
  // Create half space
  Vector3<double> normal = Vector3<double>(1, 1, 1).normalized();
  Vector3<double> point = normal * (-kRadius - 0.1);
  EXPECT_THROW(state_.RegisterGeometry(
      source_id_, frame_id,
      make_unique<GeometryInstance<double>>(
          Isometry3<double>::Identity(),
          make_unique<HalfSpace>(normal, point))), std::logic_error);
}

// This introduces a single half spaces. The half space has a normal in the
// direction <1, 2, 3> but is pushed back so that none of the spheres intersect
// with it.
TEST_F(GeometryEngineStubTest, CollisionsHalfSpaceNoCollide) {
  SetUpAxisSpheres();

  // Create half space
  Vector3<double> normal = Vector3<double>(1, 2, 3);
  Vector3<double> point = normal * (-kRadius - 0.1);
  GeometryId plane_id = state_.RegisterAnchoredGeometry(
      source_id_, make_unique<GeometryInstance<double>>(
                      Isometry3<double>::Identity(),
                      make_unique<HalfSpace>(normal, point)));

  std::vector<GeometryId> anchored_ids;
  anchored_ids.push_back(plane_id);
  std::vector<Contact<double>> contacts;
  const std::vector<GeometryId>& dynamic_ids =
      state_tester_.get_index_to_id_map();
  EXPECT_TRUE(state_tester_.get_engine()->ComputeContact(
      dynamic_ids, anchored_ids, &contacts));
  EXPECT_EQ(contacts.size(), 0);
}

// This introduces a single half spaces. The half space has a normal in the
// direction <1, 2, 3>. It is placed so that it intersects with the sphere
// located at an arbitrary location with a penetration depth of 0.1 m.
// This implicitly tests the auto normalization of the HalfSpace constructor.
TEST_F(GeometryEngineStubTest, CollisionsHalfSpaceCollide) {
  // Add single sphere at the origin.
  source_id_ = SourceId::get_new_id();
  state_.RegisterNewSource(source_id_, "axis-aligned spheres");
  FrameId frame_id = state_.RegisterFrame(
      source_id_, GeometryFrame<double>("sphere" ,
                                        Isometry3<double>::Identity()));
  GeometryId sphere_id = state_.RegisterGeometry(
      source_id_, frame_id,
      make_unique<GeometryInstance<double>>(Isometry3<double>::Identity(),
                                            make_sphere(kRadius)));
  Isometry3<double> pose = Isometry3<double>::Identity();
  Vector3<double> p_WS(0.5, 0.7, 0.9);
  pose.translation() = p_WS;
  poses_.push_back(pose);
  state_tester_.get_engine()->UpdateWorldPoses(poses_);

  // Add half space offset from the sphere such that they intersect.
  const double penetration = 0.1;
  Vector3<double> direction = Vector3<double>(1, 2, 3);
  Vector3<double> normal = direction.normalized();
  Vector3<double> point = pose.translation() - normal * (kRadius - penetration);
  GeometryId plane_id = state_.RegisterAnchoredGeometry(
      source_id_, make_unique<GeometryInstance<double>>(
                      Isometry3<double>::Identity(),
                      make_unique<HalfSpace>(direction, point)));

  std::vector<GeometryId> anchored_ids;
  anchored_ids.push_back(plane_id);
  std::vector<Contact<double>> contacts;
  const std::vector<GeometryId>& dynamic_ids =
      state_tester_.get_index_to_id_map();
  EXPECT_TRUE(state_tester_.get_engine()->ComputeContact(
      dynamic_ids, anchored_ids, &contacts));
  EXPECT_EQ(contacts.size(), 1);
  // Contact between sphere and half plane
  //  Sphere will always be first.
  EXPECT_EQ(contacts[0].id_A, sphere_id);
  EXPECT_EQ(contacts[0].id_B, plane_id);
  EXPECT_FLOAT_EQ(contacts[0].depth, penetration);
  EXPECT_TRUE(CompareMatrices(contacts[0].p_WCa,
                              p_WS - normal * kRadius));
  EXPECT_TRUE(CompareMatrices(
      contacts[0].p_WCb,
      p_WS - normal * (kRadius - penetration), 1e-14));
  EXPECT_TRUE(CompareMatrices(contacts[0].nhat_AcBc_W, -normal));
}

// TODO(SeanCurtis-TRI):
//  1. Intersect with coincident spheres.  Throw an exception.

}  // namespace
}  // namespace geometry
}  // namespace drake
