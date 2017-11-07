#include "drake/geometry/geometry_engine_stub.h"

#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/is_dynamic_castable.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/test/query_test_utility.h"

namespace drake {
namespace geometry {

template <typename T>
class GeometryEngineStubTester {
 public:
  explicit GeometryEngineStubTester(const GeometryEngineStub<T>* engine)
      : engine_(engine) {}

  const std::vector<copyable_unique_ptr<stub_shapes::EngineShape<T>>>&
  get_owned_geometries() const {
    return engine_->owned_geometries_;
  }
  const std::vector<stub_shapes::EngineShape<T>*>& get_geometries() const {
    return engine_->geometries_;
  }
  const std::vector<stub_shapes::EngineShape<T>*>& get_anchored_geometries()
  const {
    return engine_->anchored_geometries_;
  }

 private:
  const GeometryEngineStub<T>* engine_;
};

namespace {

using std::make_unique;
using std::vector;
using stub_shapes::EngineHalfSpace;
using stub_shapes::EngineShape;
using stub_shapes::EngineSphere;
using stub_shapes::OwnedIndex;

using GEngine = GeometryEngineStub<double>;

class GeometryEngineStubTest : public test::GeometryQueryTest {};

// Test that the engine sphere is transmogrifiable.
GTEST_TEST(EngineShapeTest, TransmogrifySphere) {
  double radius = 1.5;
  OwnedIndex index{1};
  EngineSphere<double> sphere_d{index, radius};
  EngineShape<double>* base = &sphere_d;
  EXPECT_TRUE(is_dynamic_castable<EngineSphere<double>>(base));
  auto base_a = sphere_d.ToAutoDiff();
  EXPECT_TRUE(is_dynamic_castable<EngineSphere<AutoDiffXd>>(base_a));
  EngineSphere<AutoDiffXd>* sphere_a =
      dynamic_cast<EngineSphere<AutoDiffXd>*>(base_a.get());
  EXPECT_EQ(sphere_a->radius(), sphere_d.radius());
}

// Test that the engine half space is transmogrifiable.
GTEST_TEST(EngineShapeTest, TransmogrifyHalfSpace) {
  Vector3<double> normal(1, 2, 3);
  normal.normalize();
  Vector3<double> point(10, 20, 30);
  OwnedIndex index{1};
  EngineHalfSpace<double> half_space_d{index, normal, point};
  EngineHalfSpace<double>* base = &half_space_d;
  EXPECT_TRUE(is_dynamic_castable<EngineHalfSpace<double>>(base));
  auto base_a = half_space_d.ToAutoDiff();
  EXPECT_TRUE(is_dynamic_castable<EngineHalfSpace<AutoDiffXd>>(base_a));
  EngineHalfSpace<AutoDiffXd>* half_space_a =
      dynamic_cast<EngineHalfSpace<AutoDiffXd>*>(base_a.get());
  auto& normal_ad = half_space_a->normal();
  for (int i = 0; i < 3; ++i)
    EXPECT_EQ(ExtractDoubleOrThrow(normal_ad[0]), half_space_d.normal()[0]);
  EXPECT_EQ(ExtractDoubleOrThrow(half_space_a->d()), half_space_d.d());
}

// Transmogrify the stub engine, bringing all of the registered geometry along.
// Performs a simple distance query and confirms the derivative is as expected.
TEST_F(GeometryEngineStubTest, TransmogrifyEngine) {
  // Set up the experiment.
  SetUpAxisSpheres();
  GeometryEngineStub<double>* stub_engine =
      dynamic_cast<GeometryEngineStub<double>*>(state_tester_.get_engine());
  ASSERT_NE(stub_engine, nullptr);
  std::vector<NearestPair<AutoDiffXd>> results;
  vector<GeometryIndex> query_indices = {GeometryIndex(2),
                                         GeometryIndex(1)};
  const std::vector<GeometryId>& sphere_ids =
      state_tester_.get_index_to_id_map();

  // Transmogrify and set the derivatives for sphere 1's position.
  std::unique_ptr<GeometryEngineStub<AutoDiffXd>> engine_ad{
      dynamic_cast<GeometryEngineStub<AutoDiffXd>*>(
          stub_engine->ToAutoDiff().release())};
  for (int i = 0; i < 3; ++i) {
    VectorX<double> derivs = VectorX<double>::Zero(3);
    derivs(i) = 1;
    engine_ad->get_mutable_poses()[1].translation()(i).derivatives() = derivs;
  }

  EXPECT_TRUE(engine_ad->ComputePairwiseClosestPoints(sphere_ids, query_indices,
                                                      &results));
  EXPECT_EQ(results.size(), 1u);

  // Examine the gradient of the distance between the two bodies (spheres with
  // centers and radii of a, aᵣ, b, and, bᵣ, respectively).
  //
  // d = ‖ᵇrᵃ‖₂ - aᵣ - bᵣ = ‖a - b‖₂ - aᵣ - bᵣ
  //
  // The gradient of this quantity w.r.t. the position of body a is:
  //
  // ∇ₐd = <a₀ - b₀, a₁ - b₁, a₂ - b₂> / ‖ᵇrᵃ‖₂
  //
  // In this case, bodies a and b are at:
  //  a: [1, 0, 0]
  //  b: [0, 1, 0]
  // So, the expected gradient is:
  //
  // ∇ₐd = <1 - 0, 0 - 1, 0 - 0> / √2 = <1, -1, 0> / √2

  EXPECT_EQ(results[0].distance.derivatives()(0), 1 / sqrt(2));
  EXPECT_EQ(results[0].distance.derivatives()(1), -1 / sqrt(2));
  EXPECT_EQ(results[0].distance.derivatives()(2), 0);
}

TEST_F(GeometryEngineStubTest, Constructor) {
  GEngine engine;
  EXPECT_EQ(engine.get_update_input_size(), 0);
}

// Confirms that the copy constructor creates the appropriate, independent stub.
TEST_F(GeometryEngineStubTest, CopyConstructor) {
  SetUpAxisSpheres();
  state_->RegisterAnchoredGeometry(
      source_id_, make_unique<GeometryInstance>(Isometry3<double>::Identity(),
                                                make_sphere(1.0)));

  GEngine engine(*dynamic_cast<GEngine*>(state_tester_.get_engine()));
  GeometryEngineStubTester<double> src(
      dynamic_cast<GEngine*>(state_tester_.get_engine()));
  GeometryEngineStubTester<double> copy(&engine);

  // Confirms that the *owned* data is equivalent but different instances.
  const auto& src_owned = src.get_owned_geometries();
  const auto& copy_owned = copy.get_owned_geometries();
  EXPECT_EQ(copy_owned.size(), src_owned.size());
  for (size_t i = 0; i < copy_owned.size(); ++i) {
    EXPECT_NE(copy_owned[i].get(), src_owned[i].get());
    EXPECT_EQ(copy_owned[i]->get_type(), src_owned[i]-> get_type());
    EXPECT_EQ(copy_owned[i]->get_index(), src_owned[i]->get_index());
    EXPECT_EQ(copy_owned[i]->get_index(), i);
  }

  // Confirms that the *geometry* data is equivalent but different instances.
  const auto& src_geometry = src.get_geometries();
  const auto& copy_geometry = copy.get_geometries();
  EXPECT_EQ(copy_geometry.size(), src_geometry.size());
  for (size_t i = 0; i < copy_geometry.size(); ++i) {
    EXPECT_NE(copy_geometry[i], src_geometry[i]);
    OwnedIndex owned_index = copy_geometry[i]->get_index();
    EXPECT_EQ(copy_owned[owned_index].get(), copy_geometry[i]);
  }

  // Confirms that the *geometry* data is equivalent but different instances.
  const auto& src_anchored = src.get_anchored_geometries();
  const auto& copy_anchored = copy.get_anchored_geometries();
  EXPECT_EQ(copy_anchored.size(), src_anchored.size());
  for (size_t i = 0; i < copy_anchored.size(); ++i) {
    EXPECT_NE(copy_anchored[i], src_anchored[i]);
    OwnedIndex owned_index = copy_anchored[i]->get_index();
    EXPECT_EQ(copy_owned[owned_index].get(), copy_anchored[i]);
  }
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
  vector<GeometryIndexPair> query_pairs;
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
  std::vector<GeometryId> anchored_ids;
  const std::vector<GeometryId>& dynamic_ids =
      state_tester_.get_index_to_id_map();
  std::vector<PenetrationAsPointPair<double>> penetrations =
      state_tester_.get_engine()->ComputePenetration(dynamic_ids, anchored_ids);
  EXPECT_EQ(penetrations.size(), 0);
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

  FrameId frame_id = state_->RegisterFrame(
      source_id_, GeometryFrame("collider", Isometry3<double>::Identity()));
  GeometryId collider_id = state_->RegisterGeometry(
      source_id_, frame_id,
      make_unique<GeometryInstance>(Isometry3<double>::Identity(),
                                    make_sphere(collider_radius)));
  state_tester_.get_engine()->UpdateWorldPoses(poses_);
  // Perform collision
  std::vector<GeometryId> anchored_ids;
  const std::vector<GeometryId>& dynamic_ids =
      state_tester_.get_index_to_id_map();
  std::vector<PenetrationAsPointPair<double>> penetrations =
      state_tester_.get_engine()->ComputePenetration(dynamic_ids, anchored_ids);
  EXPECT_EQ(penetrations.size(), 2);
  // Contact between sphere at origin with collider.
  EXPECT_EQ(penetrations[0].id_A, state_tester_.get_index_to_id_map()[0]);
  EXPECT_EQ(penetrations[0].id_B, collider_id);
  EXPECT_FLOAT_EQ(penetrations[0].depth, collide_depth);
  EXPECT_TRUE(CompareMatrices(penetrations[0].p_WCa,
                              p_WO + Vector3<double>(kRadius, 0, 0)));
  EXPECT_TRUE(CompareMatrices(
      penetrations[0].p_WCb,
      pose.translation() - Vector3<double>(collider_radius, 0, 0)));
  EXPECT_TRUE(CompareMatrices(penetrations[0].nhat_AB_W,
                              (pose.translation() - p_WO).normalized()));
  // Contact between sphere at <1, 0, 0> with collider.
  EXPECT_EQ(penetrations[1].id_A, state_tester_.get_index_to_id_map()[1]);
  EXPECT_EQ(penetrations[1].id_B, collider_id);
  EXPECT_FLOAT_EQ(penetrations[1].depth, collide_depth);
  EXPECT_TRUE(CompareMatrices(penetrations[0].p_WCa,
                              p_WX - Vector3<double>(kRadius, 0, 0)));
  EXPECT_TRUE(CompareMatrices(
      penetrations[0].p_WCb,
      pose.translation() - Vector3<double>(collider_radius, 0, 0)));
  EXPECT_TRUE(CompareMatrices(penetrations[1].nhat_AB_W,
                              (pose.translation() - p_WX).normalized()));
}

// This introduces a single half spaces. The half space has a normal in the
// direction <1, 2, 3> but is pushed back so that none of the spheres intersect
// with it.
TEST_F(GeometryEngineStubTest, CollisionsHalfSpaceNoCollide) {
  SetUpAxisSpheres();

  // Create half space
  Vector3<double> normal = Vector3<double>(1, 2, 3);
  Vector3<double> point = normal * (-kRadius - 0.1);
  GeometryId plane_id = state_->RegisterAnchoredGeometry(
      source_id_, make_unique<GeometryInstance>(
                      HalfSpace::MakePose(normal, point),
                      make_unique<HalfSpace>()));

  std::vector<GeometryId> anchored_ids;
  anchored_ids.push_back(plane_id);
  const std::vector<GeometryId>& dynamic_ids =
      state_tester_.get_index_to_id_map();
  std::vector<PenetrationAsPointPair<double>> penetrations =
      state_tester_.get_engine()->ComputePenetration(dynamic_ids, anchored_ids);
  EXPECT_EQ(penetrations.size(), 0);
}

// This introduces a single half spaces. The half space has a normal in the
// direction <1, 2, 3>. It is placed so that it intersects with the sphere
// located at an arbitrary location with a penetration depth of 0.1 m.
// This implicitly tests the auto normalization of the HalfSpace constructor.
TEST_F(GeometryEngineStubTest, CollisionsHalfSpaceCollide) {
  // Add single sphere at the origin.
  source_id_ = state_->RegisterNewSource("axis-aligned spheres");
  FrameId frame_id = state_->RegisterFrame(
      source_id_, GeometryFrame("sphere", Isometry3<double>::Identity()));
  GeometryId sphere_id = state_->RegisterGeometry(
      source_id_, frame_id,
      make_unique<GeometryInstance>(Isometry3<double>::Identity(),
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
  GeometryId plane_id = state_->RegisterAnchoredGeometry(
      source_id_,
      make_unique<GeometryInstance>(HalfSpace::MakePose(normal, point),
                                    make_unique<HalfSpace>()));

  std::vector<GeometryId> anchored_ids;
  anchored_ids.push_back(plane_id);
  const std::vector<GeometryId>& dynamic_ids =
      state_tester_.get_index_to_id_map();
  std::vector<PenetrationAsPointPair<double>> penetrations =
      state_tester_.get_engine()->ComputePenetration(dynamic_ids, anchored_ids);
  EXPECT_EQ(penetrations.size(), 1);
  // Contact between sphere and half plane
  //  Sphere will always be first.
  EXPECT_EQ(penetrations[0].id_A, sphere_id);
  EXPECT_EQ(penetrations[0].id_B, plane_id);
  EXPECT_FLOAT_EQ(penetrations[0].depth, penetration);
  EXPECT_TRUE(CompareMatrices(penetrations[0].p_WCa,
                              p_WS - normal * kRadius));
  EXPECT_TRUE(CompareMatrices(
      penetrations[0].p_WCb,
      p_WS - normal * (kRadius - penetration), 1e-14));
  EXPECT_TRUE(CompareMatrices(penetrations[0].nhat_AB_W, -normal));
}

// TODO(SeanCurtis-TRI):
//  1. Intersect with coincident spheres.  Throw an exception.

}  // namespace
}  // namespace geometry
}  // namespace drake
