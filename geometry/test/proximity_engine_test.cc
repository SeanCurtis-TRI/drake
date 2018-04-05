#include "drake/geometry/proximity_engine.h"

#include <utility>

#include <fcl/fcl.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace geometry {
namespace internal {

class ProximityEngineTester {
 public:
  ProximityEngineTester() = delete;

  template <typename T>
  static bool IsDeepCopy(const ProximityEngine<T>& test_engine,
                         const ProximityEngine<T>& ref_engine) {
    return ref_engine.IsDeepCopy(test_engine);
  }

  template <typename T>
  static Vector3<double> GetDynamicTranslation(
      int index, const ProximityEngine<T>& engine) {
    return engine.GetDynamicTranslation(index);
  }

  template <typename T>
  static Vector3<double> GetAnchoredTranslation(
      int index, const ProximityEngine<T>& engine) {
    return engine.GetAnchoredTranslation(index);
  }

  template <typename T>
  static int GetDynamicGeometryIndex(int index,
                                     const ProximityEngine<T>& engine) {
    return engine.GetDynamicGeometryIndex(index);
  }

  template <typename T>
  static int GetAnchoredGeometryIndex(int index,
                                      const ProximityEngine<T>& engine) {
    return engine.GetAnchoredGeometryIndex(index);
  }
};

namespace {

using Eigen::Translation3d;
using std::move;

// Test simple addition of dynamic geometry.
GTEST_TEST(ProximityEngineTests, AddDynamicGeometry) {
  ProximityEngine<double> engine;
  Sphere sphere{0.5};
  GeometryIndex index = engine.AddDynamicGeometry(sphere);
  EXPECT_EQ(index, 0);
  EXPECT_EQ(engine.num_geometries(), 1);
  EXPECT_EQ(engine.num_anchored(), 0);
  EXPECT_EQ(engine.num_dynamic(), 1);
}

// Tests simple addition of anchored geometry.
GTEST_TEST(ProximityEngineTests, AddAchoredGeometry) {
  ProximityEngine<double> engine;
  Sphere sphere{0.5};
  Isometry3<double> pose = Isometry3<double>::Identity();
  AnchoredGeometryIndex index = engine.AddAnchoredGeometry(sphere, pose);
  EXPECT_EQ(index, 0);
  EXPECT_EQ(engine.num_geometries(), 1);
  EXPECT_EQ(engine.num_anchored(), 1);
  EXPECT_EQ(engine.num_dynamic(), 0);
}

// Tests addition of both dynamic and anchored geometry.
GTEST_TEST(ProximityEngineTests, AddMixedGeometry) {
  ProximityEngine<double> engine;
  Sphere sphere{0.5};
  Isometry3<double> pose = Isometry3<double>::Identity();
  AnchoredGeometryIndex a_index = engine.AddAnchoredGeometry(sphere, pose);
  EXPECT_EQ(a_index, 0);
  GeometryIndex g_index = engine.AddDynamicGeometry(sphere);
  EXPECT_EQ(g_index, 0);
  EXPECT_EQ(engine.num_geometries(), 2);
  EXPECT_EQ(engine.num_anchored(), 1);
  EXPECT_EQ(engine.num_dynamic(), 1);
}

// Tests the copy semantics of the ProximityEngine -- the copy is a complete,
// deep copy.
GTEST_TEST(ProximityEngineTests, CopySemantics) {
  ProximityEngine<double> ref_engine;
  Sphere sphere{0.5};
  Isometry3<double> pose = Isometry3<double>::Identity();
  AnchoredGeometryIndex a_index = ref_engine.AddAnchoredGeometry(sphere, pose);
  EXPECT_EQ(a_index, 0);
  GeometryIndex g_index = ref_engine.AddDynamicGeometry(sphere);
  EXPECT_EQ(g_index, 0);

  ProximityEngine<double> copy_construct(ref_engine);
  ProximityEngineTester::IsDeepCopy(copy_construct, ref_engine);

  ProximityEngine<double> copy_assign;
  copy_assign = ref_engine;
  ProximityEngineTester::IsDeepCopy(copy_assign, ref_engine);
}

// Tests the move semantics of the ProximityEngine -- the source is restored to
// default state.
GTEST_TEST(ProximityEngineTests, MoveSemantics) {
  ProximityEngine<double> engine;
  Sphere sphere{0.5};
  Isometry3<double> pose = Isometry3<double>::Identity();
  AnchoredGeometryIndex a_index = engine.AddAnchoredGeometry(sphere, pose);
  EXPECT_EQ(a_index, 0);
  GeometryIndex g_index = engine.AddDynamicGeometry(sphere);
  EXPECT_EQ(g_index, 0);

  ProximityEngine<double> move_construct(move(engine));
  EXPECT_EQ(move_construct.num_geometries(), 2);
  EXPECT_EQ(move_construct.num_anchored(), 1);
  EXPECT_EQ(move_construct.num_dynamic(), 1);
  EXPECT_EQ(engine.num_geometries(), 0);
  EXPECT_EQ(engine.num_anchored(), 0);
  EXPECT_EQ(engine.num_dynamic(), 0);

  ProximityEngine<double> move_assign;
  move_assign = move(move_construct);
  EXPECT_EQ(move_assign.num_geometries(), 2);
  EXPECT_EQ(move_assign.num_anchored(), 1);
  EXPECT_EQ(move_assign.num_dynamic(), 1);
  EXPECT_EQ(move_construct.num_geometries(), 0);
  EXPECT_EQ(move_construct.num_anchored(), 0);
  EXPECT_EQ(move_construct.num_dynamic(), 0);
}

// Penetration tests

// A scene with no geometry reports no penetrations.
GTEST_TEST(ProximityEngineTests, PenetrationOnEmptyScene) {
  ProximityEngine<double> engine;
  std::vector<GeometryId> empty_map;

  auto results = engine.ComputePointPairPenetration(empty_map, empty_map);
  EXPECT_EQ(results.size(), 0);
}

// A scene with a single anchored geometry reports no penetrations.
GTEST_TEST(ProximityEngineTests, PenetrationSingleAnchored) {
  ProximityEngine<double> engine;
  std::vector<GeometryId> dynamic_map;
  std::vector<GeometryId> anchored_map;

  Sphere sphere{0.5};
  Isometry3<double> pose = Isometry3<double>::Identity();
  AnchoredGeometryIndex index = engine.AddAnchoredGeometry(sphere, pose);
  anchored_map.push_back(GeometryId::get_new_id());
  EXPECT_EQ(index, 0);
  auto results = engine.ComputePointPairPenetration(dynamic_map, anchored_map);
  EXPECT_EQ(results.size(), 0);
}

// Tests that anchored geometry aren't collided against each other -- even if
// they actually *are* in penetration.
GTEST_TEST(ProximityEngineTests, PenetrationMultipleAnchored) {
  ProximityEngine<double> engine;
  std::vector<GeometryId> dynamic_map;
  std::vector<GeometryId> anchored_map;

  const double radius = 0.5;
  Sphere sphere{radius};
  Isometry3<double> pose = Isometry3<double>::Identity();
  AnchoredGeometryIndex index1 = engine.AddAnchoredGeometry(sphere, pose);
  anchored_map.push_back(GeometryId::get_new_id());
  EXPECT_EQ(index1, 0);
  pose.translation() << 1.8 * radius, 0, 0;
  AnchoredGeometryIndex index2 = engine.AddAnchoredGeometry(sphere, pose);
  anchored_map.push_back(GeometryId::get_new_id());
  EXPECT_EQ(index2, 1);
  auto results = engine.ComputePointPairPenetration(dynamic_map, anchored_map);
  EXPECT_EQ(results.size(), 0);
}

// These tests validate collisions between spheres. This does *not* test against
// other geometry types because we assume FCL works. This merely confirms that
// the ProximityEngine functions provide the correct mapping.

// Common class for evaluating a simple penetration case between two spheres.
// The variations are in sphere *ownership* (see below).
class SimplePenetrationTest : public ::testing::Test {
 protected:
  // Moves the dynamic sphere to either a penetrating or non-penetrating
  // position. The sphere is indicated by its engine `index` which belongs to
  // the given `source_id`. If `is_colliding` is true, the sphere is placed in
  // a colliding configuration.
  //
  // Non-colliding state
  //       y           x = free_x_
  //        │          │
  //       *│*         o o
  //    *   │   *   o       o
  //   *    │    * o         o
  // ──*────┼────*─o─────────o───────── x
  //   *    │    * o         o
  //    *   │   *   o       o
  //       *│*         o o
  //
  // Colliding state
  //       y       x = colliding_x_
  //        │      │
  //       *│*    o o
  //    *   │  o*      o
  //   *    │ o  *      o
  // ──*────┼─o──*──────o────────────── x
  //   *    │ o  *      o
  //    *   │  o*      o
  //       *│*    o o
  void MoveDynamicSphere(int index, bool is_colliding,
                         ProximityEngine<double>* engine = nullptr) {
    engine = engine == nullptr ? &engine_ : engine;
    std::vector<Isometry3<double>> poses(engine->num_dynamic(),
                                         Isometry3<double>::Identity());
    const double x_pos = is_colliding ? colliding_x_ : free_x_;
    poses[index] = Isometry3<double>(Translation3d{x_pos, 0, 0});
    engine->UpdateWorldPoses(poses);
  }

  // Compute penetration and confirm that a single penetration with the expected
  // properties was found. Provide the engine indices of the sphere located at
  // the origin and the sphere positioned to be in collision.
  template <typename T>
  void ExpectPenetration(GeometryId origin_sphere, GeometryId colliding_sphere,
                         ProximityEngine<T>* engine) {
    std::vector<PenetrationAsPointPair<double>> results =
        engine->ComputePointPairPenetration(dynamic_map_, anchored_map_);
    EXPECT_EQ(results.size(), 1);
    const PenetrationAsPointPair<double> penetration = results[0];

    // There are no guarantees as to the ordering of which element is A and
    // which is B. This test enforces an order for validation.

    // First confirm membership
    EXPECT_TRUE((penetration.id_A == origin_sphere &&
                 penetration.id_B == colliding_sphere) ||
                (penetration.id_A == colliding_sphere &&
                 penetration.id_B == origin_sphere));

    // Assume A => origin_sphere and b => colliding_sphere
    // NOTE: In this current version, penetration is only reported in double.
    PenetrationAsPointPair<double> expected;
    expected.id_A = origin_sphere;  // located at origin
    expected.id_B = colliding_sphere;  // located at [1.5R, 0, 0]
    expected.depth = 2 * radius_ - colliding_x_;
    expected.p_WCa = Vector3<double>{radius_, 0, 0};
    expected.p_WCb = Vector3<double>{colliding_x_ - radius_, 0, 0};
    expected.nhat_BA_W = -Vector3<double>::UnitX();

    // Reverse if previous order assumption is false
    if (penetration.id_A == colliding_sphere) {
      Vector3<double> temp;
      // Swap the indices
      expected.id_A = colliding_sphere;
      expected.id_B = origin_sphere;
      // Swap the points
      temp = expected.p_WCa;
      expected.p_WCa = expected.p_WCb;
      expected.p_WCb = temp;
      // Reverse the normal
      expected.nhat_BA_W = -expected.nhat_BA_W;
      // Penetration depth is same either way; do nothing.
    }

    EXPECT_EQ(penetration.id_A, expected.id_A);
    EXPECT_EQ(penetration.id_B, expected.id_B);
    EXPECT_EQ(penetration.depth, expected.depth);
    EXPECT_TRUE(CompareMatrices(penetration.p_WCa, expected.p_WCa, 1e-13,
                                MatrixCompareType::absolute));
    EXPECT_TRUE(CompareMatrices(penetration.p_WCb, expected.p_WCb, 1e-13,
                                MatrixCompareType::absolute));
    EXPECT_TRUE(CompareMatrices(penetration.nhat_BA_W, expected.nhat_BA_W,
                                1e-13, MatrixCompareType::absolute));
  }

  // Compute penetration and confirm that none were found.
  void ExpectNoPenetration(ProximityEngine<double>* engine = nullptr) {
    engine = engine == nullptr ? &engine_ : engine;
    std::vector<PenetrationAsPointPair<double>> results =
        engine->ComputePointPairPenetration(dynamic_map_, anchored_map_);
    EXPECT_EQ(results.size(), 0);
  }

  ProximityEngine<double> engine_;
  std::vector<GeometryId> dynamic_map_;
  std::vector<GeometryId> anchored_map_;
  const double radius_{0.5};
  const Sphere sphere_{radius_};
  const double free_x_{2.5 * radius_};
  const double colliding_x_{1.5 * radius_};
};

// Tests collision between dynamic and anchored sphere. One case colliding, one
// case *not* colliding.
TEST_F(SimplePenetrationTest, PenetrationDynamicAndAnchored) {
  // Set up anchored geometry
  Isometry3<double> pose = Isometry3<double>::Identity();
  AnchoredGeometryIndex anchored_index =
      engine_.AddAnchoredGeometry(sphere_, pose);
  GeometryId origin_id = GeometryId::get_new_id();
  anchored_map_.push_back(origin_id);
  EXPECT_EQ(anchored_index, 0);

  // Set up dynamic geometry
  GeometryIndex dynamic_index = engine_.AddDynamicGeometry(sphere_);
  GeometryId dynamic_id = GeometryId::get_new_id();
  dynamic_map_.push_back(dynamic_id);
  EXPECT_EQ(dynamic_index, 0);
  EXPECT_EQ(engine_.num_geometries(), 2);

  // Non-colliding case
  MoveDynamicSphere(dynamic_index, false /* not colliding */);
  ExpectNoPenetration();

  // Colliding case
  MoveDynamicSphere(dynamic_index, true /* colliding */);
  ExpectPenetration(origin_id, dynamic_id, &engine_);

  // Test colliding case on copy.
  ProximityEngine<double> copy_engine(engine_);
  ExpectPenetration(origin_id, dynamic_id, &copy_engine);

  // Test AutoDiffXd converted engine
  std::unique_ptr<ProximityEngine<AutoDiffXd>> ad_engine =
      engine_.ToAutoDiffXd();
  ExpectPenetration(origin_id, dynamic_id, ad_engine.get());
}

// Performs the same collision test between two dynamic spheres which belong to
// the same source
TEST_F(SimplePenetrationTest, PenetrationDynamicAndDynamicSingleSource) {
  GeometryIndex origin_index = engine_.AddDynamicGeometry(sphere_);
  GeometryId origin_id = GeometryId::get_new_id();
  dynamic_map_.push_back(origin_id);
  EXPECT_EQ(origin_index, 0);
  std::vector<Isometry3<double>> poses{Isometry3<double>::Identity()};
  engine_.UpdateWorldPoses(poses);

  GeometryIndex collide_index = engine_.AddDynamicGeometry(sphere_);
  GeometryId collide_id = GeometryId::get_new_id();
  dynamic_map_.push_back(collide_id);
  EXPECT_EQ(collide_index, 1);
  EXPECT_EQ(engine_.num_geometries(), 2);

  // Non-colliding case
  MoveDynamicSphere(collide_index, false /* not colliding */);
  ExpectNoPenetration();

  // Colliding case
  MoveDynamicSphere(collide_index, true /* colliding */);
  ExpectPenetration(origin_id, collide_id, &engine_);

  // Test colliding case on copy.
  ProximityEngine<double> copy_engine(engine_);
  ExpectPenetration(origin_id, collide_id, &copy_engine);

  // Test AutoDiffXd converted engine
  std::unique_ptr<ProximityEngine<AutoDiffXd>> ad_engine =
      engine_.ToAutoDiffXd();
  ExpectPenetration(origin_id, collide_id, ad_engine.get());
}

GTEST_TEST(BoxPenetrationTest, BoxPlaneTest) {
  using std::abs;
  ProximityEngine<double> engine;
  std::vector<GeometryId> dynamic_map_;
  std::vector<GeometryId> anchored_map_;

  // Set up anchored geometry
  Isometry3<double> pose = Isometry3<double>::Identity();
  AnchoredGeometryIndex anchored_index =
      engine.AddAnchoredGeometry(HalfSpace(), pose);
  GeometryId plane_id = GeometryId::get_new_id();
  anchored_map_.push_back(plane_id);
  EXPECT_EQ(anchored_index, 0);

  const double size = 1.0;
  Box box{size, size, size};
  GeometryIndex origin_index = engine.AddDynamicGeometry(box);
  GeometryId box_id = GeometryId::get_new_id();
  dynamic_map_.push_back(box_id);
  EXPECT_EQ(origin_index, 0);
  std::vector<Isometry3<double>> poses{Isometry3<double>::Identity()};
  engine.UpdateWorldPoses(poses);

  // Determines if the given point `p_WP` lies on the given `box` with the
  // pose `X_WB` (within a threshold).
  auto expect_point_on_box = [](const Box& box, const Vector3<double>& p_WP,
                                const Isometry3<double>& X_WB,
                                double threshold) {
    Vector3<double> p_BP = X_WB.inverse() * p_WP;
    // Determine which plane it lies on
    if (abs(p_BP(0)) - box.width() / 2 < threshold) {
      EXPECT_LE(abs(p_BP(1)), box.size()(1) / 2);
      EXPECT_LE(abs(p_BP(2)), box.size()(2) / 2);
    } else if (abs(p_BP(1)) - box.depth() / 2 < threshold) {
      EXPECT_LE(abs(p_BP(0)), box.size()(0) / 2);
      EXPECT_LE(abs(p_BP(2)), box.size()(2) / 2);
    } else if (abs(p_BP(2)) - box.height() / 2 < threshold) {
      EXPECT_LE(abs(p_BP(0)), box.size()(0) / 2);
      EXPECT_LE(abs(p_BP(1)), box.size()(1) / 2);
    } else {
      GTEST_FAIL() << "Point doesn't lie on any of the box's planes: ("
        << p_WP(0) << ", " << p_WP(1) << ", " << p_WP(2) << ")";
    }
  };

  // Helper function to confirm two penetration point results are the same.
  // `unique_contact` is true if it is known that the contact points must
  // be unique.
  auto expect_eq_results = [&box, expect_point_on_box](
      const auto& test, const auto& expected, const Isometry3<double>& X_WB) {
    Vector3<double> normal;
    Vector3<double> p_WCa;
    Vector3<double> p_WCb;
    if (test.id_A == expected.id_A && test.id_B == expected.id_B) {
      normal = expected.nhat_BA_W;
      p_WCa = test.p_WCa;
      p_WCb = test.p_WCb;
    } else if (test.id_A == expected.id_B && test.id_B == expected.id_A) {
      normal = -expected.nhat_BA_W;
      p_WCa = test.p_WCb;
      p_WCb = test.p_WCa;
    } else {
      GTEST_FAIL() << "Test and expected ids don't match";
    }
    // Normal and depth should be "exact" match.
    EXPECT_TRUE(CompareMatrices(test.nhat_BA_W, normal, 1e-13,
                                MatrixCompareType::absolute));
    EXPECT_EQ(test.depth, expected.depth);
    // If the deepest point is not unique (i.e., if it lies on an face or edge)
    // we can't know exactly what contact points will be reported.
    // However, we can determine if p_WCa lies on the box, and if p_WCb is the
    // projection of P_WCa onto the z=0 plane.
    expect_point_on_box(box, p_WCa, X_WB, 1e-13);
    EXPECT_TRUE(CompareMatrices(p_WCb, Vector3<double>{p_WCa(0), p_WCa(1), 0},
                                1e-13, MatrixCompareType::absolute));
  };

  // Initial conditions - note the contact points are *not* used.
  PenetrationAsPointPair<double> expected{
      box_id, plane_id, Vector3<double>{0, 0, -size / 2},
      Vector3<double>{0, 0, 0}, Vector3<double>{0, 0, 1}, size / 2};

  std::vector<PenetrationAsPointPair<double>> results =
      engine.ComputePointPairPenetration(dynamic_map_, anchored_map_);
  EXPECT_EQ(results.size(), 1);
  // Face-face collision.
  expect_eq_results(results[0], expected, poses[0]);

  // Move box half the distance out of collision -- everything should be the
  // same except half distance.
  poses[0] = Isometry3<double>{Translation3<double>{0, 0, size / 4}};
  engine.UpdateWorldPoses(poses);
  results = engine.ComputePointPairPenetration(dynamic_map_, anchored_map_);
  EXPECT_EQ(results.size(), 1);
  expected.depth = size / 4;
  // face-face collision.
  expect_eq_results(results[0], expected, poses[0]);

  // Move box out of collision.
  poses[0] = Isometry3<double>{Translation3<double>{0, 0, size}};
  engine.UpdateWorldPoses(poses);
  results = engine.ComputePointPairPenetration(dynamic_map_, anchored_map_);
  EXPECT_EQ(results.size(), 0);

  // Rotate the box 45 degrees around the y-axis.
  poses[0] =
      Isometry3<double>{AngleAxis<double>(M_PI_4, Vector3<double>::UnitY())};
  engine.UpdateWorldPoses(poses);
  results = engine.ComputePointPairPenetration(dynamic_map_, anchored_map_);
  EXPECT_EQ(results.size(), 1);
  expected.depth = size / sqrt(2);
  // Edge face collision
  expect_eq_results(results[0], expected, poses[0]);

  // Rotate the box 45 degrees around the y-axis and then around the x-axis.
  // Puts a single point deeply into the plane
  poses[0] =
      Isometry3<double>{AngleAxis<double>(M_PI_4, Vector3<double>::UnitX())} *
      Isometry3<double>{AngleAxis<double>(M_PI_4, Vector3<double>::UnitY())};
  engine.UpdateWorldPoses(poses);
  results = engine.ComputePointPairPenetration(dynamic_map_, anchored_map_);
  EXPECT_EQ(results.size(), 1);
  expected.depth = size * (sqrt(2) + 2) / 4;
  // Edge face collision
  expect_eq_results(results[0], expected, poses[0]);
}

// This should be the sme as the PlaneBoxTest -- except the role of plane is
// played by a large box. Same results, but different code path.
GTEST_TEST(BoxPenetration, BoxBoxTest) {
  using std::abs;
  ProximityEngine<double> engine;
  std::vector<GeometryId> dynamic_map_;
  std::vector<GeometryId> anchored_map_;

  // Set up anchored geometry
  const double plane_size = 30;
  Isometry3<double> pose{Translation3<double>{0, 0, -plane_size / 2}};
  AnchoredGeometryIndex anchored_index =
      engine.AddAnchoredGeometry(Box(plane_size, plane_size, plane_size), pose);
  EXPECT_EQ(anchored_index, 0);
  GeometryId plane_id = GeometryId::get_new_id();
  anchored_map_.push_back(plane_id);

  const double size = 1.0;
  Box box{size, size, size};
  GeometryIndex origin_index = engine.AddDynamicGeometry(box);
  EXPECT_EQ(origin_index, 0);
  GeometryId box_id = GeometryId::get_new_id();
  dynamic_map_.push_back(box_id);

  // Determines if the given point `p_WP` lies on the given `box` with the
  // pose `X_WB` (within a threshold).
  auto expect_point_on_box = [](const Box& box, const Vector3<double>& p_WP,
                                const Isometry3<double>& X_WB,
                                double threshold) {
    Vector3<double> p_BP = X_WB.inverse() * p_WP;
    // Determine which plane it lies on
    if (abs(p_BP(0)) - box.width() / 2 < threshold) {
      EXPECT_LE(abs(p_BP(1)), box.size()(1) / 2);
      EXPECT_LE(abs(p_BP(2)), box.size()(2) / 2);
    } else if (abs(p_BP(1)) - box.depth() / 2 < threshold) {
      EXPECT_LE(abs(p_BP(0)), box.size()(0) / 2);
      EXPECT_LE(abs(p_BP(2)), box.size()(2) / 2);
    } else if (abs(p_BP(2)) - box.height() / 2 < threshold) {
      EXPECT_LE(abs(p_BP(0)), box.size()(0) / 2);
      EXPECT_LE(abs(p_BP(1)), box.size()(1) / 2);
    } else {
      GTEST_FAIL() << "Point doesn't lie on any of the box's planes: ("
                   << p_WP(0) << ", " << p_WP(1) << ", " << p_WP(2) << ")";
    }
  };

  // Helper function to confirm two penetration point results are the same.
  // `unique_contact` is true if it is known that the contact points must
  // be unique.
  auto expect_eq_results = [&box, expect_point_on_box](
      const auto& test, const auto& expected, const Isometry3<double>& X_WB) {
    Vector3<double> normal;
    Vector3<double> p_WCa;
    Vector3<double> p_WCb;
    if (test.id_A == expected.id_A && test.id_B == expected.id_B) {
      normal = expected.nhat_BA_W;
      p_WCa = test.p_WCa;
      p_WCb = test.p_WCb;
    } else if (test.id_A == expected.id_B && test.id_B == expected.id_A) {
      normal = -expected.nhat_BA_W;
      p_WCa = test.p_WCb;
      p_WCb = test.p_WCa;
    } else {
      GTEST_FAIL() << "Test and expected ids don't match. Expected: ("
          << expected.id_A << ", " << expected.id_B << "), Received: ("
          << test.id_A << ", " << test.id_B << ")";
    }
    // Normal and depth should be "exact" match.
    EXPECT_TRUE(CompareMatrices(test.nhat_BA_W, normal, 1e-13,
                                MatrixCompareType::absolute));
    EXPECT_NEAR(test.depth, expected.depth, 1e-13);
    // If the deepest point is not unique (i.e., if it lies on an face or edge)
    // we can't know exactly what contact points will be reported.
    // However, we can determine if p_WCa lies on the box, and if p_WCb is the
    // projection of P_WCa onto the z=0 plane.
    expect_point_on_box(box, p_WCa, X_WB, 1e-13);
    EXPECT_TRUE(CompareMatrices(p_WCb, Vector3<double>{p_WCa(0), p_WCa(1), 0},
                                1e-13, MatrixCompareType::absolute));
  };

  // Initial conditions - note the contact points are *not* used.
  PenetrationAsPointPair<double> expected{
      box_id, plane_id,  // id_A and id_B, respectively
      Vector3<double>{0, 0, -size / 2},  // p_WCa
      Vector3<double>{0, 0, 0},  // p_WCb
      Vector3<double>{0, 0, 1},  // nhat_BA_W
      size / 2};  // penetration depth

  std::cout << "Box id: " << box_id << ", plane id: " << plane_id << "\n";
  std::vector<Isometry3<double>> poses{Isometry3<double>::Identity()};
  engine.UpdateWorldPoses(poses);
  std::vector<PenetrationAsPointPair<double>> results =
      engine.ComputePointPairPenetration(dynamic_map_, anchored_map_);
  EXPECT_EQ(results.size(), 1);
  // Face-face collision.
  std::cerr << "Case 1\n";
  std::cerr << "  A:     " << results[0].id_A << "\n";
  std::cerr << "  B:     " << results[0].id_B << "\n";
  std::cerr << "  p_WCa: " << results[0].p_WCa.transpose() << "\n";
  std::cerr << "  p_WCb: " << results[0].p_WCb.transpose() << "\n";
  std::cerr << "  n_hat: " << results[0].nhat_BA_W.transpose() << "\n";
  std::cerr << "  d:     " << results[0].depth << "\n";
  expect_eq_results(results[0], expected, poses[0]);

  // Move box half the distance out of collision -- everything should be the
  // same except half distance.
  poses[0] = Isometry3<double>{Translation3<double>{0, 0, size / 4}};
  engine.UpdateWorldPoses(poses);
  results = engine.ComputePointPairPenetration(dynamic_map_, anchored_map_);
  EXPECT_EQ(results.size(), 1);
  expected.depth = size / 4;
  // face-face collision.
  expect_eq_results(results[0], expected, poses[0]);

  // Move box out of collision.
  poses[0] = Isometry3<double>{Translation3<double>{0, 0, size}};
  engine.UpdateWorldPoses(poses);
  results = engine.ComputePointPairPenetration(dynamic_map_, anchored_map_);
  EXPECT_EQ(results.size(), 0);

  // Rotate the box 45 degrees around the y-axis.
  poses[0] =
      Isometry3<double>{AngleAxis<double>(M_PI_4, Vector3<double>::UnitY())};
  engine.UpdateWorldPoses(poses);
  results = engine.ComputePointPairPenetration(dynamic_map_, anchored_map_);
  EXPECT_EQ(results.size(), 1);
  expected.depth = size / sqrt(2);
  // Edge-face collision
  expect_eq_results(results[0], expected, poses[0]);

  // Rotate the box 45 degrees around the y-axis and then around the x-axis.
  // Puts a single point deeply into the plane
  poses[0] =
      Isometry3<double>{AngleAxis<double>(M_PI_4, Vector3<double>::UnitX())} *
      Isometry3<double>{AngleAxis<double>(M_PI_4, Vector3<double>::UnitY())};
  engine.UpdateWorldPoses(poses);
  results = engine.ComputePointPairPenetration(dynamic_map_, anchored_map_);
  EXPECT_EQ(results.size(), 1);
  expected.depth = size * (sqrt(2) + 2) / 4;
  // Vertex-vertex collision
  expect_eq_results(results[0], expected, poses[0]);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
