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

using Eigen::AngleAxisd;
using Eigen::Isometry3d;
using Eigen::Translation3d;
using Eigen::Vector3d;

using std::move;

// Tests for manipulating the population of the proximity engine.  -------------

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

// Tests for copy/move semantics.  ---------------------------------------------

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

// Penetration tests -- testing data flow; not testing the value of the query. -

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
// This tests that collisions that are expected are reported between
//   1. dynamic-anchored
//   2. dynamic-dynamic
// It uses spheres to confirm that the collision results are as expected.
// These are the only sphere-sphere tests because the assumption is that if you
// can get *any* sphere-sphere collision test right, you can get all of them
// right.
// NOTE: FCL does not document the case where two spheres have coincident
// centers; therefore it is not tested here.
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
    ASSERT_EQ(results.size(), 1);
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

// Robust Box-Primitive tests. Tests collision of the box with other primitives
// in a uniform framework. These tests parallel tests located in fcl.

// This performs a very specific test. It collides an oriented box with a
// surface that is tangent to the z = 0 plane. The box is a cube with unit size.
// It is oriented and positioned so that a single corner, placed on the z axis,
// is the point on the cube that most deeply penetrates the tangent plane.
//
// It is approximately *this* picture
//        ┆  ╱╲
//        ┆ ╱  ╲
//        ┆╱    ╲
//       ╱┆╲    ╱
//      ╱ ┆ ╲  ╱
//     ╱  ┆  ╲╱
//     ╲  ┆  ╱
//      ╲ ┆ ╱
//  _____╲┆╱_____    ╱____ With small penetration depth of d
//  ░░░░░░┆░░░░░░    ╲
//  ░░░░░░┆░░░░░░
//  ░░░Tangent░░░
//  ░░░░shape░░░░
//  ░░interior░░░
//  ░░░░░░┆░░░░░░
//
// We can use this against various *convex* shapes to determine uniformity of
// behavior. As long as the convex tangent shape *touches* the z = 0 plane at
// (0, 0, 0), and the shape is *large* compared to the penetration depth, then
// we should get a fixed, known contact. Specifically, if we assume the tangent
// shape is A and the box is B, then
//   - the normal is (0, 0, 1) from plane into the box,
//   - the penetration depth is the specified `depth` used to configure the
//     position of the box, and
//   - the contact position is (0, 0, -depth / 2.
//
// Every convex shape type can be used as the tangent shape as follows:
//   - plane: simply define the z = 0 plane.
//   - box: define a box whose top face lies on the z = 0 and encloses the
//     origin.
//   - sphere: place the sphere at (0, 0 -radius).
//   - cylinder: There are two valid configurations (radius & length >> depth).
//     - Place the cylinder at (0, 0, -length/2).
//     - Rotate the cylinder so that its length axis is parallel with the z = 0
//       plane and the displace downward (0, 0, -radius)
// TODO(SeanCurtis-TRI): Add other shapes as they become available.
//
// NOTE: This *isn't* a ::testing::Test class because we need a bit more control
// over the implementations based on scalar type.
class BoxPenetrationTest : public ::testing::Test {
 protected:
  void SetUp() {
    // Confirm all of the constants are consistent (in case someone tweaks them
    // later).
    EXPECT_GT(kRadius, kDepth * 100);
    EXPECT_GT(kLength, kDepth * 100);

    // Compute the pose of the colliding box.
    // a. Orient the box so that the corner at -x, -y, -z lies in the most -z
    //    extent.
    box_pose_ = AngleAxisd{-M_PI_4, Vector3d::UnitY()} *
        AngleAxisd{M_PI_4, Vector3d::UnitX()};
    // b. Translate it so that the transformed corner lies at (0, 0, -d).
    Vector3d corner{-0.5, -0.5, -0.5};  // The colliding box has unit length.
    Vector3d rotated_corner = box_pose_.linear() * corner;
    Vector3d offset = -rotated_corner + Vector3d{0, 0, -kDepth};
    box_pose_.translation() = offset;
    // c. Test the transform; the initial corner should end up in the expected
    //    position.
    Vector3d target_corner = box_pose_ * corner;
    EXPECT_NEAR(target_corner(0), 0, 1e-15);
    EXPECT_NEAR(target_corner(1), 0, 1e-15);
    EXPECT_NEAR(target_corner(2), -kDepth, 1e-15);

    // Configure the expected penetration characterization.
    expected_penetration_.p_WCa << 0, 0, 0;  // Tangent plane
    expected_penetration_.p_WCb << 0, 0, -kDepth;  // Cube
    expected_penetration_.nhat_BA_W << 0, 0, -1;  // From cube into plane
    expected_penetration_.depth = kDepth;
    // NOTE: The ids are set by the individual calling tests.
  }

  enum TangentShape {
    TangentPlane,
    TangentSphere,
    TangentBox,
    TangentStandingCylinder,
    TangentProneCylinder
  };

  void TestCollision(TangentShape shape, double tolerance) {
    GeometryIndex tangent_index = engine_.AddDynamicGeometry(get_shape(shape));
    GeometryId tangent_id = GeometryId::get_new_id();
    dynamic_map_.push_back(tangent_id);

    GeometryIndex box_index = engine_.AddDynamicGeometry(box_);
    GeometryId box_id = GeometryId::get_new_id();
    dynamic_map_.push_back(box_id);

    // Confirm that there are no other geometries interfering.
    EXPECT_EQ(tangent_index, 0);
    EXPECT_EQ(box_index, 1);

    // Update the poses of the geometry.
    std::vector<Isometry3d> poses(engine_.num_dynamic(),
                                  Isometry3d::Identity());
    poses[tangent_index] = get_pose(shape);
    poses[box_index] = box_pose_;
    engine_.UpdateWorldPoses(poses);
    std::vector<PenetrationAsPointPair<double>> results =
        engine_.ComputePointPairPenetration(dynamic_map_, anchored_map_);

    ASSERT_EQ(results.size(), 1u) << "Against tangent " << shape_name(shape);
    // TODO(SeanCurtis-TRI): Test penetration data.
    const PenetrationAsPointPair<double>& contact = results[0];
    Vector3d normal;
    Vector3d p_Ac;
    Vector3d p_Bc;
    if (contact.id_A == tangent_id && contact.id_B == box_id) {
      // The documented encoding of expected_penetration_.
      normal = expected_penetration_.nhat_BA_W;
      p_Ac = expected_penetration_.p_WCa;
      p_Bc = expected_penetration_.p_WCb;
    } else if ( contact.id_A == box_id && contact.id_B == tangent_id) {
      // The reversed encoding of expected_penetration_.
      normal = -expected_penetration_.nhat_BA_W;
      p_Ac = expected_penetration_.p_WCb;
      p_Bc = expected_penetration_.p_WCa;
    } else {
      GTEST_FAIL() << "Wrong geometry ids reported in contact for tangent "
        << shape_name(shape)
        << ". Expected " << tangent_id << " and " << box_id << ". Got "
        << contact.id_A << " and " << contact.id_B;
    }
    EXPECT_TRUE(CompareMatrices(contact.nhat_BA_W, normal, tolerance))
              << "Against tangent " << shape_name(shape);
    EXPECT_TRUE(CompareMatrices(contact.p_WCa, p_Ac, tolerance))
              << "Against tangent " << shape_name(shape);
    EXPECT_TRUE(CompareMatrices(contact.p_WCb, p_Bc, tolerance))
              << "Against tangent " << shape_name(shape);
    EXPECT_NEAR(contact.depth, expected_penetration_.depth, tolerance)
              << "Against tangent " << shape_name(shape);
  }

  // The pose of the box which should be colliding against the tangent plane.
  Isometry3d box_pose_;

  // Test constants. Geometric measures must be much larger than depth. The test
  // enforces a ratio of at least 100. Using these in a GTEST precludes the
  // possibility of being constexpr initialized.
  static const double kDepth;
  static const double kRadius;
  static const double kLength;

  // The various geometries used in the collision test.
  const Box box_{1, 1, 1};
  const Sphere tangent_sphere_{kRadius};
  const Box tangent_box_{kLength, kLength, kLength};
  const HalfSpace tangent_plane_;  // Default construct the z = 0 plane.
  const Cylinder tangent_cylinder_{kRadius, kLength};

  // The expected collision result -- assumes that A is the tangent object and
  // B is the colliding box.
  PenetrationAsPointPair<double> expected_penetration_;

 private:
  // Map enumeration to string for error messages.
  static const char* shape_name(TangentShape shape) {
    switch (shape) {
      case TangentPlane:
        return "plane";
      case TangentSphere:
        return "sphere";
      case TangentBox:
        return "box";
      case TangentStandingCylinder:
        return "standing cylinder";
      case TangentProneCylinder:
        return "prone cylinder";
    }
    return "undefined shape";
  }

  // Map enumeration to the configured shapes.
  const Shape& get_shape(TangentShape shape) {
    switch (shape) {
      case TangentPlane:
        return tangent_plane_;
      case TangentSphere:
        return tangent_sphere_;
      case TangentBox:
        return tangent_box_;
      case TangentStandingCylinder:
      case TangentProneCylinder:
        return tangent_cylinder_;
    }
  }

  // Map enumeration to tangent pose.
  Isometry3d get_pose(TangentShape shape) {
    Isometry3d pose = Isometry3d::Identity();
    switch (shape) {
      case TangentPlane:
        break;  // leave it at the identity
      case TangentSphere:
        pose.translation() = Vector3d{0, 0, -kRadius};
        break;
      case TangentBox:
      case TangentStandingCylinder:
        pose.translation() = Vector3d{0, 0, -kLength / 2};
        break;
      case TangentProneCylinder:
        pose = AngleAxisd{M_PI_2, Vector3d::UnitX()};
        pose.translation() = Vector3d{0, 0, -kRadius};
        break;
    }
    return pose;
  }

  ProximityEngine<double> engine_;
  std::vector<GeometryId> dynamic_map_;
  std::vector<GeometryId> anchored_map_;
};

// See documentation. All geometry constants must be >= kDepth * 100.
const double BoxPenetrationTest::kDepth = 1e-3;
const double BoxPenetrationTest::kRadius = 1.0;
const double BoxPenetrationTest::kLength = 10.0;
#if 0
TEST_F(BoxPenetrationTest, TangentPlane) {
  TestCollision(TangentPlane, 1e-12);
}

TEST_F(BoxPenetrationTest, TangentBox) {
  TestCollision(TangentBox, 1e-12);
}
#endif
TEST_F(BoxPenetrationTest, TangentSphere) {
  // TODO(SeanCurtis-TRI): There are underlying fcl issues that prevent the
  // collision result from being more precise
  TestCollision(TangentSphere, 1e-12);
}
#if 0
TEST_F(BoxPenetrationTest, TangentStandingCylinder) {
  TestCollision(TangentStandingCylinder, 1e-12);
}

TEST_F(BoxPenetrationTest, TangentProneCylinder) {
  TestCollision(TangentProneCylinder, 1e-12);
}
#endif
#if 0
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

  // TODO(SeanCurtis-TRI): Is this true? Contact points aren't used?
  // Initial conditions - note the contact points p_WCa and p_WCb are not used.
  PenetrationAsPointPair<double> expected{
      box_id, plane_id,  // id_A and id_B, respectively
      Vector3<double>{0, 0, -size / 2},  // p_WCa
      Vector3<double>{0, 0, 0},  // p_WCb
      Vector3<double>{0, 0, 1},  // nhat_BA_W
      size / 2};  // penetration depth

  std::vector<Isometry3<double>> poses{Isometry3<double>::Identity()};
  engine.UpdateWorldPoses(poses);
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

// Explicitly test box-sphere intersection to confirm correct behavior; this
// will catch upstream changes which may break Drake functionality (history
// has shown that FCL testing doesn't necessarily catch bugs).
class BoxSphereTests : public ::testing::Test {
 protected:
  void SetUp() override {
    // Set up sphere as anchored geometry
    AnchoredGeometryIndex sphere_index = engine_.AddAnchoredGeometry(
        Sphere(kRadius), Isometry3<double>::Identity());
    EXPECT_EQ(sphere_index, 0);
    sphere_id_ = GeometryId::get_new_id();
    anchored_map_.push_back(sphere_id_);

    Box box{kSize, kSize, kSize};
    GeometryIndex box_index = engine_.AddDynamicGeometry(box);
    EXPECT_EQ(box_index, 0);
    box_id_ = GeometryId::get_new_id();
    dynamic_map_.push_back(box_id_);

    // A single dynamic pose for the box.
    poses_ = {Isometry3<double>::Identity()};
  }

  // Updates geometry based on current value of dynamic poses, perform collision
  // detection, and report penetration as point pair results.
  std::vector<PenetrationAsPointPair<double>> UpdateAndCollide() {
    engine_.UpdateWorldPoses(poses_);
    return engine_.ComputePointPairPenetration(dynamic_map_, anchored_map_);
  }

  // Utility function for comparing reported penetration against expected
  // penetration. Does it in a way that doesn't care whether the reported
  // pair is (sphere, box) or (box, sphere).
  void ExpectPenetration(const PenetrationAsPointPair<double>& test,
                         const PenetrationAsPointPair<double>& expected,
                         const char* test_source,
                         double threshold = kThreshold) {
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
      GTEST_FAIL() << test_source << ": Test and expected ids don't match. "
                   << "Expected: (" << expected.id_A << ", " << expected.id_B
                   << "), Received: (" << test.id_A << ", " << test.id_B << ")";
    }

    EXPECT_TRUE(CompareMatrices(test.p_WCa, p_WCa, threshold,
                                MatrixCompareType::absolute))
        << test_source << ": contact point on A doesn't match";
    EXPECT_TRUE(CompareMatrices(test.p_WCb, p_WCb, threshold,
                                MatrixCompareType::absolute))
        << test_source << ": contact point on B doesn't match";

    EXPECT_TRUE(CompareMatrices(test.nhat_BA_W, normal, threshold,
                                MatrixCompareType::absolute))
        << test_source << ": normals don't match";
    EXPECT_NEAR(test.depth, expected.depth, threshold)
        << test_source << ": depths don't match";
  };

  // NOTE: gtest and Eigen will attempt to take references of these by default.
  // This disallows inline-defined constexpr values.
  static const double kRadius;
  static const double kSize;
  static const double kThreshold;
  ProximityEngine<double> engine_;
  GeometryId sphere_id_{};
  GeometryId box_id_{};
  std::vector<GeometryId> dynamic_map_;
  std::vector<GeometryId> anchored_map_;
  std::vector<Isometry3<double>> poses_;
};

const double BoxSphereTests::kRadius = 1.0;
const double BoxSphereTests::kSize = 1.0;
const double BoxSphereTests::kThreshold = 1e-14;

// The simple case where the box and sphere *don't* intersect.
TEST_F(BoxSphereTests, BoxSphereNotColliding) {
  // See BoxSphereColliding for the same configuration but with collision.

  // Case 1(a-c): Simply move the box along the world-axes so it is *not* in
  // collision. The closest point should lie *on* the center of the box's face
  const double clear_offset = kRadius + kSize / 2 + kThreshold;

  // Along x-axis
  poses_[0].translation() << clear_offset, 0, 0;
  std::vector<PenetrationAsPointPair<double>> results = UpdateAndCollide();
  EXPECT_EQ(results.size(), 0);

  // Along y-axis
  poses_[0].translation() << 0, clear_offset, 0;
  results = UpdateAndCollide();
  EXPECT_EQ(results.size(), 0);

  // Along z-axis
  poses_[0].translation() << 0, 0, clear_offset;
  results = UpdateAndCollide();
  EXPECT_EQ(results.size(), 0);

  // Case 2(d-f): Rotate the box so that, along the axis of translation, one
  // of the corners is maximally extended. That's a different set of rotation
  // for each axis of displacement, but should give me the *same* clearance.
  //
  // For a given axis, we should have *this* picture
  //        ┆  ╱╲
  //        ┆ ╱  ╲
  //        ┆╱    ╲
  //       ╱┆╲    ╱
  //      ╱ ┆ ╲  ╱
  //     ╱  ┆  ╲╱
  //     ╲  ┆  ╱
  //      ╲ ┆ ╱
  //       ╲┆╱    ╱____ With epsilon separation
  //      **┆**   ╲
  //    **  ┆  **
  //   *    ┆    *
  // ┄*┄┄┄┄┄┆┄┄┄┄┄*┄┄┄┄┄
  //   *    ┆    *
  //    **  ┆  **
  //      **┆**
  //        ┆
  //
  // That requires *just* the right rotation and offset. Considering on corner
  // (-1/2, -1/2, -1/2), we want to put in the most negative x, y, and z
  // positions, respectively.
  // For the x-axis: rotate -45 around z and then 45 around y
  // For the y-axis: rotate 45 around z and then -45 around x
  // For the z-axis: rotate -45 around y and then 45 around x
  //
  // Then for axis A, the box needs to be offset the negative amount of the
  // corner along each other axis.  E.g., If the rotated corner is at:
  // <r_x, r_y, r_z>, then for the x-axis test, the offset will be:
  // <0, -r_y, -r_z>.
  Vector3<double> corner{-kSize / 2, -kSize / 2, -kSize / 2};
  poses_[0].linear() = (AngleAxis<double>{M_PI_4, Vector3<double>::UnitY()} *
                        AngleAxis<double>{-M_PI_4, Vector3<double>::UnitZ()})
                           .matrix();
  Vector3<double> rotated_corner = poses_[0].linear() * corner;
  const double angled_clear_offset =
      rotated_corner.norm() + kThreshold + kRadius;

  // Along x-axis -- uses the defining orientation.
  poses_[0].translation() << angled_clear_offset, -rotated_corner(1),
      -rotated_corner(2);
  results = UpdateAndCollide();
  EXPECT_EQ(results.size(), 0) << "Depth: " << results[0].depth;

  // Along y-axis
  poses_[0].linear() = (AngleAxis<double>{-M_PI_4, Vector3<double>::UnitX()} *
                        AngleAxis<double>{M_PI_4, Vector3<double>::UnitZ()})
                           .matrix();
  rotated_corner = poses_[0].linear() * corner;
  poses_[0].translation() << -rotated_corner(0), angled_clear_offset,
      -rotated_corner(2);
  results = UpdateAndCollide();
  EXPECT_EQ(results.size(), 0) << "Depth: " << results[0].depth;

  // Along z-axis
  poses_[0].linear() = (AngleAxis<double>{M_PI_4, Vector3<double>::UnitX()} *
                        AngleAxis<double>{-M_PI_4, Vector3<double>::UnitY()})
                           .matrix();
  poses_[0].translation() << -rotated_corner(0), -rotated_corner(1),
      angled_clear_offset;
  results = UpdateAndCollide();
  EXPECT_EQ(results.size(), 0) << "Depth: " << results[0].depth;
}

// The simple case where the box and sphere *don't* intersect.
TEST_F(BoxSphereTests, BoxSphereColliding) {
  // TODO(SeanCurtis-TRI): FCL uses GJK algorithm to compute collision between
  // box and sphere. It has *severe* numerical issues that can allow relatively
  // large deviations in the normal definition. As such, we require a larger
  // error threshold.

  // TODO(SeanCurtis-TRI): The GJK solver for fcl has a threshold that is hard
  // coded to 1e-6 (and can't be changed in the CollisionRequest. See FCL
  // issues #278 and #280. Until one or both of those merges, 1e-6 is the
  // shallowest collision that can be detected (and even then there is
  // significant error).
  const double bad_sphere_tolerance = 3e-4;
  const double depth = 1e-4;

  // TODO(SeanCurtis-TRI): Add test that shows shallower collision is not
  // reported with a note saying it should be removed when FCL behavior is
  // fixed.

  PenetrationAsPointPair<double> expected{
      box_id_,
      sphere_id_,                // id_A and id_B, respectively
      Vector3<double>{0, 0, 0},  // p_WCa
      Vector3<double>{0, 0, 0},  // p_WCb
      Vector3<double>{0, 0, 0},  // nhat_BA_W
      depth};                    // penetration depth

  // Case 1(a-c): Simply move the box along the world-axes so it is *not* in
  // collision. The closest point should lie *on* the center of the box's face
  const double offset = kRadius + kSize / 2 - depth;

  // Along x-axis - expect
  expected.p_WCa << kRadius - depth, 0, 0;  // On box
  expected.p_WCb << kRadius, 0, 0;          // On sphere
  expected.nhat_BA_W << 1, 0, 0;            // From sphere, into box
  poses_[0].translation() << offset, 0, 0;
  std::vector<PenetrationAsPointPair<double>> results = UpdateAndCollide();
  ASSERT_EQ(results.size(), 1);
  ExpectPenetration(results[0], expected, "Axis-oriented, x-displacement",
                    bad_sphere_tolerance);

  // Along y-axis
  expected.p_WCa << 0, kRadius - depth, 0;  // On box
  expected.p_WCb << 0, kRadius, 0;          // On sphere
  expected.nhat_BA_W << 0, 1, 0;            // From sphere, into box
  poses_[0].translation() << 0, offset, 0;
  results = UpdateAndCollide();
  ASSERT_EQ(results.size(), 1);
  ExpectPenetration(results[0], expected, "Axis-oriented, y-displacement",
                    bad_sphere_tolerance);

  // Along z-axis
  expected.p_WCa << 0, 0, kRadius - depth;  // On box
  expected.p_WCb << 0, 0, kRadius;          // On sphere
  expected.nhat_BA_W << 0, 0, 1;            // From sphere, into box
  poses_[0].translation() << 0, 0, offset;
  results = UpdateAndCollide();
  ASSERT_EQ(results.size(), 1);
  ExpectPenetration(results[0], expected, "Axis-oriented, z-displacement",
                    bad_sphere_tolerance);

  // Case 2(d-f): Perform the same rotations as in the non-colliding box-sphere
  // tests (BoxSphereNotColliding). The penetration depth should be the same
  // fixed penetration depth.
  Vector3<double> corner{-kSize / 2, -kSize / 2, -kSize / 2};
  poses_[0].linear() = (AngleAxis<double>{M_PI_4, Vector3<double>::UnitY()} *
                        AngleAxis<double>{-M_PI_4, Vector3<double>::UnitZ()})
                           .matrix();
  Vector3<double> rotated_corner = poses_[0].linear() * corner;
  const double angled_clear_offset =
      -rotated_corner(0) + kRadius - depth;

  // Along x-axis -- uses the defining orientation.
  expected.p_WCa << kRadius - depth, 0, 0;  // On box
  expected.p_WCb << kRadius, 0, 0;          // On sphere
  expected.nhat_BA_W << 1, 0, 0;            // From sphere, into box
  poses_[0].translation() << angled_clear_offset, -rotated_corner(1),
      -rotated_corner(2);
  const Vector3<double> moved_corner = poses_[0] * corner;
  // Confirm that the transform puts the intersecting point at (c, 0, 0) where
  // c is depth distance into the sphere's radius.
  EXPECT_NEAR(moved_corner(0), kRadius - depth, 1e-15);
  EXPECT_NEAR(moved_corner(1), 0, 1e-15);
  EXPECT_NEAR(moved_corner(2), 0, 1e-15);

  results = UpdateAndCollide();
  ASSERT_EQ(results.size(), 1);
  ExpectPenetration(results[0], expected, "Angled, x-displacement",
                    bad_sphere_tolerance);
}
#endif
}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
