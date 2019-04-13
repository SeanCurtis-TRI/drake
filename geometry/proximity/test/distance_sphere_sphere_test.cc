#include "drake/geometry/proximity/distance_sphere_sphere.h"

#include <limits>
#include <memory>

#include <fcl/fcl.h>
#include <gtest/gtest.h>

#include "drake/math/autodiff_gradient.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace geometry {
namespace internal {
namespace signed_distance {
namespace {

using Eigen::Vector3d;
using math::RigidTransform;
using math::RigidTransformd;
using math::RotationMatrix;
using std::make_unique;
using std::unique_ptr;

// Performs a point-to-shape signed-distance query and tests the result. This
// particularly focuses on the AutoDiff-valued version of these methods. It
// does a limited smoke-test on the results.
//
// It computes ddistance_dp_WQ and compares it with the reported grad_W value
// with the assumption that they should be the same.
//
// The caller provides the nearest point p_GN_G (in the geometry frame G) and
// the offset from the nearest point to the query point Q (p_NQ_G) also in the
// geometry frame G. The query point is inferred from these two values. It
// performs the query and examines the results. It generally assumes
// non-negative signed distance. If the point is inside (i.e., negative signed
// distance), then the additional boolean `is_inside` should be set to true.
// To test robustness, the frame G can have an arbitrary pose in the world
// frame (defined by X_WG). Finally, an absolute tolerance is provided to
// define the scope of correctness.
//
// Note: this is *not* the complete test; we should also confirm that the
// derivatives of grad_W and the witness points are likewise correct.
// TODO(hongkai.dai): Extend these tests to test the AutoDiff derivatives of
// the reported gradient and witness points w.r.t. arbitrary basis as well.
template<typename ShapeA, typename ShapeB>
class ShapeShapeAutoDiffSignedDistanceTester {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ShapeShapeAutoDiffSignedDistanceTester)

  // Constructs a tester for a given shape G, pose in world X_WG, and tolerance.
  // The shape must be non-null and must persist beyond the life of the tester
  // as a reference to the shape will be stored.
  ShapeShapeAutoDiffSignedDistanceTester(const ShapeA* shapeA,
                                         const ShapeB* shapeB,
                                         const RigidTransformd& X_WB,
                                         double tolerance)
      : shapeA_(*shapeA),
        shapeB_(*shapeB),
        X_WB_(X_WB),
        tolerance_(tolerance) {}

  // Perform the test with the particular N and Q.
  ::testing::AssertionResult Test(const Vector3d& p_WA_W,
                                  const Vector3d& p_BCb_W,
                                  const Vector3d& p_CbCa_W,
                                  bool is_inside = false) {
    const double sign = is_inside ? -1 : 1;
    const double expected_distance = sign * p_CbCa_W.norm();

    ::testing::AssertionResult failure = ::testing::AssertionFailure();
    bool error = false;

    Vector3<AutoDiffXd> p_WA_W_ad = math::initializeAutoDiff(p_WA_W);
    RigidTransform<AutoDiffXd> X_WA_ad = RigidTransform<AutoDiffXd>(p_WA_W_ad);

    Witness<AutoDiffXd> witness =
        Compute(shapeA_, X_WA_ad, shapeB_, X_WB_.cast<AutoDiffXd>());
    if (std::abs(witness.distance.value() - expected_distance) > tolerance_) {
      error = true;
      failure << "The difference between expected distance and tested distance "
                 "is greater than the given tolerance:\n"
              << "  Expected distance: " << expected_distance << "\n"
              << "  Tested distance: " << witness.distance.value() << "\n"
              << "  tolerance: " << tolerance_ << "\n"
              << "  difference: "
              << (std::abs(witness.distance.value() - expected_distance));
    }
    // The hand-computed `nhat_BA_F` value should match the autodiff-computed
    // gradient.
    if (witness.distance.derivatives().size() != 3) {
      if (error) failure << "\n";
      error = true;
      failure << "Test distance has no derivatives";
    }

    const Vector3d ddistance_dp_WQ = witness.distance.derivatives();
    const Vector3d nhat_BA_F = math::autoDiffToValueMatrix(witness.nhat_BA_F);
    if (nhat_BA_F.array().isNaN().any()) {
      if (error) failure << "\n";
      error = true;
      failure << "Hand-computed gradient contains NaN: "
              << nhat_BA_F.transpose();
    }
    auto gradient_compare =
        CompareMatrices(ddistance_dp_WQ, nhat_BA_F, tolerance_);
    if (!gradient_compare) {
      if (error) failure << "\n";
      error = true;
      failure << "ddistance_dp_WQ does not match nhat_BA_F\n"
              << gradient_compare.message();
    }

    if (!error) return ::testing::AssertionSuccess();
    return failure;
  }

 private:
  const ShapeA& shapeA_;
  const ShapeB& shapeB_;
  const RigidTransformd X_WB_;
  const double tolerance_{std::numeric_limits<double>::epsilon()};
};

class SphereSphereTest : public ::testing::Test {
 protected:
  using Tester =
      ShapeShapeAutoDiffSignedDistanceTester<fcl::Sphered, fcl::Sphered>;

  // Runs a test to compute the signed distance between spheres. Sphere B has
  // a hard-coded radius and is given  some arbitrary, non-identity pose.
  // Sphere A is likewise centered away from sphere B in some arbitrary,
  // non-trivial direction. The radius of A and the signed distance *between*
  // surfaces is given.
  //
  // The scene is configured, the signed distance evaluated, and the results
  // evaluated using ShapeShapeAutoDiffSignedDistanceTester.
  ::testing::AssertionResult RunTest(double radius_A, double signed_distance) {
    fcl::Sphered sphere_A{radius_A};
    fcl::Sphered sphere_B{kRadiusB};
    const RotationMatrix<double> R_WB(
        AngleAxis<double>(M_PI / 5, Vector3d{1, 2, 3}.normalized()));
    const Vector3d p_WB{0.5, 1.25, -2};
    const RigidTransform<double> X_WB(R_WB, p_WB);

    // An arbitrary direction away from the origin that *isn't* aligned with the
    // frame basis.
    const Vector3d vhat_CbCa_W = Vector3d{2, -3, 6}.normalized();
    Vector3d p_CbCa_W = signed_distance * vhat_CbCa_W;
    Vector3d p_BCb_W = p_WB + sphere_B.radius * vhat_CbCa_W;
    const Vector3d p_BA_W = p_BCb_W + p_CbCa_W + sphere_A.radius * vhat_CbCa_W;

    Tester tester(&sphere_A, &sphere_B, X_WB, kEps);
    return tester.Test(p_BA_W, p_BCb_W, p_CbCa_W, signed_distance < 0);
  }

  static const double kRadiusB;
  static const double kEps;
};

const double SphereSphereTest::kRadiusB = 0.6;
const double SphereSphereTest::kEps = 4 * std::numeric_limits<double>::epsilon();

// Sphere A is a "point" (a zero-radius sphere), outside sphere B.
TEST_F(SphereSphereTest, PointSeparated) {
  const double radius_A{0.0};
  const double distance{1.5};
  EXPECT_TRUE(RunTest(radius_A, distance));
}

// Sphere A is a "point" (a zero-radius sphere), on the surface of sphere B.
TEST_F(SphereSphereTest, PointTouching) {
  const double radius_A{0.0};
  const double distance{0};
  EXPECT_TRUE(RunTest(radius_A, distance));
}

// Sphere A is a "point" (a zero-radius sphere), inside sphere B.
TEST_F(SphereSphereTest, PointPenetrating) {
const double radius_A{0.0};
const double distance{-0.1};
EXPECT_TRUE(RunTest(radius_A, distance));
}

// Sphere A is a "point" (a zero-radius sphere), inside sphere B.
TEST_F(SphereSphereTest, PointAtCenter) {
  const double radius_A{0.0};
  const double distance{-kRadiusB};
  EXPECT_TRUE(RunTest(radius_A, distance));
}

// Sphere A has volume and is completely outside sphere B.
TEST_F(SphereSphereTest, Separated) {
  const double radius_A{0.25};
  const double distance{1.5};
  EXPECT_TRUE(RunTest(radius_A, distance));
}

// Sphere A has volume and its surface touches sphere B's surface.
TEST_F(SphereSphereTest, Touching) {
  const double radius_A{0.25};
  const double distance{0};
  EXPECT_TRUE(RunTest(radius_A, distance));
}

// Sphere A has volume and is penetrating into sphere B.
TEST_F(SphereSphereTest, Penetrating) {
const double radius_A{0.25};
const double distance{-0.1};
EXPECT_TRUE(RunTest(radius_A, distance));
}

// Sphere A has volume and its center is at B's center.
TEST_F(SphereSphereTest, CoincidentCenters) {
  const double radius_A{0.25};
  const double distance{-(radius_A + kRadiusB)};
  EXPECT_TRUE(RunTest(radius_A, distance));
}

}  // namespace
}  // namespace signed_distance
}  // namespace internal
}  // namespace geometry
}  // namespace drake
