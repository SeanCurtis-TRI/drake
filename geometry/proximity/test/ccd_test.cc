#include "drake/geometry/proximity/ccd.h"

#include <array>
#include <cmath>
#include <limits>
#include <string>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/random.h"
#include "drake/math/random_rotation.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;
using math::RigidTransformd;
using math::RotationMatrixd;

constexpr double kInf = std::numeric_limits<double>::infinity();

/* Formats the eight points of a CCD query as re-runnable C++ literals so a
 failing (e.g. randomized) case can be reproduced as a standalone regression
 test. */
std::string FormatQuery(const std::array<const char*, 8>& names,
                        const std::array<Vector3d, 8>& x) {
  std::string out;
  for (int i = 0; i < 8; ++i) {
    out += fmt::format("  const Vector3d {}({:.17g}, {:.17g}, {:.17g});\n",
                       names[i], x[i].x(), x[i].y(), x[i].z());
  }
  return out;
}

constexpr std::array<const char*, 8> kPointTriangleNames = {
    "p0", "v00", "v10", "v20", "p1", "v01", "v11", "v21"};
constexpr std::array<const char*, 8> kEdgeEdgeNames = {
    "p00", "p10", "q00", "q10", "p01", "p11", "q01", "q11"};

/* Runs point_triangle_ccd and checks the boolean result (and, for hits, the
 time of impact) against expectations. On mismatch the full query is dumped. */
::testing::AssertionResult CheckPointTriangle(const std::array<Vector3d, 8>& x,
                                              bool expected_hit,
                                              double expected_toi = -1.0,
                                              double toi_tolerance = 1e-12) {
  double toi = kInf;
  const bool hit =
      point_triangle_ccd(x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7], &toi);
  if (hit != expected_hit) {
    return ::testing::AssertionFailure()
           << "point_triangle_ccd returned " << hit << ", expected "
           << expected_hit << " (toi = " << toi << ") for query:\n"
           << FormatQuery(kPointTriangleNames, x);
  }
  if (expected_hit && expected_toi >= 0.0 &&
      std::abs(toi - expected_toi) > toi_tolerance) {
    return ::testing::AssertionFailure()
           << "point_triangle_ccd toi = " << toi << ", expected "
           << expected_toi << " ± " << toi_tolerance << " for query:\n"
           << FormatQuery(kPointTriangleNames, x);
  }
  return ::testing::AssertionSuccess();
}

/* Runs edge_edge_ccd; same contract as CheckPointTriangle. */
::testing::AssertionResult CheckEdgeEdge(const std::array<Vector3d, 8>& x,
                                         bool expected_hit,
                                         double expected_toi = -1.0,
                                         double toi_tolerance = 1e-12) {
  double toi = kInf;
  const bool hit =
      edge_edge_ccd(x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7], &toi);
  if (hit != expected_hit) {
    return ::testing::AssertionFailure()
           << "edge_edge_ccd returned " << hit << ", expected " << expected_hit
           << " (toi = " << toi << ") for query:\n"
           << FormatQuery(kEdgeEdgeNames, x);
  }
  if (expected_hit && expected_toi >= 0.0 &&
      std::abs(toi - expected_toi) > toi_tolerance) {
    return ::testing::AssertionFailure()
           << "edge_edge_ccd toi = " << toi << ", expected " << expected_toi
           << " ± " << toi_tolerance << " for query:\n"
           << FormatQuery(kEdgeEdgeNames, x);
  }
  return ::testing::AssertionSuccess();
}

/* Applies a rigid transform (and uniform scale) to all eight points of a
 query. Both CCD predicates and the time of impact are invariant under this
 family of transforms (in exact arithmetic). */
std::array<Vector3d, 8> TransformQuery(const std::array<Vector3d, 8>& x,
                                       const RigidTransformd& X, double scale) {
  std::array<Vector3d, 8> result;
  for (int i = 0; i < 8; ++i) {
    result[i] = X * (scale * x[i]);
  }
  return result;
}

/* ============================ point-triangle ============================ */

/* A canonical hit: static triangle in the z = 0 plane, point descending
 through its interior. All coordinates are dyadic and the query is symmetric
 about the origin (centroid = 0, max centered norm = 4 = 2²), so the internal
 shift/scale conditioning is exact and the linear coplanarity root t = 1/2 is
 computed exactly. */
std::array<Vector3d, 8> CanonicalPointTriangleHit() {
  const Vector3d v0(-2, -2, 0);
  const Vector3d v1(2, -2, 0);
  const Vector3d v2(0, 4, 0);
  const Vector3d p0(0, 0, 2);
  const Vector3d p1(0, 0, -2);
  return {p0, v0, v1, v2, p1, v0, v1, v2};
}

/* A canonical miss: the same descending motion, but far outside the
 triangle's interior. Coplanarity occurs at t = 1/2 but containment fails. */
std::array<Vector3d, 8> CanonicalPointTriangleMiss() {
  const Vector3d v0(-2, -2, 0);
  const Vector3d v1(2, -2, 0);
  const Vector3d v2(0, 4, 0);
  const Vector3d p0(10, 10, 2);
  const Vector3d p1(10, 10, -2);
  return {p0, v0, v1, v2, p1, v0, v1, v2};
}

GTEST_TEST(PointTriangleCcdTest, ClearHit) {
  EXPECT_TRUE(CheckPointTriangle(CanonicalPointTriangleHit(), true, 0.5));
}

GTEST_TEST(PointTriangleCcdTest, ClearMissOutsideTriangle) {
  EXPECT_TRUE(CheckPointTriangle(CanonicalPointTriangleMiss(), false));
}

GTEST_TEST(PointTriangleCcdTest, ClearMissParallelMotion) {
  // The point moves parallel to the (static) triangle's plane at z = 1; the
  // query is never coplanar.
  const Vector3d v0(-2, -2, 0);
  const Vector3d v1(2, -2, 0);
  const Vector3d v2(0, 4, 0);
  const Vector3d p0(0, 0, 1);
  const Vector3d p1(0.5, 0.25, 1);
  EXPECT_TRUE(CheckPointTriangle({p0, v0, v1, v2, p1, v0, v1, v2}, false));
}

GTEST_TEST(PointTriangleCcdTest, SymmetricMotionHit) {
  // Both the point and the triangle move (towards each other); impact at the
  // z = 0 plane at t = 1/2.
  const Vector3d v00(-2, -2, -1), v01(-2, -2, 1);
  const Vector3d v10(2, -2, -1), v11(2, -2, 1);
  const Vector3d v20(0, 4, -1), v21(0, 4, 1);
  const Vector3d p0(0, 0, 1), p1(0, 0, -1);
  EXPECT_TRUE(
      CheckPointTriangle({p0, v00, v10, v20, p1, v01, v11, v21}, true, 0.5));
}

GTEST_TEST(PointTriangleCcdTest, HitNearTimeZero) {
  // The point starts just above the plane and descends: impact at
  // t = ε / (1 + ε) ≈ 1e-6.
  const double eps = 1e-6;
  const Vector3d v0(-2, -2, 0);
  const Vector3d v1(2, -2, 0);
  const Vector3d v2(0, 4, 0);
  const Vector3d p0(0, 0, eps);
  const Vector3d p1(0, 0, -1);
  EXPECT_TRUE(CheckPointTriangle({p0, v0, v1, v2, p1, v0, v1, v2}, true,
                                 eps / (1 + eps), 1e-12));
}

GTEST_TEST(PointTriangleCcdTest, HitNearTimeOne) {
  // The point ends just below the plane: impact at t = 1/(1+ε) ≈ 1 - 1e-6.
  const double eps = 1e-6;
  const Vector3d v0(-2, -2, 0);
  const Vector3d v1(2, -2, 0);
  const Vector3d v2(0, 4, 0);
  const Vector3d p0(0, 0, 1);
  const Vector3d p1(0, 0, -eps);
  EXPECT_TRUE(CheckPointTriangle({p0, v0, v1, v2, p1, v0, v1, v2}, true,
                                 1.0 / (1 + eps), 1e-12));
}

GTEST_TEST(PointTriangleCcdTest, EarliestRootIsReported) {
  // A deforming triangle whose plane sweeps past the moving point twice. The
  // coplanarity polynomial is the quadratic 16·(-t² + t - 3/16), with roots
  // t = 1/4 and t = 3/4; containment holds at both. The earliest time of
  // impact (1/4) must be the one reported.
  //
  // Derivation: v0 pinned at the origin, v1 = (4,0,4t), v2 = (0,4,4t) gives
  // plane normal n(t) = (-16t, -16t, 16); with p(t) = (1+t, 1, 3t - 3/16),
  // (p - v0)·n = 16·(-t² + t - 3/16).
  const Vector3d v00(0, 0, 0), v01(0, 0, 0);
  const Vector3d v10(4, 0, 0), v11(4, 0, 4);
  const Vector3d v20(0, 4, 0), v21(0, 4, 4);
  const Vector3d p0(1, 1, -0.1875), p1(2, 1, 2.8125);
  EXPECT_TRUE(CheckPointTriangle({p0, v00, v10, v20, p1, v01, v11, v21}, true,
                                 0.25, 1e-9));
}

GTEST_TEST(PointTriangleCcdTest, TangentialGrazeHit) {
  // A grazing contact: the coplanarity polynomial is -16(t - 1/2)² — an exact
  // double root with no sign change, which bracketing alone cannot see. The
  // interval solver's conservative boundary acceptance (the double root
  // coincides with a derivative root) must report it (COR-7).
  //
  // Construction: same deforming-triangle family as EarliestRootIsReported,
  // with the point's z-motion tuned so the parabola just touches zero:
  // z(t) = 3t - 1/4 gives f/16 = z - t² - 2t = -(t - 1/2)².
  const Vector3d v00(0, 0, 0), v01(0, 0, 0);
  const Vector3d v10(4, 0, 0), v11(4, 0, 4);
  const Vector3d v20(0, 4, 0), v21(0, 4, 4);
  const Vector3d p0(1, 1, -0.25), p1(2, 1, 2.75);
  EXPECT_TRUE(CheckPointTriangle({p0, v00, v10, v20, p1, v01, v11, v21}, true,
                                 0.5, 1e-6));
}

GTEST_TEST(PointTriangleCcdTest, TangentialNearMissReportedConservatively) {
  // The same graze lifted off contact by 1e-13: mathematically no impact, but
  // far inside rounding noise for any real query — the conservative solver
  // reports it (the safe direction). A *clear* miss (1e-4 clearance) must
  // still be collision-free.
  const Vector3d v00(0, 0, 0), v01(0, 0, 0);
  const Vector3d v10(4, 0, 0), v11(4, 0, 4);
  const Vector3d v20(0, 4, 0), v21(0, 4, 4);
  const Vector3d p0_graze(1, 1, -0.25 - 1e-13), p1_graze(2, 1, 2.75 - 1e-13);
  EXPECT_TRUE(CheckPointTriangle(
      {p0_graze, v00, v10, v20, p1_graze, v01, v11, v21}, true, 0.5, 1e-5));
  const Vector3d p0_miss(1, 1, -0.25 - 1e-4), p1_miss(2, 1, 2.75 - 1e-4);
  EXPECT_TRUE(CheckPointTriangle(
      {p0_miss, v00, v10, v20, p1_miss, v01, v11, v21}, false));
}

GTEST_TEST(PointTriangleCcdTest, StationaryCoplanarQuery) {
  // Fully static query with the point in the triangle's plane: the
  // coplanarity polynomial is identically zero, so roots carry no
  // information. The C2 containment-sampling fallback reports the inside
  // case as a contact at t = 0 (before the fix this was a silent miss); the
  // outside case remains collision-free.
  const Vector3d v0(-2, -2, 0);
  const Vector3d v1(2, -2, 0);
  const Vector3d v2(0, 4, 0);
  const Vector3d p_inside(0, 0, 0);
  const Vector3d p_outside(10, 10, 0);
  EXPECT_TRUE(CheckPointTriangle({p_inside, v0, v1, v2, p_inside, v0, v1, v2},
                                 true, 0.0, 0.0));
  EXPECT_TRUE(CheckPointTriangle({p_outside, v0, v1, v2, p_outside, v0, v1, v2},
                                 false));
}

GTEST_TEST(PointTriangleCcdTest, AllPointsCoincidentIsContact) {
  // Degenerate input (COR-4): all eight points coincide for all t. Coincident
  // points are in contact, so this must conservatively report a hit at
  // toi = 0 (pre-fix, the 1/0 = ∞ internal scale factor produced NaN
  // coefficients and the root finder threw).
  const Vector3d p(0.5, 0.25, -1);
  double toi = kInf;
  EXPECT_TRUE(point_triangle_ccd(p, p, p, p, p, p, p, p, &toi));
  EXPECT_EQ(toi, 0.0);
}

GTEST_TEST(PointTriangleCcdTest, FastTranslationSmallRotationHit) {
  // COR-1 regression: fast point translation combined with a tiny
  // (rotation-like) non-uniform vertex motion produces a coplanarity cubic
  // whose leading coefficient is tiny relative to the others but not exactly
  // zero. The closed-form cubic branches are catastrophically ill-conditioned
  // there and lose the genuine crossing at t ≈ 1/2 (missed collision).
  const double delta = std::ldexp(1.0, -28);  // 2^-28 ≈ 3.7e-9.
  const Vector3d v00(-2, -2, 0), v01(-2, -2, 0);
  const Vector3d v10(2, -2, 0), v11(2, -2, delta);
  const Vector3d v20(0, 4, 0), v21(delta, 4, 0);
  const Vector3d p0(0, 0, 2), p1(0, delta, -2);
  EXPECT_TRUE(CheckPointTriangle({p0, v00, v10, v20, p1, v01, v11, v21}, true,
                                 0.5, 1e-3));
}

GTEST_TEST(PointTriangleCcdTest, GrazingEndAcceptedConservatively) {
  // COR-2: the point descends toward the plane but stops just short of it
  // (4e-10 above at t = 1); the mathematical crossing time is 1 + 2e-10,
  // just outside [0, 1]. The tolerance-widened acceptance gate must flag
  // this as a hit with toi clamped to 1 — the conservative direction for a
  // non-penetration guarantee (pre-fix, the hard [0, 1] gate dropped it, and
  // by the same mechanism dropped true roots returned as 1 + O(rounding)).
  const Vector3d v0(-2, -2, 0);
  const Vector3d v1(2, -2, 0);
  const Vector3d v2(0, 4, 0);
  const Vector3d p0(0, 0, 2);
  const Vector3d p1(0, 0, 4e-10);
  EXPECT_TRUE(
      CheckPointTriangle({p0, v0, v1, v2, p1, v0, v1, v2}, true, 1.0, 0.0));
}

GTEST_TEST(PointTriangleCcdTest, GrazingStartAcceptedConservatively) {
  // COR-2, t ≈ 0 side: the point starts just above the plane (4e-10) and
  // ascends away from it; the mathematical crossing time is -2e-10, just
  // outside [0, 1]. Accepted and clamped to toi = 0.
  const Vector3d v0(-2, -2, 0);
  const Vector3d v1(2, -2, 0);
  const Vector3d v2(0, 4, 0);
  const Vector3d p0(0, 0, 4e-10);
  const Vector3d p1(0, 0, 2);
  EXPECT_TRUE(
      CheckPointTriangle({p0, v0, v1, v2, p1, v0, v1, v2}, true, 0.0, 0.0));
}

/* ============================== edge-edge ============================== */

/* A canonical hit: edge P static along the x-axis, edge Q along the y-axis
 descending through it; the segments cross at the origin at t = 1/2. The
 query is symmetric (centroid = 0) with max centered norm exactly 2. */
std::array<Vector3d, 8> CanonicalEdgeEdgeHit() {
  const Vector3d p0(-2, 0, 0), p1(2, 0, 0);
  const Vector3d q00(0, -1, 1), q10(0, 1, 1);
  const Vector3d q01(0, -1, -1), q11(0, 1, -1);
  return {p0, p1, q00, q10, p0, p1, q01, q11};
}

/* A canonical miss: the same descending motion displaced along x so the
 (coplanar at t = 1/2) segments never intersect. */
std::array<Vector3d, 8> CanonicalEdgeEdgeMiss() {
  const Vector3d p0(-2, 0, 0), p1(2, 0, 0);
  const Vector3d q00(5, -1, 1), q10(5, 1, 1);
  const Vector3d q01(5, -1, -1), q11(5, 1, -1);
  return {p0, p1, q00, q10, p0, p1, q01, q11};
}

GTEST_TEST(EdgeEdgeCcdTest, ClearHit) {
  EXPECT_TRUE(CheckEdgeEdge(CanonicalEdgeEdgeHit(), true, 0.5));
}

GTEST_TEST(EdgeEdgeCcdTest, ClearMissSeparatedSegments) {
  EXPECT_TRUE(CheckEdgeEdge(CanonicalEdgeEdgeMiss(), false));
}

GTEST_TEST(EdgeEdgeCcdTest, HitNearTimeZero) {
  const double eps = 1e-6;
  const Vector3d p0(-2, 0, 0), p1(2, 0, 0);
  const Vector3d q00(0, -1, eps), q10(0, 1, eps);
  const Vector3d q01(0, -1, -1), q11(0, 1, -1);
  EXPECT_TRUE(CheckEdgeEdge({p0, p1, q00, q10, p0, p1, q01, q11}, true,
                            eps / (1 + eps), 1e-12));
}

GTEST_TEST(EdgeEdgeCcdTest, HitNearTimeOne) {
  const double eps = 1e-6;
  const Vector3d p0(-2, 0, 0), p1(2, 0, 0);
  const Vector3d q00(0, -1, 1), q10(0, 1, 1);
  const Vector3d q01(0, -1, -eps), q11(0, 1, -eps);
  EXPECT_TRUE(CheckEdgeEdge({p0, p1, q00, q10, p0, p1, q01, q11}, true,
                            1.0 / (1 + eps), 1e-12));
}

GTEST_TEST(EdgeEdgeCcdTest, ParallelEdgesSweepingThroughEachOther) {
  // Parallel edges are *always* coplanar with each other (the triple product
  // is identically zero), so root finding carries no information here; the
  // C2 proximity-sampling fallback detects edge Q sweeping directly through
  // edge P within the shared plane, overlapping it at t = 1/2. (Before the
  // fix this was a silent miss.)
  const Vector3d p0(-2, 0, 0), p1(2, 0, 0);
  const Vector3d q00(-1, 1, 0), q10(1, 1, 0);
  const Vector3d q01(-1, -1, 0), q11(1, -1, 0);
  EXPECT_TRUE(
      CheckEdgeEdge({p0, p1, q00, q10, p0, p1, q01, q11}, true, 0.5, 0.0));
}

GTEST_TEST(EdgeEdgeCcdTest, ParallelEdgesPassingBySeparatedPlanes) {
  // Parallel edges in distinct parallel planes; Q translates along the shared
  // direction. Never a collision; the identically-zero polynomial early-out
  // gives the right answer for the wrong reason.
  const Vector3d p0(-2, 0, 0), p1(2, 0, 0);
  const Vector3d q00(-1, 1, 1), q10(1, 1, 1);
  const Vector3d q01(0, 1, 1), q11(2, 1, 1);
  EXPECT_TRUE(CheckEdgeEdge({p0, p1, q00, q10, p0, p1, q01, q11}, false));
}

GTEST_TEST(EdgeEdgeCcdTest, AllPointsCoincidentIsContact) {
  // See PointTriangleCcdTest.AllPointsCoincidentIsContact (COR-4).
  const Vector3d p(0.5, 0.25, -1);
  double toi = kInf;
  EXPECT_TRUE(edge_edge_ccd(p, p, p, p, p, p, p, p, &toi));
  EXPECT_EQ(toi, 0.0);
}

GTEST_TEST(EdgeEdgeCcdTest, FastTranslationSmallRotationHit) {
  // COR-1 regression for the edge-edge kernel; see
  // PointTriangleCcdTest.FastTranslationSmallRotationHit. Edge Q descends
  // fast through edge P while P's endpoints drift by a tiny non-uniform
  // amount, giving a relatively-tiny (but nonzero) cubic leading coefficient.
  const double delta = std::ldexp(1.0, -28);  // 2^-28 ≈ 3.7e-9.
  const Vector3d p00(-2, 0, 0), p01(-2, 0, delta);
  const Vector3d p10(2, 0, 0), p11(2, delta, 0);
  const Vector3d q00(0, -1, 1), q10(0, 1, 1);
  const Vector3d q01(0, -1, -1), q11(delta, 1, -1);
  EXPECT_TRUE(
      CheckEdgeEdge({p00, p10, q00, q10, p01, p11, q01, q11}, true, 0.5, 1e-3));
}

/* ========================= randomized invariance ========================= */

/* The CCD verdict and time of impact are invariant under rigid transforms and
 uniform scaling of all eight points. Push the canonical known-answer queries
 through random members of that family; any failure prints the full query for
 use as a standalone regression case. */
GTEST_TEST(CcdInvarianceTest, RandomTransformedQueries) {
  RandomGenerator generator(1234);
  std::uniform_real_distribution<double> uniform(-1.0, 1.0);
  std::uniform_real_distribution<double> log_scale(-2.0, 2.0);
  constexpr int kNumCases = 250;

  for (int i = 0; i < kNumCases; ++i) {
    const RotationMatrixd R = math::UniformlyRandomRotationMatrix(&generator);
    const Vector3d offset(10 * uniform(generator), 10 * uniform(generator),
                          10 * uniform(generator));
    const RigidTransformd X(R, offset);
    const double scale = std::pow(10.0, log_scale(generator));

    // The transformed known-answer roots are no longer exact; allow a modest
    // tolerance on toi.
    EXPECT_TRUE(CheckPointTriangle(
        TransformQuery(CanonicalPointTriangleHit(), X, scale), true, 0.5,
        1e-9));
    EXPECT_TRUE(CheckPointTriangle(
        TransformQuery(CanonicalPointTriangleMiss(), X, scale), false));
    EXPECT_TRUE(CheckEdgeEdge(TransformQuery(CanonicalEdgeEdgeHit(), X, scale),
                              true, 0.5, 1e-9));
    EXPECT_TRUE(CheckEdgeEdge(TransformQuery(CanonicalEdgeEdgeMiss(), X, scale),
                              false));
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
