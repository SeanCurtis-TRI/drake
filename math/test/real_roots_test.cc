#include "drake/math/real_roots.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/random.h"

namespace drake {
namespace math {
namespace {

using std::isfinite;
using std::isnan;

::testing::AssertionResult AreBothNaN(const std::array<double, 2>& roots) {
  if (isnan(roots[0]) && isnan(roots[1])) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure()
         << "Expected both NaN, got [" << roots[0] << ", " << roots[1] << "]";
}

::testing::AssertionResult AreAllNan(const std::array<double, 3>& roots) {
  if (isnan(roots[0]) && isnan(roots[1]) && isnan(roots[2])) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure()
         << "Expected all NaN, got [" << roots[0] << ", " << roots[1] << ", "
         << roots[2] << "]";
}

constexpr double kTol = std::numeric_limits<double>::epsilon();

GTEST_TEST(QuadraticRealRootsTest, DegenerateConstantNonZero) {
  // a = 0, b = 0, c != 0 -> no roots -> both NaN.
  const std::array<double, 2> roots = quadratic_real_roots(0.0, 0.0, 1.0);
  EXPECT_TRUE(AreBothNaN(roots));
}

GTEST_TEST(QuadraticRealRootsTest, DegenerateConstantZero) {
  // a = 0, b = 0, c = 0 -> identically zero polynomial. Every point is a
  // root; the implementation cannot enumerate them and returns all NaN.
  const std::array<double, 2> roots = quadratic_real_roots(0.0, 0.0, 0.0);
  EXPECT_TRUE(AreBothNaN(roots));
}

GTEST_TEST(QuadraticRealRootsTest, LinearCase) {
  // a = 0, b != 0 -> linear root at -c / b, second is NaN.
  const std::array<double, 2> roots = quadratic_real_roots(0.0, 2.0, -6.0);
  ASSERT_TRUE(isfinite(roots[0]));
  EXPECT_EQ(roots[0], 6.0 / 2.0);
  EXPECT_TRUE(isnan(roots[1]));
}

GTEST_TEST(QuadraticRealRootsTest, DoubleRootAtZero) {
  // b = 0, c = 0 -> double root at 0.
  const std::array<double, 2> roots = quadratic_real_roots(1.0, 0.0, 0.0);
  EXPECT_EQ(roots[0], 0.0);
  EXPECT_EQ(roots[1], 0.0);
}

GTEST_TEST(QuadraticRealRootsTest, NoRealRootsFromBSymmetric) {
  // b = 0, a*c > 0 -> no real roots.
  const std::array<double, 2> roots = quadratic_real_roots(1.0, 0.0, 1.0);
  EXPECT_TRUE(AreBothNaN(roots));
}

GTEST_TEST(QuadraticRealRootsTest, SymmetricNonZeroRootsFromBSymmetric) {
  // b = 0, a*c < 0 -> roots are ±sqrt(-c/a), ordered ascending.
  const std::array<double, 2> roots = quadratic_real_roots(1.0, 0.0, -4.0);
  ASSERT_TRUE(isfinite(roots[0]));
  ASSERT_TRUE(isfinite(roots[1]));
  EXPECT_NEAR(roots[0], -2.0, kTol);
  EXPECT_NEAR(roots[1], 2.0, kTol);
}

GTEST_TEST(QuadraticRealRootsTest, CZeroNegativeFirst) {
  // c = 0 and -b/a < 0 -> {x0, 0}.
  const std::array<double, 2> roots = quadratic_real_roots(1.0, 2.0, 0.0);
  // Polynomial x^2 + 2x = x(x+2) -> roots -2 and 0.
  EXPECT_EQ(roots[0], -2.0);
  EXPECT_EQ(roots[1], 0.0);
}

GTEST_TEST(QuadraticRealRootsTest, CZeroPositiveFirst) {
  // c = 0 and -b/a >= 0 -> {0, x0}.
  const std::array<double, 2> roots = quadratic_real_roots(-1.0, 2.0, 0.0);
  // Polynomial -x^2 + 2x = -x(x-2) -> roots 0 and 2.
  EXPECT_EQ(roots[0], 0.0);
  EXPECT_EQ(roots[1], 2.0);
}

GTEST_TEST(QuadraticRealRootsTest, GeneralNoRealRoots) {
  // Discriminant < 0.
  const std::array<double, 2> roots = quadratic_real_roots(1.0, 2.0, 5.0);
  EXPECT_TRUE(AreBothNaN(roots));
}

GTEST_TEST(QuadraticRealRootsTest, GeneralTwoRealRootsAscendingOrder) {
  //  x^2 - 5x + 6 = 0 -> roots 2 and 3.
  const std::array<double, 2> roots = quadratic_real_roots(1.0, -5.0, 6.0);
  ASSERT_TRUE(isfinite(roots[0]));
  ASSERT_TRUE(isfinite(roots[1]));
  EXPECT_NEAR(roots[0], 2.0, kTol);
  EXPECT_NEAR(roots[1], 3.0, kTol);
}

GTEST_TEST(QuadraticRealRootsTest, GeneralTwoRealRootsAlreadyInOrder) {
  // b > 0 branch: x^2 + x - 6 = 0 -> roots -3 and 2.
  const std::array<double, 2> roots = quadratic_real_roots(1.0, 1.0, -6.0);
  ASSERT_TRUE(isfinite(roots[0]));
  ASSERT_TRUE(isfinite(roots[1]));
  EXPECT_NEAR(roots[0], -3.0, kTol);
  EXPECT_NEAR(roots[1], 2.0, kTol);
}

GTEST_TEST(QuadraticRealRootsTest, NonFiniteCoefficientsThrows) {
  const double inf = std::numeric_limits<double>::infinity();
  const double NaN = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(quadratic_real_roots(inf, 1.0, 1.0),
               std::runtime_error);  // non-finite a
  EXPECT_THROW(quadratic_real_roots(1.0, inf, 1.0),
               std::runtime_error);  // non-finite b
  EXPECT_THROW(quadratic_real_roots(1.0, 1.0, inf),
               std::runtime_error);  // non-finite c
  EXPECT_THROW(quadratic_real_roots(NaN, 1.0, 1.0),
               std::runtime_error);  // non-finite a
  EXPECT_THROW(quadratic_real_roots(1.0, NaN, 1.0),
               std::runtime_error);  // non-finite b
  EXPECT_THROW(quadratic_real_roots(1.0, 1.0, NaN),
               std::runtime_error);  // non-finite c
}

GTEST_TEST(CubicRealRootsTest, AllNaNForNonZeroConstantOnly) {
  // a = b = c = 0, d != 0 -> constant != 0 -> no roots, all NaN.
  const std::array<double, 3> roots = cubic_real_roots(0.0, 0.0, 0.0, 1.0);
  EXPECT_TRUE(AreAllNan(roots));
}

GTEST_TEST(CubicRealRootsTest, IdenticallyZeroPolynomial) {
  // a = b = c = d = 0 -> identically zero polynomial. Every point is a root;
  // the implementation cannot enumerate them and returns all NaN.
  const std::array<double, 3> roots = cubic_real_roots(0.0, 0.0, 0.0, 0.0);
  EXPECT_TRUE(AreAllNan(roots));
}

GTEST_TEST(CubicRealRootsTest, LinearDegenerateCase) {
  // a = b = 0, c != 0 -> linear equation cx + d = 0.
  const std::array<double, 3> roots = cubic_real_roots(0.0, 0.0, 2.0, -6.0);
  // 2x - 6 = 0 -> x = 3.
  ASSERT_TRUE(isfinite(roots[0]));
  EXPECT_EQ(roots[0], 3.0);
  EXPECT_TRUE(isnan(roots[1]));
  EXPECT_TRUE(isnan(roots[2]));
}

GTEST_TEST(CubicRealRootsTest, QuadraticDegenerateCase) {
  // a = 0, b != 0 -> quadratic case handled by quadratic_real_roots.
  // x^2 - 5x + 6 = 0 -> roots 2 and 3.
  const std::array<double, 3> roots = cubic_real_roots(0.0, 1.0, -5.0, 6.0);
  ASSERT_TRUE(isfinite(roots[0]));
  ASSERT_TRUE(isfinite(roots[1]));
  EXPECT_TRUE(isnan(roots[2]));
  EXPECT_NEAR(roots[0], 2.0, kTol);
  EXPECT_NEAR(roots[1], 3.0, kTol);
}

GTEST_TEST(CubicRealRootsTest, CubicWithZeroConstantAndNoAdditionalRealRoots) {
  // d = 0, a != 0 -> root at 0 plus roots of quadratic_real_roots(a, b, c).
  // Use a quadratic with no real roots: x^2 + x + 1.
  // Cubic: x * (x^2 + x + 1) = x^3 + x^2 + x -> roots: 0 and complex pair.
  const std::array<double, 3> roots = cubic_real_roots(1.0, 1.0, 1.0, 0.0);
  // Exactly one real root at 0, others NaN.
  EXPECT_EQ(roots[0], 0.0);
  EXPECT_TRUE(isnan(roots[1]));
  EXPECT_TRUE(isnan(roots[2]));
}

GTEST_TEST(CubicRealRootsTest, CubicWithZeroConstantAndThreeRealRoots) {
  // d = 0, a != 0, quadratic has two real roots.
  // x * (x^2 - 5x + 6) = x^3 - 5x^2 + 6x -> roots 0, 2, 3.
  const std::array<double, 3> roots = cubic_real_roots(1.0, -5.0, 6.0, 0.0);
  // This path should sort the roots when both quadratic roots are finite.
  ASSERT_TRUE(isfinite(roots[0]));
  ASSERT_TRUE(isfinite(roots[1]));
  ASSERT_TRUE(isfinite(roots[2]));
  EXPECT_NEAR(roots[0], 0.0, kTol);
  EXPECT_NEAR(roots[1], 2.0, kTol);
  EXPECT_NEAR(roots[2], 3.0, kTol);
}

GTEST_TEST(CubicRealRootsTest, ThreeDistinctRealRootsGeneralCase) {
  using std::abs;
  using std::max;
  const std::array<double, 3> expected_roots = {-3.45, 0.0678, 0.12};

  const double a = 1.0;
  const double b = -expected_roots[0] - expected_roots[1] - expected_roots[2];
  const double c = expected_roots[0] * expected_roots[1] +
                   expected_roots[1] * expected_roots[2] +
                   expected_roots[2] * expected_roots[0];
  const double d = -expected_roots[0] * expected_roots[1] * expected_roots[2];
  const std::array<double, 3> roots = cubic_real_roots(a, b, c, d);

  // Should return three real roots, sorted.
  ASSERT_TRUE(isfinite(roots[0]));
  ASSERT_TRUE(isfinite(roots[1]));
  ASSERT_TRUE(isfinite(roots[2]));
  EXPECT_NEAR(roots[0], expected_roots[0], kTol);
  EXPECT_NEAR(roots[1], expected_roots[1], kTol);
  EXPECT_NEAR(roots[2], expected_roots[2], kTol);

  const double expected_residual =
      kTol * (4 * abs(a) + 3 * abs(b) + 2 * abs(c) + abs(d));

  for (double r : roots) {
    const double f = ((a * r + b) * r + c) * r + d;
    EXPECT_NEAR(f, 0.0, expected_residual);
  }
}

GTEST_TEST(CubicRealRootsTest, OneRealRootGeneralCase) {
  // x^3 - x + 1 = 0 has one real root (near -0.682...).
  const std::array<double, 3> roots = cubic_real_roots(1.0, 0.0, -1.0, 1.0);
  ASSERT_TRUE(isfinite(roots[0]));
  EXPECT_TRUE(isnan(roots[1]));
  EXPECT_TRUE(isnan(roots[2]));

  // Check the root satisfies f(x) ≈ 0.
  const double r = roots[0];
  const double f = r * (r * r - 1.0) + 1.0;
  EXPECT_NEAR(f, 0.0, kTol);
}

GTEST_TEST(CubicRealRootsTest, DoubleRootSpecialCase) {
  // (x - 1)^2 (x - 2) = x^3 - 4x^2 + 5x - 2.
  // Double root at x = 1, simple root at 2.
  const std::array<double, 3> roots = cubic_real_roots(1.0, -4.0, 5.0, -2.0);

  // Special-case branch should detect double root and sort.
  ASSERT_TRUE(isfinite(roots[0]));
  ASSERT_TRUE(isfinite(roots[1]));
  ASSERT_TRUE(isfinite(roots[2]));
  EXPECT_NEAR(roots[0], 1.0, kTol);
  EXPECT_NEAR(roots[1], 1.0, kTol);
  EXPECT_NEAR(roots[2], 2.0, kTol);

  for (double r : roots) {
    const double f = ((r - 4.0) * r + 5.0) * r - 2.0;
    EXPECT_NEAR(f, 0.0, kTol);
  }
}

GTEST_TEST(CubicRealRootsTest, ScalingDoesNotChangeRoots) {
  // Scaling all coefficients by a common factor must not (materially) change
  // the roots. Use well-separated simple roots so the roots are
  // well-conditioned in the coefficients: f(x) = (x-1)(x-2)(x+3)
  //                                            = x^3 - 7x + 6.
  const double a = 1.0;
  const double b = 0.0;
  const double c = -7.0;
  const double d = 6.0;

  const double s = 123.456;

  const std::array<double, 3> roots = cubic_real_roots(a, b, c, d);
  const auto scaled_roots = cubic_real_roots(s * a, s * b, s * c, s * d);

  // The scaled coefficients round at machine precision; for simple roots the
  // induced root perturbation is a small multiple of that.
  constexpr double kScaleTol = 16 * kTol;
  EXPECT_NEAR(roots[0], scaled_roots[0], kScaleTol * 3.0);
  EXPECT_NEAR(roots[1], scaled_roots[1], kScaleTol * 1.0);
  EXPECT_NEAR(roots[2], scaled_roots[2], kScaleTol * 2.0);
}

GTEST_TEST(CubicRealRootsTest, RelativeLeadingCoefficientDemotion) {
  // COR-1 regression: a leading coefficient that is tiny *relative* to the
  // other coefficients makes the closed-form (Cardano/trigonometric) branches
  // catastrophically ill-conditioned. Empirically (at the pre-fix HEAD):
  //   cubic_real_roots(1e-9,  1, -5, 6) = [-1e9,   NaN, NaN]   (roots lost)
  //   cubic_real_roots(1e-14, 1, -5, 6) = [-1e14,  2.5, 2.5]   (fabricated)
  // The true near-origin roots (of x^2 - 5x + 6) are 2 and 3. With relative
  // demotion the quadratic path recovers them; the huge third root ~ -b/a is
  // intentionally not reported (documented in the header).
  for (const double a : {0.0, 1e-18, 1e-16, 1e-14, 1e-12, 1e-10, 1e-8}) {
    SCOPED_TRACE(fmt::format("a = {}", a));
    const std::array<double, 3> roots = cubic_real_roots(a, 1.0, -5.0, 6.0);
    ASSERT_TRUE(isfinite(roots[0]));
    ASSERT_TRUE(isfinite(roots[1]));
    // Dropping the a·x³ term perturbs the roots by O(|a·x³ / f'(x)|).
    const double shift_bound = 64 * std::max(a * 27, kTol);
    EXPECT_NEAR(roots[0], 2.0, shift_bound);
    EXPECT_NEAR(roots[1], 3.0, shift_bound);
  }
}

GTEST_TEST(CubicRealRootsTest, FiniteRootsAreAscendingRandomized) {
  // COR-6 regression (as a property test): the finite roots must be returned
  // in ascending order — the single post-sort polish step must not be able to
  // reorder them. Exercise random coefficient sets and random
  // root-constructed polynomials (including clustered roots).
  RandomGenerator generator(1234);
  std::uniform_real_distribution<double> uniform(-1.0, 1.0);
  constexpr int kNumCases = 2000;
  for (int i = 0; i < kNumCases; ++i) {
    std::array<double, 3> roots;
    if (i % 2 == 0) {
      // Random coefficients.
      roots = cubic_real_roots(uniform(generator), uniform(generator),
                               uniform(generator), uniform(generator));
    } else {
      // Root-constructed with a clustered pair: (x-r0)(x-r0-eps)(x-r1).
      const double r0 = uniform(generator);
      const double r1 = uniform(generator);
      const double eps_cluster = std::pow(10.0, -8.0 * uniform(generator) - 4);
      const double s0 = r0;
      const double s1 = r0 + eps_cluster;
      const double s2 = r1;
      roots = cubic_real_roots(1.0, -(s0 + s1 + s2),
                               s0 * s1 + s1 * s2 + s2 * s0, -s0 * s1 * s2);
    }
    for (int k = 0; k + 1 < 3; ++k) {
      if (isfinite(roots[k]) && isfinite(roots[k + 1])) {
        EXPECT_LE(roots[k], roots[k + 1]) << fmt::format(
            "case {}: roots = [{}, {}, {}]", i, roots[0], roots[1], roots[2]);
      }
    }
  }
}

/* ======================= cubic_real_roots_interval ======================= */

/* Builds the coefficients of (x - r0)(x - r1)(x - r2). */
std::array<double, 4> CubicFromRoots(double r0, double r1, double r2) {
  return {1.0, -(r0 + r1 + r2), r0 * r1 + r1 * r2 + r2 * r0, -r0 * r1 * r2};
}

GTEST_TEST(CubicRealRootsIntervalTest, ThreeInteriorRoots) {
  const std::array<double, 4> k = CubicFromRoots(0.2, 0.5, 0.8);
  const std::array<double, 3> roots =
      cubic_real_roots_interval(k[0], k[1], k[2], k[3], 0.0, 1.0);
  ASSERT_TRUE(isfinite(roots[0]) && isfinite(roots[1]) && isfinite(roots[2]));
  EXPECT_NEAR(roots[0], 0.2, 1e-12);
  EXPECT_NEAR(roots[1], 0.5, 1e-12);
  EXPECT_NEAR(roots[2], 0.8, 1e-12);
}

GTEST_TEST(CubicRealRootsIntervalTest, RootsOutsideIntervalExcluded) {
  const std::array<double, 4> k = CubicFromRoots(2.0, 3.0, -1.0);
  const std::array<double, 3> roots =
      cubic_real_roots_interval(k[0], k[1], k[2], k[3], 0.0, 1.0);
  EXPECT_TRUE(AreAllNan(roots));
}

GTEST_TEST(CubicRealRootsIntervalTest, ExactRootsAtIntervalEndpoints) {
  {
    // Root exactly at t_min = 0.
    const std::array<double, 4> k = CubicFromRoots(0.0, 0.5, 2.0);
    const std::array<double, 3> roots =
        cubic_real_roots_interval(k[0], k[1], k[2], k[3], 0.0, 1.0);
    ASSERT_TRUE(isfinite(roots[0]) && isfinite(roots[1]));
    EXPECT_TRUE(isnan(roots[2]));
    EXPECT_NEAR(roots[0], 0.0, 1e-12);
    EXPECT_NEAR(roots[1], 0.5, 1e-12);
  }
  {
    // Root exactly at t_max = 1.
    const std::array<double, 4> k = CubicFromRoots(1.0, 0.25, -3.0);
    const std::array<double, 3> roots =
        cubic_real_roots_interval(k[0], k[1], k[2], k[3], 0.0, 1.0);
    ASSERT_TRUE(isfinite(roots[0]) && isfinite(roots[1]));
    EXPECT_NEAR(roots[0], 0.25, 1e-12);
    EXPECT_NEAR(roots[1], 1.0, 1e-12);
  }
}

GTEST_TEST(CubicRealRootsIntervalTest, TangentialDoubleRoot) {
  // (x - 0.4)²(x - 2): a double root at 0.4 with no sign change. The double
  // root coincides with a derivative root — a sub-interval boundary — where
  // |f| is (near) zero, so the conservative boundary acceptance reports it.
  const std::array<double, 4> k = {1.0, -2.8, 1.76, -0.32};
  const std::array<double, 3> roots =
      cubic_real_roots_interval(k[0], k[1], k[2], k[3], 0.0, 1.0);
  ASSERT_TRUE(isfinite(roots[0]));
  EXPECT_NEAR(roots[0], 0.4, 1e-6);
}

GTEST_TEST(CubicRealRootsIntervalTest, NearTangencyReportedConservatively) {
  // (x - 0.4)²(x - 2) - 1e-14: never crosses zero near 0.4 (the parabola is
  // lifted off the axis), so there is genuinely no real root there — but a
  // grazing CCD contact rounds into exactly this shape, and the solver must
  // err toward reporting it.
  const std::array<double, 4> k = {1.0, -2.8, 1.76, -0.32 - 1e-14};
  const std::array<double, 3> roots =
      cubic_real_roots_interval(k[0], k[1], k[2], k[3], 0.0, 1.0);
  ASSERT_TRUE(isfinite(roots[0]));
  EXPECT_NEAR(roots[0], 0.4, 1e-6);
}

GTEST_TEST(CubicRealRootsIntervalTest, TinyLeadingCoefficient) {
  // The COR-1 regime that defeats the closed-form solver: genuine roots at
  // 0.25 and 0.75 with a relatively-negligible cubic term. Bracketing does
  // not care about the leading coefficient's conditioning.
  for (const double a : {1e-8, 1e-12, 1e-16, 1e-20, 1e-30}) {
    SCOPED_TRACE(fmt::format("a = {}", a));
    const std::array<double, 3> roots =
        cubic_real_roots_interval(a, 1.0, -1.0, 0.1875, 0.0, 1.0);
    ASSERT_TRUE(isfinite(roots[0]) && isfinite(roots[1]));
    EXPECT_NEAR(roots[0], 0.25, 1e-7);
    EXPECT_NEAR(roots[1], 0.75, 1e-7);
  }
}

GTEST_TEST(CubicRealRootsIntervalTest, IdenticallyZeroPolynomial) {
  const std::array<double, 3> roots =
      cubic_real_roots_interval(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  EXPECT_TRUE(AreAllNan(roots));
}

GTEST_TEST(CubicRealRootsIntervalTest, InvalidInputsThrow) {
  const double inf = std::numeric_limits<double>::infinity();
  const double NaN = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(cubic_real_roots_interval(inf, 1, 1, 1, 0, 1),
               std::runtime_error);
  EXPECT_THROW(cubic_real_roots_interval(1, NaN, 1, 1, 0, 1),
               std::runtime_error);
  EXPECT_THROW(cubic_real_roots_interval(1, 1, 1, 1, NaN, 1),
               std::runtime_error);
  EXPECT_THROW(cubic_real_roots_interval(1, 1, 1, 1, 0, -inf),
               std::runtime_error);
  EXPECT_THROW(cubic_real_roots_interval(1, 1, 1, 1, 1, 1),
               std::runtime_error);  // t_min >= t_max.
  EXPECT_THROW(cubic_real_roots_interval(1, 1, 1, 1, 2, 1),
               std::runtime_error);  // t_min >= t_max.
}

GTEST_TEST(CubicRealRootsIntervalTest, RandomizedAgreesWithClosedForm) {
  // For well-conditioned cubics (unit leading coefficient, well-separated
  // roots away from the interval boundaries), the interval solver must find
  // exactly the closed-form roots that lie inside [0, 1].
  RandomGenerator generator(1234);
  std::uniform_real_distribution<double> uniform(-1.5, 1.5);
  constexpr int kNumCases = 1000;
  int tested = 0;
  for (int i = 0; i < kNumCases; ++i) {
    double r0 = uniform(generator);
    double r1 = uniform(generator);
    double r2 = uniform(generator);
    // Enforce separation between roots and distance from the boundaries so
    // the expected in-interval root set is unambiguous.
    const double sep = 0.05;
    if (std::abs(r0 - r1) < sep || std::abs(r1 - r2) < sep ||
        std::abs(r0 - r2) < sep) {
      continue;
    }
    auto near_boundary = [sep](double r) {
      return std::abs(r) < sep || std::abs(r - 1.0) < sep;
    };
    if (near_boundary(r0) || near_boundary(r1) || near_boundary(r2)) continue;
    ++tested;

    const std::array<double, 4> k = CubicFromRoots(r0, r1, r2);
    const std::array<double, 3> interval_roots =
        cubic_real_roots_interval(k[0], k[1], k[2], k[3], 0.0, 1.0);

    std::vector<double> expected;
    for (double r : {r0, r1, r2}) {
      if (r > 0.0 && r < 1.0) expected.push_back(r);
    }
    std::sort(expected.begin(), expected.end());

    int num_found = 0;
    while (num_found < 3 && isfinite(interval_roots[num_found])) ++num_found;
    ASSERT_EQ(num_found, static_cast<int>(expected.size())) << fmt::format(
        "case {}: constructed roots [{}, {}, {}], got [{}, {}, {}]", i, r0, r1,
        r2, interval_roots[0], interval_roots[1], interval_roots[2]);
    for (int j = 0; j < num_found; ++j) {
      EXPECT_NEAR(interval_roots[j], expected[j], 1e-9) << fmt::format(
          "case {}: constructed roots [{}, {}, {}]", i, r0, r1, r2);
    }
  }
  // Make sure the filters left a meaningful sample.
  EXPECT_GT(tested, 300);
}

GTEST_TEST(CubicRealRootsTest, NonFiniteCoefficientsThrows) {
  const double inf = std::numeric_limits<double>::infinity();
  const double NaN = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(cubic_real_roots(inf, 1.0, 1.0, 1.0),
               std::runtime_error);  // non-finite a
  EXPECT_THROW(cubic_real_roots(1.0, inf, 1.0, 1.0),
               std::runtime_error);  // non-finite b
  EXPECT_THROW(cubic_real_roots(1.0, 1.0, inf, 1.0),
               std::runtime_error);  // non-finite c
  EXPECT_THROW(cubic_real_roots(1.0, 1.0, 1.0, inf),
               std::runtime_error);  // non-finite d
  EXPECT_THROW(cubic_real_roots(NaN, 1.0, 1.0, 1.0),
               std::runtime_error);  // non-finite a
  EXPECT_THROW(cubic_real_roots(1.0, NaN, 1.0, 1.0),
               std::runtime_error);  // non-finite b
  EXPECT_THROW(cubic_real_roots(1.0, 1.0, NaN, 1.0),
               std::runtime_error);  // non-finite c
  EXPECT_THROW(cubic_real_roots(1.0, 1.0, 1.0, NaN),
               std::runtime_error);  // non-finite d
}

}  // namespace
}  // namespace math
}  // namespace drake
