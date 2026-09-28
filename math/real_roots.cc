#include "drake/math/real_roots.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <utility>

#include <fmt/format.h>

#include "drake/common/drake_throw.h"
#include "drake/multibody/contact_solvers/newton_with_bisection.h"

namespace drake {
namespace math {

constexpr double NaN = std::numeric_limits<double>::quiet_NaN();
constexpr double sqrt3 = std::numbers::sqrt3;

std::array<double, 2> quadratic_real_roots(double a, double b, double c) {
  using std::copysign;
  using std::sqrt;

  DRAKE_THROW_UNLESS(std::isfinite(a));
  DRAKE_THROW_UNLESS(std::isfinite(b));
  DRAKE_THROW_UNLESS(std::isfinite(c));

  // Handle degenerate cases.
  if (a == 0) {
    // Non-zero constant polynomial has no roots.
    if (b == 0 && c != 0) {
      return {NaN, NaN};
    } else if (b == 0 && c == 0) {
      // Technically infinite roots, but we'll treat this as no roots.
      return {NaN, NaN};
    }
    // Linear case.
    return {-c / b, NaN};
  }
  if (b == 0) {
    // Double root at x = 0.
    if (c == 0) {
      return {0.0, 0.0};
    }
    const double x0_sq = -c / a;
    // No real roots.
    if (x0_sq < 0) {
      return {NaN, NaN};
    }
    const double x0 = sqrt(x0_sq);
    return {-x0, x0};
  }
  if (c == 0) {
    const double x0 = -b / a;
    if (x0 < 0) {
      return {x0, 0.0};
    } else {
      return {0.0, x0};
    }
  }

  // General case.
  const double discriminant = b * b - 4.0 * a * c;

  // No real roots.
  if (discriminant < 0) {
    return {NaN, NaN};
  }
  // Avoid catastrophic cancellation.
  const double q = -0.5 * (b + copysign(sqrt(discriminant), b));
  const double x0 = q / a;
  const double x1 = c / q;
  if (x0 < x1) {
    return {x0, x1};
  }
  return {x1, x0};
}

namespace {

/* Polishes each finite root with one iteration of Halley's method (falling
 back to Newton's method), evaluating the *full* cubic ax³ + bx² + cx + d,
 then re-sorts so the ascending-order contract survives polishing. The finite
 roots always occupy a prefix of the array (NaN padding is at the tail); only
 that prefix is sorted, since std::sort on NaNs is undefined behavior. */
void PolishAndSortRoots(double a, double b, double c, double d,
                        std::array<double, 3>* roots_in_out) {
  using std::isfinite;
  std::array<double, 3>& roots = *roots_in_out;
  for (auto& x : roots) {
    // Skip any NaN values.
    if (!isfinite(x)) continue;

    const double f = ((a * x + b) * x + c) * x + d;
    const double df = (3 * a * x + 2 * b) * x + c;
    if (df != 0) {
      const double d2f = 6 * a * x + 2 * b;
      const double denom = 2 * df * df - f * d2f;
      if (denom != 0) {
        // Halley's method.
        x -= 2 * f * df / denom;
      } else {
        // Newton's method.
        x -= f / df;
      }
    }
  }
  int num_finite = 0;
  while (num_finite < 3 && isfinite(roots[num_finite])) ++num_finite;
  std::sort(roots.begin(), roots.begin() + num_finite);
}

}  // namespace

// Solves for the real roots of ax^3 + bx^2 + cx + d = 0.
// Adapted from:
//   https://github.com/boostorg/math/blob/develop/include/boost/math/tools/cubic_roots.hpp.
// Follows Numerical Recipes, Chapter 5, section 6.
// Adds scaling to avoid overflow/underflow. See line 1080 of RPOLY:
//   Jenkins, Michael A. "Algorithm 493: Zeros of a real polynomial [c2]." ACM
//   Transactions on Mathematical Software (TOMS) 1.2 (1975): 178-189.
std::array<double, 3> cubic_real_roots(double a, double b, double c, double d) {
  using std::abs;
  using std::acos;
  using std::cbrt;
  using std::clamp;
  using std::cos;
  using std::isfinite;
  using std::max;
  using std::sqrt;

  DRAKE_THROW_UNLESS(isfinite(a));
  DRAKE_THROW_UNLESS(isfinite(b));
  DRAKE_THROW_UNLESS(isfinite(c));
  DRAKE_THROW_UNLESS(isfinite(d));

  std::array<double, 3> roots = {NaN, NaN, NaN};

  const double m = max(max(abs(a), abs(b)), max(abs(c), abs(d)));
  // Identically-zero polynomial: every x is a root; none can be enumerated.
  // (This also guards std::ilogbl(0) below, which returns FP_ILOGB0 and would
  // otherwise poison the scale factor.)
  if (m == 0) {
    return roots;
  }
  // Scale the coefficients by an exact power of 2 to control
  // overflow/underflow without losing precision. After scaling, the largest
  // coefficient magnitude lies in [0.5, 1).
  const int e = std::ilogbl(m);
  double s = std::scalbn(1.0, -e);
  a *= s;
  b *= s;
  c *= s;
  d *= s;

  // Degree demotion, on *relative* coefficient magnitude. The closed-form
  // branches below are catastrophically ill-conditioned when the leading
  // coefficient is negligible relative to the trailing ones — empirically,
  // for |a| / max(|b|,|c|,|d|) anywhere below ~1e-7 the near-origin roots
  // come back with garbage digits, fabricated multiplicities, or are lost
  // outright (e.g. cubic_real_roots(1e-9, 1, -5, 6) = {-1e9, NaN, NaN},
  // losing the genuine roots 2 and 3). Treating the negligible leading
  // term as zero recovers the near-origin roots to O(tol·|x³/f'(x)|); the
  // demoted root(s) of magnitude ≳ 1/tol are not reported (see the header).
  // The retained roots are polished against the full cubic below.
  constexpr double kDegeneracyTol = 1e-7;
  const double max_bcd = max(abs(b), max(abs(c), abs(d)));
  if (abs(a) <= kDegeneracyTol * max_bcd) {
    const double max_cd = max(abs(c), abs(d));
    if (abs(b) <= kDegeneracyTol * max_cd) {
      if (abs(c) <= kDegeneracyTol * abs(d)) {
        // Effectively a non-zero constant: no roots. (m > 0 rules out the
        // identically-zero polynomial here.)
        return roots;
      }
      // Effectively linear.
      roots[0] = -d / c;
    } else {
      // Effectively quadratic.
      auto [x0, x1] = quadratic_real_roots(b, c, d);
      roots[0] = x0;
      roots[1] = x1;
    }
    PolishAndSortRoots(a, b, c, d, &roots);
    return roots;
  }
  if (d == 0) {
    auto [x0, x1] = quadratic_real_roots(a, b, c);
    roots[0] = 0;
    roots[1] = x0;
    roots[2] = x1;
    PolishAndSortRoots(a, b, c, d, &roots);
    return roots;
  }

  // General case;
  const double p = b / a;
  const double q = c / a;
  const double r = d / a;
  const double Q = (p * p - 3 * q) / 9;
  const double R = (2 * p * p * p - 9 * p * q + 27 * r) / 54;
  if (R * R < Q * Q * Q) {
    const double rtQ = sqrt(Q);
    const double theta = acos(clamp(R / (Q * rtQ), -1.0, 1.0)) / 3;
    const double st = sin(theta);
    const double ct = cos(theta);
    roots[0] = -2 * rtQ * ct - p / 3;
    roots[1] = -rtQ * (-ct + sqrt3 * st) - p / 3;
    roots[2] = rtQ * (ct + sqrt3 * st) - p / 3;
  } else {
    const double arg = R * R - Q * Q * Q;
    const double A = (R >= 0 ? -1 : 1) * cbrt(abs(R) + sqrt(arg));
    double B = 0;
    if (A != 0) {
      B = Q / A;
    }
    roots[0] = A + B - p / 3;
    // Special case: double real root.
    if (A == B || arg == 0) {
      roots[1] = -A - p / 3;
      roots[2] = roots[1];
    }
  }
  PolishAndSortRoots(a, b, c, d, &roots);
  return roots;
}

std::array<double, 3> cubic_real_roots_interval(double a, double b, double c,
                                                double d, double t_min,
                                                double t_max) {
  using std::abs;
  using std::isfinite;
  using std::max;

  DRAKE_THROW_UNLESS(isfinite(a));
  DRAKE_THROW_UNLESS(isfinite(b));
  DRAKE_THROW_UNLESS(isfinite(c));
  DRAKE_THROW_UNLESS(isfinite(d));
  DRAKE_THROW_UNLESS(isfinite(t_min) && isfinite(t_max) && t_min < t_max);

  std::array<double, 3> interval_roots = {NaN, NaN, NaN};

  const double m = max(max(abs(a), abs(b)), max(abs(c), abs(d)));
  // Identically-zero polynomial: every t is a root; none can be enumerated.
  // (Also guards std::ilogbl(0) below.)
  if (m == 0) {
    return interval_roots;
  }
  // Scale the coefficients by an exact power of 2 to control
  // overflow/underflow without losing precision. After scaling, the largest
  // coefficient magnitude lies in [0.5, 1).
  const int e = std::ilogbl(m);
  double s = std::scalbn(1.0, -e);
  a *= s;
  b *= s;
  c *= s;
  d *= s;

  const auto f = [a, b, c, d](double t) {
    return ((a * t + b) * t + c) * t + d;
  };

  /* The boundaries of the monotonic sub-intervals of [t_min, t_max]: the
   interval endpoints plus any derivative roots inside. (NaN derivative roots
   fail both comparisons and are excluded.) quadratic_real_roots() returns
   ascending roots, so `boundary` is ascending by construction. */
  const std::array<double, 2> df_roots = quadratic_real_roots(3 * a, 2 * b, c);
  std::array<double, 4> boundary;
  int num_boundaries = 0;
  boundary[num_boundaries++] = t_min;
  if (df_roots[0] > t_min && df_roots[0] < t_max) {
    boundary[num_boundaries++] = df_roots[0];
  }
  if (df_roots[1] > t_min && df_roots[1] < t_max &&
      df_roots[1] != df_roots[0]) {
    boundary[num_boundaries++] = df_roots[1];
  }
  boundary[num_boundaries++] = t_max;

  /* |f| at or below this threshold at a boundary is treated as a root there.
   With the coefficients pre-scaled to [0.5, 1), |f| = O(1) on |t| ≲ 1, so
   this is an (effectively relative) tolerance ~1e4·ε. It is what makes the
   solver conservative for tangential (double-root) contacts: an exact double
   root coincides with a derivative root — one of the boundaries — and a
   *near* double root that never quite crosses zero has no sign change for
   bracketing to find, but leaves |f| tiny at that boundary. Reporting the
   boundary as a root errs toward "collision", the safe direction for the CCD
   caller. */
  constexpr double kBoundaryRootTol = 1e-12;

  int count = 0;
  for (int i = 0; i < num_boundaries; ++i) {
    if (abs(f(boundary[i])) <= kBoundaryRootTol && count < 3) {
      interval_roots[count++] = boundary[i];
    }
  }

  /* Bracket-solve each sub-interval whose endpoint values straddle zero. f is
   monotonic on each sub-interval, so a straddling one holds exactly one root.
   An endpoint with f *exactly* zero is skipped — by monotonicity its interval
   holds no additional root, and the boundary pass above already reported it.
   An endpoint with f tiny-but-nonzero still brackets: its true crossing can
   lie well inside the interval (the derivative vanishes at interior
   boundaries), so the boundary root recorded above is not a substitute. Such
   an interval reports both the (conservative) boundary root and the bracketed
   true root. */
  std::array<double, 3> bracketed = {NaN, NaN, NaN};
  int num_bracketed = 0;
  for (int i = 0; i + 1 < num_boundaries; ++i) {
    const double t_lower = boundary[i];
    const double t_upper = boundary[i + 1];
    const double f_lower = f(t_lower);
    const double f_upper = f(t_upper);
    if (f_lower == 0 || f_upper == 0) continue;
    if (!(std::signbit(f_lower) ^ std::signbit(f_upper))) continue;

    multibody::contact_solvers::internal::Bracket bracket(t_lower, f_lower,
                                                          t_upper, f_upper);
    const double x_guess = 0.5 * (t_lower + t_upper);
    const double x_tolerance = 1e-14;
    const double f_tolerance = 1e-14;
    const int max_iterations = 100;

    // The callable is passed as std::function; capturing one pointer (8
    // bytes, trivially copyable) stays within its small-buffer optimization —
    // capturing the four coefficients directly would heap-allocate on every
    // bracket solve in this hot path.
    const std::array<double, 4> coeffs{a, b, c, d};
    const std::array<double, 4>* k = &coeffs;
    const auto [root, iterations] =
        multibody::contact_solvers::internal::DoNewtonWithBisectionFallback(
            [k](double t) {
              const double ft =
                  (((*k)[0] * t + (*k)[1]) * t + (*k)[2]) * t + (*k)[3];
              const double dft = (3 * (*k)[0] * t + 2 * (*k)[1]) * t + (*k)[2];
              return std::make_pair(ft, dft);
            },
            bracket, x_guess, x_tolerance, f_tolerance, max_iterations);

    if (num_bracketed < 3) {
      bracketed[num_bracketed++] = root;
    }
  }

  // Merge (both sequences are ascending; at most 3 roots survive — a cubic
  // has at most 3 real roots, and any overflow beyond 3 candidates can only
  // come from multiply-reported near-degenerate boundaries, where dropping
  // the latest (largest) candidates is the right resolution for an
  // earliest-first consumer).
  if (num_bracketed > 0) {
    std::array<double, 3> merged = {NaN, NaN, NaN};
    int mi = 0, ri = 0, bi = 0;
    while (mi < 3 && (ri < count || bi < num_bracketed)) {
      if (ri < count &&
          (bi >= num_bracketed || interval_roots[ri] <= bracketed[bi])) {
        merged[mi++] = interval_roots[ri++];
      } else {
        merged[mi++] = bracketed[bi++];
      }
    }
    interval_roots = merged;
  }
  return interval_roots;
}

}  // namespace math
}  // namespace drake
