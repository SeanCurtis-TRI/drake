#pragma once

#include <array>

namespace drake {
namespace math {

/* Computes the real roots of the quadratic a·x² + b·x + c = 0.

 Returns the roots in ascending order, padded with NaN: two real roots as
 {x0, x1} with x0 ≤ x1; a single (e.g. linear-case) root as {x0, NaN}; no
 real roots (including the identically-zero polynomial, whose roots cannot
 be enumerated) as {NaN, NaN}. Finite roots always precede the NaN padding.

 @throws std::exception if any coefficient is not finite. */
std::array<double, 2> quadratic_real_roots(double a, double b, double c);

/* Computes the real roots of the cubic a·x³ + b·x² + c·x + d = 0.

 Returns the roots in ascending order, padded with NaN (finite roots always
 precede the padding). The identically-zero polynomial returns all NaN.

 Robustness: coefficients are internally rescaled by a power of two, and a
 leading coefficient that is negligible *relative* to the trailing ones
 (|a| ≤ 1e-7·max(|b|,|c|,|d|), similarly cascading for b and c) demotes the
 polynomial to the lower degree. In that regime the closed-form cubic
 solution is catastrophically ill-conditioned and the demoted evaluation is
 strictly more accurate for the near-origin roots; the price is that the
 demoted root(s), of magnitude ≳ 1e7 relative to the others, are not
 reported. Callers that need guaranteed root isolation on a bounded interval
 should use cubic_real_roots_interval() instead.

 @throws std::exception if any coefficient is not finite. */
std::array<double, 3> cubic_real_roots(double a, double b, double c, double d);

/* Computes the real roots of the cubic a·x³ + b·x² + c·x + d = 0 inside the
 interval [t_min, t_max], by bracketed Newton-with-bisection on the monotonic
 sub-intervals delimited by the roots of the derivative. Returns the roots in
 ascending order, padded with NaN. The identically-zero polynomial returns
 all NaN.

 Unlike cubic_real_roots(), this solver never relies on the closed-form
 (ill-conditioned) cubic formulas, so it is robust to relatively-tiny leading
 coefficients, and it is deliberately *conservative*: a sub-interval boundary
 (an interval endpoint or an interior derivative root) where |f| ≤ 1e-12
 (with coefficients pre-scaled so the largest magnitude is in [0.5, 1)) is
 reported as a root. This catches tangential double roots that produce no
 sign change — at the price of occasionally reporting a spurious near-root,
 or both a boundary near-root and the adjacent true crossing. When more than
 three candidates arise this way, the three smallest are returned
 (earliest-first bias, matching the CCD consumer).

 @throws std::exception if any coefficient or bound is not finite, or if
 t_min >= t_max. */
std::array<double, 3> cubic_real_roots_interval(double a, double b, double c,
                                                double d, double t_min,
                                                double t_max);

}  // namespace math
}  // namespace drake
