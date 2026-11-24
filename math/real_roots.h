#pragma once

#include <array>

namespace drake {
namespace math {

/** Computes the real roots of the quadratic polynomial a⋅x² + b⋅x + c.

The roots are returned in ascending order, with NaN padding for missing
roots, and finite roots always come before the padding:
 - two real roots (including a repeated root): `{x₀, x₁}` with x₀ ≤ x₁;
 - one real root (i.e., when `a` is zero and the polynomial is linear):
   `{x₀, NaN}`;
 - no real roots: `{NaN, NaN}`. This includes the identically-zero
   polynomial, whose roots cannot be enumerated.

@throws std::exception if any coefficient is not finite. */
std::array<double, 2> quadratic_real_roots(double a, double b, double c);

/** Computes the real roots of the cubic polynomial a⋅x³ + b⋅x² + c⋅x + d.

The roots are returned in ascending order, with NaN padding for missing
roots, and finite roots always come before the padding. The
identically-zero polynomial returns all NaN.

The coefficients are internally rescaled by a power of two. When the leading
coefficient is negligible relative to the others (|a| ≤ 10⁻⁷⋅max(|b|, |c|,
|d|), and likewise for `b` relative to `c` and `d`, and so on), the
polynomial is treated as having lower degree. In that regime the closed-form
cubic solution is badly ill-conditioned, and the lower-degree solution is more
accurate for the roots of moderate magnitude. The cost is that the dropped
root(s), whose magnitudes are roughly 10⁷ times larger than the others, are
not reported. Use cubic_real_roots_interval() when every root in a bounded
interval must be found.

@throws std::exception if any coefficient is not finite. */
std::array<double, 3> cubic_real_roots(double a, double b, double c, double d);

/** Computes the real roots of the cubic polynomial a⋅x³ + b⋅x² + c⋅x + d that
lie between `t_min` and `t_max`, inclusive.

The roots of the derivative split the interval into sub-intervals on which
the polynomial is monotonic, and each sub-interval that brackets a sign change
is solved with Newton's method safeguarded by bisection. The roots are
returned in ascending order, with NaN padding for missing roots. The
identically-zero polynomial returns all NaN.

Unlike cubic_real_roots(), this function never uses the closed-form cubic
formulas, so it remains accurate when the leading coefficient is tiny relative
to the others.

This function is deliberately conservative. A sub-interval boundary (an
interval endpoint or an interior root of the derivative) where |f| ≤ 10⁻¹²,
with the coefficients scaled so that the largest has magnitude in [0.5, 1), is
reported as a root. This finds tangential (double) roots, which have no sign
change to bracket. As a consequence, it may occasionally report a near-root
that is not a true root, or report both a boundary near-root and the adjacent
true root. If this yields more than three candidates, the three smallest are
returned.

@throws std::exception if any coefficient or bound is not finite, or if
        `t_min >= t_max`. */
std::array<double, 3> cubic_real_roots_interval(double a, double b, double c,
                                                double d, double t_min,
                                                double t_max);

}  // namespace math
}  // namespace drake
