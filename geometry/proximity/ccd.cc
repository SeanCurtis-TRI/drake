#include "drake/geometry/proximity/ccd.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include "drake/math/real_roots.h"

namespace drake {
namespace geometry {
namespace internal {

namespace {

/*
  Test whether a point lies inside a triangle using barycentric coordinates
  (when the point is assumed to be co-planar with the triangle).

  Let e0 = t1 - t0, e1 = t2 - t0, and r = p - t0.

  This function solves for x = [u, v] that minimizes the squared distance:

    || r - B·x ||^2   where B = [e0; e1]

  This least squares minimizer is given by:

    B·Bᵀ x = B·r

  This implementation builds that 2×2 system and solves it with LDLᵀ.
  If the solution satisfies u ∈ [0,1] and v ∈ [0,1] and u + v < 1, the point p
  lies inside the triangle [t0, t1, t2].
*/
bool is_coplanar_point_inside_triangle(const Vector3d& p, const Vector3d& t0,
                                       const Vector3d& t1, const Vector3d& t2) {
  Eigen::Matrix<double, 2, 3> B;
  B.row(0) = t1 - t0;
  B.row(1) = t2 - t0;
  const Eigen::Matrix2d A = B * B.transpose();
  const Eigen::Vector2d b = B * (p - t0);
  const Eigen::Vector2d x = A.ldlt().solve(b);
  return x[0] >= -1e-8 && x[1] >= -1e-8 && x[0] + x[1] <= 1 + 1e-8;
}

/*
  Squared distance between segments [p1, q1] and [p2, q2] via clamped
  closest points.

  Adapted from:
    Ericson, Christer. Real-time collision detection. Crc Press, 2004.
    Section 5.1.9

  Unlike a normal-equations line-line solve, this handles parallel,
  degenerate (point-like), and endpoint-closest configurations robustly —
  those are exactly the configurations the near-coplanar CCD fallback has to
  survive. */
double segment_segment_squared_distance(const Vector3d& p1, const Vector3d& q1,
                                        const Vector3d& p2,
                                        const Vector3d& q2) {
  constexpr double kEps2 = std::numeric_limits<double>::epsilon() *
                           std::numeric_limits<double>::epsilon();
  double s, t;
  const Vector3d d1 = q1 - p1;  // Direction vector of segment 1.
  const Vector3d d2 = q2 - p2;  // Direction vector of segment 2.
  const Vector3d r = p1 - p2;
  const double a = d1.dot(d1);  // Squared length of segment 1.
  const double e = d2.dot(d2);  // Squared length of segment 2.
  const double f = d2.dot(r);

  if (a <= kEps2 && e <= kEps2) {
    // Both segments degenerate into points.
    return r.squaredNorm();
  }
  if (a <= kEps2) {
    // First segment degenerates into a point.
    s = 0;
    t = std::clamp(f / e, 0.0, 1.0);
  } else {
    const double c = d1.dot(r);
    if (e <= kEps2) {
      // Second segment degenerates into a point.
      t = 0;
      s = std::clamp(-c / a, 0.0, 1.0);
    } else {
      // The general non-degenerate case starts here.
      const double b = d1.dot(d2);
      const double denom = a * e - b * b;  // Always non-negative.

      // If segments not parallel, compute closest point on L1 to L2 and
      // clamp to segment 1. Else pick arbitrary s (here 0).
      if (denom != 0) {
        s = std::clamp((b * f - c * e) / denom, 0.0, 1.0);
      } else {
        s = 0;
      }

      // Closest point on L2 to segment-1 point at s.
      t = (b * s + f) / e;

      // If t is outside [0, 1], clamp it and recompute s.
      if (t < 0) {
        t = 0;
        s = std::clamp(-c / a, 0.0, 1.0);
      } else if (t > 1) {
        t = 1;
        s = std::clamp((b - c) / a, 0.0, 1.0);
      }
    }
  }
  return (p1 + s * d1 - p2 - t * d2).squaredNorm();
}

/*
  Test whether two (near-)co-planar 3D segments (nearly) intersect: their
  clamped closest-point distance must be below tolerance. Robust to parallel
  and degenerate segments (a previous normal-equations implementation was
  singular exactly in the parallel-overlap case that the near-coplanar
  fallback must detect). Endpoint-to-endpoint grazing contacts within
  tolerance also count — the conservative direction. */
bool are_coplanar_edges_intersecting(const Vector3d& ea0, const Vector3d& ea1,
                                     const Vector3d& eb0, const Vector3d& eb1) {
  constexpr double kDistTol2 = 1e-8 * 1e-8;
  return segment_segment_squared_distance(ea0, ea1, eb0, eb1) < kDistTol2;
}

/*
  Let ([x,y,z] := x⋅(y × z)) be the scalar triple product.
  With A(t) = a + tα, B(t) = b + tβ, C(t) = c + tγ

     A(t)⋅(B(t) × C(t)) = [a + tα, b + tβ, c + tγ] = p₀ + p₁ t + p₂ t² + p₃ t³

  with coefficients:

    p₀ = [a,b,c]                      =  a⋅(b × c),
    p₁ = [α,b,c] + [a,β,c] + [a,b,γ]  =  α⋅(b × c) + a⋅(β × c) + a⋅(b × γ)
    p₂ = [α,β,c] + [α,b,γ] + [a,β,γ]  =  α⋅(β × c) + α⋅(b × γ) + a⋅(β × γ)
    p₃ = [α,β,γ]                      =  α⋅(β × γ)

  simplifying:

    p₀ =  a⋅X₀,
    p₁ =  α⋅X₀ + a⋅X₁
    p₂ =  α⋅X₁ + a⋅X₂
    p₃ =  α⋅X₂

  Where:
    X₀ = (b × c)
    X₁ = (β × c) + (b × γ)
    X₂ = (β × γ)
*/
std::array<double, 4> cubic(const Vector3d& a, const Vector3d& b,
                            const Vector3d& c, const Vector3d& alpha,
                            const Vector3d& beta, const Vector3d& gamma) {
  const Vector3d X0 = b.cross(c);
  const Vector3d X1 = beta.cross(c) + b.cross(gamma);
  const Vector3d X2 = beta.cross(gamma);
  return {a.dot(X0), alpha.dot(X0) + a.dot(X1), alpha.dot(X1) + a.dot(X2),
          alpha.dot(X2)};
}

}  // namespace

constexpr double c_tol = 1e-14;

/* Tolerance for accepting a coplanarity root just outside [0, 1]. A true
 impact at t ≈ 0 or t ≈ 1 can round to a root marginally outside the step
 interval; silently dropping it is a missed collision — the one error class
 this query must avoid. Accepted roots are clamped back into [0, 1] before
 the containment test, so the cost of the widened gate is (at most) a rare
 conservative false positive. */
constexpr double kTimeTol = 1e-9;

/*
  Given a point p(t) and a triangle [v₀(t), v₁(t), v₂(t)],
  where the parameteric form of each point is:

    p(t)  = p₀  + t⋅(p₁ - p₀)
    v₀(t) = v₀₀ + t⋅(v₀₁ - v₀₀)
    v₁(t) = v₁₀ + t⋅(v₁₁ - v₁₀)
    v₂(t) = v₂₀ + t⋅(v₂₁ - v₂₀)

  If the p(t) intesects the triangle [v₀(t), v₁(t), v₂(t)] on t = [0, 1],
  then it must happen when all points are co-planar. In other words:

    A(t)⋅(B(t) × C(t)) = 0

  Where:

    A(t) =  p(t) - v₀(t) = a + t⋅α
    B(t) = v₁(t) - v₀(t) = b + t⋅β
    C(t) = v₂(t) - v₀(t) = c + t⋅γ

    a = p₀  - v₀₀
    b = v₁₀ - v₀₀
    c = v₂₀ - v₀₀
    α = (p₁ - p₀)   - (v₀₁ - v₀₀)
    β = (v₁₁ - v₁₀) - (v₀₁ - v₀₀)
    γ = (v₂₁ - v₂₀) - (v₀₁ - v₀₀)

  This function:
    - Forms the polynomial A(t)⋅(B(t) × C(t))
    - Solves for the real roots in the interval [0, 1]
    - For each valid root, r, in ascending order:
        if p(r) is inside the triangle [v₀(r), v₁(r), v₂(r)]
          set toi = r
          return true
    - return false
*/
bool point_triangle_ccd(const Vector3d& p0, const Vector3d& v00,
                        const Vector3d& v10, const Vector3d& v20,
                        const Vector3d& p1, const Vector3d& v01,
                        const Vector3d& v11, const Vector3d& v21, double* toi) {
  // Scale and shift all points to improve numerical stability.
  const Vector3d centroid = (p0 + v00 + v10 + v20 + p1 + v01 + v11 + v21) / 8.0;
  Vector3d p0_s = p0 - centroid;
  Vector3d v00_s = v00 - centroid;
  Vector3d v10_s = v10 - centroid;
  Vector3d v20_s = v20 - centroid;
  Vector3d p1_s = p1 - centroid;
  Vector3d v01_s = v01 - centroid;
  Vector3d v11_s = v11 - centroid;
  Vector3d v21_s = v21 - centroid;
  const double max_norm =
      std::max({p0_s.norm(), v00_s.norm(), v10_s.norm(), v20_s.norm(),
                p1_s.norm(), v01_s.norm(), v11_s.norm(), v21_s.norm()});
  // Degenerate extent: all eight points coincide (for all t). Coincident
  // points are in contact; report it rather than dividing by zero.
  if (max_norm == 0) {
    *toi = 0;
    return true;
  }
  const double s = 1.0 / max_norm;

  p0_s *= s;
  v00_s *= s;
  v10_s *= s;
  v20_s *= s;
  p1_s *= s;
  v01_s *= s;
  v11_s *= s;
  v21_s *= s;

  const Vector3d delta_p = p1_s - p0_s;
  const Vector3d delta_v0 = v01_s - v00_s;
  const Vector3d delta_v1 = v11_s - v10_s;
  const Vector3d delta_v2 = v21_s - v20_s;
  const Vector3d a = p0_s - v00_s;
  const Vector3d b = v10_s - v00_s;
  const Vector3d c = v20_s - v00_s;
  const Vector3d alpha = delta_p - delta_v0;
  const Vector3d beta = delta_v1 - delta_v0;
  const Vector3d gamma = delta_v2 - delta_v0;

  const std::array<double, 4> f = cubic(a, b, c, alpha, beta, gamma);

  const auto test_containment = [&](double t) {
    const Vector3d p_t = p0_s + t * delta_p;
    const Vector3d v0_t = v00_s + t * delta_v0;
    const Vector3d v1_t = v10_s + t * delta_v1;
    const Vector3d v2_t = v20_s + t * delta_v2;
    return is_coplanar_point_inside_triangle(p_t, v0_t, v1_t, v2_t);
  };

  if (std::abs(f[0]) <= c_tol && std::abs(f[1]) <= c_tol &&
      std::abs(f[2]) <= c_tol && std::abs(f[3]) <= c_tol) {
    // The coplanarity polynomial is (numerically) identically zero: the point
    // stays in the triangle's plane for the whole step, and roots carry no
    // information. (The inputs are pre-normalized to unit radius, so c_tol is
    // effectively a relative tolerance.) Sample containment across the step
    // instead of declaring "no collision" (which silently missed genuine
    // in-plane contacts). A transit that starts, ends, and holds containment
    // only strictly between samples can still be missed — the principled fix
    // for in-plane motion is a distance-based method (e.g. ACCD).
    for (const double t : {0.0, 0.25, 0.5, 0.75, 1.0}) {
      if (test_containment(t)) {
        *toi = t;
        return true;
      }
    }
    return false;
  }

  // Bracketed root isolation on the (widened) step interval. Unlike the
  // closed-form cubic solver, it is robust to a relatively-tiny leading
  // coefficient (fast translation + small rotation) and conservatively
  // reports tangential (double-root) grazes. Roots arrive ascending with NaN
  // padding, so the first containment hit is the earliest time of impact.
  for (const double r : math::cubic_real_roots_interval(
           f[3], f[2], f[1], f[0], -kTimeTol, 1.0 + kTimeTol)) {
    if (std::isnan(r)) break;
    const double rc = std::clamp(r, 0.0, 1.0);
    if (test_containment(rc)) {
      *toi = rc;
      return true;
    }
  }

  return false;
}

/*
  Given edges [p₀(t), p₁(t)] and [q₀(t), q₁(t)]
  where the parameteric form of each point is:

    p₀(t) = p₀₀ + t⋅(p₀₁ - p₀₀)
    p₁(t) = p₁₀ + t⋅(p₁₁ - p₁₀)
    q₀(t) = q₀₀ + t⋅(q₀₁ - q₀₀)
    q₁(t) = q₁₀ + t⋅(q₁₁ - q₁₀)

  If the edges intesect for some t = [0, 1], then it must happen when all points
  are co-planar. In other words:

    A(t)⋅(B(t) × C(t)) = 0

  Where:

    A(t) = p₁(t) - p₀(t) = a + t⋅α
    B(t) = q₀(t) - p₀(t) = b + t⋅β
    C(t) = q₁(t) - p₀(t) = c + t⋅γ

    a = p₁₀ - p₀₀
    b = q₀₀ - p₀₀
    c = q₁₀ - p₀₀
    α = (p₁₁ - p₁₀) - (p₀₁ - p₀₀)
    β = (q₀₁ - q₀₀) - (p₀₁ - p₀₀)
    γ = (q₁₁ - q₁₀) - (p₀₁ - p₀₀)

  This function:
    - Forms the polynomial A(t)⋅(B(t) × C(t))
    - Solves for the real roots in the interval [0, 1]
    - For each valid root, r, in ascending order:
        if the edges intersect at r
          set toi = r
          return true
    - return false
*/
bool edge_edge_ccd(const Vector3d& p00, const Vector3d& p10,
                   const Vector3d& q00, const Vector3d& q10,
                   const Vector3d& p01, const Vector3d& p11,
                   const Vector3d& q01, const Vector3d& q11, double* toi) {
  // Scale and shift all points to improve numerical stability.
  const Vector3d centroid =
      (p00 + p10 + q00 + q10 + p01 + p11 + q01 + q11) / 8.0;

  Vector3d p00_s = p00 - centroid;
  Vector3d p10_s = p10 - centroid;
  Vector3d q00_s = q00 - centroid;
  Vector3d q10_s = q10 - centroid;
  Vector3d p01_s = p01 - centroid;
  Vector3d p11_s = p11 - centroid;
  Vector3d q01_s = q01 - centroid;
  Vector3d q11_s = q11 - centroid;

  const double max_norm =
      std::max({p00_s.norm(), p10_s.norm(), q00_s.norm(), q10_s.norm(),
                p01_s.norm(), p11_s.norm(), q01_s.norm(), q11_s.norm()});
  // Degenerate extent: all eight points coincide (for all t). Coincident
  // points are in contact; report it rather than dividing by zero.
  if (max_norm == 0) {
    *toi = 0;
    return true;
  }
  const double s = 1.0 / max_norm;

  p00_s *= s;
  p10_s *= s;
  q00_s *= s;
  q10_s *= s;
  p01_s *= s;
  p11_s *= s;
  q01_s *= s;
  q11_s *= s;

  const Vector3d delta_p0 = p01_s - p00_s;
  const Vector3d delta_p1 = p11_s - p10_s;
  const Vector3d delta_q0 = q01_s - q00_s;
  const Vector3d delta_q1 = q11_s - q10_s;
  const Vector3d a = p10_s - p00_s;
  const Vector3d b = q00_s - p00_s;
  const Vector3d c = q10_s - p00_s;
  const Vector3d alpha = delta_p1 - delta_p0;
  const Vector3d beta = delta_q0 - delta_p0;
  const Vector3d gamma = delta_q1 - delta_p0;

  const std::array<double, 4> f = cubic(a, b, c, alpha, beta, gamma);

  const auto test_containment = [&](double t) {
    const Vector3d p0_t = p00_s + t * delta_p0;
    const Vector3d p1_t = p10_s + t * delta_p1;
    const Vector3d q0_t = q00_s + t * delta_q0;
    const Vector3d q1_t = q10_s + t * delta_q1;
    return are_coplanar_edges_intersecting(p0_t, p1_t, q0_t, q1_t);
  };

  if (std::abs(f[0]) <= c_tol && std::abs(f[1]) <= c_tol &&
      std::abs(f[2]) <= c_tol && std::abs(f[3]) <= c_tol) {
    // Identically-zero coplanarity polynomial (co-planar motion — including
    // *any* pair of parallel edges, which are always co-planar). Sample the
    // segment-segment proximity across the step instead of declaring "no
    // collision"; see the discussion in point_triangle_ccd(). Note that an
    // in-plane transversal crossing is instantaneous (a measure-zero event
    // between samples can be missed); the principled fix is a distance-based
    // method (e.g. ACCD).
    for (const double t : {0.0, 0.25, 0.5, 0.75, 1.0}) {
      if (test_containment(t)) {
        *toi = t;
        return true;
      }
    }
    return false;
  }

  // See the matching comment in point_triangle_ccd().
  for (const double r : math::cubic_real_roots_interval(
           f[3], f[2], f[1], f[0], -kTimeTol, 1.0 + kTimeTol)) {
    if (std::isnan(r)) break;
    const double rc = std::clamp(r, 0.0, 1.0);
    if (test_containment(rc)) {
      *toi = rc;
      return true;
    }
  }

  return false;
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
