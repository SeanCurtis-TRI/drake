#include "drake/geometry/proximity/test/ccd_test_configurations.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <fmt/format.h>

#include "drake/math/random_rotation.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;
using math::RigidTransformd;

double Uniform(RandomGenerator* gen, double lo, double hi) {
  std::uniform_real_distribution<double> dist(lo, hi);
  return dist(*gen);
}

double LogUniform(RandomGenerator* gen, double lo_exp, double hi_exp) {
  return std::pow(10.0, Uniform(gen, lo_exp, hi_exp));
}

Vector3d UniformBox(RandomGenerator* gen, double half_width) {
  return Vector3d(Uniform(gen, -half_width, half_width),
                  Uniform(gen, -half_width, half_width),
                  Uniform(gen, -half_width, half_width));
}

/* Composes the family decorations shared by all constructed cases, applied
 in the *canonical* frame before the final rigid transform:
 - a common linear drift D·t added to all end positions (rigid translation of
   the whole query — the relative geometry is unchanged);
 - for kFastTranslationTinyRotation, a large drift plus per-point tiny end
   perturbations (breaking the exact degree degeneracies the same way small
   rotations do). When `perturb_dirs` is provided (edge-edge queries), each
   point is perturbed only along its given direction: sliding endpoints along
   their own edge lines leaves the constructed crossing exact, while an
   isotropic perturbation would break it into a near-miss beyond the
   intersection tolerance. Point-triangle queries pass nullptr (their
   containment test projects into the triangle plane, so isotropic
   perturbations keep the constructed truth). */
void DecorateMotion(CcdQueryFamily family, RandomGenerator* gen,
                    std::array<Vector3d, 8>* x,
                    const std::array<Vector3d, 8>* perturb_dirs = nullptr) {
  double drift_scale = 2.0;
  double perturbation = 0.0;
  if (family == CcdQueryFamily::kFastTranslationTinyRotation) {
    drift_scale = Uniform(gen, 50.0, 500.0);
    perturbation = LogUniform(gen, -13.0, -5.0);
  }
  const Vector3d drift = UniformBox(gen, drift_scale);
  for (int i = 4; i < 8; ++i) {
    (*x)[i] += drift;
    if (perturbation > 0) {
      if (perturb_dirs != nullptr) {
        (*x)[i] += perturbation * Uniform(gen, -1.0, 1.0) * (*perturb_dirs)[i];
      } else {
        (*x)[i] += perturbation * UniformBox(gen, 1.0);
      }
    }
  }
}

/* Applies the family's final placement: random rotation, bounded random
 translation, and the family's scale; optionally snaps to the dyadic grid. */
void PlaceQuery(CcdQueryFamily family, RandomGenerator* gen,
                std::array<Vector3d, 8>* x) {
  double scale = LogUniform(gen, -0.5, 0.5);
  if (family == CcdQueryFamily::kTinyScale) scale = LogUniform(gen, -6.5, -5.5);
  if (family == CcdQueryFamily::kHugeScale) scale = LogUniform(gen, 5.5, 6.5);
  const RigidTransformd X(math::UniformlyRandomRotationMatrix(gen),
                          UniformBox(gen, 5.0));
  for (auto& p : *x) {
    p = X * (scale * p);
  }
  if (family == CcdQueryFamily::kDyadic) {
    // Snap to k/2^20. Only meaningful near unit scale; the snap displaces
    // points by ≲ 1e-6, well inside every constructed margin.
    const double grid = std::ldexp(1.0, 20);
    for (auto& p : *x) {
      p = (p * grid).array().round().matrix() / grid;
    }
  }
}

double PickToi(CcdQueryFamily family, RandomGenerator* gen) {
  if (family == CcdQueryFamily::kBoundaryTime) {
    const double eps = LogUniform(gen, -7.5, -6.5);
    return (Uniform(gen, 0, 1) < 0.5) ? eps : 1.0 - eps;
  }
  return Uniform(gen, 0.1, 0.9);
}

double ToiTolerance(CcdQueryFamily family) {
  switch (family) {
    case CcdQueryFamily::kFastTranslationTinyRotation:
      return 1e-3;  // The tiny perturbations shift the crossing slightly.
    case CcdQueryFamily::kDyadic:
      return 1e-4;  // The grid snap moves the crossing slightly.
    default:
      return 1e-6;
  }
}

}  // namespace

const char* to_string(CcdQueryFamily family) {
  switch (family) {
    case CcdQueryFamily::kClean:
      return "kClean";
    case CcdQueryFamily::kFastTranslationTinyRotation:
      return "kFastTranslationTinyRotation";
    case CcdQueryFamily::kBoundaryTime:
      return "kBoundaryTime";
    case CcdQueryFamily::kTinyScale:
      return "kTinyScale";
    case CcdQueryFamily::kHugeScale:
      return "kHugeScale";
    case CcdQueryFamily::kDyadic:
      return "kDyadic";
  }
  return "unknown";
}

CcdQueryCase MakeVertexFaceHit(CcdQueryFamily family, RandomGenerator* gen) {
  // Canonical frame: a static triangle in the z = 0 plane; the point crosses
  // the plane at t* through a barycentric location with a wide (0.15)
  // interior margin.
  const Vector3d v0(-1, -1, 0), v1(1, -1, 0), v2(0, 1, 0);
  const double u = Uniform(gen, 0.15, 0.55);
  const double w = Uniform(gen, 0.15, 0.85 - u);
  const Vector3d xy = v0 + u * (v1 - v0) + w * (v2 - v0);

  const double t_star = PickToi(family, gen);
  const double speed = Uniform(gen, 0.5, 4.0);
  // z(t) = speed·(t* - t): zero exactly at t*.
  const Vector3d p0 = xy + Vector3d(0, 0, speed * t_star);
  const Vector3d p1 = xy + Vector3d(0, 0, speed * (t_star - 1.0));

  CcdQueryCase c;
  c.x = {p0, v0, v1, v2, p1, v0, v1, v2};
  DecorateMotion(family, gen, &c.x);
  PlaceQuery(family, gen, &c.x);
  c.expected_hit = true;
  c.expected_toi = t_star;
  c.toi_tolerance = ToiTolerance(family);
  return c;
}

CcdQueryCase MakeVertexFaceMiss(CcdQueryFamily family, RandomGenerator* gen) {
  const Vector3d v0(-1, -1, 0), v1(1, -1, 0), v2(0, 1, 0);
  CcdQueryCase c;
  if (Uniform(gen, 0, 1) < 0.5) {
    // Containment miss: the point crosses the triangle's plane, but at a
    // barycentric location outside the triangle by a wide (0.3) margin. The
    // triangle is static and the point linear, so the crossing is the only
    // coplanarity time.
    const double u = Uniform(gen, 1.3, 2.0);
    const double w = Uniform(gen, 0.15, 0.5);
    const Vector3d xy = v0 + u * (v1 - v0) + w * (v2 - v0);
    const double t_star = PickToi(family, gen);
    const double speed = Uniform(gen, 0.5, 4.0);
    const Vector3d p0 = xy + Vector3d(0, 0, speed * t_star);
    const Vector3d p1 = xy + Vector3d(0, 0, speed * (t_star - 1.0));
    c.x = {p0, v0, v1, v2, p1, v0, v1, v2};
  } else {
    // Clearance miss: the point stays at least 0.2 above the plane for the
    // whole (widened) step.
    const Vector3d xy0 = UniformBox(gen, 1.0);
    const Vector3d xy1 = UniformBox(gen, 1.0);
    const double z0 = Uniform(gen, 0.2, 2.0);
    const double z1 = Uniform(gen, 0.2, 2.0);
    const Vector3d p0(xy0.x(), xy0.y(), z0);
    const Vector3d p1(xy1.x(), xy1.y(), z1);
    c.x = {p0, v0, v1, v2, p1, v0, v1, v2};
  }
  DecorateMotion(family, gen, &c.x);
  PlaceQuery(family, gen, &c.x);
  c.expected_hit = false;
  return c;
}

CcdQueryCase MakeEdgeEdgeHit(CcdQueryFamily family, RandomGenerator* gen) {
  // Canonical frame: edge P static along the x axis; edge Q lies in a plane
  // parallel to z = 0 and translates down along z, crossing P transversally
  // at t*. Both crossing parameters keep a wide (0.15) interior margin, and
  // the edge directions keep a wide non-parallelism margin.
  const Vector3d p0(-1, 0, 0), p1(1, 0, 0);
  const double s_p = Uniform(gen, 0.15, 0.85);
  const Vector3d cross_point(-1 + 2 * s_p, 0, 0);

  const double theta = Uniform(gen, 0.3, M_PI - 0.3);
  const Vector3d dq(std::cos(theta), std::sin(theta), 0);
  const double length = Uniform(gen, 0.5, 3.0);
  const double s_q = Uniform(gen, 0.15, 0.85);

  const double t_star = PickToi(family, gen);
  const double speed = Uniform(gen, 0.5, 4.0);
  const Vector3d q_start_offset(0, 0, speed * t_star);
  const Vector3d q_end_offset(0, 0, speed * (t_star - 1.0));
  const Vector3d q_lo = cross_point - s_q * length * dq;
  const Vector3d q_hi = cross_point + (1.0 - s_q) * length * dq;

  CcdQueryCase c;
  c.x = {p0, p1, q_lo + q_start_offset, q_hi + q_start_offset,
         p0, p1, q_lo + q_end_offset,   q_hi + q_end_offset};
  const Vector3d dp(1, 0, 0);
  const std::array<Vector3d, 8> dirs = {dp, dp, dq, dq, dp, dp, dq, dq};
  DecorateMotion(family, gen, &c.x, &dirs);
  PlaceQuery(family, gen, &c.x);
  c.expected_hit = true;
  c.expected_toi = t_star;
  c.toi_tolerance = ToiTolerance(family);
  return c;
}

CcdQueryCase MakeEdgeEdgeMiss(CcdQueryFamily family, RandomGenerator* gen) {
  const Vector3d p0(-1, 0, 0), p1(1, 0, 0);
  CcdQueryCase c;
  if (Uniform(gen, 0, 1) < 0.5) {
    // Containment miss: the crossing happens beyond P's endpoint by a wide
    // (0.3) margin along x.
    const double sign = (Uniform(gen, 0, 1) < 0.5) ? -1.0 : 1.0;
    const Vector3d cross_point(sign * Uniform(gen, 1.3, 2.0), 0, 0);
    const double theta = Uniform(gen, 0.3, M_PI - 0.3);
    const Vector3d dq(std::cos(theta), std::sin(theta), 0);
    const double length = Uniform(gen, 0.25, 0.6);  // Short: stays far away.
    const double s_q = Uniform(gen, 0.3, 0.7);
    const double t_star = PickToi(family, gen);
    const double speed = Uniform(gen, 0.5, 4.0);
    const Vector3d q_lo = cross_point - s_q * length * dq;
    const Vector3d q_hi = cross_point + (1.0 - s_q) * length * dq;
    const Vector3d z0(0, 0, speed * t_star);
    const Vector3d z1(0, 0, speed * (t_star - 1.0));
    c.x = {p0, p1, q_lo + z0, q_hi + z0, p0, p1, q_lo + z1, q_hi + z1};
  } else {
    // Clearance miss: edge Q stays at least 0.2 above the z = 0 plane.
    const double theta = Uniform(gen, 0.3, M_PI - 0.3);
    const Vector3d dq(std::cos(theta), std::sin(theta), 0);
    const double length = Uniform(gen, 0.5, 3.0);
    const Vector3d center0 = UniformBox(gen, 1.0) + Vector3d(0, 0, 1.5);
    const Vector3d center1 = UniformBox(gen, 1.0) + Vector3d(0, 0, 1.5);
    auto clamp_z = [](Vector3d p) {
      p.z() = std::max(p.z(), 0.2);
      return p;
    };
    c.x = {p0,
           p1,
           clamp_z(center0 - 0.5 * length * dq),
           clamp_z(center0 + 0.5 * length * dq),
           p0,
           p1,
           clamp_z(center1 - 0.5 * length * dq),
           clamp_z(center1 + 0.5 * length * dq)};
  }
  DecorateMotion(family, gen, &c.x);
  PlaceQuery(family, gen, &c.x);
  c.expected_hit = false;
  return c;
}

CcdQueryCase MakeRandomQuery(RandomGenerator* gen) {
  CcdQueryCase c;
  for (auto& p : c.x) {
    p = UniformBox(gen, 1.0);
  }
  return c;
}

std::string ToRerunnableString(const CcdQueryCase& c) {
  std::string out;
  for (int i = 0; i < 8; ++i) {
    out += fmt::format("  const Vector3d x{}({:.17g}, {:.17g}, {:.17g});\n", i,
                       c.x[i].x(), c.x[i].y(), c.x[i].z());
  }
  if (c.expected_hit.has_value()) {
    out += fmt::format("  // expected_hit = {}, expected_toi = {}\n",
                       *c.expected_hit, c.expected_toi);
  }
  return out;
}

double SegmentSegmentSquaredDistance(const Vector3d& p1, const Vector3d& q1,
                                     const Vector3d& p2, const Vector3d& q2) {
  // Ericson, Real-time collision detection, Section 5.1.9 (clamped closest
  // points); mirrors the production implementation inside ccd.cc.
  constexpr double kEps2 = std::numeric_limits<double>::epsilon() *
                           std::numeric_limits<double>::epsilon();
  double s, t;
  const Vector3d d1 = q1 - p1;
  const Vector3d d2 = q2 - p2;
  const Vector3d r = p1 - p2;
  const double a = d1.dot(d1);
  const double e = d2.dot(d2);
  const double f = d2.dot(r);

  if (a <= kEps2 && e <= kEps2) {
    return r.squaredNorm();
  }
  if (a <= kEps2) {
    s = 0;
    t = std::clamp(f / e, 0.0, 1.0);
  } else {
    const double c = d1.dot(r);
    if (e <= kEps2) {
      t = 0;
      s = std::clamp(-c / a, 0.0, 1.0);
    } else {
      const double b = d1.dot(d2);
      const double denom = a * e - b * b;
      if (denom != 0) {
        s = std::clamp((b * f - c * e) / denom, 0.0, 1.0);
      } else {
        s = 0;
      }
      t = (b * s + f) / e;
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

}  // namespace internal
}  // namespace geometry
}  // namespace drake
