/* Differential and constructed-truth testing for the CCD narrowphase
 kernels, against oracles that are INDEPENDENT of the kernels' own numerics:

 1. Constructed ground truth: query generators (ccd_test_configurations)
    build hits and misses with wide margins by construction, biased into the
    adversarial regimes (fast translation + tiny rotation, boundary-time
    impacts, extreme scales, dyadic-exact coordinates). The hard guarantee
    asserted here is ZERO FALSE NEGATIVES on constructed hits: a false
    negative is a missed collision, the one error class the Barrier CENIC
    non-penetration guarantee cannot tolerate. Constructed misses have
    margins far beyond the kernels' conservative tolerances, so they are also
    asserted strictly (guarding against a trivially-conservative kernel).

 2. fcl's interval-Newton CCD (fcl::detail::Intersect<double>::intersect_VF /
    intersect_EE), an independently-implemented solver for the identical
    problem statement. Verdict disagreements where OUR kernel reports miss
    and fcl reports hit are adjudicated by a dense-time sampler; a confirmed
    miss fails the test. (fcl is tolerance-based, not exact, so its
    unconfirmed extra hits are only counted.)

 Run a soak with: bazel run //geometry/proximity:ccd_differential_test -- \
   --num_trials=100000 */

#include <algorithm>
#include <cmath>
#include <limits>

#include <fcl/narrowphase/detail/traversal/collision/intersect.h>
#include <fmt/format.h>
#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "drake/geometry/proximity/ccd.h"
#include "drake/geometry/proximity/test/ccd_test_configurations.h"

DEFINE_int32(num_trials, 500,
             "Constructed-truth trials per (family, kernel, verdict) cell and "
             "random queries per comparison suite.");

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;

constexpr double kInf = std::numeric_limits<double>::infinity();

bool RunPointTriangle(const CcdQueryCase& c, double* toi) {
  return point_triangle_ccd(c.x[0], c.x[1], c.x[2], c.x[3], c.x[4], c.x[5],
                            c.x[6], c.x[7], toi);
}

bool RunEdgeEdge(const CcdQueryCase& c, double* toi) {
  return edge_edge_ccd(c.x[0], c.x[1], c.x[2], c.x[3], c.x[4], c.x[5], c.x[6],
                       c.x[7], toi);
}

/* fcl's argument order is (triangle..., point) versus our (point,
 triangle...). */
bool RunFclVF(const CcdQueryCase& c, double* toi) {
  Vector3d contact_point = Vector3d::Zero();
  // N.B. vendor_cxx strips default arguments; all parameters are explicit.
  return fcl::detail::Intersect<double>::intersect_VF(
      c.x[1], c.x[2], c.x[3], c.x[0], c.x[5], c.x[6], c.x[7], c.x[4], toi,
      &contact_point, true);
}

bool RunFclEE(const CcdQueryCase& c, double* toi) {
  Vector3d contact_point = Vector3d::Zero();
  return fcl::detail::Intersect<double>::intersect_EE(
      c.x[0], c.x[1], c.x[2], c.x[3], c.x[4], c.x[5], c.x[6], c.x[7], toi,
      &contact_point, true);
}

Vector3d Lerp(const Vector3d& a, const Vector3d& b, double t) {
  return a + t * (b - a);
}

/* Dense-time adjudicator for point-triangle: reports true only when a
 plane-side sign flip of the point occurs between adjacent samples while the
 projected barycentric location is strictly inside the triangle — i.e. a
 crossing that any correct CCD must report. Deliberately strict, so a "true"
 verdict is trustworthy evidence of a missed collision. */
bool DenseVFHit(const CcdQueryCase& c, int num_samples = 4096) {
  double prev_side = 0.0;
  bool prev_valid = false;
  bool prev_inside = false;
  for (int k = 0; k <= num_samples; ++k) {
    const double t = static_cast<double>(k) / num_samples;
    const Vector3d p = Lerp(c.x[0], c.x[4], t);
    const Vector3d v0 = Lerp(c.x[1], c.x[5], t);
    const Vector3d v1 = Lerp(c.x[2], c.x[6], t);
    const Vector3d v2 = Lerp(c.x[3], c.x[7], t);
    const Vector3d e0 = v1 - v0;
    const Vector3d e1 = v2 - v0;
    const Vector3d n = e0.cross(e1);
    const double side = n.dot(p - v0);
    // Strictly-interior barycentric test (margin 0.02).
    Eigen::Matrix2d A;
    A << e0.dot(e0), e0.dot(e1), e0.dot(e1), e1.dot(e1);
    const Eigen::Vector2d rhs(e0.dot(p - v0), e1.dot(p - v0));
    const Eigen::Vector2d uv = A.ldlt().solve(rhs);
    const bool inside = uv[0] >= 0.02 && uv[1] >= 0.02 &&
                        uv[0] + uv[1] <= 0.98 && n.squaredNorm() > 0;
    if (prev_valid && std::signbit(prev_side) != std::signbit(side) &&
        prev_side != 0 && side != 0 && prev_inside && inside) {
      return true;
    }
    prev_side = side;
    prev_valid = true;
    prev_inside = inside;
  }
  return false;
}

/* Dense-time adjudicator for edge-edge: reports true only when the sampled
 segment-segment distance dips below a strict threshold (relative to the
 query's extent). */
bool DenseEEHit(const CcdQueryCase& c, int num_samples = 4096) {
  double scale = 0.0;
  for (const auto& p : c.x) scale = std::max(scale, p.norm());
  const double threshold2 = 1e-8 * scale * scale;
  for (int k = 0; k <= num_samples; ++k) {
    const double t = static_cast<double>(k) / num_samples;
    const Vector3d p0 = Lerp(c.x[0], c.x[4], t);
    const Vector3d p1 = Lerp(c.x[1], c.x[5], t);
    const Vector3d q0 = Lerp(c.x[2], c.x[6], t);
    const Vector3d q1 = Lerp(c.x[3], c.x[7], t);
    if (SegmentSegmentSquaredDistance(p0, p1, q0, q1) <= threshold2) {
      return true;
    }
  }
  return false;
}

/* ========================== fcl link smoke tests ========================= */

GTEST_TEST(FclOracleSmoke, StaticTriangleTriangle) {
  const Vector3d p1(-1, -1, 0), p2(1, -1, 0), p3(0, 1, 0);
  const Vector3d q1(0, -0.2, -1), q2(0.2, 0.2, 1), q3(-0.2, 0.2, 1);
  EXPECT_TRUE(fcl::detail::Intersect<double>::intersect_Triangle(
      p1, p2, p3, q1, q2, q3, nullptr, nullptr, nullptr, nullptr));
  const Vector3d r1(10, 10, 5), r2(12, 10, 5), r3(10, 12, 5);
  EXPECT_FALSE(fcl::detail::Intersect<double>::intersect_Triangle(
      p1, p2, p3, r1, r2, r3, nullptr, nullptr, nullptr, nullptr));
}

GTEST_TEST(FclOracleSmoke, MovingVertexFace) {
  const Vector3d a(-2, -2, 0), b(2, -2, 0), c(0, 4, 0);
  const Vector3d p0(0, 0, 2), p1(0, 0, -2);
  double collision_time{-1};
  Vector3d contact_point = Vector3d::Zero();
  const bool hit = fcl::detail::Intersect<double>::intersect_VF(
      a, b, c, p0, a, b, c, p1, &collision_time, &contact_point, true);
  EXPECT_TRUE(hit);
  EXPECT_NEAR(collision_time, 0.5, 1e-6);
}

/* ========================= constructed ground truth ====================== */

struct KernelCase {
  const char* name;
  CcdQueryCase (*make_hit)(CcdQueryFamily, RandomGenerator*);
  CcdQueryCase (*make_miss)(CcdQueryFamily, RandomGenerator*);
  bool (*run)(const CcdQueryCase&, double*);
};

constexpr KernelCase kKernelCases[] = {
    {"point_triangle", &MakeVertexFaceHit, &MakeVertexFaceMiss,
     &RunPointTriangle},
    {"edge_edge", &MakeEdgeEdgeHit, &MakeEdgeEdgeMiss, &RunEdgeEdge},
};

GTEST_TEST(CcdConstructedTruthTest, NoFalseNegativesOrPositives) {
  RandomGenerator generator(1234);
  for (const auto& kernel : kKernelCases) {
    for (const CcdQueryFamily family : kAllCcdQueryFamilies) {
      int false_negatives = 0;
      int false_positives = 0;
      int toi_errors = 0;
      for (int i = 0; i < FLAGS_num_trials; ++i) {
        {
          const CcdQueryCase hit = kernel.make_hit(family, &generator);
          double toi = kInf;
          if (!kernel.run(hit, &toi)) {
            ++false_negatives;
            ADD_FAILURE() << fmt::format(
                "FALSE NEGATIVE: {} {} trial {} missed a constructed hit "
                "(expected toi {}):\n{}",
                kernel.name, to_string(family), i, hit.expected_toi,
                ToRerunnableString(hit));
          } else if (hit.expected_toi >= 0 &&
                     std::abs(toi - hit.expected_toi) > hit.toi_tolerance) {
            ++toi_errors;
            ADD_FAILURE() << fmt::format(
                "TOI ERROR: {} {} trial {}: toi = {}, expected {} ± {}:\n{}",
                kernel.name, to_string(family), i, toi, hit.expected_toi,
                hit.toi_tolerance, ToRerunnableString(hit));
          }
        }
        {
          const CcdQueryCase miss = kernel.make_miss(family, &generator);
          double toi = kInf;
          if (kernel.run(miss, &toi)) {
            ++false_positives;
            ADD_FAILURE() << fmt::format(
                "FALSE POSITIVE: {} {} trial {} hit a constructed miss "
                "(toi = {}):\n{}",
                kernel.name, to_string(family), i, toi,
                ToRerunnableString(miss));
          }
        }
        // Bail out early if something is systematically broken.
        if (false_negatives + false_positives + toi_errors > 10) {
          GTEST_FAIL() << "aborting after >10 failures in " << kernel.name
                       << "/" << to_string(family);
        }
      }
    }
  }
}

/* ====================== differential vs fcl's kernels ==================== */

GTEST_TEST(CcdFclDifferentialTest, NoConfirmedMissedCollisions) {
  RandomGenerator generator(1234);
  struct Stats {
    int agree = 0;
    int ours_only = 0;        // We hit, fcl misses: conservative, acceptable.
    int fcl_unconfirmed = 0;  // fcl hits, we miss, sampler cannot confirm.
  };
  Stats vf_stats, ee_stats;

  auto compare = [](const CcdQueryCase& c,
                    bool (*ours)(const CcdQueryCase&, double*),
                    bool (*fcl_run)(const CcdQueryCase&, double*),
                    bool (*dense)(const CcdQueryCase&, int), Stats* stats,
                    const char* name) {
    double toi_ours = kInf;
    double toi_fcl = kInf;
    const bool hit_ours = ours(c, &toi_ours);
    const bool hit_fcl = fcl_run(c, &toi_fcl);
    if (hit_ours == hit_fcl) {
      ++stats->agree;
    } else if (hit_ours) {
      ++stats->ours_only;
    } else {
      // fcl reports a collision we do not. Adjudicate with the strict
      // dense-time sampler: if it confirms a genuine crossing, our kernel
      // has a real missed collision.
      if (dense(c, 4096)) {
        ADD_FAILURE() << fmt::format(
            "CONFIRMED MISSED COLLISION ({}): fcl toi = {}:\n{}", name, toi_fcl,
            ToRerunnableString(c));
      } else {
        ++stats->fcl_unconfirmed;
      }
    }
  };

  for (int i = 0; i < 4 * FLAGS_num_trials; ++i) {
    const CcdQueryCase c = MakeRandomQuery(&generator);
    compare(c, &RunPointTriangle, &RunFclVF, &DenseVFHit, &vf_stats,
            "point_triangle");
    compare(c, &RunEdgeEdge, &RunFclEE, &DenseEEHit, &ee_stats, "edge_edge");
  }
  // Also push the constructed adversarial families through the comparison.
  for (const CcdQueryFamily family : kAllCcdQueryFamilies) {
    for (int i = 0; i < FLAGS_num_trials; ++i) {
      compare(MakeVertexFaceHit(family, &generator), &RunPointTriangle,
              &RunFclVF, &DenseVFHit, &vf_stats, "point_triangle");
      compare(MakeEdgeEdgeHit(family, &generator), &RunEdgeEdge, &RunFclEE,
              &DenseEEHit, &ee_stats, "edge_edge");
    }
  }

  fmt::print(
      "VF: agree {}, ours-only {}, fcl-unconfirmed {}\n"
      "EE: agree {}, ours-only {}, fcl-unconfirmed {}\n",
      vf_stats.agree, vf_stats.ours_only, vf_stats.fcl_unconfirmed,
      ee_stats.agree, ee_stats.ours_only, ee_stats.fcl_unconfirmed);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
