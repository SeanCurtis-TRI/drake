#pragma once

#include <array>
#include <optional>
#include <string>

#include "drake/common/eigen_types.h"
#include "drake/common/random.h"

namespace drake {
namespace geometry {
namespace internal {

/* One CCD query: eight points. For point-triangle the order is
 (p0, v00, v10, v20, p1, v01, v11, v21); for edge-edge it is
 (p00, p10, q00, q10, p01, p11, q01, q11) — matching the kernel signatures in
 ccd.h. */
struct CcdQueryCase {
  std::array<Eigen::Vector3d, 8> x;
  /* Ground truth established *by construction* (with wide margins, so it
   survives the rounding of the applied random transforms); nullopt means
   unknown — use an oracle. */
  std::optional<bool> expected_hit;
  /* Valid only when expected_hit == true; negative means "hit, but the time
   of impact is not certified". */
  double expected_toi{-1.0};
  /* Absolute tolerance on expected_toi. */
  double toi_tolerance{1e-6};
};

/* Adversarial families for importance-biased differential testing. Every
 family still carries constructed ground truth. */
enum class CcdQueryFamily {
  kClean,                        // Well-conditioned, moderate scale.
  kFastTranslationTinyRotation,  // COR-1 regime: large common translation
                                 // plus per-vertex drift of magnitude
                                 // 1e-13..1e-5.
  kBoundaryTime,                 // Impact within 1e-7 of t = 0 or t = 1.
  kTinyScale,                    // Query coordinates scaled by ~1e-6.
  kHugeScale,                    // Query coordinates scaled by ~1e6.
  kDyadic,                       // Coordinates snapped to k/2^20 so that
                                 // low-order arithmetic is exact.
};

constexpr CcdQueryFamily kAllCcdQueryFamilies[] = {
    CcdQueryFamily::kClean,        CcdQueryFamily::kFastTranslationTinyRotation,
    CcdQueryFamily::kBoundaryTime, CcdQueryFamily::kTinyScale,
    CcdQueryFamily::kHugeScale,    CcdQueryFamily::kDyadic,
};

const char* to_string(CcdQueryFamily family);

/* Constructed-truth generators. Hits cross with wide containment margins;
 misses either miss containment by a wide margin at the (single) coplanarity
 time or keep a wide clearance for the whole step. */
CcdQueryCase MakeVertexFaceHit(CcdQueryFamily family, RandomGenerator* gen);
CcdQueryCase MakeVertexFaceMiss(CcdQueryFamily family, RandomGenerator* gen);
CcdQueryCase MakeEdgeEdgeHit(CcdQueryFamily family, RandomGenerator* gen);
CcdQueryCase MakeEdgeEdgeMiss(CcdQueryFamily family, RandomGenerator* gen);

/* Unconstrained query (all eight points uniform in [-1, 1]³); no ground
 truth. */
CcdQueryCase MakeRandomQuery(RandomGenerator* gen);

/* Squared distance between segments [p1, q1] and [p2, q2] (clamped closest
 points, robust to parallel/degenerate segments). Test-support copy for
 building oracles; the production twin lives inside ccd.cc. */
double SegmentSegmentSquaredDistance(const Eigen::Vector3d& p1,
                                     const Eigen::Vector3d& q1,
                                     const Eigen::Vector3d& p2,
                                     const Eigen::Vector3d& q2);

/* Formats the eight points as re-runnable C++ literals so any failing case
 can be reproduced as a standalone regression test. */
std::string ToRerunnableString(const CcdQueryCase& c);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
