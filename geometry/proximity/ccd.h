#pragma once

#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {
namespace internal {

/* Default per-substep rotation bound (π/2 radians) for the rotation-adaptive
 subdivision of the CCD feasibility queries; see
 ProximityEngine::IsFeasibleTrajectory(). The E6a study data supports π/2
 with wide margin: the unsubdivided linear check missed collisions only for
 per-step rotations θ ≥ 5.6 rad in that scene. An adversarial scene (obstacle
 very close to the rotation axis) could require a smaller value; callers can
 tighten per query. */
inline constexpr double kDefaultCcdMaxSubstepRotation = 1.57079632679489661923;

using Eigen::Vector3d;

/* Continuous collision detection between a moving point p and a moving
 triangle [v₀, v₁, v₂], with all points moving on linear trajectories over
 t ∈ [0, 1] between their given start (suffix 0) and end (suffix 1)
 positions.

 Returns true if the point crosses the triangle's interior at some
 t ∈ [0, 1] (evaluated at the roots of the coplanarity condition, with a
 small conservative tolerance at the interval boundaries); on a hit, *toi
 contains the earliest such time (clamped to [0, 1]). On a miss, *toi is
 untouched. */
bool point_triangle_ccd(const Vector3d& p0, const Vector3d& v00,
                        const Vector3d& v10, const Vector3d& v20,
                        const Vector3d& p1, const Vector3d& v01,
                        const Vector3d& v11, const Vector3d& v21, double* toi);

/* Continuous collision detection between moving edges [p₀, p₁] and [q₀, q₁];
 same trajectory model and contract as point_triangle_ccd(). */
bool edge_edge_ccd(const Vector3d& p00, const Vector3d& p10,
                   const Vector3d& q00, const Vector3d& q10,
                   const Vector3d& p01, const Vector3d& p11,
                   const Vector3d& q01, const Vector3d& q11, double* toi);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
