#pragma once

#include <unordered_map>

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/hydroelastic_internal.h"

namespace drake {
namespace geometry {
namespace internal {
namespace hydroelastic {

/* Narrowphase feasibility (linear CCD) between one pair of compliant
 geometries that carry rigid-core collision meshes. Works in double
 exclusively: the caller (ProximityEngine) converts the scalar-typed pose
 maps once per query, rather than per primitive pair.

 Vertex positions are interpolated linearly between the two pose maps'
 endpoint placements; candidate vertex-face and edge-edge pairs come from the
 meshes' moving-AABB BVHs. */
class FeasibilityCalculator {
 public:
  /* Constructs the fully-specified calculator. All parameters are aliased
   and must remain valid (and unmodified) at least as long as this instance.

   @param geometries  The set of all hydroelastic geometric representations.
   @param X_WGs_prev  Poses at the start of the step for (at least) every
                      geometry this calculator will be queried on.
   @param X_WGs_next  Poses at the end of the step, ditto. */
  FeasibilityCalculator(
      Geometries* geometries,
      const std::unordered_map<GeometryId, math::RigidTransformd>* X_WGs_prev,
      const std::unordered_map<GeometryId, math::RigidTransformd>* X_WGs_next)
      : geometries_(*geometries),
        X_WGs_prev_(*X_WGs_prev),
        X_WGs_next_(*X_WGs_next) {
    DRAKE_DEMAND(geometries != nullptr);
    DRAKE_DEMAND(X_WGs_prev != nullptr);
    DRAKE_DEMAND(X_WGs_next != nullptr);
  }

  /* Returns true if the two geometries do not collide over the
     (linearly-interpolated) trajectories of their respective mesh vertices.

     @param id_A     Id of the first object in the pair (order insignificant).
     @param id_B     Id of the second object in the pair (order insignificant).
      */
  bool IsFeasibleTrajectory(GeometryId id_A, GeometryId id_B);

  /* Returns the earliest time of impact in [0, 1] between the two
   geometries' interpolated trajectories, or +infinity if they do not
   collide. */
  double FeasibilityTimeOfImpact(GeometryId id_A, GeometryId id_B);

 private:
  bool IsFeasibleTrajectoryVertexFace(GeometryId id_A, int v_A, GeometryId id_B,
                                      int t_B) const;

  bool IsFeasibleTrajectoryEdgeEdge(GeometryId id_A, int e_A, GeometryId id_B,
                                    int e_B) const;

  double FeasibilityTimeOfImpactVertexFace(GeometryId id_A, int v_A,
                                           GeometryId id_B, int t_B) const;

  double FeasibilityTimeOfImpactEdgeEdge(GeometryId id_A, int e_A,
                                         GeometryId id_B, int e_B) const;

  /* The hydroelastic geometric representations.  */
  Geometries& geometries_;
  const std::unordered_map<GeometryId, math::RigidTransformd>& X_WGs_prev_;
  const std::unordered_map<GeometryId, math::RigidTransformd>& X_WGs_next_;
};

}  // namespace hydroelastic
}  // namespace internal
}  // namespace geometry
}  // namespace drake
