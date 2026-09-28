#include "drake/geometry/proximity/feasibility_calculator.h"

#include <algorithm>
#include <array>
#include <limits>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/ccd.h"

namespace drake {
namespace geometry {
namespace internal {
namespace hydroelastic {

using Eigen::Vector3d;
using math::RigidTransformd;

bool FeasibilityCalculator::IsFeasibleTrajectoryVertexFace(GeometryId id_A,
                                                           int v_A,
                                                           GeometryId id_B,
                                                           int t_B) const {
  double t;

  const RigidTransformd& X_WA0 = X_WGs_prev_.at(id_A);
  const RigidTransformd& X_WA1 = X_WGs_next_.at(id_A);
  const RigidTransformd& X_WB0 = X_WGs_prev_.at(id_B);
  const RigidTransformd& X_WB1 = X_WGs_next_.at(id_B);

  const CompliantMesh& mesh_A =
      geometries_.compliant_geometry(id_A).compliant_mesh();
  const CompliantMesh& mesh_B =
      geometries_.compliant_geometry(id_B).compliant_mesh();

  const SurfaceTriangle& triB = mesh_B.collision_mesh().element(t_B);
  const std::array<int, 3>& v_B = triB.vertices();

  const Vector3d a_0 = X_WA0 * mesh_A.collision_mesh().vertex(v_A);
  const Vector3d a_1 = X_WA1 * mesh_A.collision_mesh().vertex(v_A);
  const Vector3d b0_0 = X_WB0 * mesh_B.collision_mesh().vertex(v_B[0]);
  const Vector3d b1_0 = X_WB0 * mesh_B.collision_mesh().vertex(v_B[1]);
  const Vector3d b2_0 = X_WB0 * mesh_B.collision_mesh().vertex(v_B[2]);
  const Vector3d b0_1 = X_WB1 * mesh_B.collision_mesh().vertex(v_B[0]);
  const Vector3d b1_1 = X_WB1 * mesh_B.collision_mesh().vertex(v_B[1]);
  const Vector3d b2_1 = X_WB1 * mesh_B.collision_mesh().vertex(v_B[2]);

  if (point_triangle_ccd(a_0, b0_0, b1_0, b2_0, a_1, b0_1, b1_1, b2_1, &t)) {
    return false;
  }

  return true;
}

double FeasibilityCalculator::FeasibilityTimeOfImpactVertexFace(GeometryId id_A,
                                                                int v_A,
                                                                GeometryId id_B,
                                                                int t_B) const {
  double t;

  const RigidTransformd& X_WA0 = X_WGs_prev_.at(id_A);
  const RigidTransformd& X_WA1 = X_WGs_next_.at(id_A);
  const RigidTransformd& X_WB0 = X_WGs_prev_.at(id_B);
  const RigidTransformd& X_WB1 = X_WGs_next_.at(id_B);

  const CompliantMesh& mesh_A =
      geometries_.compliant_geometry(id_A).compliant_mesh();
  const CompliantMesh& mesh_B =
      geometries_.compliant_geometry(id_B).compliant_mesh();

  const SurfaceTriangle& triB = mesh_B.collision_mesh().element(t_B);
  const std::array<int, 3>& v_B = triB.vertices();

  const Vector3d a_0 = X_WA0 * mesh_A.collision_mesh().vertex(v_A);
  const Vector3d a_1 = X_WA1 * mesh_A.collision_mesh().vertex(v_A);
  const Vector3d b0_0 = X_WB0 * mesh_B.collision_mesh().vertex(v_B[0]);
  const Vector3d b1_0 = X_WB0 * mesh_B.collision_mesh().vertex(v_B[1]);
  const Vector3d b2_0 = X_WB0 * mesh_B.collision_mesh().vertex(v_B[2]);
  const Vector3d b0_1 = X_WB1 * mesh_B.collision_mesh().vertex(v_B[0]);
  const Vector3d b1_1 = X_WB1 * mesh_B.collision_mesh().vertex(v_B[1]);
  const Vector3d b2_1 = X_WB1 * mesh_B.collision_mesh().vertex(v_B[2]);

  if (point_triangle_ccd(a_0, b0_0, b1_0, b2_0, a_1, b0_1, b1_1, b2_1, &t)) {
    return t;
  }
  return std::numeric_limits<double>::infinity();
}

bool FeasibilityCalculator::IsFeasibleTrajectoryEdgeEdge(GeometryId id_A,
                                                         int e_A,
                                                         GeometryId id_B,
                                                         int e_B) const {
  double t;

  const CompliantMesh& mesh_A =
      geometries_.compliant_geometry(id_A).compliant_mesh();
  const CompliantMesh& mesh_B =
      geometries_.compliant_geometry(id_B).compliant_mesh();

  const RigidTransformd& X_WA0 = X_WGs_prev_.at(id_A);
  const RigidTransformd& X_WA1 = X_WGs_next_.at(id_A);
  const RigidTransformd& X_WB0 = X_WGs_prev_.at(id_B);
  const RigidTransformd& X_WB1 = X_WGs_next_.at(id_B);

  const auto [v_A0, v_A1] = mesh_A.collision_mesh().edge(e_A);
  const auto [v_B0, v_B1] = mesh_B.collision_mesh().edge(e_B);

  const Vector3d a0_0 = X_WA0 * mesh_A.collision_mesh().vertex(v_A0);
  const Vector3d a1_0 = X_WA0 * mesh_A.collision_mesh().vertex(v_A1);
  const Vector3d a0_1 = X_WA1 * mesh_A.collision_mesh().vertex(v_A0);
  const Vector3d a1_1 = X_WA1 * mesh_A.collision_mesh().vertex(v_A1);
  const Vector3d b0_0 = X_WB0 * mesh_B.collision_mesh().vertex(v_B0);
  const Vector3d b1_0 = X_WB0 * mesh_B.collision_mesh().vertex(v_B1);
  const Vector3d b0_1 = X_WB1 * mesh_B.collision_mesh().vertex(v_B0);
  const Vector3d b1_1 = X_WB1 * mesh_B.collision_mesh().vertex(v_B1);

  if (edge_edge_ccd(a0_0, a1_0, b0_0, b1_0, a0_1, a1_1, b0_1, b1_1, &t)) {
    return false;
  }

  return true;
}

double FeasibilityCalculator::FeasibilityTimeOfImpactEdgeEdge(GeometryId id_A,
                                                              int e_A,
                                                              GeometryId id_B,
                                                              int e_B) const {
  double t;

  const CompliantMesh& mesh_A =
      geometries_.compliant_geometry(id_A).compliant_mesh();
  const CompliantMesh& mesh_B =
      geometries_.compliant_geometry(id_B).compliant_mesh();

  const RigidTransformd& X_WA0 = X_WGs_prev_.at(id_A);
  const RigidTransformd& X_WA1 = X_WGs_next_.at(id_A);
  const RigidTransformd& X_WB0 = X_WGs_prev_.at(id_B);
  const RigidTransformd& X_WB1 = X_WGs_next_.at(id_B);

  const auto [v_A0, v_A1] = mesh_A.collision_mesh().edge(e_A);
  const auto [v_B0, v_B1] = mesh_B.collision_mesh().edge(e_B);

  const Vector3d a0_0 = X_WA0 * mesh_A.collision_mesh().vertex(v_A0);
  const Vector3d a1_0 = X_WA0 * mesh_A.collision_mesh().vertex(v_A1);
  const Vector3d a0_1 = X_WA1 * mesh_A.collision_mesh().vertex(v_A0);
  const Vector3d a1_1 = X_WA1 * mesh_A.collision_mesh().vertex(v_A1);
  const Vector3d b0_0 = X_WB0 * mesh_B.collision_mesh().vertex(v_B0);
  const Vector3d b1_0 = X_WB0 * mesh_B.collision_mesh().vertex(v_B1);
  const Vector3d b0_1 = X_WB1 * mesh_B.collision_mesh().vertex(v_B0);
  const Vector3d b1_1 = X_WB1 * mesh_B.collision_mesh().vertex(v_B1);

  if (edge_edge_ccd(a0_0, a1_0, b0_0, b1_0, a0_1, a1_1, b0_1, b1_1, &t)) {
    return t;
  }

  return std::numeric_limits<double>::infinity();
}

bool FeasibilityCalculator::IsFeasibleTrajectory(GeometryId id_A,
                                                 GeometryId id_B) {
  CompliantMesh& soft_A =
      geometries_.mutable_compliant_geometry(id_A).mutable_compliant_mesh();
  CompliantMesh& soft_B =
      geometries_.mutable_compliant_geometry(id_B).mutable_compliant_mesh();

  std::vector<std::pair<int, int>> vertex_A_face_B_pairs =
      soft_A.mutable_collision_mesh_vertex_bvh().GetMovingCollisionCandidates(
          soft_B.mutable_collision_mesh_face_bvh());
  for (const auto& [v_A, f_B] : vertex_A_face_B_pairs) {
    if (!IsFeasibleTrajectoryVertexFace(id_A, v_A, id_B, f_B)) {
      return false;
    }
  }

  std::vector<std::pair<int, int>> vertex_B_face_A_pairs =
      soft_B.mutable_collision_mesh_vertex_bvh().GetMovingCollisionCandidates(
          soft_A.mutable_collision_mesh_face_bvh());
  for (const auto& [v_B, f_A] : vertex_B_face_A_pairs) {
    if (!IsFeasibleTrajectoryVertexFace(id_B, v_B, id_A, f_A)) {
      return false;
    }
  }

  std::vector<std::pair<int, int>> edge_a_edge_B_pairs =
      soft_A.mutable_collision_mesh_edge_bvh().GetMovingCollisionCandidates(
          soft_B.mutable_collision_mesh_edge_bvh());
  for (const auto& [e_A, e_B] : edge_a_edge_B_pairs) {
    if (!IsFeasibleTrajectoryEdgeEdge(id_A, e_A, id_B, e_B)) {
      return false;
    }
  }

  return true;
}

double FeasibilityCalculator::FeasibilityTimeOfImpact(GeometryId id_A,
                                                      GeometryId id_B) {
  using std::min;
  CompliantMesh& soft_A =
      geometries_.mutable_compliant_geometry(id_A).mutable_compliant_mesh();
  CompliantMesh& soft_B =
      geometries_.mutable_compliant_geometry(id_B).mutable_compliant_mesh();
  double t_impact = std::numeric_limits<double>::infinity();
  std::vector<std::pair<int, int>> vertex_A_face_B_pairs =
      soft_A.mutable_collision_mesh_vertex_bvh().GetMovingCollisionCandidates(
          soft_B.mutable_collision_mesh_face_bvh());
  for (const auto& [v_A, f_B] : vertex_A_face_B_pairs) {
    t_impact =
        min(t_impact, FeasibilityTimeOfImpactVertexFace(id_A, v_A, id_B, f_B));
  }
  std::vector<std::pair<int, int>> vertex_B_face_A_pairs =
      soft_B.mutable_collision_mesh_vertex_bvh().GetMovingCollisionCandidates(
          soft_A.mutable_collision_mesh_face_bvh());
  for (const auto& [v_B, f_A] : vertex_B_face_A_pairs) {
    t_impact =
        min(t_impact, FeasibilityTimeOfImpactVertexFace(id_B, v_B, id_A, f_A));
  }

  std::vector<std::pair<int, int>> edge_a_edge_B_pairs =
      soft_A.mutable_collision_mesh_edge_bvh().GetMovingCollisionCandidates(
          soft_B.mutable_collision_mesh_edge_bvh());
  for (const auto& [e_A, e_B] : edge_a_edge_B_pairs) {
    t_impact =
        min(t_impact, FeasibilityTimeOfImpactEdgeEdge(id_A, e_A, id_B, e_B));
  }
  return t_impact;
}

}  // namespace hydroelastic
}  // namespace internal
}  // namespace geometry
}  // namespace drake
