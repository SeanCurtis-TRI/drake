/* Engine-level tests for the CCD feasibility queries
 (ProximityEngine::IsFeasibleTrajectory / FeasibilityTimeOfImpact), focused on
 the rotation-adaptive conservative subdivision (S1) added in the CCD
 robustness campaign.

 The scene mirrors the E6a study: a thin square "blade" spins about the world
 z axis; a smaller thin square "obstacle" stands in the x = 0 plane inside
 the swept annulus of the blade's corners. A large per-step rotation makes
 the blade's linear vertex chords contract through the middle of the annulus
 and miss the obstacle; the subdivided check must catch the collision. */

#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity_engine.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;
using math::RigidTransformd;
using math::RotationMatrixd;

using PoseMap = std::unordered_map<GeometryId, RigidTransformd>;

constexpr double kInf = std::numeric_limits<double>::infinity();

class ProximityEngineCcdTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const std::string obj =
        FindResourceOrThrow("drake/geometry/test/ccd_square.obj");

    ProximityProperties props;
    AddCompliantHydroelasticProperties(/* resolution_hint = */ 1.0,
                                       /* hydroelastic_modulus = */ 1e8,
                                       &props);
    props.AddProperty(kHydroGroup, kMargin, 1e-4);
    props.AddProperty(kHydroGroup, kBarrier, 1e-4);

    // Blade: 0.2 x 0.2 m thin square in the z = 0 plane (corner radius
    // ~0.1414 m), spinning about the world z axis (poses supplied per query).
    blade_id_ = GeometryId::get_new_id();
    engine_.AddDynamicGeometry(Mesh(obj, 0.1), RigidTransformd(), blade_id_,
                               props);

    // Obstacle: 0.1 x 0.1 m thin square in the x = 0 plane, spanning
    // y ∈ [0.102, 0.202], z ∈ [-0.05, 0.05] — strictly outside the blade at
    // angle 0 (the blade's reach at x = 0 is its apothem, 0.1), but inside
    // the blade corners' swept annulus r ∈ [0.1, 0.1414].
    obstacle_id_ = GeometryId::get_new_id();
    X_WO_ = RigidTransformd(RotationMatrixd::MakeYRotation(M_PI / 2),
                            Vector3d(0, 0.152, 0));
    engine_.AddDynamicGeometry(Mesh(obj, 0.05), X_WO_, obstacle_id_, props);
  }

  /* Poses for a step where the blade spins from angle 0 to `theta` about the
   world z axis and the obstacle holds still. */
  std::pair<PoseMap, PoseMap> MakeSpinStep(double theta) const {
    PoseMap X_prev, X_next;
    X_prev[blade_id_] = RigidTransformd();
    X_next[blade_id_] = RigidTransformd(RotationMatrixd::MakeZRotation(theta),
                                        Vector3d::Zero());
    X_prev[obstacle_id_] = X_WO_;
    X_next[obstacle_id_] = X_WO_;
    return {X_prev, X_next};
  }

  ProximityEngine<double> engine_;
  GeometryId blade_id_;
  GeometryId obstacle_id_;
  RigidTransformd X_WO_;
};

TEST_F(ProximityEngineCcdTest, HydroelasticMeshStats) {
  // Both geometries are thin squares (2 triangles each) carrying rigid-core
  // collision meshes, plus extruded barrier volume meshes.
  const HydroelasticMeshStats stats = engine_.ComputeHydroelasticMeshStats();
  EXPECT_EQ(stats.num_geometries, 2);
  EXPECT_EQ(stats.num_surface_triangles, 4);  // 2 triangles per square.
  EXPECT_GT(stats.num_tetrahedra, 0);         // Extruded barrier layers.
}

TEST_F(ProximityEngineCcdTest, ParticipantIds) {
  const std::vector<GeometryId> ids = engine_.GetCcdParticipantGeometryIds();
  ASSERT_EQ(ids.size(), 2u);
  // Sorted order.
  EXPECT_LT(ids[0], ids[1]);
  EXPECT_TRUE((ids[0] == blade_id_ && ids[1] == obstacle_id_) ||
              (ids[0] == obstacle_id_ && ids[1] == blade_id_));
}

TEST_F(ProximityEngineCcdTest, SmallRotationIsFeasible) {
  // A small spin that keeps the blade clear of the obstacle: feasible with
  // or without subdivision. (N.B. the rotated square's reach along +y at
  // x = 0 grows as apothem / cos(β) = 0.1 / cos(0.1) ≈ 0.1005, still clear
  // of the obstacle's lower edge at 0.102; by β = 0.2 it would graze it.)
  const auto [X_prev, X_next] = MakeSpinStep(0.1);
  EXPECT_TRUE(engine_.IsFeasibleTrajectory(X_prev, X_next));
  EXPECT_EQ(engine_.FeasibilityTimeOfImpact(X_prev, X_next), kInf);
}

TEST_F(ProximityEngineCcdTest, LargeRotationRequiresSubdivision) {
  // A 3 rad spin sweeps the blade's corners through the obstacle (the first
  // corner arc crosses the x = 0 plane at spin angle π/4, i.e. t ≈ 0.26, at
  // radius 0.1414 — well inside the obstacle's span), but the corners'
  // *linear chords* contract toward the blade's center and reach only
  // y ≈ 0.013 at x = 0 — the geometric S1 gap. With the default π/2
  // subdivision (N = 2), the first sub-chord crosses x = 0 at y ≈ 0.1036,
  // inside the obstacle's [0.102, 0.202] span: caught. (Chords of a
  // ψ-radian sub-arc under-sweep the radius by a factor cos(ψ/2); the
  // obstacle placement gives both verdicts >1e-3 geometric margin.)
  const auto [X_prev, X_next] = MakeSpinStep(3.0);

  // Unsubdivided (max_substep_rotation > θ forces N = 1): missed collision.
  EXPECT_TRUE(engine_.IsFeasibleTrajectory(X_prev, X_next,
                                           /* max_substep_rotation = */ 10.0));
  EXPECT_EQ(engine_.FeasibilityTimeOfImpact(X_prev, X_next, 10.0), kInf);

  // Default subdivision (π/2 → N = 2): the collision is caught, at a time
  // of impact consistent with the true first contact at t ≈ 0.26.
  EXPECT_FALSE(engine_.IsFeasibleTrajectory(X_prev, X_next));
  const double toi = engine_.FeasibilityTimeOfImpact(X_prev, X_next);
  EXPECT_GT(toi, 0.15);
  EXPECT_LT(toi, 0.40);
}

TEST_F(ProximityEngineCcdTest, RepeatedQueriesAreIdempotent) {
  const auto [X_prev, X_next] = MakeSpinStep(3.0);
  const double toi_1 = engine_.FeasibilityTimeOfImpact(X_prev, X_next);
  const double toi_2 = engine_.FeasibilityTimeOfImpact(X_prev, X_next);
  EXPECT_EQ(toi_1, toi_2);
  // Interleave with a differently-shaped query (N = 1) and repeat.
  const auto [X_prev_small, X_next_small] = MakeSpinStep(0.1);
  EXPECT_TRUE(engine_.IsFeasibleTrajectory(X_prev_small, X_next_small));
  EXPECT_EQ(engine_.FeasibilityTimeOfImpact(X_prev, X_next), toi_1);
}

TEST_F(ProximityEngineCcdTest, TranslationalImpactTimeIsAccurate) {
  // Pure translation (no subdivision): the blade starts clear of the
  // obstacle along +x and translates through it. The blade's leading edge
  // (0.1 m ahead of its center, which travels x = 0.2 - 0.4t) reaches the
  // obstacle plane x = 0 at t = 0.25 exactly.
  PoseMap X_prev, X_next;
  X_prev[blade_id_] = RigidTransformd(Vector3d(0.2, 0.152, 0));
  X_next[blade_id_] = RigidTransformd(Vector3d(-0.2, 0.152, 0));
  X_prev[obstacle_id_] = X_WO_;
  X_next[obstacle_id_] = X_WO_;
  EXPECT_FALSE(engine_.IsFeasibleTrajectory(X_prev, X_next));
  const double toi = engine_.FeasibilityTimeOfImpact(X_prev, X_next);
  EXPECT_NEAR(toi, 0.25, 1e-6);

  // Translating well away from the obstacle is feasible.
  PoseMap X_prev_clear = X_prev, X_next_clear = X_next;
  X_prev_clear[blade_id_] = RigidTransformd(Vector3d(0.2, 0.152, 1.0));
  X_next_clear[blade_id_] = RigidTransformd(Vector3d(-0.2, 0.152, 1.0));
  EXPECT_TRUE(engine_.IsFeasibleTrajectory(X_prev_clear, X_next_clear));
  EXPECT_EQ(engine_.FeasibilityTimeOfImpact(X_prev_clear, X_next_clear), kInf);
}

TEST_F(ProximityEngineCcdTest, InvalidSubstepRotationThrows) {
  const auto [X_prev, X_next] = MakeSpinStep(0.1);
  EXPECT_THROW(engine_.IsFeasibleTrajectory(X_prev, X_next, 0.0),
               std::exception);
  EXPECT_THROW(engine_.FeasibilityTimeOfImpact(X_prev, X_next, -1.0),
               std::exception);
}

GTEST_TEST(ProximityEngineCcdEmptyTest, NoParticipantsIsFeasible) {
  ProximityEngine<double> engine;
  const PoseMap empty;
  EXPECT_TRUE(engine.GetCcdParticipantGeometryIds().empty());
  EXPECT_TRUE(engine.IsFeasibleTrajectory(empty, empty));
  EXPECT_EQ(engine.FeasibilityTimeOfImpact(empty, empty), kInf);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
