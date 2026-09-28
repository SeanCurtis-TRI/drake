/* Kinematic study of the linear-CCD approximation under rotation (Demo 5).

CENIC's barrier/CCD feasibility check assumes each mesh vertex travels in a
straight line between the poses at the step endpoints. Under large rotations
(θ = ω·dt) the true screw motion deviates from these chords, so the linear
check can miss collisions ("rotation gap", see
multibody/cenic/barrier_cenic_thin_objects.md, item S1).

Scene: a thin square "blade" (codimensional mesh, 0.2 x 0.2 m) rotating about
the world z-axis, and a small thin "obstacle" plate standing inside the swept
annulus of the blade's corners. For each (ω, dt) grid point the blade rotates
by θ = ω·dt between the two query endpoints. We compare:

 - linear:  one CCD feasibility query between the endpoints (what CENIC does).
 - curved:  the same query subdivided into N rotation substeps (ground truth
            for N large, since the per-substep rotation → 0).

A "false negative" is a pair where the linear query reports feasible but the
subdivided query detects a collision. We also time the query as a function of
the substep count N, giving an empirical cost model for a
conservative-subdivision curved CCD.

Outputs two TSV files:
  --grid_output: omega, dt, theta, linear_feasible, linear_toi,
                 curved_feasible, curved_toi, false_negative, t_linear_us
  --cost_output: theta, N, feasible, t_us  (query cost vs substep count)
*/

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <gflags/gflags.h>

#include "drake/common/eigen_types.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsing/package_map.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

DEFINE_string(grid_output, "ccd_grid.tsv",
              "Output TSV for the (omega, dt) false-negative grid.");
DEFINE_string(cost_output, "ccd_cost.tsv",
              "Output TSV for the query-cost-vs-substeps ladder.");
DEFINE_int32(grid_size, 24, "Grid resolution per axis.");
DEFINE_int32(truth_substeps, 64,
             "Substep count used as the curved-motion ground truth.");
DEFINE_double(omega_min, 0.5, "Minimum angular velocity [rad/s].");
DEFINE_double(omega_max, 128.0, "Maximum angular velocity [rad/s].");
DEFINE_double(dt_min, 1e-3, "Minimum step size [s].");
DEFINE_double(dt_max, 0.5, "Maximum step size [s].");

namespace drake {
namespace examples {
namespace {

using geometry::GeometryId;
using geometry::Mesh;
using geometry::QueryObject;
using geometry::SceneGraph;
using geometry::SceneGraphConfig;
using math::RigidTransformd;
using math::RotationMatrixd;
using multibody::AddMultibodyPlantSceneGraph;
using multibody::CoulombFriction;
using multibody::MultibodyPlant;
using multibody::PackageMap;
using multibody::RigidBody;
using multibody::SpatialInertia;
using systems::Context;
using systems::DiagramBuilder;

using PoseMap = std::unordered_map<GeometryId, math::RigidTransform<double>>;
using steady_clock = std::chrono::steady_clock;

struct Scene {
  std::unique_ptr<systems::Diagram<double>> diagram;
  MultibodyPlant<double>* plant{};
  const RigidBody<double>* blade{};
  std::unique_ptr<Context<double>> context;
  Context<double>* plant_context{};
};

Scene MakeScene() {
  Scene scene;
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);

  const std::string square_obj = PackageMap().ResolveUrl(
      "package://drake/examples/integrators/square.obj");

  // Blade: free thin square plate, 0.2 x 0.2 m in the z = 0 plane of its
  // body frame. Corner radius = 0.1 * sqrt(2) ≈ 0.1414 m.
  scene.blade = &plant.AddRigidBody(
      "blade", SpatialInertia<double>::SolidBoxWithMass(0.05, 0.2, 0.2, 1e-3));
  plant.RegisterCollisionGeometry(*scene.blade, RigidTransformd(),
                                  Mesh(square_obj, 0.1), "blade_collision",
                                  CoulombFriction<double>(0.5, 0.5));

  // Obstacle: small thin square (0.1 x 0.1 m) anchored to the world, standing
  // in the y-z plane (normal along x), centered at (0, 0.13, 0). Its z = 0
  // cross-section spans y in [0.08, 0.18], overlapping the blade corners'
  // swept annulus r in [0.1, 0.1414] but clear of the blade at angle 0.
  const RigidTransformd X_WO(RotationMatrixd::MakeYRotation(M_PI / 2),
                             Eigen::Vector3d(0.0, 0.13, 0.0));
  plant.RegisterCollisionGeometry(plant.world_body(), X_WO,
                                  Mesh(square_obj, 0.05), "obstacle_collision",
                                  CoulombFriction<double>(0.5, 0.5));
  plant.Finalize();

  // Thin-object barrier model on all geometries.
  SceneGraphConfig sg_config;
  sg_config.default_proximity_properties.compliance_type = "compliant";
  sg_config.default_proximity_properties.hydroelastic_modulus = 1e8;
  sg_config.default_proximity_properties.margin = 1e-4;
  sg_config.default_proximity_properties.barrier = 1e-4;
  scene_graph.set_config(sg_config);

  scene.plant = &plant;
  scene.diagram = builder.Build();
  scene.context = scene.diagram->CreateDefaultContext();
  scene.plant_context = &scene.plant->GetMyMutableContextFromRoot(
      scene.context.get());
  return scene;
}

/* Returns all geometry poses with the blade rotated by `angle` about the
world z-axis (blade center at the world origin). */
PoseMap PosesAtAngle(const Scene& scene, double angle) {
  scene.plant->SetFreeBodyPose(
      scene.plant_context, *scene.blade,
      RigidTransformd(RotationMatrixd::MakeZRotation(angle),
                      Eigen::Vector3d::Zero()));
  const auto& query_object =
      scene.plant->get_geometry_query_input_port().Eval<QueryObject<double>>(
          *scene.plant_context);
  return query_object.GetAllPosesInWorld();
}

const QueryObject<double>& GetQueryObject(const Scene& scene) {
  return scene.plant->get_geometry_query_input_port().Eval<QueryObject<double>>(
      *scene.plant_context);
}

struct SubdividedResult {
  bool feasible{true};
  // Fraction of [0, theta] completed before first impact (1.0 if feasible).
  double toi{1.0};
};

/* Runs the linear CCD feasibility query on N substeps of the rotation from 0
to theta. Ground truth for large N. */
SubdividedResult SubdividedFeasibility(const Scene& scene, double theta,
                                       int num_substeps) {
  SubdividedResult result;
  PoseMap X_prev = PosesAtAngle(scene, 0.0);
  for (int k = 0; k < num_substeps; ++k) {
    const double angle_next = theta * (k + 1) / num_substeps;
    PoseMap X_next = PosesAtAngle(scene, angle_next);
    const QueryObject<double>& qo = GetQueryObject(scene);
    const double toi = qo.FeasibilityTimeOfImpact(X_prev, X_next);
    if (toi <= 1.0) {
      result.feasible = false;
      result.toi = (k + std::max(0.0, toi)) / num_substeps;
      return result;
    }
    X_prev = std::move(X_next);
  }
  return result;
}

int do_main() {
  Scene scene = MakeScene();

  // Log-spaced grids.
  const int n = FLAGS_grid_size;
  auto logspace = [n](double lo, double hi, int i) {
    return lo * std::pow(hi / lo, static_cast<double>(i) / (n - 1));
  };

  std::ofstream grid_ofs(FLAGS_grid_output);
  grid_ofs << "omega\tdt\ttheta\tlinear_feasible\tlinear_toi\t"
              "curved_feasible\tcurved_toi\tfalse_negative\tt_linear_us\n";

  int num_false_negatives = 0;
  for (int i = 0; i < n; ++i) {
    const double omega = logspace(FLAGS_omega_min, FLAGS_omega_max, i);
    for (int j = 0; j < n; ++j) {
      const double dt = logspace(FLAGS_dt_min, FLAGS_dt_max, j);
      const double theta = omega * dt;

      // Linear query (what CENIC does), timed.
      PoseMap X_prev = PosesAtAngle(scene, 0.0);
      PoseMap X_next = PosesAtAngle(scene, theta);
      const QueryObject<double>& qo = GetQueryObject(scene);
      const auto t_start = steady_clock::now();
      const double linear_toi = qo.FeasibilityTimeOfImpact(X_prev, X_next);
      const double t_linear_us =
          std::chrono::duration<double, std::micro>(steady_clock::now() -
                                                    t_start)
              .count();
      const bool linear_feasible = linear_toi > 1.0;

      // Subdivided ground truth.
      const SubdividedResult curved =
          SubdividedFeasibility(scene, theta, FLAGS_truth_substeps);

      const bool false_negative = linear_feasible && !curved.feasible;
      num_false_negatives += false_negative;

      grid_ofs << fmt::format(
          "{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\n", omega, dt, theta,
          linear_feasible ? 1 : 0, std::min(linear_toi, 1.0), // clamp inf
          curved.feasible ? 1 : 0, curved.toi, false_negative ? 1 : 0,
          t_linear_us);
    }
  }
  grid_ofs.close();
  fmt::print("Grid written to {} ({} false negatives / {} cells)\n",
             FLAGS_grid_output, num_false_negatives, n * n);

  // Cost ladder: query cost vs substep count, for a set of representative
  // rotation magnitudes (including collision-rich ones).
  std::ofstream cost_ofs(FLAGS_cost_output);
  cost_ofs << "theta\tN\tfeasible\tt_us\n";
  const std::vector<double> thetas = {0.1, 0.5, 1.0, M_PI / 2, M_PI,
                                      3 * M_PI / 2};
  const std::vector<int> substep_counts = {1, 2, 4, 8, 16, 32, 64};
  constexpr int kRepeats = 5;
  for (const double theta : thetas) {
    for (const int N : substep_counts) {
      double best_us = std::numeric_limits<double>::infinity();
      bool feasible = true;
      for (int rep = 0; rep < kRepeats; ++rep) {
        const auto t_start = steady_clock::now();
        const SubdividedResult r = SubdividedFeasibility(scene, theta, N);
        const double t_us = std::chrono::duration<double, std::micro>(
                                steady_clock::now() - t_start)
                                .count();
        best_us = std::min(best_us, t_us);
        feasible = r.feasible;
      }
      cost_ofs << fmt::format("{}\t{}\t{}\t{}\n", theta, N, feasible ? 1 : 0,
                              best_us);
    }
  }
  cost_ofs.close();
  fmt::print("Cost ladder written to {}\n", FLAGS_cost_output);

  return 0;
}

}  // namespace
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "Kinematic study of the linear-CCD rotation gap for thin objects.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::do_main();
}
