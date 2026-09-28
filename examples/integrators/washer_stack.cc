/* P1 washer stack (PAPER_NOTES §5): N thin coaxial washers dropped onto a
 thin table plate. Near-coplanar barrier contact everywhere — the
 conditioning-brutal case for the barycentric dual quadrature of the
 mesh-barrier formulation. Success = a stable stack; the interesting trend is
 solver conditioning (iterations, Hessian factorizations) and CCD activity as
 N grows.

 Writes one TSV row per invocation (with a header) to --output, or stdout.

 Example:
   bazel run //examples/integrators:washer_stack -- --num_washers=8 \
     --sim_time=1.0 --accuracy=1e-2
*/

#include <chrono>
#include <cstdio>
#include <string>
#include <variant>
#include <vector>

#include <fmt/format.h>
#include <gflags/gflags.h>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/cenic/cenic_integrator.h"
#include "drake/multibody/parsing/package_map.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

DEFINE_int32(num_washers, 5, "Number of washers in the stack.");
DEFINE_double(sim_time, 1.0, "Simulated time [s].");
DEFINE_double(accuracy, 1e-2, "Integrator target accuracy.");
DEFINE_double(max_dt, 0.01, "Maximum step size [s].");
DEFINE_bool(use_toi, false, "Use CCD time-of-impact for step selection.");
DEFINE_double(modulus, 1e8, "Hydroelastic modulus [Pa].");
DEFINE_double(margin, 1e-4, "Barrier margin [m].");
DEFINE_double(barrier, 1e-4, "Barrier thickness [m].");
DEFINE_string(output, "", "TSV output path ('' = stdout).");

namespace drake {
namespace examples {
namespace {

using geometry::SceneGraphConfig;
using math::RigidTransformd;
using multibody::AddMultibodyPlantSceneGraph;
using multibody::CenicIntegrator;
using multibody::CoulombFriction;
using multibody::PackageMap;
using multibody::RigidBody;
using multibody::SpatialInertia;
using systems::Context;
using systems::DiagramBuilder;
using systems::Simulator;

/* Washer geometry: washer.obj is an annulus with outer radius 1, inner
 radius 0.5, in the z = 0 plane; kScale makes it a 6 cm-diameter washer. */
constexpr double kScale = 0.03;
constexpr double kOuterRadius = 1.0 * kScale;
constexpr double kMass = 0.02;

double GetStat(const std::vector<systems::NamedStatistic>& stats,
               const std::string& name) {
  for (const auto& [key, value] : stats) {
    if (key == name) {
      if (std::holds_alternative<int64_t>(value)) {
        return static_cast<double>(std::get<int64_t>(value));
      }
      return std::get<double>(value);
    }
  }
  return std::numeric_limits<double>::quiet_NaN();
}

int DoMain() {
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);

  const std::string washer_obj = PackageMap().ResolveUrl(
      "package://drake/examples/integrators/washer.obj");
  const std::string square_obj = PackageMap().ResolveUrl(
      "package://drake/examples/integrators/square.obj");

  // Table: 1 x 1 m thin plate at z = 0, anchored to the world.
  plant.RegisterCollisionGeometry(
      plant.world_body(), RigidTransformd(), geometry::Mesh(square_obj, 0.5),
      "table_collision", CoulombFriction<double>(0.5, 0.5));

  std::vector<const RigidBody<double>*> washers;
  for (int i = 0; i < FLAGS_num_washers; ++i) {
    const auto& washer = plant.AddRigidBody(
        fmt::format("washer_{}", i),
        SpatialInertia<double>::SolidCylinderWithMass(
            kMass, kOuterRadius, 1e-3, Eigen::Vector3d::UnitZ()));
    plant.RegisterCollisionGeometry(washer, RigidTransformd(),
                                    geometry::Mesh(washer_obj, kScale),
                                    fmt::format("washer_{}_collision", i),
                                    CoulombFriction<double>(0.5, 0.5));
    washers.push_back(&washer);
  }
  plant.Finalize();

  // Thin-object barrier model on all geometries.
  SceneGraphConfig sg_config;
  sg_config.default_proximity_properties.compliance_type = "compliant";
  sg_config.default_proximity_properties.hydroelastic_modulus = FLAGS_modulus;
  sg_config.default_proximity_properties.margin = FLAGS_margin;
  sg_config.default_proximity_properties.barrier = FLAGS_barrier;
  scene_graph.set_config(sg_config);

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();
  Context<double>& plant_context =
      plant.GetMyMutableContextFromRoot(context.get());

  // Drop heights: staggered slightly above the table so the washers settle
  // into a coaxial near-coplanar stack.
  for (int i = 0; i < FLAGS_num_washers; ++i) {
    plant.SetFreeBodyPose(
        &plant_context, *washers[i],
        RigidTransformd(Eigen::Vector3d(0, 0, 0.002 + 0.004 * i)));
  }

  Simulator<double> simulator(*diagram, std::move(context));
  auto& integrator = simulator.reset_integrator<CenicIntegrator<double>>();
  integrator.set_maximum_step_size(FLAGS_max_dt);
  integrator.set_fixed_step_mode(false);
  integrator.set_target_accuracy(FLAGS_accuracy);
  auto params = integrator.get_solver_parameters();
  params.use_toi = FLAGS_use_toi;
  integrator.SetSolverParameters(params);
  simulator.Initialize();

  const auto t_start = std::chrono::steady_clock::now();
  simulator.AdvanceTo(FLAGS_sim_time);
  const double wall_s =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start)
          .count();

  // Stack outcome: every washer coaxial (small xy offset) and settled low.
  const Context<double>& plant_context_final =
      plant.GetMyContextFromRoot(simulator.get_context());
  double top_z = 0;
  bool stable = true;
  for (int i = 0; i < FLAGS_num_washers; ++i) {
    const auto& X_WW =
        plant.EvalBodyPoseInWorld(plant_context_final, *washers[i]);
    const Eigen::Vector3d& p = X_WW.translation();
    if (!p.allFinite() || p.head<2>().norm() > 0.5 * kOuterRadius ||
        p.z() > 0.05) {
      stable = false;
    }
    top_z = std::max(top_z, p.z());
  }

  const auto stats = integrator.GetStatisticsSummary();
  const std::string header =
      "num_washers\taccuracy\tuse_toi\tsteps\tsolver_iterations\t"
      "hessian_factorizations\tccd_rejections_full\tccd_rejections_half1\t"
      "ccd_rejections_half2\twall_s\ttop_z\tstable\n";
  const std::string row = fmt::format(
      "{}\t{:g}\t{}\t{:.0f}\t{:.0f}\t{:.0f}\t{:.0f}\t{:.0f}\t{:.0f}\t{:.3f}\t"
      "{:.5f}\t{}\n",
      FLAGS_num_washers, FLAGS_accuracy, FLAGS_use_toi,
      GetStat(stats, "integrator_num_steps_taken"),
      GetStat(stats, "cenic_total_solver_iterations"),
      GetStat(stats, "cenic_total_hessian_factorizations"),
      GetStat(stats, "cenic_num_feasibility_rejections_full"),
      GetStat(stats, "cenic_num_feasibility_rejections_half1"),
      GetStat(stats, "cenic_num_feasibility_rejections_half2"), wall_s, top_z,
      stable ? 1 : 0);

  if (FLAGS_output.empty()) {
    fmt::print("{}{}", header, row);
  } else {
    std::FILE* f = std::fopen(FLAGS_output.c_str(), "w");
    DRAKE_THROW_UNLESS(f != nullptr);
    fmt::print(f, "{}{}", header, row);
    std::fclose(f);
  }
  return stable ? 0 : 1;
}

}  // namespace
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "P1 washer stack: N thin coaxial washers settle on a thin table under "
      "the mesh-barrier model (near-coplanar conditioning stress test).");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::DoMain();
}
