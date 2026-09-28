/* P2 sheet-through-slot (PAPER_NOTES §5): a thin sheet slides along a
 prismatic axis through a slot whose per-side clearance is a small multiple
 of (margin + barrier) — stressing sustained tangential sliding of two
 opposing barrier layers. Measures CCD activity, solver effort, and whether
 the sheet traverses the slot.

 (The proposed "max pressure" metric needs per-step contact-surface
 introspection that the ICF barrier layer does not expose yet;
 TODO(joemasterjohn): record it once exposed.)

 Writes one TSV row per invocation (with a header) to --output, or stdout.

 Example:
   bazel run //examples/integrators:sheet_through_slot -- \
     --clearance_factor=2 --speed=0.1 --sim_time=3
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
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

DEFINE_double(clearance_factor, 2.0,
              "Per-side slot clearance, in multiples of (margin + barrier).");
DEFINE_double(speed, 0.1, "Initial sheet speed along +x [m/s].");
DEFINE_double(sim_time, 3.0, "Simulated time [s].");
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
using math::RotationMatrixd;
using multibody::AddMultibodyPlantSceneGraph;
using multibody::CenicIntegrator;
using multibody::CoulombFriction;
using multibody::PackageMap;
using multibody::PrismaticJoint;
using multibody::SpatialInertia;
using systems::Context;
using systems::DiagramBuilder;
using systems::Simulator;

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

  const std::string square_obj = PackageMap().ResolveUrl(
      "package://drake/examples/integrators/square.obj");

  // Slot: two 0.1 x 0.1 m plates in the y-z plane (normal along x, the sheet
  // travel direction), one above and one below the sheet's z = 0 travel
  // plane, each offset by the clearance.
  const double clearance =
      FLAGS_clearance_factor * (FLAGS_margin + FLAGS_barrier);
  const double plate_half = 0.05;
  for (const double sign : {1.0, -1.0}) {
    const RigidTransformd X_WP(
        RotationMatrixd::MakeYRotation(M_PI / 2),
        Eigen::Vector3d(0, 0, sign * (clearance + plate_half)));
    plant.RegisterCollisionGeometry(
        plant.world_body(), X_WP, geometry::Mesh(square_obj, plate_half),
        fmt::format("slot_plate_{}", sign > 0 ? "upper" : "lower"),
        CoulombFriction<double>(0.5, 0.5));
  }

  // Sheet: 0.08 x 0.08 m thin square in the z = 0 plane of its body frame,
  // on a prismatic rail along x.
  const auto& sheet = plant.AddRigidBody(
      "sheet",
      SpatialInertia<double>::SolidBoxWithMass(0.02, 0.08, 0.08, 1e-3));
  plant.RegisterCollisionGeometry(
      sheet, RigidTransformd(), geometry::Mesh(square_obj, 0.04),
      "sheet_collision", CoulombFriction<double>(0.5, 0.5));
  const auto& slide = plant.AddJoint<PrismaticJoint>(
      "slide", plant.world_body(), {}, sheet, {}, Eigen::Vector3d::UnitX());

  // No gravity: the stress is the tangential sliding between the two barrier
  // layers, with the sheet held on the slot's midplane by the rail.
  plant.mutable_gravity_field().set_gravity_vector(Eigen::Vector3d::Zero());
  plant.Finalize();

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

  // Start fully outside the slot, moving toward it.
  const double x0 = -0.15;
  slide.set_translation(&plant_context, x0);
  slide.set_translation_rate(&plant_context, FLAGS_speed);

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

  const Context<double>& plant_context_final =
      plant.GetMyContextFromRoot(simulator.get_context());
  const double x_final = slide.get_translation(plant_context_final);
  // Traversed = the sheet's trailing edge cleared the slot plane.
  const bool traversed = x_final > 0.04;

  const auto stats = integrator.GetStatisticsSummary();
  const std::string header =
      "clearance_factor\tspeed\taccuracy\tuse_toi\tsteps\tsolver_iterations\t"
      "hessian_factorizations\tccd_rejections_full\tccd_rejections_half1\t"
      "ccd_rejections_half2\twall_s\tx_final\ttraversed\n";
  const std::string row = fmt::format(
      "{:g}\t{:g}\t{:g}\t{}\t{:.0f}\t{:.0f}\t{:.0f}\t{:.0f}\t{:.0f}\t{:.0f}\t"
      "{:.3f}\t{:.5f}\t{}\n",
      FLAGS_clearance_factor, FLAGS_speed, FLAGS_accuracy, FLAGS_use_toi,
      GetStat(stats, "integrator_num_steps_taken"),
      GetStat(stats, "cenic_total_solver_iterations"),
      GetStat(stats, "cenic_total_hessian_factorizations"),
      GetStat(stats, "cenic_num_feasibility_rejections_full"),
      GetStat(stats, "cenic_num_feasibility_rejections_half1"),
      GetStat(stats, "cenic_num_feasibility_rejections_half2"), wall_s, x_final,
      traversed ? 1 : 0);

  if (FLAGS_output.empty()) {
    fmt::print("{}{}", header, row);
  } else {
    std::FILE* f = std::fopen(FLAGS_output.c_str(), "w");
    DRAKE_THROW_UNLESS(f != nullptr);
    fmt::print(f, "{}{}", header, row);
    std::fclose(f);
  }
  return traversed ? 0 : 1;
}

}  // namespace
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "P2 sheet-through-slot: a thin sheet slides between two opposing "
      "barrier layers with clearance of a few (margin + barrier).");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::DoMain();
}
