/** @file
 Test scenario to evaluate the cost of collision between Convex geometries.

 This creates a grid of unit spheres. The unit sphere can be either an idealized
 unit sphere or a discrete representation of the unit sphere (to varying
 degrees of refinment). The spheres are oriented randomly and spin around their
 z-axes with random speeds. Each unit sphere is in contact with the spheres
 adjancet to it in the grid along the x-, y-, and z-axes. A simulation is run
 for a fixed time period and metrics are collected:

   - Time spent advancing the time.
   - Total calls to the collision query.
   - Total number of collisions found.

 This scenario is created in such a way so as to focus as much of the runtime
 as possible on just the collision queries. For convenience, the scenario can
 be visualized (including the reported contacts). For clear characterization of
 the collision cost, it should be profiled with visualization disabled.

 The methodology for profiling is as follows. For a fixed number of spheres in
 the grid, we vary the representation. We want to see how the cost of collision
 queries change as the convex shape complexity (number of vertices) changes.
 The various sphere representations are given by the --model_index command-line
 parameter. Index 0 represents the ideal sphere. Indices 1 through 7 (inclusive)
 are a sequence of discrete sphere approximations in order of _increasing_
 coarseness.

   - Executing with index 0 provides the idealized baseline -- a discrete
     approximation of the sphere will never be as fast as the sphere itself.
     Convex shapes pass through the general convexity algorithm with the
     attendant overhead.
   - We examine how the per-collision cost decreases as complexity (vertex
     count) decreases.

 All contacts are between two Convex shapes (i.e., for any contact, we know
 we're playing 2X the cost of a single Convex geometry in contact).
 */
#include <chrono>
#include <memory>
#include <random>
#include <vector>

#include <fmt/format.h>
#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/profiling/contact_result_maker.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/math/random_rotation.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/systems/analysis/explicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

namespace drake {
namespace geometry {
namespace profiling {
namespace convex {

using Eigen::AngleAxisd;
using Eigen::Vector3d;
using Eigen::Vector4d;
using geometry::AddContactMaterial;
using geometry::Convex;
using geometry::FrameId;
using geometry::FramePoseVector;
using geometry::GeometryFrame;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::IllustrationProperties;
using geometry::ProximityProperties;
using geometry::SceneGraph;
using lcm::DrakeLcm;
using math::RigidTransformd;
using math::RotationMatrixd;
using std::make_unique;
using std::vector;
using systems::Context;
using systems::DiagramBuilder;
using systems::ExplicitEulerIntegrator;
using systems::LeafSystem;
using systems::OutputPort;
using systems::lcm::LcmPublisherSystem;

DEFINE_double(simulation_time, 10.0,
              "Desired duration of the simulatio in seconds.");
// TODO(SeanCurtis-TRI): Make sure I have models to support this flag.
DEFINE_int32(
    convex_model, 0,
    "The index of the convex model to use. Must be in the range [0, 7]. Zero "
    "is an ideal unit sphere, increasing index values map to coarser and "
    "coarser approximations of the sphere");
DEFINE_double(realtime_rate, 0,
              "The fraction of realtime to run; defaults to 0");
// TODO(SeanCurtis-TRI): Make sure this is resolved properly.
DEFINE_int32(grid_size, 2,
             "The number of models along each axis of the rectangular grid. "
             "There will be grid_size ** 3 models.");
DEFINE_double(contact_period, 1.0 / 60,
              "The period (in seconds) at which contact is evaluated; defaults "
              "to 1/60s (60 Hz)");
DEFINE_bool(visualization, false, "Visualizes geometry and contact if true");
DEFINE_double(collision_period, 1e-3,
              "The period between collision query evaluations");
DEFINE_int32(seed, -1,
             "An optional seed for the random number generator. Used to define "
             "geometry orientaitons and angular velocities; non-ositive values "
             "will trigger use of the default seed");

/** Defines a matrix of rotation convex geometries and provides changing poses
  for those geometries w.r.t. time.  */
class ConvexMatrixSystem : public LeafSystem<double> {
 public:
  ConvexMatrixSystem(int model_index, int grid_size,
                     SceneGraph<double>* scene_graph) {
    source_id_ = scene_graph->RegisterSource("ConvexLattice");
    RegisterGeometries(model_index, grid_size, scene_graph);

    geometry_pose_output_port_ = &this->DeclareAbstractOutputPort(
        "geometry_pose", &ConvexMatrixSystem::CalcFramePoseOutput);
  }

  const OutputPort<double>& get_geometry_pose_output_port() const {
    return *geometry_pose_output_port_;
  }

  SourceId source_id() const { return source_id_; }

 private:
  /* Registers all of the geometries based on the requested count. */
  void RegisterGeometries(int model_index, int grid_size,
                          SceneGraph<double>* scene_graph) {
    int seed = FLAGS_seed;
    if (seed < 0) {
      seed = std::chrono::system_clock::now().time_since_epoch().count();
    }
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<double> uniform(-M_PI, M_PI);

    ProximityProperties prox_props;
    AddContactMaterial(1e8, {}, {}, &prox_props);
    IllustrationProperties illus_props;
    illus_props.AddProperty("phong", "diffuse", Vector4d(0.8, 0.1, 0.1, 0.25));
    // const Convex convex = ModelByIndex(model_index);
    auto make_shape = ModelFactory(model_index);
    int f = 0;
    for (const Vector3d& p_WF : CalcGeometryPositions(grid_size)) {
      FrameId f_id = scene_graph->RegisterFrame(
          source_id_, GeometryFrame(fmt::format("frame{}", f)));
      GeometryId g_id = scene_graph->RegisterGeometry(
          source_id_, f_id,
          make_unique<GeometryInstance>(RigidTransformd(), make_shape(),
                                        fmt::format("convex{}", f)));
      scene_graph->AssignRole(source_id_, g_id, prox_props);
      scene_graph->AssignRole(source_id_, g_id, illus_props);
      const RotationMatrixd R_WF0 =
          math::UniformlyRandomRotationMatrix(&generator);
      const Vector3d& w_axis_W = R_WF0.col(2);
      const double w_speed = uniform(generator);

      const AngleAxisd omega{w_speed, w_axis_W};
      RigidTransformd X_WF0{R_WF0, p_WF};
      frames_.emplace_back(Frame{f_id, omega, X_WF0});
      ++f;
    }
  }

  /* Creates the positions of the vertices on a rectangular lattice with
   `grid_size` points along each axis. The lattice is centered on the world
   origin. There will be grid_size ** 3 total positions.  The space between
   adjacent lattice vertices is 1.8 (adjacent in the axis directions and not
   diagonally). This distance is sufficient for two unit spheres to be
   intersecting.  */
  vector<Vector3d> CalcGeometryPositions(int grid_size) {
    DRAKE_DEMAND(grid_size >= 1);
    constexpr double distance = 1.8;
    const double lattice_size = (grid_size - 1) * distance;
    const Vector3d min_corner = Vector3d::Constant(-lattice_size / 2);
    vector<Vector3d> vertices;
    for (int i = 0; i < grid_size; ++i) {
      for (int j = 0; j < grid_size; ++j) {
        for (int k = 0; k < grid_size; ++k) {
          vertices.emplace_back(min_corner + Vector3d{i, j, k} * distance);
        }
      }
    }
    return vertices;
  }

  /* Returns a function that will create a shape based on the given
   `model_index`.
     0: Perfect sphere.
     1 - 7: coarse to fine approximations of spheres.
     The result should converge to the perfect sphere as the value increases.
  */
  std::function<std::unique_ptr<Shape>()> ModelFactory(int model_index) {
    std::string file_name;
    switch (model_index) {
      case 0:
        return []() { return make_unique<Sphere>(1.0); };
      case 1:
        file_name = "sphere5.obj";
        break;
      case 2:
        file_name = "sphere4.obj";
        break;
      case 3:
        file_name = "sphere3.obj";
        break;
      case 4:
        file_name = "sphere2.obj";
        break;
      case 5:
        file_name = "sphere1.obj";
        break;
      case 6:
        // TODO(SeanCurtis-TRI): Consider swapping cube and tet for a single
        //  octahedron. The cube extends beyond the unit sphere, and the tet
        //  is *far* too small. However, it does represent a test with the
        //  absolute minimum number of vertices. Alternatively, make it bigger
        //  so I get a comparable number of contacts reported as with the
        //  spheres.
        file_name = "cube.obj";
        break;
      case 7:
        file_name = "tet.obj";
        break;
      default:
        throw std::runtime_error("Model index should be in the range [0, 7]");
    }
    const std::string file_path =
        FindResourceOrThrow("drake/geometry/profiling/meshes/" + file_name);
    return [file_path]() { return make_unique<Convex>(file_path, 1.0); };
  }

  /* Calculates the output poses of all registered geometry frames. */
  void CalcFramePoseOutput(const Context<double>& context,
                           FramePoseVector<double>* poses) const {
    const double t = context.get_time();
    poses->clear();
    for (const Frame& frame : frames_) {
      const RigidTransformd X_F0Fi(RotationMatrixd(
          AngleAxisd(frame.omega.angle() * t, frame.omega.axis())));
      const RigidTransformd X_WFi = frame.X_WF0 * X_F0Fi;
      poses->set_value(frame.id, X_WFi);
    }
  }

  SourceId source_id_{};
  struct Frame {
    // The id of the frame.
    FrameId id;
    // The constant angular velocity of the frame.
    AngleAxisd omega;
    // The initial pose of the frame.
    RigidTransformd X_WF0;
  };
  vector<Frame> frames_;
  const OutputPort<double>* geometry_pose_output_port_{};
};

int do_main() {
  /*
   1. Instantiate SceneGraph.
   2. Instantiate geometries based on the parameters.
   3. Add visualizer
   4. Add contact puller (visualizer?)
   5. Launch simulation.
   */
  DiagramBuilder<double> builder;

  auto& scene_graph = *builder.AddSystem<SceneGraph<double>>();
  auto& convex_matrix = *builder.AddSystem<ConvexMatrixSystem>(
      FLAGS_convex_model, FLAGS_grid_size, &scene_graph);
  builder.Connect(convex_matrix.get_geometry_pose_output_port(),
                  scene_graph.get_source_pose_port(convex_matrix.source_id()));

  ContactResultMaker* contact_results{nullptr};
  // For simplicitly; construct DrakeLcm whether we visualize or not.
  DrakeLcm lcm;
  if (FLAGS_visualization) {
    std::cout << "Visualization enabled\n";
    contact_results = builder.AddSystem<ContactResultMaker>(
        ContactResultMaker::kPointContact);

    // Visualize geometry.
    ConnectDrakeVisualizer(&builder, scene_graph, &lcm);

    // Visualize contacts.
    auto& contact_to_lcm = *builder.AddSystem(
        LcmPublisherSystem::Make<lcmt_contact_results_for_viz>(
            "CONTACT_RESULTS", &lcm, FLAGS_collision_period));
    builder.Connect(*contact_results, contact_to_lcm);
  } else {
    std::cout << "Visualization disabled\n";
    contact_results = builder.AddSystem<ContactResultMaker>(
        FLAGS_collision_period, ContactResultMaker::kPointContact);
  }
  builder.Connect(scene_graph.get_query_output_port(),
                  contact_results->get_geometry_query_port());

  auto diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);
  // We set the maximum step size as large as the simulation time so that the
  // steps will be dominated by the contact events.
  simulator.reset_integrator<ExplicitEulerIntegrator<double>>(
      FLAGS_simulation_time);
  simulator.set_target_realtime_rate(FLAGS_realtime_rate);
  // simulator.Initialize();
  using clock = std::chrono::steady_clock;
  const clock::time_point start = clock::now();
  simulator.AdvanceTo(FLAGS_simulation_time);
  const clock::time_point end = clock::now();
  const double wall_clock_time =
      std::chrono::duration<double>(end - start).count();
  std::cout << "Stats for Simulator::AdvanceTo()\n";
  std::cout << "  Model index:           " << FLAGS_convex_model << "\n";
  std::cout << "  Simulation time (s):   " << FLAGS_simulation_time << "\n";
  std::cout << "  Wall clocktime (s):    " << wall_clock_time << "\n";
  std::cout << "  Simulation steps:      " << simulator.get_num_steps_taken()
            << "\n";
  const auto& stats = contact_results->point_pair_stats();
  std::cout << "  Collision evaluations: " << stats.evaluations << "\n";
  std::cout << "  Total contacts:        " << stats.total_contacts << "\n";
  return 0;
}

}  // namespace convex
}  // namespace profiling
}  // namespace geometry
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::geometry::profiling::convex::do_main();
}
