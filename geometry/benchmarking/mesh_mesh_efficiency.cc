/** @file
 A simple binary for exercising and visualizing computation of ContactSurfaces.
 This is decoupled from dynamics so that just the geometric components can be
 evaluated in as light-weight a fashion as possible.

 This can serve as a test bed for evaluating the various cases of the
 ContactSurface-computing algorithms. Simply swap the geometry types (moving
 and anchored) and their properties to see the effect on contact surface.  */

#include <memory>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/profiler.h"
#include "drake/common/value.h"
#include "drake/geometry/frame_kinematics_vector.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/shape_specification.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/math/rigid_transform.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

namespace drake {
namespace geometry {
namespace internal {
/** @defgroup mesh_intersection_efficiency Mesh Intersection Efficiency
 @ingrou proximity_queries

 This benchmark attempts to measure the efficiency of the tet-tri intersection.

 It creates a scenario of a ball intersecting a "ground" surface and measures
 the costs of computing the contact surface. The scenario has the following
 features:

   - The rigid ground is located at z = 0 in the world frame.
   - The ground can be represented in one of two ways:
     - a mesh consisting of a single large triangle or
     - an infinite half space.
   - The soft ball penetrates half its radius distance into the ground.
     - the resolution of the soft volume mesh can be varied.
   - The ball moves along a small path
     - the path is fully contained in both half space and triangle
       representations of the ground and
     - never changes height.
   - The patch is *always* the same; always the same tets intersecting the
     ground in exactly the same way -- the patch merely transforms around
     the surface. Therefore, the work of each patch calculation is the same.

  We assess the efficiency of the tetrahedron-triangle intersection efficiency
  by comparing it with the tetrahedron-plane implementation. We know that the
  plane is a _simpler_ version of the triangle. It therefore serves as a
  floor on how well tet-tri intersection can perform.

  <h2>Running with profiling</h2>

  <h3>Building with profiling enabled</h3>

  To build with the profiler enabled, you must _recompile_, e.g.:

  ```
  bazel run --copt -DENABLE_TIMERS geometry/benchmarking:mesh_mesh_efficiency
  ```

  <h3>Interpreting the profiler results</h3>

  The profiler is incorporated into this benchmark. With the profiler enabled,
  it will capture high resolution measurements of several operations:

    - Total simulation time (the call to simulator.AdvanceTo()) -- this excludes
      all diagram building.
      - All calls to QueryObject::ComputeContactSurface()
        - All calls to the tri-tet/plane-tet function.
      - All calls to ContactResultMaker::CalcContactResults()

   The results will be output in a table that looks like this (sorry for the
   poor formatting):

   ```
Time (s)   Samples   Total Time (s)  Label
0.0002131649568405266      6001      1.279202906  Process Contact Surface
0.00020712580319946677      6001      1.242961945  ComputeContactSurfaces
4.3154817466319717e-07   2184364      0.942658297  SliceTetWithPlane
    1.808158485         1      1.808158485  Main Simulation Loop

   ```

   The columns are:

      - Time (s): The average time per invocation of the labeled operation.
      - Samples: The number of times the label was called.
      - Total Time (s): The _total_ time in seconds spent in the labeled
        operation.
      - Label: The label applied to the measured quantity.

   Things to note:

     - The number of invocations of `Process Contact Surface` and
       `ComputeContactSurfaces` should always match.
     - There should always be 1 invocation for `Main Simulation Loop`.
     - Whether `--ground_as_mesh` is `true` or `false`, there should be the same
       number of invocations of the `Tet-Tri intersection` or
       `SliceTetWithPlane` labels, respectively. These are the number of
       tet-feature pairs that were produced by the BVH culling; it should always
       be the same.
     - The primitive intersection operations that are measured have equivalent
       effect. They both intersect a tet with its primitive (tri or plane) and
       evaluate the mesh field on the intersection vertices.
     - Increasing the resolution of the ball mesh (by decreasing the `--length`
       parameter) will increase the cost of the contact surface and the number
       of calls to the primitive intersection test.
 */

using Eigen::Vector3d;
using Eigen::Vector4d;
using lcm::DrakeLcm;
using math::RigidTransformd;
using std::make_unique;
using systems::BasicVector;
using systems::Context;
using systems::ContinuousState;
using systems::DiagramBuilder;
using systems::lcm::LcmPublisherSystem;
using systems::LeafSystem;
using systems::Simulator;

DEFINE_double(simulation_time, 10.0,
              "Desired duration of the simulation in seconds.");
DEFINE_bool(real_time, true, "Set to false to run as fast as possible");
DEFINE_double(length, 1.0,
              "Measure of sphere edge length -- smaller numbers produce a "
              "denser, more expensive mesh");
DEFINE_bool(ground_as_mesh, false,
            "If true, the ground plane is a large triangle that encloses the "
            "sphere's motion, otherwise a half space");

/** Moves a ball on the z = k plane in the world frame W. k is some amount less
 than or equal to the balls radius. This guarantees a consistent contact patch
 with the ground plane (z = 0).

 @system{MovingBall,, @output_port{geometry_pose} }
 */
class MovingBall final : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MovingBall)

  explicit MovingBall(SceneGraph<double>* scene_graph) {
    // Add geometry for a ball that moves based on sinusoidal derivatives.
    source_id_ = scene_graph->RegisterSource("moving_ball");
    frame_id_ =
        scene_graph->RegisterFrame(source_id_, GeometryFrame("moving_frame"));
    geometry_id_ = scene_graph->RegisterGeometry(
        source_id_, frame_id_,
        make_unique<GeometryInstance>(RigidTransformd(),
                                      make_unique<Sphere>(1.0), "ball"));

    ProximityProperties prox_props;
    AddContactMaterial(1e8, {}, {}, &prox_props);
    AddSoftHydroelasticProperties(FLAGS_length, &prox_props);
    scene_graph->AssignRole(source_id_, geometry_id_, prox_props);

    IllustrationProperties illus_props;
    illus_props.AddProperty("phong", "diffuse", Vector4d(0.8, 0.1, 0.1, 0.25));
    scene_graph->AssignRole(source_id_, geometry_id_, illus_props);

    geometry_pose_port_ =
        this->DeclareAbstractOutputPort("geometry_pose",
                                        &MovingBall::CalcFramePoseOutput)
            .get_index();
  }

  SourceId source_id() const { return source_id_; }

  const systems::OutputPort<double>& get_geometry_pose_output_port() const {
    return systems::System<double>::get_output_port(geometry_pose_port_);
  }

 private:
  void CalcFramePoseOutput(
      const Context<double>& context, FramePoseVector<double>* poses) const {
    RigidTransformd pose;
    pose.set_translation({std::cos(context.get_time()), 0.0, 0.5});
    *poses = {{frame_id_, pose}};
  }

  SourceId source_id_;
  FrameId frame_id_;
  GeometryId geometry_id_;

  int geometry_pose_port_{-1};
};

/** A system that evaluates contact surfaces from SceneGraph and outputs a fake
 ContactResults with the actual contact surfaces.

 @system{ContactResultMaker,
   @intput_port{query_object},
   @output_port{contact_result}
 }
 */
class ContactResultMaker final : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ContactResultMaker)

  explicit ContactResultMaker(bool use_strict_hydro = true)
      : use_strict_hydro_{use_strict_hydro} {
    geometry_query_input_port_ =
        this->DeclareAbstractInputPort("query_object",
                                       Value<QueryObject<double>>())
            .get_index();
    contact_result_output_port_ =
        this->DeclareAbstractOutputPort("contact_result",
                                        &ContactResultMaker::CalcContactResults)
            .get_index();
  }

  const systems::InputPort<double>& get_geometry_query_port() const {
    return systems::System<double>::get_input_port(geometry_query_input_port_);
  }

 private:
  void CalcContactResults(const Context<double>& context,
                          lcmt_contact_results_for_viz* results) const {
    static const common::TimerIndex timer = addTimer("Process Contact Surface");
    startTimer(timer);
    const auto& query_object =
        get_geometry_query_port().Eval<QueryObject<double>>(context);
    std::vector<ContactSurface<double>> surfaces;
    std::vector<PenetrationAsPointPair<double>> points;
    if (use_strict_hydro_) {
      surfaces = query_object.ComputeContactSurfaces();
    } else {
      query_object.ComputeContactSurfacesWithFallback(&surfaces, &points);
    }
    const int num_surfaces = static_cast<int>(surfaces.size());
    const int num_pairs = static_cast<int>(points.size());

    auto& msg = *results;
    msg.timestamp = context.get_time() * 1e6;  // express in microseconds.
    msg.num_point_pair_contacts = num_pairs;
    msg.point_pair_contact_info.resize(num_pairs);
    msg.num_hydroelastic_contacts = num_surfaces;
    msg.hydroelastic_contacts.resize(num_surfaces);

    auto write_double3 = [](const Vector3d& src, double* dest) {
      dest[0] = src(0);
      dest[1] = src(1);
      dest[2] = src(2);
    };

    // Contact surfaces.
    for (int i = 0; i < num_surfaces; ++i) {
      lcmt_hydroelastic_contact_surface_for_viz& surface_msg =
          msg.hydroelastic_contacts[i];

      surface_msg.body1_name = "Id_" + to_string(surfaces[i].id_M());
      surface_msg.body2_name = "Id_" + to_string(surfaces[i].id_N());

      const SurfaceMesh<double>& mesh_W = surfaces[i].mesh_W();
      surface_msg.num_triangles = mesh_W.num_faces();
      surface_msg.triangles.resize(surface_msg.num_triangles);

      // Loop through each contact triangle on the contact surface.
      for (SurfaceFaceIndex j(0); j < surface_msg.num_triangles; ++j) {
        lcmt_hydroelastic_contact_surface_tri_for_viz& tri_msg =
            surface_msg.triangles[j];

        // Get the three vertices.
        const auto& face = mesh_W.element(j);
        const SurfaceVertex<double>& vA = mesh_W.vertex(face.vertex(0));
        const SurfaceVertex<double>& vB = mesh_W.vertex(face.vertex(1));
        const SurfaceVertex<double>& vC = mesh_W.vertex(face.vertex(2));

        write_double3(vA.r_MV(), tri_msg.p_WA);
        write_double3(vB.r_MV(), tri_msg.p_WB);
        write_double3(vC.r_MV(), tri_msg.p_WC);

        tri_msg.pressure_A = surfaces[i].EvaluateE_MN(face.vertex(0));
        tri_msg.pressure_B = surfaces[i].EvaluateE_MN(face.vertex(1));
        tri_msg.pressure_C = surfaces[i].EvaluateE_MN(face.vertex(2));
      }
    }

    // Point pairs.
    for (int i = 0; i < num_pairs; ++i) {
      lcmt_point_pair_contact_info_for_viz& info_msg =
          msg.point_pair_contact_info[i];
      info_msg.timestamp = msg.timestamp;
      const PenetrationAsPointPair<double>& pair = points[i];

      info_msg.body1_name = query_object.inspector().GetName(pair.id_A);
      info_msg.body1_name = query_object.inspector().GetName(pair.id_B);

      // Fake contact *force* data from strictly contact data. Contact point
      // is midway between the two contact points and force = normal.
      const Vector3d contact_point = (pair.p_WCa + pair.p_WCb) / 2.0;
      write_double3(contact_point, info_msg.contact_point);
      write_double3(pair.nhat_BA_W, info_msg.contact_force);
      write_double3(pair.nhat_BA_W, info_msg.normal);
    }
    lapTimer(timer);
  }

  int geometry_query_input_port_{-1};
  int contact_result_output_port_{-1};
  const bool use_strict_hydro_{true};
};

int do_main() {
  DiagramBuilder<double> builder;

  auto& scene_graph = *builder.AddSystem<SceneGraph<double>>();

  // Add the bouncing ball.
  auto& moving_ball = *builder.AddSystem<MovingBall>(&scene_graph);
  builder.Connect(moving_ball.get_geometry_pose_output_port(),
                  scene_graph.get_source_pose_port(moving_ball.source_id()));

  // Add a large box, such that intersection occurs at the edge.
  const SourceId source_id = scene_graph.RegisterSource("world");
  GeometryId ground_id;
  const RigidTransformd X_WB;  // identity.
  ProximityProperties rigid_props;
  if (FLAGS_ground_as_mesh) {
    ground_id = scene_graph.RegisterAnchoredGeometry(
        source_id, make_unique<GeometryInstance>(
                       X_WB,
                       make_unique<Mesh>(
                           FindResourceOrThrow(
                               "drake/geometry/benchmarking/big_triangle.obj"),
                           1.0),
                       "ground"));

  } else {
    ground_id = scene_graph.RegisterAnchoredGeometry(
        source_id, make_unique<GeometryInstance>(X_WB, make_unique<HalfSpace>(),
                                                 "ground"));
  }
  AddRigidHydroelasticProperties(&rigid_props);
  scene_graph.AssignRole(source_id, ground_id, rigid_props);
  IllustrationProperties illustration_box;
  illustration_box.AddProperty("phong", "diffuse",
                               Vector4d{0.5, 0.5, 0.45, 0.5});
  scene_graph.AssignRole(source_id, ground_id, illustration_box);

  // Make and visualize contacts.
  auto& contact_results =
      *builder.AddSystem<ContactResultMaker>(true /* use strict hydro */);
  builder.Connect(scene_graph.get_query_output_port(),
                  contact_results.get_geometry_query_port());

  // Now visualize.
  DrakeLcm lcm;

  // Visualize geometry.
  ConnectDrakeVisualizer(&builder, scene_graph, &lcm);

  // Visualize contacts.
  auto& contact_to_lcm =
      *builder.AddSystem(LcmPublisherSystem::Make<lcmt_contact_results_for_viz>(
          "CONTACT_RESULTS", &lcm, 1.0 / 60));
  builder.Connect(contact_results, contact_to_lcm);

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);

  simulator.get_mutable_integrator().set_maximum_step_size(0.002);
  simulator.set_target_realtime_rate(FLAGS_real_time ? 1.f : 0.f);
  simulator.Initialize();
  const common::TimerIndex main_loop = addTimer("Main Simulation Loop");
  startTimer(main_loop);
  simulator.AdvanceTo(FLAGS_simulation_time);
  lapTimer(main_loop);
  std::cout << TableOfAverages() << "\n";
  return 0;
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::geometry::internal::do_main();
}
