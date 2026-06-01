#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/math/spatial_algebra.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/multibody/tree/unit_inertia.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/visualization/visualization_config_functions.h"

DEFINE_double(target_realtime_rate, 0.25,
              "Desired rate relative to real time.");
DEFINE_double(simulation_time, 8.0, "Desired simulation duration in seconds.");
DEFINE_double(time_step, 0.001,
              "Discrete update period for the MultibodyPlant.");
DEFINE_bool(visualize, true, "Whether to publish visualization geometry.");

DEFINE_int32(num_links, 80,
             "Number of rigid links used to approximate the belt.");
DEFINE_double(drum_spacing, 2.0, "Distance between drum centers, in meters.");
DEFINE_double(drum_radius, 0.16, "Radius of each conveyor drum, in meters.");
DEFINE_double(belt_width, 0.44, "Belt width along the y-axis, in meters.");
DEFINE_double(belt_thickness, 0.035, "Belt link thickness, in meters.");
DEFINE_bool(include_center_support, true,
            "Whether to include the frictionless box between the drums.");
DEFINE_double(link_mass, 0.03, "Mass of each belt link, in kg.");
DEFINE_double(target_belt_speed, 0.5, "Target belt speed in m/s.");
DEFINE_double(belt_speed_kp, 50.0,
              "Proportional gain for the belt speed controller.");
DEFINE_double(max_drive_force, 25.0,
              "Maximum drive force applied to the driven link, in N.");
DEFINE_int32(driven_link, 3,
             "Index of the belt link that receives the body-frame force.");

DEFINE_double(box_size, 0.16, "Edge length of the carried box, in meters.");
DEFINE_double(box_mass, 0.1, "Mass of the carried box, in kg.");

namespace drake {
namespace examples {
namespace conveyor_belt {
namespace {

using drake::geometry::Box;
using drake::geometry::Cylinder;
using drake::math::RigidTransformd;
using drake::math::RollPitchYawd;
using drake::multibody::BodyIndex;
using drake::multibody::CoulombFriction;
using drake::multibody::DiscreteContactApproximation;
using drake::multibody::ExternallyAppliedSpatialForce;
using drake::multibody::MultibodyPlant;
using drake::multibody::RevoluteJoint;
using drake::multibody::RigidBody;
using drake::multibody::SpatialForce;
using drake::multibody::SpatialInertia;
using drake::multibody::SpatialVelocity;
using drake::multibody::UnitInertia;
using Eigen::Vector3d;

struct BeltPoint {
  Vector3d p_WB;
  // Body pitch angle about world y. With this convention, the body x-axis is
  // tangent to the belt path.
  double pitch{};
};

class BeltSpeedController final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BeltSpeedController);

  BeltSpeedController(BodyIndex body_index, double target_speed, double kp,
                      double max_force, const Vector3d& application_point_B)
      : systems::LeafSystem<double>(),
        body_index_(body_index),
        target_speed_(target_speed),
        kp_(kp),
        max_force_(max_force),
        application_point_B_(application_point_B) {
    body_poses_input_port_ =
        this->DeclareAbstractInputPort("body_poses",
                                       Value<std::vector<RigidTransformd>>())
            .get_index();
    body_spatial_velocities_input_port_ =
        this->DeclareAbstractInputPort(
                "body_spatial_velocities",
                Value<std::vector<SpatialVelocity<double>>>())
            .get_index();
    this->DeclareAbstractOutputPort("spatial_force",
                                    &BeltSpeedController::CalcSpatialForce);
  }

  const systems::InputPort<double>& get_body_poses_input_port() const {
    return this->get_input_port(body_poses_input_port_);
  }

  const systems::InputPort<double>& get_body_spatial_velocities_input_port()
      const {
    return this->get_input_port(body_spatial_velocities_input_port_);
  }

  const systems::OutputPort<double>& get_spatial_force_output_port() const {
    return this->get_output_port(0);
  }

 private:
  void CalcSpatialForce(
      const systems::Context<double>& context,
      std::vector<ExternallyAppliedSpatialForce<double>>* output) const {
    output->resize(1);
    const RigidTransformd& X_WB =
        get_body_poses_input_port().Eval<std::vector<RigidTransformd>>(
            context)[body_index_];
    const SpatialVelocity<double>& V_WB =
        get_body_spatial_velocities_input_port()
            .Eval<std::vector<SpatialVelocity<double>>>(context)[body_index_];

    const Vector3d xhat_W = X_WB.rotation() * Vector3d::UnitX();
    const double measured_speed = xhat_W.dot(V_WB.translational());
    const double force_magnitude = std::clamp(
        kp_ * (target_speed_ - measured_speed), -max_force_, max_force_);

    SpatialForce<double> F_Bq_B = SpatialForce<double>::Zero();
    F_Bq_B.translational() = force_magnitude * Vector3d::UnitX();

    ExternallyAppliedSpatialForce<double>& force = output->front();
    force.body_index = body_index_;
    force.p_BoBq_B = application_point_B_;
    force.F_Bq_W = X_WB.rotation() * F_Bq_B;
  }

  BodyIndex body_index_;
  double target_speed_{};
  double kp_{};
  double max_force_{};
  Vector3d application_point_B_;
  systems::InputPortIndex body_poses_input_port_;
  systems::InputPortIndex body_spatial_velocities_input_port_;
};

BeltPoint CalcBeltPoint(double s, double drum_spacing,
                        double centerline_radius) {
  const double straight = drum_spacing;
  const double arc = M_PI * centerline_radius;
  const double perimeter = 2.0 * straight + 2.0 * arc;
  s = std::fmod(s, perimeter);
  if (s < 0.0) {
    s += perimeter;
  }

  const double left_x = -0.5 * drum_spacing;
  const double right_x = 0.5 * drum_spacing;
  if (s < straight) {
    return {.p_WB = Vector3d(left_x + s, 0.0, centerline_radius), .pitch = 0.0};
  }
  s -= straight;

  if (s < arc) {
    const double alpha = M_PI / 2.0 - s / centerline_radius;
    const Vector3d p_WB(right_x + centerline_radius * std::cos(alpha), 0.0,
                        centerline_radius * std::sin(alpha));
    return {.p_WB = p_WB, .pitch = s / centerline_radius};
  }
  s -= arc;

  if (s < straight) {
    return {.p_WB = Vector3d(right_x - s, 0.0, -centerline_radius),
            .pitch = M_PI};
  }
  s -= straight;

  const double alpha = -M_PI / 2.0 - s / centerline_radius;
  const Vector3d p_WB(left_x + centerline_radius * std::cos(alpha), 0.0,
                      centerline_radius * std::sin(alpha));
  return {.p_WB = p_WB, .pitch = M_PI + s / centerline_radius};
}

RigidTransformd MakePose(const BeltPoint& point) {
  return RigidTransformd(RollPitchYawd(0.0, point.pitch, 0.0), point.p_WB);
}

Vector3d ExpressPointInBody(const BeltPoint& body_pose_point,
                            const Vector3d& point_W) {
  const RigidTransformd X_WB = MakePose(body_pose_point);
  return X_WB.inverse() * point_W;
}

void AddDrumGeometry(MultibodyPlant<double>* plant, std::string name,
                     double x_WD, double drum_radius, double belt_width,
                     const CoulombFriction<double>& friction,
                     const Vector4<double>& color) {
  const RigidTransformd X_WD(RollPitchYawd(-M_PI / 2.0, 0.0, 0.0),
                             Vector3d(x_WD, 0.0, 0.0));
  plant->RegisterCollisionGeometry(plant->world_body(), X_WD,
                                   Cylinder(drum_radius, 1.15 * belt_width),
                                   name + "_collision", friction);
  plant->RegisterVisualGeometry(plant->world_body(), X_WD,
                                Cylinder(drum_radius, 1.15 * belt_width),
                                name + "_visual", color);
}

void AddCenterSupportGeometry(MultibodyPlant<double>* plant,
                              double drum_spacing, double drum_radius,
                              double belt_width,
                              const CoulombFriction<double>& friction,
                              const Vector4<double>& color) {
  const Box support_shape(drum_spacing, 1.15 * belt_width, 2.0 * drum_radius);
  plant->RegisterCollisionGeometry(plant->world_body(),
                                   RigidTransformd::Identity(), support_shape,
                                   "center_support_collision", friction);
  plant->RegisterVisualGeometry(plant->world_body(),
                                RigidTransformd::Identity(), support_shape,
                                "center_support_visual", color);
}

int do_main() {
  DRAKE_DEMAND(FLAGS_num_links >= 12);
  DRAKE_DEMAND(FLAGS_time_step > 0.0);
  DRAKE_DEMAND(FLAGS_drum_spacing > 0.0);
  DRAKE_DEMAND(FLAGS_drum_radius > 0.0);
  DRAKE_DEMAND(FLAGS_belt_thickness > 0.0);
  DRAKE_DEMAND(FLAGS_belt_width > 0.0);
  DRAKE_DEMAND(FLAGS_belt_speed_kp >= 0.0);
  DRAKE_DEMAND(FLAGS_max_drive_force >= 0.0);
  const int driven_link =
      (FLAGS_driven_link % FLAGS_num_links + FLAGS_num_links) % FLAGS_num_links;

  systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] =
      multibody::AddMultibodyPlantSceneGraph(&builder, FLAGS_time_step);
  plant.set_discrete_contact_approximation(DiscreteContactApproximation::kSap);

  const CoulombFriction<double> belt_friction(1.0, 0.9);
  const CoulombFriction<double> drum_friction(0.0, 0.0);
  const CoulombFriction<double> box_friction(1.1, 1.0);

  const double centerline_radius =
      FLAGS_drum_radius + 0.5 * FLAGS_belt_thickness;
  const double perimeter =
      2.0 * FLAGS_drum_spacing + 2.0 * M_PI * centerline_radius;
  const double pitch = perimeter / FLAGS_num_links;
  const double link_length = 0.96 * pitch;

  std::vector<const RigidBody<double>*> links;
  links.reserve(FLAGS_num_links);
  std::vector<BeltPoint> link_points;
  link_points.reserve(FLAGS_num_links);

  for (int i = 0; i < FLAGS_num_links; ++i) {
    const std::string name = "belt_link_" + std::to_string(i);
    const UnitInertia<double> G_BBo = UnitInertia<double>::SolidBox(
        link_length, FLAGS_belt_width, FLAGS_belt_thickness);
    const SpatialInertia<double> M_BBo =
        SpatialInertia<double>::MakeFromCentralInertia(
            FLAGS_link_mass, Vector3d::Zero(), FLAGS_link_mass * G_BBo);
    const RigidBody<double>& link = plant.AddRigidBody(name, M_BBo);
    links.push_back(&link);

    const Vector4<double> color = (i == driven_link)
                                      ? Vector4<double>(0.9, 0.2, 0.1, 1.0)
                                      : Vector4<double>(0.08, 0.12, 0.14, 1.0);
    plant.RegisterCollisionGeometry(
        link, RigidTransformd::Identity(),
        Box(link_length, FLAGS_belt_width, FLAGS_belt_thickness),
        name + "_collision", belt_friction);
    plant.RegisterVisualGeometry(
        link, RigidTransformd::Identity(),
        Box(link_length, FLAGS_belt_width, FLAGS_belt_thickness),
        name + "_visual", color);

    link_points.push_back(CalcBeltPoint((i + 0.5) * pitch, FLAGS_drum_spacing,
                                        centerline_radius));
  }

  plant.SetDefaultFloatingBaseBodyPose(*links.front(),
                                       MakePose(link_points[0]));

  for (int i = 0; i < FLAGS_num_links - 1; ++i) {
    const BeltPoint joint_point =
        CalcBeltPoint((i + 1) * pitch, FLAGS_drum_spacing, centerline_radius);
    const Vector3d p_PF = ExpressPointInBody(link_points[i], joint_point.p_WB);
    const Vector3d p_BM =
        ExpressPointInBody(link_points[i + 1], joint_point.p_WB);
    const std::string joint_name = "belt_pin_" + std::to_string(i);
    plant.AddJoint<RevoluteJoint>(joint_name, *links[i], RigidTransformd(p_PF),
                                  *links[i + 1], RigidTransformd(p_BM),
                                  Vector3d::UnitY(), 0.02);
    RevoluteJoint<double>& joint =
        plant.GetMutableJointByName<RevoluteJoint>(joint_name);
    joint.set_default_angle(link_points[i + 1].pitch - link_points[i].pitch);
  }

  const BeltPoint closure_point =
      CalcBeltPoint(0.0, FLAGS_drum_spacing, centerline_radius);
  const Vector3d p_last =
      ExpressPointInBody(link_points.back(), closure_point.p_WB);
  const Vector3d p_first =
      ExpressPointInBody(link_points.front(), closure_point.p_WB);
  const double closure_axis_offset = 0.2 * FLAGS_belt_width;
  plant.AddBallConstraint(
      *links.back(), p_last + closure_axis_offset * Vector3d::UnitY(),
      *links.front(), p_first + closure_axis_offset * Vector3d::UnitY());
  plant.AddBallConstraint(
      *links.back(), p_last - closure_axis_offset * Vector3d::UnitY(),
      *links.front(), p_first - closure_axis_offset * Vector3d::UnitY());

  Eigen::Vector4d support_rgb(0.6, 0.62, 0.66, 1.0);
  AddDrumGeometry(&plant, "left_drum", -0.5 * FLAGS_drum_spacing,
                  FLAGS_drum_radius, FLAGS_belt_width, drum_friction,
                  support_rgb);
  AddDrumGeometry(&plant, "right_drum", 0.5 * FLAGS_drum_spacing,
                  FLAGS_drum_radius, FLAGS_belt_width, drum_friction,
                  support_rgb);
  if (FLAGS_include_center_support) {
    AddCenterSupportGeometry(&plant, FLAGS_drum_spacing,
                             FLAGS_drum_radius * 0.9, FLAGS_belt_width,
                             drum_friction, support_rgb);
  }

  const UnitInertia<double> G_Box = UnitInertia<double>::SolidBox(
      FLAGS_box_size, FLAGS_box_size, FLAGS_box_size);
  const SpatialInertia<double> M_Box =
      SpatialInertia<double>::MakeFromCentralInertia(
          FLAGS_box_mass, Vector3d::Zero(), FLAGS_box_mass * G_Box);
  const RigidBody<double>& box = plant.AddRigidBody("box", M_Box);
  plant.RegisterCollisionGeometry(
      box, RigidTransformd::Identity(),
      Box(FLAGS_box_size, FLAGS_box_size, FLAGS_box_size), "box_collision",
      box_friction);
  plant.RegisterVisualGeometry(
      box, RigidTransformd::Identity(),
      Box(FLAGS_box_size, FLAGS_box_size, FLAGS_box_size), "box_visual",
      Vector4<double>(0.16, 0.38, 0.72, 1.0));
  const double box_z =
      centerline_radius + FLAGS_belt_thickness + 0.5 * FLAGS_box_size + 0.005;
  plant.SetDefaultFloatingBaseBodyPose(
      box, RigidTransformd(Vector3d(-0.3 * FLAGS_drum_spacing, 0.0, box_z)));

  plant.Finalize();

  BeltSpeedController* speed_controller =
      builder.AddSystem<BeltSpeedController>(
          links.at(driven_link)->index(), FLAGS_target_belt_speed,
          FLAGS_belt_speed_kp, FLAGS_max_drive_force, Vector3d::Zero());
  builder.Connect(plant.get_body_poses_output_port(),
                  speed_controller->get_body_poses_input_port());
  builder.Connect(plant.get_body_spatial_velocities_output_port(),
                  speed_controller->get_body_spatial_velocities_input_port());
  builder.Connect(speed_controller->get_spatial_force_output_port(),
                  plant.get_applied_spatial_force_input_port());

  if (FLAGS_visualize) {
    visualization::VisualizationConfig visualization_config;
    visualization_config.publish_contacts = false;
    visualization::ApplyVisualizationConfig(visualization_config, &builder);
  }

  auto diagram = builder.Build();
  auto diagram_context = diagram->CreateDefaultContext();

  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.AdvanceTo(FLAGS_simulation_time);

  return 0;
}

}  // namespace
}  // namespace conveyor_belt
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A literal coarse conveyor belt: box links pinned in a closed loop "
      "around two fixed drums, carrying a free box. Launch meldis before "
      "running this example to view it.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::conveyor_belt::do_main();
}
