#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/value.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/math/spatial_algebra.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/prismatic_spring.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/multibody/tree/unit_inertia.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/bus_value.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/visualization/visualization_config_functions.h"

DEFINE_double(target_realtime_rate, 1, "Desired rate relative to real time.");
DEFINE_double(simulation_time, 100.0, "Desired simulation duration in seconds.");
DEFINE_double(time_step, 0.001,
              "Discrete update period for the MultibodyPlant.");
DEFINE_bool(visualize, true, "Whether to publish visualization geometry.");

DEFINE_string(conveyor_belts, "both",
              "Which conveyor belt(s) to add: 'physical', 'virtual', or "
              "'both'.");
DEFINE_int32(num_links, 80,
             "Number of rigid links used to approximate the belt.");
DEFINE_double(drum_spacing, 2.0, "Distance between drum centers, in meters.");
DEFINE_double(drum_radius, 0.16, "Radius of each conveyor drum, in meters.");
DEFINE_double(belt_width, 0.44, "Belt width along the y-axis, in meters.");
DEFINE_double(belt_thickness, 0.035, "Belt link thickness, in meters.");
DEFINE_bool(include_center_support, true,
            "Whether to include the frictionless box between the drums.");
DEFINE_double(link_mass, 0.03, "Mass of each belt link, in kg.");
DEFINE_string(contact_model, "point",
              "Contact model to use: 'point' or 'hydro'.");
DEFINE_double(hydro_resolution_hint, 0.02,
              "Resolution hint for hydroelastic contact geometries, in m.");
DEFINE_double(hydroelastic_modulus, 1.0e5,
              "Hydroelastic modulus for compliant contact geometries, in Pa.");
DEFINE_double(hunt_crossley_dissipation, 10.0,
              "Hunt-Crossley dissipation for contact geometries, in s/m.");
DEFINE_double(target_belt_speed, 0.5, "Target belt speed in m/s.");
DEFINE_double(belt_speed_kp, 50.0,
              "Proportional gain for the belt speed controller.");
DEFINE_double(belt_speed_ki, 50.0,
              "Integral gain for the belt speed controller, in N/(m/s)/s.");
DEFINE_double(max_drive_force, 25.0,
              "Maximum drive force applied to the driven link, in N.");
DEFINE_double(max_integral_drive_force, 10.0,
              "Maximum integral drive force contribution, in N.");
DEFINE_int32(driven_link, 3,
             "Index of the belt link that receives the body-frame force.");
DEFINE_double(surface_velocity_belt_y, 0.8,
              "World y-position of the mesh belt with surface velocity, in m.");

DEFINE_double(box_size, 0.16, "Edge length of the carried box, in meters.");
DEFINE_double(box_mass, 0.1, "Mass of the carried box, in kg.");
DEFINE_double(button_travel, 0.06,
              "Maximum travel for each spring-loaded button, in meters.");
DEFINE_double(button_press_threshold, 0.015,
              "Button travel that latches the belt direction, in meters.");
DEFINE_double(button_release_threshold, 0.0075,
              "Button travel below which the latch can re-arm, in meters.");
DEFINE_double(button_stiffness, 40.0,
              "Spring stiffness for each conveyor-end button, in N/m.");
DEFINE_double(button_damping, 1.0,
              "Viscous damping for each conveyor-end button, in N s/m.");

namespace drake {
namespace examples {
namespace conveyor_belt {
namespace {

using drake::geometry::Box;
using drake::geometry::Cylinder;
using drake::geometry::Mesh;
using drake::geometry::ProximityProperties;
using drake::math::RigidTransformd;
using drake::math::RollPitchYawd;
using drake::multibody::BodyIndex;
using drake::multibody::ContactModel;
using drake::multibody::CoulombFriction;
using drake::multibody::DiscreteContactApproximation;
using drake::multibody::ExternallyAppliedSpatialForce;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::PrismaticJoint;
using drake::multibody::PrismaticSpring;
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

enum class ConveyorBeltSelection {
  kPhysical,
  kVirtual,
  kBoth,
};

class BeltSpeedController final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BeltSpeedController);

  BeltSpeedController(BodyIndex body_index, double kp, double ki,
                      double max_force,
                      double max_integral_force,
                      const Vector3d& application_point_B,
                      double update_period)
      : systems::LeafSystem<double>(),
        body_index_(body_index),
        kp_(kp),
        ki_(ki),
        max_force_(max_force),
        max_integral_force_(max_integral_force),
        application_point_B_(application_point_B),
        update_period_(update_period) {
    target_speed_input_port_ =
        this->DeclareVectorInputPort("target_speed", 1).get_index();
    body_poses_input_port_ =
        this->DeclareAbstractInputPort("body_poses",
                                       Value<std::vector<RigidTransformd>>())
            .get_index();
    body_spatial_velocities_input_port_ =
        this->DeclareAbstractInputPort(
                "body_spatial_velocities",
                Value<std::vector<SpatialVelocity<double>>>())
            .get_index();
    integral_force_state_index_ = this->DeclareDiscreteState(1);
    previous_target_speed_state_index_ = this->DeclareDiscreteState(1);
    this->DeclarePeriodicDiscreteUpdateEvent(
        update_period, 0.0, &BeltSpeedController::UpdateIntegralForce);
    this->DeclareAbstractOutputPort("spatial_force",
                                    &BeltSpeedController::CalcSpatialForce);
  }

  const systems::InputPort<double>& get_target_speed_input_port() const {
    return this->get_input_port(target_speed_input_port_);
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
  double CalcTargetSpeed(const systems::Context<double>& context) const {
    return get_target_speed_input_port().Eval(context)(0);
  }

  double CalcMeasuredSpeed(const systems::Context<double>& context) const {
    const RigidTransformd& X_WB =
        get_body_poses_input_port().Eval<std::vector<RigidTransformd>>(
            context)[body_index_];
    const SpatialVelocity<double>& V_WB =
        get_body_spatial_velocities_input_port()
            .Eval<std::vector<SpatialVelocity<double>>>(context)[body_index_];

    const Vector3d xhat_W = X_WB.rotation() * Vector3d::UnitX();
    return xhat_W.dot(V_WB.translational());
  }

  double CalcForceMagnitude(const systems::Context<double>& context) const {
    const double target_speed = CalcTargetSpeed(context);
    const double measured_speed = CalcMeasuredSpeed(context);
    const double speed_error = target_speed - measured_speed;
    double integral_force =
        context.get_discrete_state(integral_force_state_index_).GetAtIndex(0);
    const double previous_target_speed =
        context.get_discrete_state(previous_target_speed_state_index_)
            .GetAtIndex(0);
    if (target_speed * previous_target_speed < 0.0) {
      integral_force = 0.0;
    }
    return std::clamp(kp_ * speed_error + integral_force, -max_force_,
                      max_force_);
  }

  void UpdateIntegralForce(const systems::Context<double>& context,
                           systems::DiscreteValues<double>* updates) const {
    const double target_speed = CalcTargetSpeed(context);
    const double measured_speed = CalcMeasuredSpeed(context);
    const double speed_error = target_speed - measured_speed;
    const double previous_target_speed =
        context.get_discrete_state(previous_target_speed_state_index_)
            .GetAtIndex(0);
    double integral_force =
        context.get_discrete_state(integral_force_state_index_).GetAtIndex(0);
    if (target_speed * previous_target_speed < 0.0) {
      integral_force = 0.0;
    }
    const double force_unsaturated = kp_ * speed_error + integral_force;
    const bool saturated_high = force_unsaturated > max_force_;
    const bool saturated_low = force_unsaturated < -max_force_;
    const bool should_integrate =
        (!saturated_high && !saturated_low) ||
        (saturated_high && speed_error < 0.0) ||
        (saturated_low && speed_error > 0.0);

    double next_integral_force = integral_force;
    if (should_integrate) {
      next_integral_force += update_period_ * ki_ * speed_error;
      next_integral_force =
          std::clamp(next_integral_force, -max_integral_force_,
                     max_integral_force_);
    }
    updates->get_mutable_vector(integral_force_state_index_)
        .SetAtIndex(0, next_integral_force);
    updates->get_mutable_vector(previous_target_speed_state_index_)
        .SetAtIndex(0, target_speed);
  }

  void CalcSpatialForce(
      const systems::Context<double>& context,
      std::vector<ExternallyAppliedSpatialForce<double>>* output) const {
    output->resize(1);
    const RigidTransformd& X_WB =
        get_body_poses_input_port().Eval<std::vector<RigidTransformd>>(
            context)[body_index_];
    const double force_magnitude = CalcForceMagnitude(context);

    SpatialForce<double> F_Bq_B = SpatialForce<double>::Zero();
    F_Bq_B.translational() = force_magnitude * Vector3d::UnitX();

    ExternallyAppliedSpatialForce<double>& force = output->front();
    force.body_index = body_index_;
    force.p_BoBq_B = application_point_B_;
    force.F_Bq_W = X_WB.rotation() * F_Bq_B;
  }

  BodyIndex body_index_;
  double kp_{};
  double ki_{};
  double max_force_{};
  double max_integral_force_{};
  Vector3d application_point_B_;
  double update_period_{};
  systems::InputPortIndex target_speed_input_port_;
  systems::InputPortIndex body_poses_input_port_;
  systems::InputPortIndex body_spatial_velocities_input_port_;
  systems::DiscreteStateIndex integral_force_state_index_;
  systems::DiscreteStateIndex previous_target_speed_state_index_;
};

class ButtonSpeedSelector final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ButtonSpeedSelector);

  ButtonSpeedSelector(int state_size, int left_button_q_index,
                      int right_button_q_index, double initial_speed,
                      double press_threshold, double release_threshold,
                      double update_period)
      : systems::LeafSystem<double>(),
        left_button_q_index_(left_button_q_index),
        right_button_q_index_(right_button_q_index),
        speed_magnitude_(std::abs(initial_speed)),
        press_threshold_(press_threshold),
        release_threshold_(release_threshold) {
    const double initial_direction = initial_speed < 0.0 ? -1.0 : 1.0;
    state_input_port_ =
        this->DeclareVectorInputPort("plant_state", state_size).get_index();
    selector_state_index_ =
        this->DeclareDiscreteState(Vector3d(initial_direction, 0.0, 0.0));
    this->DeclarePeriodicDiscreteUpdateEvent(
        update_period, 0.0, &ButtonSpeedSelector::UpdateSelection);
    this->DeclareVectorOutputPort("desired_speed", 1,
                                  &ButtonSpeedSelector::CalcDesiredSpeed);
  }

  const systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_input_port_);
  }

  const systems::OutputPort<double>& get_desired_speed_output_port() const {
    return this->get_output_port(0);
  }

 private:
  bool CalcPressed(double travel, bool was_pressed) const {
    if (was_pressed) {
      return travel > release_threshold_;
    }
    return travel >= press_threshold_;
  }

  void UpdateSelection(const systems::Context<double>& context,
                       systems::DiscreteValues<double>* updates) const {
    const auto& plant_state = get_state_input_port().Eval(context);
    const systems::BasicVector<double>& selector_state =
        context.get_discrete_state(selector_state_index_);
    double direction = selector_state.GetAtIndex(0);
    const bool left_was_pressed = selector_state.GetAtIndex(1) != 0.0;
    const bool right_was_pressed = selector_state.GetAtIndex(2) != 0.0;

    const bool left_pressed =
        CalcPressed(plant_state(left_button_q_index_), left_was_pressed);
    const bool right_pressed =
        CalcPressed(plant_state(right_button_q_index_), right_was_pressed);
    const bool left_rising = left_pressed && !left_was_pressed;
    const bool right_rising = right_pressed && !right_was_pressed;
    if (left_rising != right_rising) {
      direction = left_rising ? 1.0 : -1.0;
    }

    systems::BasicVector<double>& next_state =
        updates->get_mutable_vector(selector_state_index_);
    next_state.SetAtIndex(0, direction);
    next_state.SetAtIndex(1, left_pressed ? 1.0 : 0.0);
    next_state.SetAtIndex(2, right_pressed ? 1.0 : 0.0);
  }

  void CalcDesiredSpeed(const systems::Context<double>& context,
                        systems::BasicVector<double>* output) const {
    const double direction =
        context.get_discrete_state(selector_state_index_).GetAtIndex(0);
    output->SetAtIndex(0, direction * speed_magnitude_);
  }

  int left_button_q_index_{};
  int right_button_q_index_{};
  double speed_magnitude_{};
  double press_threshold_{};
  double release_threshold_{};
  systems::InputPortIndex state_input_port_;
  systems::DiscreteStateIndex selector_state_index_;
};

class SurfaceSpeedBus final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SurfaceSpeedBus);

  explicit SurfaceSpeedBus(std::string surface_velocity_body_name)
      : surface_velocity_body_name_(std::move(surface_velocity_body_name)) {
    speed_input_port_ =
        this->DeclareVectorInputPort("desired_speed", 1).get_index();
    this->DeclareAbstractOutputPort("surface_speeds",
                                    &SurfaceSpeedBus::CalcSurfaceSpeeds);
  }

  const systems::InputPort<double>& get_speed_input_port() const {
    return this->get_input_port(speed_input_port_);
  }

  const systems::OutputPort<double>& get_surface_speeds_output_port() const {
    return this->get_output_port(0);
  }

 private:
  void CalcSurfaceSpeeds(const systems::Context<double>& context,
                         systems::BusValue* output) const {
    const double desired_speed = get_speed_input_port().Eval(context)(0);
    output->Clear();
    output->Set(surface_velocity_body_name_, Value<double>(desired_speed));
  }

  std::string surface_velocity_body_name_;
  systems::InputPortIndex speed_input_port_;
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
                     double resolution_hint, double hydroelastic_modulus,
                     double dissipation,
                     const CoulombFriction<double>& friction,
                     const Vector4<double>& color) {
  const RigidTransformd X_WD(RollPitchYawd(-M_PI / 2.0, 0.0, 0.0),
                             Vector3d(x_WD, 0.0, 0.0));
  const Cylinder drum_shape(drum_radius, 1.15 * belt_width);
  ProximityProperties proximity_props;
  geometry::AddContactMaterial(dissipation, {} /* point stiffness */, friction,
                               &proximity_props);
  geometry::AddCompliantHydroelasticProperties(
      resolution_hint, hydroelastic_modulus, &proximity_props);
  plant->RegisterCollisionGeometry(plant->world_body(), X_WD, drum_shape,
                                   name + "_collision",
                                   std::move(proximity_props));
  plant->RegisterVisualGeometry(plant->world_body(), X_WD, drum_shape,
                                name + "_visual", color);
}

void AddCenterSupportGeometry(MultibodyPlant<double>* plant,
                              double drum_spacing, double drum_radius,
                              double belt_width, double resolution_hint,
                              double hydroelastic_modulus, double dissipation,
                              const CoulombFriction<double>& friction,
                              const Vector4<double>& color) {
  const Box support_shape(drum_spacing, 1.15 * belt_width, 2.0 * drum_radius);
  ProximityProperties proximity_props;
  geometry::AddContactMaterial(dissipation, {} /* point stiffness */, friction,
                               &proximity_props);
  geometry::AddCompliantHydroelasticProperties(
      resolution_hint, hydroelastic_modulus, &proximity_props);
  plant->RegisterCollisionGeometry(
      plant->world_body(), RigidTransformd::Identity(), support_shape,
      "center_support_collision", std::move(proximity_props));
  plant->RegisterVisualGeometry(plant->world_body(),
                                RigidTransformd::Identity(), support_shape,
                                "center_support_visual", color);
}

ProximityProperties MakeRigidHydroelasticProperties(
    double resolution_hint, double dissipation,
    const CoulombFriction<double>& friction) {
  ProximityProperties properties;
  geometry::AddContactMaterial(dissipation, {} /* point stiffness */, friction,
                               &properties);
  geometry::AddRigidHydroelasticProperties(resolution_hint, &properties);
  return properties;
}

ProximityProperties MakeCompliantHydroelasticProperties(
    double resolution_hint, double hydroelastic_modulus, double dissipation,
    const CoulombFriction<double>& friction) {
  ProximityProperties properties;
  geometry::AddContactMaterial(dissipation, {} /* point stiffness */, friction,
                               &properties);
  geometry::AddCompliantHydroelasticProperties(
      resolution_hint, hydroelastic_modulus, &properties);
  return properties;
}

ContactModel ParseContactModel() {
  if (FLAGS_contact_model == "point") {
    return ContactModel::kPoint;
  }
  if (FLAGS_contact_model == "hydro" || FLAGS_contact_model == "hydroelastic") {
    return ContactModel::kHydroelastic;
  }
  throw std::runtime_error("Invalid --contact_model='" + FLAGS_contact_model +
                           "'. Expected 'point', 'hydro', or 'hydroelastic'.");
}

ConveyorBeltSelection ParseConveyorBeltSelection() {
  if (FLAGS_conveyor_belts == "physical") {
    return ConveyorBeltSelection::kPhysical;
  }
  if (FLAGS_conveyor_belts == "virtual") {
    return ConveyorBeltSelection::kVirtual;
  }
  if (FLAGS_conveyor_belts == "both") {
    return ConveyorBeltSelection::kBoth;
  }
  throw std::runtime_error("Invalid --conveyor_belts='" +
                           FLAGS_conveyor_belts +
                           "'. Expected 'physical', 'virtual', or 'both'.");
}

const RigidBody<double>& AddSurfaceVelocityBelt(
    MultibodyPlant<double>* plant, const CoulombFriction<double>& friction) {
  const auto model_instance = plant->AddModelInstance("surface_velocity_belt");
  const SpatialInertia<double> M_BBo =
      SpatialInertia<double>::SolidBoxWithMass(1.0, 1.0, 1.0, 1.0);
  const RigidBody<double>& belt =
      plant->AddRigidBody("surface_velocity_belt", model_instance, M_BBo);
  plant->WeldFrames(
      plant->world_frame(), belt.body_frame(),
      RigidTransformd(Vector3d(0.0, FLAGS_surface_velocity_belt_y, 0.0)));

  const std::string collision_mesh =
      FindResourceOrThrow("drake/examples/conveyor_belt/sagging_belt.obj");
  // The OBJ's long axis is +Y; rotate it into this body's +X belt direction.
  const RigidTransformd X_BC(RollPitchYawd(0.0, 0.0, -M_PI / 2.0),
                             Vector3d::Zero());
  plant->RegisterCollisionGeometry(
      belt, X_BC, Mesh(collision_mesh), "surface_velocity_belt_collision",
      MakeRigidHydroelasticProperties(FLAGS_hydro_resolution_hint,
                                      FLAGS_hunt_crossley_dissipation,
                                      friction));

  const std::string visual_mesh =
      FindResourceOrThrow("drake/examples/conveyor_belt/sagging_belt.gltf");
  plant->RegisterVisualGeometry(belt, RigidTransformd::Identity(),
                                Mesh(visual_mesh),
                                "surface_velocity_belt_visual");
  plant->SetSurfaceVelocityAxis(belt, Vector3d::UnitY());
  return belt;
}

const PrismaticJoint<double>& AddEndButton(
    MultibodyPlant<double>* plant, ModelInstanceIndex model_instance,
    const std::string& name, double x_WB, double y_WB, double z_WB,
    const Vector3d& press_axis_W, double button_thickness,
    double button_y_size, double button_height, double button_travel,
    double button_stiffness, double button_damping, double resolution_hint,
    double hydroelastic_modulus, double dissipation,
    const CoulombFriction<double>& friction, const Vector4<double>& color) {
  const SpatialInertia<double> M_BBo =
      SpatialInertia<double>::SolidBoxWithMass(
          0.05, button_thickness, button_y_size, button_height);
  const RigidBody<double>& button =
      plant->AddRigidBody(name, model_instance, M_BBo);
  const PrismaticJoint<double>& slider = plant->AddJoint<PrismaticJoint>(
      name + "_slider", plant->world_body(),
      RigidTransformd(Vector3d(x_WB, y_WB, z_WB)), button,
      RigidTransformd::Identity(), press_axis_W, 0.0, button_travel,
      button_damping);
  plant->AddForceElement<PrismaticSpring>(slider, 0.0, button_stiffness);

  plant->RegisterCollisionGeometry(
      button, RigidTransformd::Identity(),
      Box(button_thickness, button_y_size, button_height), name + "_collision",
      MakeCompliantHydroelasticProperties(resolution_hint, hydroelastic_modulus,
                                          dissipation, friction));
  plant->RegisterVisualGeometry(
      button, RigidTransformd::Identity(),
      Box(button_thickness, button_y_size, button_height), name + "_visual",
      color);
  return slider;
}

struct EndButtons {
  const PrismaticJoint<double>* left{};
  const PrismaticJoint<double>* right{};
};

int do_main() {
  DRAKE_DEMAND(FLAGS_time_step > 0.0);
  DRAKE_DEMAND(FLAGS_drum_spacing > 0.0);
  DRAKE_DEMAND(FLAGS_drum_radius > 0.0);
  DRAKE_DEMAND(FLAGS_belt_thickness > 0.0);
  DRAKE_DEMAND(FLAGS_belt_width > 0.0);
  DRAKE_DEMAND(FLAGS_hydro_resolution_hint > 0.0);
  DRAKE_DEMAND(FLAGS_hydroelastic_modulus > 0.0);
  DRAKE_DEMAND(FLAGS_hunt_crossley_dissipation >= 0.0);
  DRAKE_DEMAND(std::isfinite(FLAGS_target_belt_speed));
  DRAKE_DEMAND(FLAGS_belt_speed_kp >= 0.0);
  DRAKE_DEMAND(FLAGS_belt_speed_ki >= 0.0);
  DRAKE_DEMAND(FLAGS_max_drive_force >= 0.0);
  DRAKE_DEMAND(FLAGS_max_integral_drive_force >= 0.0);
  DRAKE_DEMAND(FLAGS_button_travel > 0.0);
  DRAKE_DEMAND(FLAGS_button_press_threshold > 0.0);
  DRAKE_DEMAND(FLAGS_button_press_threshold <= FLAGS_button_travel);
  DRAKE_DEMAND(FLAGS_button_release_threshold >= 0.0);
  DRAKE_DEMAND(FLAGS_button_release_threshold <
               FLAGS_button_press_threshold);
  DRAKE_DEMAND(FLAGS_button_stiffness >= 0.0);
  DRAKE_DEMAND(FLAGS_button_damping >= 0.0);
  const ConveyorBeltSelection conveyor_belts = ParseConveyorBeltSelection();
  const bool add_physical_belt =
      conveyor_belts != ConveyorBeltSelection::kVirtual;
  const bool add_virtual_belt =
      conveyor_belts != ConveyorBeltSelection::kPhysical;
  if (add_physical_belt) {
    DRAKE_DEMAND(FLAGS_num_links >= 12);
  }
  const int driven_link =
      add_physical_belt
          ? (FLAGS_driven_link % FLAGS_num_links + FLAGS_num_links) %
                FLAGS_num_links
          : 0;
  const double desired_belt_speed = FLAGS_target_belt_speed;

  systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] =
      multibody::AddMultibodyPlantSceneGraph(&builder, FLAGS_time_step);
  plant.set_discrete_contact_approximation(DiscreteContactApproximation::kSap);
  plant.set_contact_model(ParseContactModel());

  const CoulombFriction<double> belt_friction(1.0, 0.9);
  const CoulombFriction<double> drum_friction(0.0, 0.0);
  const CoulombFriction<double> box_friction(1.1, 1.0);

  const double centerline_radius =
      FLAGS_drum_radius + 0.5 * FLAGS_belt_thickness;
  std::vector<double> belt_lane_y_positions;
  if (add_physical_belt) {
    belt_lane_y_positions.push_back(0.0);
  }
  if (add_virtual_belt) {
    belt_lane_y_positions.push_back(FLAGS_surface_velocity_belt_y);
  }

  std::vector<const RigidBody<double>*> links;
  if (add_physical_belt) {
    links.reserve(FLAGS_num_links);
    std::vector<BeltPoint> link_points;
    link_points.reserve(FLAGS_num_links);
    const double perimeter =
        2.0 * FLAGS_drum_spacing + 2.0 * M_PI * centerline_radius;
    const double pitch = perimeter / FLAGS_num_links;
    const double link_length = 0.96 * pitch;

    for (int i = 0; i < FLAGS_num_links; ++i) {
      const std::string name = "belt_link_" + std::to_string(i);
      const UnitInertia<double> G_BBo = UnitInertia<double>::SolidBox(
          link_length, FLAGS_belt_width, FLAGS_belt_thickness);
      const SpatialInertia<double> M_BBo =
          SpatialInertia<double>::MakeFromCentralInertia(
              FLAGS_link_mass, Vector3d::Zero(), FLAGS_link_mass * G_BBo);
      const RigidBody<double>& link = plant.AddRigidBody(name, M_BBo);
      links.push_back(&link);

      const Vector4<double> color =
          (i == driven_link) ? Vector4<double>(0.9, 0.2, 0.1, 1.0)
                             : Vector4<double>(0.08, 0.12, 0.14, 1.0);
      plant.RegisterCollisionGeometry(
          link, RigidTransformd::Identity(),
          Box(link_length, FLAGS_belt_width, FLAGS_belt_thickness),
          name + "_collision",
          MakeCompliantHydroelasticProperties(
              FLAGS_hydro_resolution_hint, FLAGS_hydroelastic_modulus,
              FLAGS_hunt_crossley_dissipation, belt_friction));
      plant.RegisterVisualGeometry(
          link, RigidTransformd::Identity(),
          Box(link_length, FLAGS_belt_width, FLAGS_belt_thickness),
          name + "_visual", color);

      link_points.push_back(CalcBeltPoint(
          (i + 0.5) * pitch, FLAGS_drum_spacing, centerline_radius));
    }

    plant.SetDefaultFloatingBaseBodyPose(*links.front(),
                                         MakePose(link_points[0]));

    for (int i = 0; i < FLAGS_num_links - 1; ++i) {
      const BeltPoint joint_point =
          CalcBeltPoint((i + 1) * pitch, FLAGS_drum_spacing, centerline_radius);
      const Vector3d p_PF =
          ExpressPointInBody(link_points[i], joint_point.p_WB);
      const Vector3d p_BM =
          ExpressPointInBody(link_points[i + 1], joint_point.p_WB);
      const std::string joint_name = "belt_pin_" + std::to_string(i);
      plant.AddJoint<RevoluteJoint>(
          joint_name, *links[i], RigidTransformd(p_PF), *links[i + 1],
          RigidTransformd(p_BM), Vector3d::UnitY(), 0.02);
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
                    FLAGS_drum_radius, FLAGS_belt_width,
                    FLAGS_hydro_resolution_hint, FLAGS_hydroelastic_modulus,
                    FLAGS_hunt_crossley_dissipation, drum_friction,
                    support_rgb);
    AddDrumGeometry(&plant, "right_drum", 0.5 * FLAGS_drum_spacing,
                    FLAGS_drum_radius, FLAGS_belt_width,
                    FLAGS_hydro_resolution_hint, FLAGS_hydroelastic_modulus,
                    FLAGS_hunt_crossley_dissipation, drum_friction,
                    support_rgb);
    if (FLAGS_include_center_support) {
      AddCenterSupportGeometry(
          &plant, FLAGS_drum_spacing, FLAGS_drum_radius * 0.9, FLAGS_belt_width,
          FLAGS_hydro_resolution_hint, FLAGS_hydroelastic_modulus,
          FLAGS_hunt_crossley_dissipation, drum_friction, support_rgb);
    }
  }

  const UnitInertia<double> G_Box = UnitInertia<double>::SolidBox(
      FLAGS_box_size, FLAGS_box_size, FLAGS_box_size);
  const SpatialInertia<double> M_Box =
      SpatialInertia<double>::MakeFromCentralInertia(
          FLAGS_box_mass, Vector3d::Zero(), FLAGS_box_mass * G_Box);
  const ModelInstanceIndex box_model_instance =
      plant.AddModelInstance("carried_boxes");
  auto add_free_box = [&](const std::string& name, const Vector3d& p_WB,
                          const Vector4<double>& color) {
    const RigidBody<double>& free_box =
        plant.AddRigidBody(name, box_model_instance, M_Box);
    plant.RegisterCollisionGeometry(
        free_box, RigidTransformd::Identity(),
        Box(FLAGS_box_size, FLAGS_box_size, FLAGS_box_size),
        name + "_collision",
        MakeCompliantHydroelasticProperties(
            FLAGS_hydro_resolution_hint, FLAGS_hydroelastic_modulus,
            FLAGS_hunt_crossley_dissipation, box_friction));
    plant.RegisterVisualGeometry(
        free_box, RigidTransformd::Identity(),
        Box(FLAGS_box_size, FLAGS_box_size, FLAGS_box_size), name + "_visual",
        color);
    plant.SetDefaultFloatingBaseBodyPose(free_box, RigidTransformd(p_WB));
  };
  const double box_z =
      centerline_radius + FLAGS_belt_thickness + 0.5 * FLAGS_box_size + 0.005;
  if (add_physical_belt) {
    add_free_box("box", Vector3d(-0.3 * FLAGS_drum_spacing, 0.0, box_z),
                 Vector4<double>(0.16, 0.38, 0.72, 1.0));
  }
  if (add_virtual_belt) {
    add_free_box("surface_velocity_box",
                 Vector3d(-0.3 * FLAGS_drum_spacing,
                          FLAGS_surface_velocity_belt_y, box_z),
                 Vector4<double>(0.12, 0.58, 0.36, 1.0));
  }

  const RigidBody<double>* surface_velocity_belt = nullptr;
  if (add_virtual_belt) {
    surface_velocity_belt = &AddSurfaceVelocityBelt(&plant, belt_friction);
  }

  const double button_thickness = 0.08;
  const double button_height = std::max(0.10, 0.75 * FLAGS_box_size);
  const auto [min_lane_y, max_lane_y] = std::minmax_element(
      belt_lane_y_positions.begin(), belt_lane_y_positions.end());
  const double button_y_center = 0.5 * (*min_lane_y + *max_lane_y);
  const double button_y_size =
      (*max_lane_y - *min_lane_y) + 1.15 * FLAGS_belt_width;
  const double button_face_gap = 0.04;
  const double button_x =
      0.5 * FLAGS_drum_spacing + button_face_gap + 0.5 * button_thickness;
  const CoulombFriction<double> button_friction(0.0, 0.0);
  const Vector4<double> button_color(0.95, 0.68, 0.18, 1.0);
  const ModelInstanceIndex button_model_instance =
      plant.AddModelInstance("end_buttons");
  EndButtons buttons;
  buttons.left = &AddEndButton(
      &plant, button_model_instance, "left_button", -button_x,
      button_y_center, box_z, -Vector3d::UnitX(), button_thickness,
      button_y_size, button_height, FLAGS_button_travel,
      FLAGS_button_stiffness, FLAGS_button_damping, FLAGS_hydro_resolution_hint,
      FLAGS_hydroelastic_modulus, FLAGS_hunt_crossley_dissipation,
      button_friction, button_color);
  buttons.right = &AddEndButton(
      &plant, button_model_instance, "right_button", button_x, button_y_center,
      box_z, Vector3d::UnitX(), button_thickness, button_y_size, button_height,
      FLAGS_button_travel, FLAGS_button_stiffness, FLAGS_button_damping,
      FLAGS_hydro_resolution_hint, FLAGS_hydroelastic_modulus,
      FLAGS_hunt_crossley_dissipation, button_friction, button_color);

  plant.Finalize();

  ButtonSpeedSelector* speed_selector = builder.AddSystem<ButtonSpeedSelector>(
      plant.num_multibody_states(), buttons.left->position_start(),
      buttons.right->position_start(), desired_belt_speed,
      FLAGS_button_press_threshold, FLAGS_button_release_threshold,
      FLAGS_time_step);
  builder.Connect(plant.get_state_output_port(),
                  speed_selector->get_state_input_port());

  if (add_virtual_belt) {
    DRAKE_DEMAND(surface_velocity_belt != nullptr);
    SurfaceSpeedBus* surface_speed_bus = builder.AddSystem<SurfaceSpeedBus>(
        surface_velocity_belt->scoped_name().to_string());
    builder.Connect(speed_selector->get_desired_speed_output_port(),
                    surface_speed_bus->get_speed_input_port());
    builder.Connect(surface_speed_bus->get_surface_speeds_output_port(),
                    plant.get_surface_speeds_input_port());
  }

  if (add_physical_belt) {
    BeltSpeedController* speed_controller =
        builder.AddSystem<BeltSpeedController>(
            links.at(driven_link)->index(), FLAGS_belt_speed_kp,
            FLAGS_belt_speed_ki, FLAGS_max_drive_force,
            FLAGS_max_integral_drive_force, Vector3d::Zero(), FLAGS_time_step);
    builder.Connect(speed_selector->get_desired_speed_output_port(),
                    speed_controller->get_target_speed_input_port());
    builder.Connect(plant.get_body_poses_output_port(),
                    speed_controller->get_body_poses_input_port());
    builder.Connect(
        plant.get_body_spatial_velocities_output_port(),
        speed_controller->get_body_spatial_velocities_input_port());
    builder.Connect(speed_controller->get_spatial_force_output_port(),
                    plant.get_applied_spatial_force_input_port());
  }

  if (FLAGS_visualize) {
    visualization::VisualizationConfig visualization_config;
    visualization_config.publish_contacts = false;
    visualization_config.publish_inertia = false;
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
