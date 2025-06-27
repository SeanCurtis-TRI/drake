#pragma once

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/string_unordered_map.h"
#include "drake/planning/collision_checker.h"
#include "drake/planning/dof_mask.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/systems/framework/leaf_system.h"

namespace anzu {
namespace operational_space_control {

// TODO(jeremy-nimmer) After the DIK constructors are cleaned up, relocate this
// definition to be closer to the (only) constraint that needs it.
/** Filters clearance data for defining a collision constraint.

TODO(rick-poyner): maybe import a bunch of constraint math doc and paper
citations from robot_bridge/common/motion_primitive.h.

@param active_dof indicates active degrees of freedom.
@param robot_clearance the clearance summary for the current robot state.
@param dist_out[out] On return, contains the distances. If vector zero size on
  return, no collision constraint will be added. Guaranteed to be non-null on
  entry.
@param ddist_dq_out[out] On return, contains motion derivatives. The number of
  columns must match the number of active degrees of freedom. Guaranteed to be
  non-null on entry. */
using SelectDataForCollisionConstraintFunction = std::function<void(
    const drake::planning::DofMask& active_dof,
    const drake::planning::RobotClearance& robot_clearance,
    Eigen::VectorXd* dist_out, Eigen::MatrixXd* ddist_dq_out)>;

/** The %DifferentialInverseKinematicsSystem takes as input desired cartesian
poses (or cartesian velocities) for an arbitrary number of "goal" frames on the
robot, and produces a generalized velocity command as output to move the goal
frames toward the desired state. This system is stateless, but is intended to be
clocked at a known, fixed time step Δt by evaluating its output port at integer
multiples of Δt.

The velocity command is computed by solving a MathematicalProgram optimization
problem, consisting of:
- one primary objective (typically LeastSquaresCost or WeightedSpatialCost),
- typically a secondary objective (JointCenteringCost) to resolve the nullspace
  within the primary objective, and
- various optional constraints:
  - CartesianPositionLimitConstraint
  - CartesianVelocityLimitConstraint
  - CollisionConstraint
  - JointVelocityLimitConstraint
  - StabilityConstraint

In brief, we solve for `v_next` such that `Jv_TGs * v_next` is close to `Vd_TGs`
subject to the constraints, where:
- v_next is the generalized velocity command on the output port, which has the
  dimension of the number of active degrees of freedom,
- Vd_TGs is the desired spatial velocities of the goal frames (when desired
  positions are input, the desired velocity is inferred using the difference
  from the current position vs the time step),
- Jv_TGs is the jacobian relating spatial velocities to generalized velocities,
  i.e., V_TGs (rows) with respect to v_active (cols) -- v_active is the subset
  of generalized velocities for the active degrees of freedom.

For an introduction to differential inverse kinematics via optimization,
see section 10.6 of:
https://manipulation.csail.mit.edu/pick.html#diff_ik_w_constraints

@system
name: DifferentialInverseKinematicsSystem
input_ports:
- position
- nominal_posture
- desired_cartesian_velocities (optional)
- desired_cartesian_poses (optional)
output_ports:
- commanded_velocity
@endsystem

Port `position` accepts the current generalized position (for the full `plant`,
not just the active dofs).

Port `desired_cartesian_velocities` accepts desired cartesian velocities, typed
as drake::systems::BusValue where key is the name of the frame to track and the
value is the drake::multibody::SpatialVelocity<double> w.r.t the task frame
(described in the DifferentialInverseKinematicsParameters).

Port `desired_cartesian_poses` accepts desired cartesian poses, typed as
drake::systems::BusValue where key is the name of the frame to track and the
value is the drake::math::RigidTransformd spatial pose w.r.t the task frame.

Port `nominal_posture` accepts a generalized position to be used to handle
nullspace resolution; this has the dimension of the full degrees of freedom
(not just active dofs).

Port `commanded_velocity` emits generalized velocity that realizes the desired
cartesian velocities or poses within the constraints. This has the dimension of
the number of active degrees of freedom.

Either `desired_cartesian_velocities` or `desired_cartesian_poses` must be
connected. Connecting both ports will result in an exception.

@note There is no consistency check to ensure that the frames being tracked are
the same across multiple time steps.

TODO(jeremy-nimmer) This class only works correctly when the plant has v = q̇.

TODO(Aditya.Bhat): Remove `desired_cartesian_poses' input port entirely and put
that as part of a leaf system that outputs cartesian velocity. (This is not as
simple as it sounds. It means caching the plant kinematics twice (in each of the
two leaf systems), and having the logic for Vd_TG_limit spread across two
classes. Possibly the current design is actually better, and we should decide
not to do this and drop the TODO.)

= Notation =

The implementation uses "monogram notation" abbreviations throughout. See
https://drake.mit.edu/doxygen_cxx/group__multibody__quantities.html for
details. The relevant frame names are:
- B: base frame
- Gi: the i'th goal frame (per the desired_cartesian_... input port)
- T: task frame
- W: world frame

To denote desired spatial velocities, we use a "d" suffix (i.e., "Vd").
Quantities like Vd_TGi (and therefore also Vd_TGlist and Vd_TGs) refer to the
desired velocity, not the current velocity. Other quantities without the "d"
subscript (e.g., X_TGi, Jv_TGi, etc.) refer to the current kinematics.

When 's' is used as a suffix (e.g., 'Vd_TGs'), it refers to the stack of all
goals one after another, e.g., Vd_TGs refers to the concatenation of all Vd_TGi
in Vd_TGlist. You can think of the 's' either as an abbreviation for "stack" or
as a plural.

We also use the letter 'S' to refer a "multibody system", in our case the robot
portion of the controller plant (not including the environment), as defined by
the collision_checker. For example, use the notation 'Scm' to denote the robot's
center of mass.

@ingroup control_systems */
class DifferentialInverseKinematicsSystem final
    : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DifferentialInverseKinematicsSystem);

  // These nested classes are defined later in this file.
  struct CallbackDetails;
  class Recipe;
  class Ingredient;

  // These nested classes are defined later in this file.
  // Keep this list in alphabetical order.
  class CartesianPositionLimitConstraint;
  class CartesianVelocityLimitConstraint;
  class CollisionConstraint;
  class JointCenteringCost;
  class JointVelocityLimitConstraint;
  class LeastSquaresCost;

  /** (Advanced) Constructs the DifferentialInverseKinematicsSystem with a
  user-provided recipe for the mathematical program formulation. */
  DifferentialInverseKinematicsSystem(
      std::unique_ptr<const Recipe> recipe, std::string_view task_frame,
      std::shared_ptr<const drake::planning::CollisionChecker>
          collision_checker,
      const drake::planning::DofMask& active_dof, double time_step, double K_VX,
      const drake::multibody::SpatialVelocity<double>& Vd_TG_limit);

  ~DifferentialInverseKinematicsSystem() final;

  /** Gets the plant used by the controller. */
  const drake::multibody::MultibodyPlant<double>& plant() const {
    return collision_checker_->plant();
  }

  /** Gets the collision checker used by the controller. */
  const drake::planning::CollisionChecker& collision_checker() const {
    return *collision_checker_;
  }

  /** Gets the mask of active DOFs in plant() that are being controlled. */
  const drake::planning::DofMask& active_dof() const { return active_dof_; }

  /** Gets the time step used by the controller. */
  double time_step() const { return time_step_; }

  /** Gets the frame assumed on the desired_cartesian_poses input port. */
  const drake::multibody::Frame<double>& task_frame() const {
    return *task_frame_;
  }

  /** Returns the input port for the joint positions. */
  const drake::systems::InputPort<double>& get_input_port_position() const {
    return this->get_input_port(input_port_index_position_);
  }

  /** Returns the input port for the nominal joint positions to be used to
  handle nullspace resolution. This has the dimension of the full
  `plant.num_positions()`; non-active dofs will be ignored. */
  const drake::systems::InputPort<double>& get_input_port_nominal_posture()
      const {
    return this->get_input_port(input_port_index_nominal_posture_);
  }

  /** Returns the input port for the desired cartesian poses (of type
  drake::systems::BusValue containing drake::math::RigidTransformd). */
  const drake::systems::InputPort<double>&
  get_input_port_desired_cartesian_poses() const {
    return this->get_input_port(input_port_index_desired_cartesian_poses_);
  }

  /** Returns the input port for the desired cartesian velocities. */
  const drake::systems::InputPort<double>&
  get_input_port_desired_cartesian_velocities() const {
    return this->get_input_port(input_port_index_desired_cartesian_velocities_);
  }

  /** Returns the output port for the generalized velocity command that realizes
  the desired poses within the constraints. The size is equal to
  `get_active_dof().count()`. */
  const drake::systems::OutputPort<double>& get_output_port_commanded_velocity()
      const {
    return this->get_output_port(output_port_index_commanded_velocity_);
  }

 private:
  struct CartesianDesires;
  void PrepareMultibodyContext(const drake::systems::Context<double>&,
                               drake::systems::Context<double>*) const;
  void PrepareCartesianDesires(const drake::systems::Context<double>&,
                               CartesianDesires*) const;

  void CalcCommandedVelocity(const drake::systems::Context<double>& context,
                             drake::systems::BasicVector<double>* output) const;

  // Constructor arguments.
  const std::unique_ptr<const Recipe> recipe_;
  const std::shared_ptr<const drake::planning::CollisionChecker>
      collision_checker_;
  const drake::planning::DofMask active_dof_;
  const double time_step_;
  const double K_VX_;
  const drake::multibody::SpatialVelocity<double> Vd_TG_limit_;

  // Derived from constructor arguments.
  const drake::multibody::Frame<double>* const task_frame_;

  // LeafSystem plumbing, set once during the constructor and then never changed
  // again.
  drake::systems::InputPortIndex input_port_index_position_;
  drake::systems::InputPortIndex input_port_index_nominal_posture_;
  drake::systems::InputPortIndex input_port_index_desired_cartesian_poses_;
  drake::systems::InputPortIndex input_port_index_desired_cartesian_velocities_;
  drake::systems::OutputPortIndex output_port_index_commanded_velocity_;
  drake::systems::CacheIndex plant_context_cache_index_;
  drake::systems::CacheIndex cartesian_desires_cache_index_;
};

/** (Internal use only) A group of common arguments relevant to multiple
different costs and constraints within the DifferentialInverseKinematicsSystem
program formulation. Think of this struct as a short-lived pack of function
arguments that is set once and then processed by multiple helper functions; it
is not intended to be a long-lived abstract data type. */
struct DifferentialInverseKinematicsSystem::CallbackDetails {
  /** The mutable, work-in-progress optimization program. */
  drake::solvers::MathematicalProgram& mathematical_program;

  /** The decision variables being optimized. This has the dimension of the
  number of active degrees of freedom (see `active_dof`). */
  const drake::solvers::VectorXDecisionVariable& v_next;

  /** A context for the control plant, set to current positions.
  (At the moment, velocities are zero but that might change down the road.) */
  const drake::systems::Context<double>& plant_context;

  /** The collision checker for the robot being controlled.
  Note that its robot_model_instances() accessor also partitions which parts of
  the `plant` are the robot model vs its environment. */
  const drake::planning::CollisionChecker& collision_checker;

  /** The active degrees of freedom in `plant`. */
  const drake::planning::DofMask& active_dof;

  /** The control rate for DifferentialInverseKinematicsSystem (the pace at
  which velocity commands are expected to be applied). */
  const double time_step;

  /** The current value of the `nominal_posture` input port. This has the
  dimension of the full degrees of freedom (not just active dofs). */
  const Eigen::VectorXd& nominal_posture;

  /** The list of frames being controlled. */
  std::vector<const drake::multibody::Frame<double>*> frame_list;

  /** The current poses of the goal frames. */
  const std::vector<drake::math::RigidTransformd>& X_TGlist;

  /** The desired velocities of the goal frames. */
  const std::vector<drake::multibody::SpatialVelocity<double>> Vd_TGlist;

  /** The jacobian relating spatial velocities to generalized velocities, i.e.,
  V_TGs (rows) with respect to v_active (cols). */
  const Eigen::MatrixXd& Jv_TGs;
};

/** (Internal use only) A user-provided set of constraint(s) and/or cost(s) for
a DifferentialInverseKinematicsSystem recipe, to allow for user customization of
the mathematical program formulation.

TODO(jeremy-nimmer) In the future, we should remove "internal use only" so that
users can officially bake their own recipes for
DifferentialInverseKinematicsSystem.
*/
class DifferentialInverseKinematicsSystem::Ingredient {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Ingredient);
  virtual ~Ingredient();

  /** Adds this ingredient into DifferentialInverseKinematicsSystem's
  mathematical program, returning the listing of bindings for all newly-added
  costs and/or constraints.

  @param[in,out] details A group of arguments commonly used by most costs and
  constraints. */
  virtual std::vector<drake::solvers::Binding<drake::solvers::EvaluatorBase>>
  AddToProgram(CallbackDetails* details) const = 0;

  // TODO(jeremy-nimmer) Add an 'update program' virtual method (which accepts
  // the vector-of-bindings returned by `AddToProgram` plus the new `details`),
  // that can modify the costs/constraints in place, instead of creating new
  // ones. When not overridden in a subclass, the base implementation can simply
  // remove the old bindings from the program and then delegate to 'add to
  // program' to re-add them.

 protected:
  Ingredient() = default;
};

// TODO(jeremy-nimmer) Should this simply be a composite (inherit from the
// Ingredient base class)?
/** A recipe collects a list of ingredients for
DifferentialInverseKinematicsSystem, allowing the user to customize the program
being solved. */
class DifferentialInverseKinematicsSystem::Recipe {
 public:
  void AddIngredient(std::shared_ptr<const Ingredient> ingredient) {
    ingredients_.push_back(std::move(ingredient));
  }

  /** Calls DifferentialInverseKinematicsSystem::Ingredient::AddToProgram on all
  of the ingredients in this recipe. */
  std::vector<drake::solvers::Binding<drake::solvers::EvaluatorBase>>
  AddToProgram(CallbackDetails* details) const;

 private:
  std::vector<std::shared_ptr<const Ingredient>> ingredients_;
};

// ============================================================================
// The available objectives (costs).
// ============================================================================

/** Provides a primary DifferentialInverseKinematics objective to minimize
`G*| S * (V_TGs - Jv_TGs * v_next)|²`, also known as the "least squares"
formulation.

Where:
- G is `cartesian_qp_weight`; this coefficient can be used to balance the
  relative weight of multiple costs in the mathematical program.
- S is a block-diagonal selector matrix applying per-frame axis masks
  to filter out components that are not tracked.
Mask behavior:
- Each 6×6 block of `S` corresponds to a goal frame and is constructed from the
  frame's entry in `cartesian_axis_masks`. If a frame is not explicitly listed,
  all axes are enabled.
- Each axis mask is a binary Vector6d of the form
  [ωx, ωy, ωz, vx, vy, vz] ∈ {0, 1}⁶.*/
class DifferentialInverseKinematicsSystem::LeastSquaresCost final
    : public Ingredient {
 public:
  LeastSquaresCost();
  ~LeastSquaresCost() final;
  std::vector<drake::solvers::Binding<drake::solvers::EvaluatorBase>>
  AddToProgram(CallbackDetails* details) const final;

  double cartesian_qp_weight{1.0};

  /** Map from fully-scoped frame names to their 6D spatial velocity axis mask.
  Each Vector6d is a binary mask [ωx, ωy, ωz, vx, vy, vz] indicating which
  Cartesian velocity components (angular and translational) are being tracked.
  */
  drake::string_unordered_map<drake::Vector6d> cartesian_axis_masks;
};

/** There will almost inevitably be times when the jacobian `Jv_TGs` is not full
rank. Sometimes because of current position values and sometimes because
`Jv_TGs` is rectangular by construction. At those times, there's no longer a
single solution and relying on the mathematical program to pick a reasonable
solution from the large space is unreasonable.

This cost is intended to work in conjunction with the primary cost (e.g.,
LeastSquaresCost). When `Jv_TGs` is not full rank, this provides guidance for
selecting a _unique_ velocity from the space of possible solutions. If the
primary cost produces a space of optimal velocities, this secondary cost will
select the velocity from that space that brings the arm closer to the "nominal
posture":

   |P⋅(v_next - K⋅(nominal_posture_active - q_active))|²

where,
- `K` is the proportional gain of a joint-space controller that pulls toward the
  nominal posture.
- `P` is the linear map from generalized velocities to the _null space_ of
  masked Jacobian `S ⋅ Jv_TGs`. Mapping to the null space, allows refinement of
  the velocity without changing the primary cost.
- S is a block-diagonal matrix of axis masks, enabling per-axis tracking.

Notes:
- When combined with a primary cost, the primary cost should be weighed far more
  heavily to keep this secondary cost from interfering. A factor of 100X or so
  is advisable.
- For this cost to behave as expected, it is critical that the null space of
  `S ⋅ Jv_TGs` be a subspace of the null space used by the primary cost. The
  simplest way is to make sure both costs receive the same axis masks and same
  jacobians. Failure to do so would lead this cost to _fight_ the primary cost
  instead of complementing it.

For more details on this cost, see:
https://manipulation.csail.mit.edu/pick.html#diff_ik_w_constraints#joint_centering

TODO(sean.curtis) As with the other costs, this should also have a scaling
weight (e.g. what `G` is for LeastSquaresCost). */
class DifferentialInverseKinematicsSystem::JointCenteringCost final
    : public Ingredient {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(JointCenteringCost);
  JointCenteringCost();
  ~JointCenteringCost() final;
  std::vector<drake::solvers::Binding<drake::solvers::EvaluatorBase>>
  AddToProgram(CallbackDetails* details) const final;

  /** The proportional gain `K`. */
  double posture_gain{1.0};

  /** Map from fully-scoped frame names to their 6D spatial velocity axis mask.
  Each Vector6d is a binary mask [ωx, ωy, ωz, vx, vy, vz] indicating which
  Cartesian velocity components (angular and translational) are being tracked.
  */
  drake::string_unordered_map<drake::Vector6d> cartesian_axis_masks;
};

// ============================================================================
// The available constraints, in alphabetical order.
// ============================================================================

/** Constrains the goal frames to a cartesian bounding box:
`∀i lower <= p_TGi + Jv_TGi * v_next * Δt <= upper`
where:
- p_TGi is the translation component of the i'th goal point w.r.t the task
  frame. */
class DifferentialInverseKinematicsSystem::CartesianPositionLimitConstraint
    final : public Ingredient {
 public:
  CartesianPositionLimitConstraint();
  ~CartesianPositionLimitConstraint() final;
  std::vector<drake::solvers::Binding<drake::solvers::EvaluatorBase>>
  AddToProgram(CallbackDetails* details) const final;

  Eigen::VectorXd cartesian_bounds_lower;
  Eigen::VectorXd cartesian_bounds_upper;
};

/** Constrains the spatial velocities of the goal frames:
`∀i abs(Jv_TGi * v_next) <= spatial_velocity_limit` */
class DifferentialInverseKinematicsSystem::CartesianVelocityLimitConstraint
    final : public Ingredient {
 public:
  CartesianVelocityLimitConstraint();
  ~CartesianVelocityLimitConstraint() final;
  std::vector<drake::solvers::Binding<drake::solvers::EvaluatorBase>>
  AddToProgram(CallbackDetails* details) const final;

  Eigen::VectorXd spatial_velocity_limit;
};

/** Constrains the collision clearance around the robot to remain above the
safety distance:
`∀j ϕₛ <= ϕⱼ + ∂ϕⱼ/∂q_active * v_next * Δt`
where:
- ϕₛ is the safety_distance;
- ϕⱼ is the current signed distance between the robot and some j'th obstacle;
- ∂ϕⱼ/∂q_active is the gradient of ϕⱼ with respect to the active dof positions.
Obstacles beyond the influence distance are ignored.
The optional select_data_for_collision_constraint may be used to preprocess the
clearance data prior to adding the constraint (e.g., to ignore some parts).

TODO(jeremy-nimmer) This constraint should account for passive dof velocities as
well (i.e., v_passive), using the full ∂q (not just ∂q_active). */
class DifferentialInverseKinematicsSystem::CollisionConstraint final
    : public Ingredient {
 public:
  CollisionConstraint();
  ~CollisionConstraint() final;
  std::vector<drake::solvers::Binding<drake::solvers::EvaluatorBase>>
  AddToProgram(CallbackDetails* details) const final;

  double safety_distance{};
  double influence_distance{};
  SelectDataForCollisionConstraintFunction select_data_for_collision_constraint;
};

/** Constrains the generalized velocity to prevent a commanded velocity that
would push the generalized position outside its limits.

The configured joint velocity limits are not used directly. When the joint
position is near a limit, the joint velocity limit may be insufficient to
constrain `v_next` from pushing the position beyond its limits.

Instead, we use a _scaled_ velocity limit. In simple terms, the closer `q` is to
its position limit, the more velocity towards that limit is constrained. When
`q` lies at the limit boundary, the velocity in that direction would be zero.
However, the configured velocity limit should be applied when q is
"sufficiently" far away from the position limit boundary. So, we compute and
apply a scale factor to attenuate the "near" velocity limit.

The constraint is parameterized to define the domain of the attenuation; how far
away from the near limit boundary should the scaled limit be zero and how far
should it be restored to its configured value? We parameterize those two
distances as:

  - `min_margin`: the distance at which the velocity limit is scaled to be zero.
                  Must be non-negative and is typically positive.
  - `influence_margin`: the distance at which the velocity limit is
                        restored to its configured value. This value must be
                        strictly greater than `min_margin`.

The velocity limit is only ever attenuated on one boundary.

If we define `distᵢ` as the distance to the near position limit boundary, and
`near_limit` as the value of that limit, we can define the attenuation as:

```
  distᵢ = max(qᵢ_current - qᵢ_min, qᵢ_max - qᵢ_current)
  scale = clamp((distᵢ - min_margin) / (influence_margin - min_margin), 0, 1)
  scaled_near_limit = near_limit * scale.
```

Implications:
 - Because we're _attenuating_ the configured velocity limits, an infinite limit
   value won't be attenuated. It _will_ be zero when `distᵢ ≤ min_margin`.
 - The formulation above strongly requires that `q_min ≤ q ≤ q_max`. If `q` ever
   moves beyond its limits, velocity limits will not be clamped, they'll be
   magnified.
 - Position limits define an interval for q, which we'll call P. If
   `P / 2 - min_margin < influence_margin - min_margin` then one limit will
   _always_ be attenuated to be less than its configured value. */
class DifferentialInverseKinematicsSystem::JointVelocityLimitConstraint final
    : public Ingredient {
 public:
  JointVelocityLimitConstraint();
  ~JointVelocityLimitConstraint() final;
  std::vector<drake::solvers::Binding<drake::solvers::EvaluatorBase>>
  AddToProgram(CallbackDetails* details) const final;

  drake::planning::JointLimits joint_limits;
  double min_margin{0.0025};
  double influence_margin{0.1};
};

}  // namespace operational_space_control
}  // namespace anzu
