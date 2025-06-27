#include "operational_space_control/differential_inverse_kinematics_system.h"

#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/text_logging.h"
#include "drake/common/unused.h"
#include "drake/multibody/inverse_kinematics/differential_inverse_kinematics.h"
#include "drake/multibody/parsing/scoped_names.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/systems/framework/bus_value.h"

namespace anzu {
namespace operational_space_control {
namespace {

using drake::unused;
using drake::Value;
using drake::Vector6d;
using drake::math::RigidTransformd;
using drake::math::RollPitchYawd;
using drake::multibody::ComputePoseDiffInCommonFrame;
using drake::multibody::Frame;
using drake::multibody::JacobianWrtVariable;
using drake::multibody::SpatialVelocity;
using drake::multibody::parsing::GetScopedFrameByName;
using drake::planning::CollisionChecker;
using drake::planning::DofMask;
using drake::planning::JointLimits;
using drake::solvers::Binding;
using drake::solvers::EvaluatorBase;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::OsqpSolver;
using drake::solvers::SolverOptions;
using drake::solvers::VectorXDecisionVariable;
using drake::systems::BasicVector;
using drake::systems::BusValue;
using drake::systems::Context;
using Eigen::Matrix3d;
using Eigen::MatrixX3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using CallbackDetails = DifferentialInverseKinematicsSystem::CallbackDetails;

constexpr int kSpatialVelocityRows =
    SpatialVelocity<double>::kSpatialVectorSize;

constexpr int kCartesianSpaceJacobianRows = 6;

constexpr double kInf = std::numeric_limits<double>::infinity();

}  // namespace

// Let's get all of the linker boilerplate out of the way first:
// clang-format off
DifferentialInverseKinematicsSystem::Ingredient::~Ingredient() = default;
DifferentialInverseKinematicsSystem::CartesianPositionLimitConstraint::CartesianPositionLimitConstraint() = default;  // NOLINT
DifferentialInverseKinematicsSystem::CartesianPositionLimitConstraint::~CartesianPositionLimitConstraint() = default;  // NOLINT
DifferentialInverseKinematicsSystem::CartesianVelocityLimitConstraint::CartesianVelocityLimitConstraint() = default;  // NOLINT
DifferentialInverseKinematicsSystem::CartesianVelocityLimitConstraint::~CartesianVelocityLimitConstraint() = default;  // NOLINT
DifferentialInverseKinematicsSystem::CollisionConstraint::CollisionConstraint() = default;  // NOLINT
DifferentialInverseKinematicsSystem::CollisionConstraint::~CollisionConstraint() = default;  // NOLINT
DifferentialInverseKinematicsSystem::LeastSquaresCost::LeastSquaresCost() = default;  // NOLINT
DifferentialInverseKinematicsSystem::LeastSquaresCost::~LeastSquaresCost() = default;  // NOLINT
DifferentialInverseKinematicsSystem::WeightedSpatialCost::WeightedSpatialCost() = default;  // NOLINT
DifferentialInverseKinematicsSystem::WeightedSpatialCost::~WeightedSpatialCost() = default;  // NOLINT
DifferentialInverseKinematicsSystem::JointCenteringCost::JointCenteringCost() = default;  // NOLINT
DifferentialInverseKinematicsSystem::JointCenteringCost::~JointCenteringCost() = default;  // NOLINT
DifferentialInverseKinematicsSystem::JointVelocityLimitConstraint::JointVelocityLimitConstraint() = default;  // NOLINT
DifferentialInverseKinematicsSystem::JointVelocityLimitConstraint::~JointVelocityLimitConstraint() = default;  // NOLINT
DifferentialInverseKinematicsSystem::StabilityConstraint::StabilityConstraint() = default;  // NOLINT
DifferentialInverseKinematicsSystem::StabilityConstraint::~StabilityConstraint() = default;  // NOLINT
// clang-format on

/* Constructs a block-diagonal matrix selecting Cartesian velocity axes
for each goal frame, based on per-frame axis masks.

Each controlled frame Gi has a 6×6 diagonal mask matrix selecting which
spatial velocity components (angular, linear) are active. These individual
mask matrices are stacked into a single block-diagonal matrix for use
in constraints and cost terms.

For example, given N goal frames, the output matrix has size 6N × 6N and
has the form:

  [ diag(mask_G₁)     0            ...     0         ]
  [     0         diag(mask_G₂)    ...     0         ]
  [    ...           ...           ...    ...        ]
  [     0             0            ...  diag(mask_Gₙ)]

@param frame_list The list of controlled frames {Gi}, borrowed from the plant.
@param cartesian_axis_masks Map from fully-scoped frame names to their 6D axis
mask.

@return axis_selector A 6N × 6N block-diagonal matrix applying the per-frame
axis mask.
*/
MatrixXd BuildBlockDiagonalAxisSelector(
    const std::vector<const Frame<double>*>& frame_list,
    const drake::string_unordered_map<Vector6d>& cartesian_axis_masks) {
  const int num_constraints = frame_list.size() * 6;
  MatrixXd selector = MatrixXd::Zero(num_constraints, num_constraints);
  for (int i = 0; i < static_cast<int>(frame_list.size()); ++i) {
    const auto& frame_name = frame_list[i]->scoped_name().to_string();
    Vector6d axis_mask = Vector6d::Ones();
    if (cartesian_axis_masks.count(frame_name) > 0) {
      axis_mask = cartesian_axis_masks.at(frame_name);
    }
    selector.block(6 * i, 6 * i, 6, 6) = axis_mask.asDiagonal();
  }
  return selector;
}

// TODO(jeremy-nimmer) The definition order in this file has been kept intact
// during refactoring to make reviews (and git history) easier to follow. Once
// all of the edits are finished, we should (in a separate commit) shuffle the
// code order in the cc file to match the header.

// TODO(jeremy-nimmer) The inconsistent local variable names in this file (e.g.,
// `ndof` vs `num_cartesian_...`) have been kept intact during refactoring to
// make reviews (and git history) easier to follow. Once all of the edits are
// finished, we should (in a separate commit) improve the names.

// TODO(jeremy-nimmer) The pervasive use of `auto` this file has been kept
// intact during refactoring to make reviews (and git history) easier to
// follow. Once all of the edits are finished, we should (in a separate commit)
// spell out any `auto`s that are a bridge too far.

std::vector<Binding<EvaluatorBase>>
DifferentialInverseKinematicsSystem::LeastSquaresCost::AddToProgram(
    CallbackDetails* details) const {
  DRAKE_DEMAND(details != nullptr);
  auto& prog = details->mathematical_program;
  const auto& v_next = details->v_next;
  const auto& Vd_TGlist = details->Vd_TGlist;
  const auto& Jv_TGs = details->Jv_TGs;
  const int num_cart_constraints = Vd_TGlist.size() * kSpatialVelocityRows;
  const int ndof = details->active_dof.count();
  DRAKE_DEMAND(num_cart_constraints > 0);
  DRAKE_DEMAND(Jv_TGs.rows() == num_cart_constraints);
  DRAKE_DEMAND(Jv_TGs.cols() == ndof);
  static_assert(sizeof(SpatialVelocity<double>) ==
                sizeof(double) * kSpatialVelocityRows);
  const Eigen::Map<const VectorXd> Vd_TGs(Vd_TGlist[0].get_coeffs().data(),
                                          num_cart_constraints);
  const bool is_convex = true;
  // Minimize |V - J*v|² = vᵀJᵀJv - 2VᵀJv + VᵀV.
  // Note: MP::AddQuadraticCost() documents that what is being optimized is:
  //    0.5*xᵀ*Q*x + bᵀ*x + c.
  // 0.5Q = JᵀJ    -->  Q = 2JᵀJ
  // bᵀ = -2VᵀJ    -->  b = -2JᵀV
  // c = VᵀV       -->  we're choosing to omit it; it reduces the minimum cost,
  //                    but not where that cost occurs.
  const MatrixXd selector =
      BuildBlockDiagonalAxisSelector(details->frame_list, cartesian_axis_masks);
  const MatrixXd masked_Jv = selector * Jv_TGs;
  const VectorXd masked_Vd = selector * Vd_TGs;

  const MatrixXd Q_cart = 2 * masked_Jv.transpose() * masked_Jv;
  const MatrixXd b_cart = -2 * masked_Jv.transpose() * masked_Vd;
  auto binding =
      prog.AddQuadraticCost(cartesian_qp_weight * Q_cart,
                            cartesian_qp_weight * b_cart, v_next, is_convex);
  binding.evaluator()->set_description("Least squares cost");
  return std::vector<Binding<EvaluatorBase>>{std::move(binding)};
}

namespace {

/* The linear map between q̇ and v is given by matrix E_F(q) defined by:
         [ cos(y) * cos(p), -sin(y), 0, 0, 0, 0]
E_F(q) = [ sin(y) * cos(p),  cos(y), 0, 0, 0, 0]
         [         -sin(p),       0, 1, 0, 0, 0]
         [               0,       0, 0, 1, 0, 0]
         [               0,       0, 0, 0, 1, 0]
         [               0,       0, 0, 0, 0, 1]
w_FM = E_F(q) * q̇; q̇ = [ṙ, ṗ, ẏ]ᵀ */
MatrixXd ComputeE(const VectorXd& euler_angles) {
  const double sp = sin(euler_angles[1]);
  const double cp = cos(euler_angles[1]);
  const double sy = sin(euler_angles[2]);
  const double cy = cos(euler_angles[2]);

  Matrix3d E_FRot;
  // clang-format off
  E_FRot <<
    cy * cp, -sy, 0.0,
    sy * cp,  cy, 0.0,
        -sp, 0.0, 1.0;
  // clang-format on
  MatrixXd E_F = MatrixXd::Identity(6, 6);
  E_F.template topLeftCorner<3, 3>() = E_FRot;
  return E_F;
}

/* The linear map from v to q̇ is given by the inverse of E_F(q):
         [          cos(y) / cos(p),          sin(y) / cos(p), 0, 0, 0, 0]
Einv_F = [                  -sin(y),                   cos(y), 0, 0, 0, 0]
         [ sin(p) * cos(y) / cos(p), sin(p) * sin(y) / cos(p), 1, 0, 0, 0]
         [                        0,                        0, 0, 1, 0, 0]
         [                        0,                        0, 0, 0, 1, 0]
         [                        0,                        0, 0, 0, 0, 1]
such that q̇ = Einv_F(q) * w_FM; q̇ = [ṙ, ṗ, ẏ]ᵀ */
MatrixXd ComputeEInverse(const VectorXd& euler_angles) {
  const double cp = cos(euler_angles[1]);
  // Demand for the computation to be away from a state for which Einv_F is
  // singular.
  if (abs(cp) < 1.0e-3) {
    throw std::runtime_error(
        fmt::format("Singular configuration: {}", euler_angles[1]));
  }

  const double sp = sin(euler_angles[1]);
  const double sy = sin(euler_angles[2]);
  const double cy = cos(euler_angles[2]);
  const double cpi = 1.0 / cp;

  const double cy_x_cpi = cy * cpi;
  const double sy_x_cpi = sy * cpi;

  MatrixXd Einv = MatrixXd::Identity(6, 6);
  Matrix3d E_FRot;
  // clang-format off
  E_FRot <<
         cy_x_cpi,      sy_x_cpi, 0.0,
              -sy,            cy, 0.0,
    cy_x_cpi * sp, sy_x_cpi * sp, 1.0;
  // clang-format on
  Einv.template topLeftCorner<3, 3>() = E_FRot;
  return Einv;
}

/* Adds a linear equality constraint to enforce a differential inverse
kinematics relationship.

This function imposes the constraint:
   Jv_TGs * v_next = diag(cartesian_axis_mask) E(q) diag(α) E⁻¹(q) Vd_TGs

where:
- Jv_TGs is the Jacobian matrix,
- v_next is the next-step decision variable for joint velocities,
- E(q) is the linear map from derivative of euler angle to spatial velocity,
- E^{-1}(q) is the linear map from spatial velocity to derivative of euler
angle,
- alpha is a scaling factor applied element-wise, and
- Vd_TGs is the desired cartesian velocities.

@param alpha Decision variable representing the scaling factors for
transformation. */
Binding<EvaluatorBase>
AddDifferentialInverseKinematicsEqualityLinearMapConstraint(
    CallbackDetails* details,
    const Eigen::Ref<const VectorXDecisionVariable>& alpha,
    const drake::string_unordered_map<drake::Vector6d>& cartesian_axis_masks) {
  DRAKE_DEMAND(details != nullptr);
  auto& prog = details->mathematical_program;
  const auto& v_next = details->v_next;
  const auto& X_TGlist = details->X_TGlist;
  const auto& Vd_TGlist = details->Vd_TGlist;
  const auto& Jv_TGs = details->Jv_TGs;
  const int ndof = details->active_dof.count();
  const int num_cart_constraints = Vd_TGlist.size() * kSpatialVelocityRows;

  DRAKE_DEMAND(num_cart_constraints > 0);
  DRAKE_DEMAND(Jv_TGs.rows() == num_cart_constraints);
  DRAKE_DEMAND(Jv_TGs.cols() == ndof);
  DRAKE_DEMAND(X_TGlist.size() == Vd_TGlist.size());
  MatrixXd Beq = MatrixXd::Zero(num_cart_constraints, num_cart_constraints);

  const auto get_axis_selector = [&](const std::string& frame_name) {
    auto it = cartesian_axis_masks.find(frame_name);
    if (it != cartesian_axis_masks.end()) {
      return it->second;
    } else {
      return Vector6d::Ones().eval();
    }
  };

  for (int i = 0; i < ssize(X_TGlist); ++i) {
    const Vector3d rpy_TGi = RollPitchYawd(X_TGlist[i].rotation()).vector();
    const Vector6d& Vd_TGi = Vd_TGlist[i].get_coeffs();
    const std::string frame_name =
        details->frame_list[i]->scoped_name().to_string();
    const Vector6d axis_selector = get_axis_selector(frame_name);
    Beq.block(i * 6, i * 6, 6, 6) =
        axis_selector.asDiagonal() *
        (-ComputeE(rpy_TGi) *
         ((ComputeEInverse(rpy_TGi) * Vd_TGi).asDiagonal()));
  }

  MatrixXd A(num_cart_constraints, ndof + num_cart_constraints);
  A.leftCols(ndof) = BuildBlockDiagonalAxisSelector(details->frame_list,
                                                    cartesian_axis_masks) *
                     Jv_TGs;
  A.rightCols(num_cart_constraints) = Beq;
  auto binding = prog.AddLinearEqualityConstraint(
      A, VectorXd::Zero(num_cart_constraints), {v_next, alpha});
  binding.evaluator()->set_description("Linear map constraint");
  return binding;
}

}  // namespace

std::vector<Binding<EvaluatorBase>>
DifferentialInverseKinematicsSystem::CollisionConstraint::AddToProgram(
    CallbackDetails* details) const {
  DRAKE_DEMAND(details != nullptr);
  std::vector<Binding<EvaluatorBase>> result;
  auto& prog = details->mathematical_program;
  const auto& v_next = details->v_next;
  const auto& plant = details->collision_checker.plant();
  const auto& plant_context = details->plant_context;
  const auto& collision_checker = details->collision_checker;
  const auto& active_dof = details->active_dof;
  const double dt = details->time_step;
  VectorXd dist;
  MatrixXd ddist_dq;
  const auto& robot_position = plant.GetPositions(plant_context);
  // TODO(aditya.bhat): Would be better to share context rather than
  // maintaining separate.
  const auto robot_clearance =
      collision_checker.CalcRobotClearance(robot_position, influence_distance);
  if (select_data_for_collision_constraint != nullptr) {
    // Use the passed-in constraint filtering function.
    select_data_for_collision_constraint(active_dof, robot_clearance, &dist,
                                         &ddist_dq);
  } else {
    // Reduce the derivative columns to those for active dofs.
    dist = robot_clearance.distances();
    int num_active_dofs = active_dof.count();
    MatrixXd jacobians = robot_clearance.jacobians();
    ddist_dq.resize(jacobians.rows(), num_active_dofs);
    active_dof.GetColumnsFromMatrix(jacobians, &ddist_dq);
  }

  const int num_dist = dist.size();
  if (num_dist > 0) {
    // For the constraint to be well-formed, the number of derivative columns
    // must match the number of decision variables.
    DRAKE_DEMAND(ddist_dq.cols() == v_next.rows());

    // Formulate minimum distance directly using time-step:
    //   ϕᵢ + ∂ϕᵢ/∂q v Δt >= ϕₛ
    VectorXd dist_min = (safety_distance - dist.array()).matrix() / dt;
    VectorXd dist_max = VectorXd::Constant(num_dist, kInf);
    auto binding =
        prog.AddLinearConstraint(ddist_dq, dist_min, dist_max, v_next);
    binding.evaluator()->set_description("Collision constraint");
    result.push_back(std::move(binding));
  }
  return result;
}

std::vector<Binding<EvaluatorBase>>
DifferentialInverseKinematicsSystem::StabilityConstraint::AddToProgram(
    CallbackDetails* details) const {
  DRAKE_DEMAND(details != nullptr);
  auto& prog = details->mathematical_program;
  const auto& v_next = details->v_next;
  const auto& plant = details->collision_checker.plant();
  const auto& plant_context = details->plant_context;
  const auto& collision_checker = details->collision_checker;
  const auto& active_dof = details->active_dof;
  const double dt = details->time_step;
  const int v_dim = plant.num_velocities();

  // Compute robot CoM and its Jacobian.
  // TODO(jeremy-nimmer) Missing unit test coverage that we've properly used the
  // robot model CoM here instead of the world model CoM.
  Vector3d p_BScm;
  MatrixXd Jv_v_BScm(3, v_dim);
  const Vector3d p_WScm = plant.CalcCenterOfMassPositionInWorld(
      plant_context, collision_checker.robot_model_instances());
  const Frame<double>& B = plant.GetFrameByName(support_polygon_base_frame);
  const RigidTransformd X_BW =
      plant.CalcRelativeTransform(plant_context, B, plant.world_frame());
  p_BScm = X_BW * p_WScm;
  plant.CalcJacobianCenterOfMassTranslationalVelocity(
      plant_context, JacobianWrtVariable::kV, B, B, &Jv_v_BScm);

  MatrixX3d A;
  VectorXd upper_bound;
  VectorXd negative_inf;
  support_polygon_B.PopulateSupportPolygonPolyhedronAndBounds(
      &A, &upper_bound,
      /* lower_bound = */ &negative_inf);
  DRAKE_ASSERT((negative_inf.array() == -kInf).all());

  MatrixXd Jv_v_BScm_active(3, active_dof.count());
  MatrixXd Jv_v_BScm_passive(3, Jv_v_BScm.cols() - active_dof.count());
  active_dof.GetColumnsFromMatrix(Jv_v_BScm, &Jv_v_BScm_active);
  active_dof.Complement().GetColumnsFromMatrix(Jv_v_BScm, &Jv_v_BScm_passive);

  // TODO(Aditya.Bhat): Remove the assumption of passive DoFs having 0 velocity.
  unused(Jv_v_BScm_passive);
  upper_bound -= A * p_BScm;
  auto binding = prog.AddLinearConstraint(dt * A * Jv_v_BScm_active,
                                          /* lower_bound = */ negative_inf,
                                          upper_bound, v_next);
  binding.evaluator()->set_description("Stability constraint");
  return std::vector<Binding<EvaluatorBase>>{std::move(binding)};
}

namespace {

/* Scales the velocity limits for the position limits.
@param q The current joint positions, active dofs only.
@param joint_limits The joint limits.
@return joint_limits_scaled The scaled joint limits.
The velocity limits are scaled to pad against the position limits.

TODO(jeremy-nimmer) Relocate this function closer to its sole user. */
JointLimits ScaleVelocityLimitsForPositionLimits(
    const VectorXd& q, const JointLimits& joint_limits, const double min_margin,
    const double influence_margin) {
  DRAKE_ASSERT(min_margin >= 0);
  DRAKE_ASSERT(influence_margin > min_margin);
  VectorXd v_limits_lower = joint_limits.velocity_lower();
  VectorXd v_limits_upper = joint_limits.velocity_upper();
  // Scale velocity limits to pad against position limits.
  for (int i = 0; i < joint_limits.num_positions(); i++) {
    const double upper_margin = joint_limits.position_upper()[i] - q[i];
    const double lower_margin = q[i] - joint_limits.position_lower()[i];
    if (upper_margin < lower_margin) {
      // Closer to upper.
      const double scale =
          (upper_margin - min_margin) / (influence_margin - min_margin);
      v_limits_upper[i] *= std::clamp(scale, 0.0, 1.0);
    } else {
      // Closer to lower.
      const double scale =
          (lower_margin - min_margin) / (influence_margin - min_margin);
      v_limits_lower[i] *= std::clamp(scale, 0.0, 1.0);
    }
  }
  return JointLimits(joint_limits.position_lower(),
                     joint_limits.position_upper(), v_limits_lower,
                     v_limits_upper, joint_limits.acceleration_lower(),
                     joint_limits.acceleration_upper());
}

}  // namespace

std::vector<Binding<EvaluatorBase>> DifferentialInverseKinematicsSystem::
    CartesianPositionLimitConstraint::AddToProgram(
        CallbackDetails* details) const {
  DRAKE_DEMAND(details != nullptr);
  auto& prog = details->mathematical_program;
  const auto& v_next = details->v_next;
  const auto& X_TGlist = details->X_TGlist;
  const auto& Jv_TGs = details->Jv_TGs;
  const auto& active_dof = details->active_dof;
  const double dt = details->time_step;
  std::vector<Binding<EvaluatorBase>> result;
  for (int i = 0; i < ssize(X_TGlist); ++i) {
    const auto& p_TGi = X_TGlist[i].translation();
    auto binding = prog.AddLinearConstraint(
        Jv_TGs.block((6 * i) + 3, 0, 3, active_dof.count()) * dt,
        cartesian_bounds_lower - p_TGi, cartesian_bounds_upper - p_TGi, v_next);
    binding.evaluator()->set_description("Cartesian position limit");
    result.push_back(std::move(binding));
  }
  return result;
}

std::vector<Binding<EvaluatorBase>> DifferentialInverseKinematicsSystem::
    CartesianVelocityLimitConstraint::AddToProgram(
        CallbackDetails* details) const {
  DRAKE_DEMAND(details != nullptr);
  auto& prog = details->mathematical_program;
  const auto& Vd_TGlist = details->Vd_TGlist;
  const auto& Jv_TGs = details->Jv_TGs;
  const auto& v_next = details->v_next;
  const SpatialVelocity<double> max_Vd_TG{spatial_velocity_limit};
  VectorXd max_Vd_TGs(ssize(Vd_TGlist) * kSpatialVelocityRows);
  for (int i = 0; i < ssize(Vd_TGlist); ++i) {
    max_Vd_TGs.segment(kSpatialVelocityRows * i, kSpatialVelocityRows) =
        max_Vd_TG.get_coeffs();
  }
  auto binding =
      prog.AddLinearConstraint(Jv_TGs, -max_Vd_TGs, max_Vd_TGs, v_next);
  binding.evaluator()->set_description("Cartesian velocity limit");
  return std::vector<Binding<EvaluatorBase>>{std::move(binding)};
}

std::vector<Binding<EvaluatorBase>>
DifferentialInverseKinematicsSystem::JointVelocityLimitConstraint::AddToProgram(
    CallbackDetails* details) const {
  DRAKE_DEMAND(details != nullptr);
  auto& prog = details->mathematical_program;
  const auto& v_next = details->v_next;
  const auto& plant = details->collision_checker.plant();
  const auto& plant_context = details->plant_context;
  const auto& active_dof = details->active_dof;
  const auto& robot_position = plant.GetPositions(plant_context);

  JointLimits joint_limits_scaled = ScaleVelocityLimitsForPositionLimits(
      active_dof.GetFromArray(robot_position), joint_limits, min_margin,
      influence_margin);
  auto binding = prog.AddBoundingBoxConstraint(
      joint_limits_scaled.velocity_lower(),
      joint_limits_scaled.velocity_upper(), v_next);
  binding.evaluator()->set_description("Joint velocity limit");
  return std::vector<Binding<EvaluatorBase>>{std::move(binding)};
}

std::vector<Binding<EvaluatorBase>>
DifferentialInverseKinematicsSystem::JointCenteringCost::AddToProgram(
    CallbackDetails* details) const {
  DRAKE_DEMAND(details != nullptr);
  auto& prog = details->mathematical_program;
  const auto& v_next = details->v_next;
  const auto& plant = details->collision_checker.plant();
  const auto& plant_context = details->plant_context;
  const auto& active_dof = details->active_dof;
  const auto& nominal_posture = details->nominal_posture;
  const auto& Jv_TGs = details->Jv_TGs;
  std::vector<Binding<EvaluatorBase>> result;
  // Implements the joint centering cost in the form
  // |P(v_next - k(q_active_nominal - q_active))|²
  const MatrixXd cartesian_axis_selector =
      BuildBlockDiagonalAxisSelector(details->frame_list, cartesian_axis_masks);
  const Eigen::FullPivLU<MatrixXd> lu_Jv(cartesian_axis_selector * Jv_TGs);
  const int ndof = Jv_TGs.cols();
  if (lu_Jv.rank() < ndof) {
    const auto& robot_position = plant.GetPositions(plant_context);
    const MatrixXd P = lu_Jv.kernel().transpose();
    const VectorXd q_active_nominal = active_dof.GetFromArray(nominal_posture);
    const VectorXd q_active = active_dof.GetFromArray(robot_position);
    auto binding = prog.Add2NormSquaredCost(
        P, P * posture_gain * (q_active_nominal - q_active), v_next);
    binding.evaluator()->set_description("Joint centering cost");
    result.push_back(std::move(binding));
  }
  return result;
}

namespace {

void LogConstraintViolations(const MathematicalProgram& prog,
                             const MathematicalProgramResult& result) {
  auto infeasible_constraint_names =
      result.GetInfeasibleConstraintNames(prog, std::nullopt);
  if (infeasible_constraint_names.empty()) {
    infeasible_constraint_names.emplace_back("none");
  }
  drake::log()->warn(
      "QP failed to solve, returning zero velocity; the violated constraints "
      "were {}",
      fmt::join(infeasible_constraint_names, ", "));

  // Debugging information for all constraints.
  if (drake::log()->should_log(spdlog::level::debug)) {
    for (const auto& binding : prog.GetAllConstraints()) {
      const auto& constraint = binding.evaluator();

      VectorXd value = result.EvalBinding(binding);
      VectorXd lower_bound = constraint->lower_bound();
      VectorXd upper_bound = constraint->upper_bound();

      for (int i = 0; i < value.size(); ++i) {
        drake::log()->debug("{}, index {}, value = {}, bounds = [{}, {}]",
                            constraint->get_description(), i, value[i],
                            lower_bound[i], upper_bound[i]);
      }
    }
  }
}

VectorXd TrySolveQPAndFallbackToZero(const MathematicalProgram& prog,
                                     const VectorXDecisionVariable& v_next) {
  OsqpSolver solver;
  SolverOptions solver_options;
  solver_options.SetOption(OsqpSolver::id(), "adaptive_rho_interval", 0);
  auto result = solver.Solve(prog, {}, solver_options);
  if (result.is_success()) {
    return result.GetSolution(v_next);
  }
  LogConstraintViolations(prog, result);
  return VectorXd::Zero(v_next.size());
}

}  // namespace

std::vector<Binding<EvaluatorBase>>
DifferentialInverseKinematicsSystem::WeightedSpatialCost::AddToProgram(
    CallbackDetails* details) const {
  DRAKE_DEMAND(details != nullptr);
  std::vector<Binding<EvaluatorBase>> result;
  auto& prog = details->mathematical_program;
  const auto& Vd_TGlist = details->Vd_TGlist;

  VectorXDecisionVariable alpha = prog.NewContinuousVariables(
      ssize(Vd_TGlist) * kSpatialVelocityRows, "alpha");
  Binding<EvaluatorBase> binding =
      prog.AddLinearCost(-VectorXd::Ones(alpha.size()).transpose(), alpha);
  binding.evaluator()->set_description("Maximize alpha cost");
  result.push_back(std::move(binding));

  // 0 <= alpha <= 1
  binding = prog.AddBoundingBoxConstraint(VectorXd::Zero(alpha.size()),
                                          VectorXd::Ones(alpha.size()), alpha);
  binding.evaluator()->set_description("Alpha constraint");
  result.push_back(std::move(binding));

  binding = AddDifferentialInverseKinematicsEqualityLinearMapConstraint(
      details, alpha, cartesian_axis_masks);
  result.push_back(std::move(binding));

  return result;
}

/* Each value on the input port for either desired cartesian poses or desired
cartesian velocities is reprocessed into this internal value which is easier to
work with. The desires stored here are always in terms of spatial velocities; if
the input desire was for a pose, it is converted to a velocity by dividing the
pose error by the time step. All vectors have the same size (the number of
desires). */
struct DifferentialInverseKinematicsSystem::CartesianDesires {
  /* The frame being controlled; a borrowed pointer into our control plant. */
  std::vector<const Frame<double>*> frame_list;
  /* The current pose of the controlled frame. */
  std::vector<RigidTransformd> X_TGlist;
  /* The commanded velocity of the controlled frame. */
  std::vector<SpatialVelocity<double>> Vd_TGlist;
};

std::unique_ptr<DifferentialInverseKinematicsSystem>
DifferentialInverseKinematicsSystem::MakeFromParameters(
    std::shared_ptr<const CollisionChecker> collision_checker,
    const std::optional<planning::SupportPolygon>& support_polygon_B,
    const std::optional<std::string>& support_polygon_base_frame,
    const DifferentialInverseKinematicsParameters& parameters, double time_step,
    SelectDataForCollisionConstraintFunction
        select_data_for_collision_constraint) {
  DRAKE_THROW_UNLESS(collision_checker != nullptr);

  // Set up the recipe based on the parameters.
  auto recipe = std::make_unique<Recipe>();
  switch (parameters.get_diff_ik_version()) {
    case DifferentialInverseKinematicsVersion::kLeastSquares: {
      auto ingredient = std::make_shared<LeastSquaresCost>();
      ingredient->cartesian_qp_weight = parameters.get_cartesian_qp_weight();
      ingredient->cartesian_axis_masks =
          parameters.get_cartesian_axis_mask_map();
      recipe->AddIngredient(std::move(ingredient));
      break;
    }
    case DifferentialInverseKinematicsVersion::kWeightedSpatial: {
      auto ingredient = std::make_shared<WeightedSpatialCost>();
      ingredient->cartesian_axis_masks =
          parameters.get_cartesian_axis_mask_map();
      recipe->AddIngredient(std::move(ingredient));
      break;
    }
  }
  {
    auto ingredient = std::make_shared<JointCenteringCost>();
    ingredient->posture_gain = parameters.get_posture_gain();
    ingredient->cartesian_axis_masks = parameters.get_cartesian_axis_mask_map();
    recipe->AddIngredient(std::move(ingredient));
  }
  {
    auto ingredient = std::make_shared<CartesianPositionLimitConstraint>();
    ingredient->cartesian_bounds_lower =
        parameters.get_cartesian_bounds().first;
    ingredient->cartesian_bounds_upper =
        parameters.get_cartesian_bounds().second;
    recipe->AddIngredient(std::move(ingredient));
  }
  {
    auto ingredient = std::make_shared<CartesianVelocityLimitConstraint>();
    ingredient->spatial_velocity_limit =
        parameters.get_spatial_velocity_limit();
    recipe->AddIngredient(std::move(ingredient));
  }
  {
    auto ingredient = std::make_shared<JointVelocityLimitConstraint>();
    ingredient->joint_limits = parameters.get_joint_limits();
    recipe->AddIngredient(std::move(ingredient));
  }
  if (parameters.collision_avoidance_enabled()) {
    auto ingredient = std::make_shared<CollisionConstraint>();
    ingredient->safety_distance = parameters.get_safety_distance();
    ingredient->influence_distance = parameters.get_influence_distance();
    ingredient->select_data_for_collision_constraint =
        std::move(select_data_for_collision_constraint);
    recipe->AddIngredient(std::move(ingredient));
  }
  if (support_polygon_B.has_value()) {
    // TODO(jeremy.nimmer) Least Squares mode should not support / allow this
    // option, since we don't want to open-source this constraint.
    auto ingredient = std::make_shared<StabilityConstraint>();
    ingredient->support_polygon_B = support_polygon_B.value();
    ingredient->support_polygon_base_frame = support_polygon_base_frame.value();
    recipe->AddIngredient(std::move(ingredient));
  }

  return std::make_unique<DifferentialInverseKinematicsSystem>(
      std::move(recipe), parameters.get_task_frame(),
      std::move(collision_checker), parameters.get_active_dof(), time_step,
      parameters.get_K_VX(),
      SpatialVelocity<double>(parameters.get_spatial_velocity_limit()));
}

DifferentialInverseKinematicsSystem::DifferentialInverseKinematicsSystem(
    std::unique_ptr<const Recipe> recipe, const std::string_view task_frame,
    std::shared_ptr<const CollisionChecker> collision_checker,
    const drake::planning::DofMask& active_dof, const double time_step,
    const double K_VX, const SpatialVelocity<double>& Vd_TG_limit)
    : recipe_(std::move(recipe)),
      collision_checker_(std::move(collision_checker)),
      active_dof_(active_dof),
      time_step_(time_step),
      K_VX_(K_VX),
      Vd_TG_limit_(Vd_TG_limit),
      task_frame_(&GetScopedFrameByName(plant(), task_frame)) {
  input_port_index_position_ =
      this->DeclareVectorInputPort("position", plant().num_positions())
          .get_index();

  input_port_index_nominal_posture_ =
      this->DeclareVectorInputPort("nominal_posture", plant().num_positions())
          .get_index();

  input_port_index_desired_cartesian_poses_ =
      this->DeclareAbstractInputPort("desired_cartesian_poses",
                                     Value<BusValue>{})
          .get_index();

  input_port_index_desired_cartesian_velocities_ =
      this->DeclareAbstractInputPort("desired_cartesian_velocities",
                                     Value<BusValue>{})
          .get_index();

  // Declare cache entry for the multibody plant context.
  auto plant_context = plant().CreateDefaultContext();
  plant_context_cache_index_ =
      this->DeclareCacheEntry(
              "plant_context_cache", *plant_context,
              &DifferentialInverseKinematicsSystem::PrepareMultibodyContext,
              {this->input_port_ticket(get_input_port_position().get_index())})
          .cache_index();

  // Declare cache entry for the reprocessed input from either the
  // desired cartesian poses or desired cartesian velocities.
  cartesian_desires_cache_index_ =
      this->DeclareCacheEntry(
              "cartesian_desires_cache", CartesianDesires{},
              &DifferentialInverseKinematicsSystem::PrepareCartesianDesires,
              {this->input_port_ticket(
                   get_input_port_desired_cartesian_poses().get_index()),
               this->input_port_ticket(
                   get_input_port_desired_cartesian_velocities().get_index()),
               this->cache_entry_ticket(plant_context_cache_index_)})
          .cache_index();

  output_port_index_commanded_velocity_ =
      this->DeclareVectorOutputPort(
              "commanded_velocity", active_dof_.count(),
              &DifferentialInverseKinematicsSystem::CalcCommandedVelocity,
              {this->all_input_ports_ticket(),
               this->cache_entry_ticket(plant_context_cache_index_),
               this->cache_entry_ticket(cartesian_desires_cache_index_)})
          .get_index();
}

DifferentialInverseKinematicsSystem::~DifferentialInverseKinematicsSystem() =
    default;

void DifferentialInverseKinematicsSystem::PrepareMultibodyContext(
    const Context<double>& context, Context<double>* plant_context) const {
  const VectorXd& position = get_input_port_position().Eval(context);

  // Assert that the input port values are finite. One possible reason it could
  // be NaN is if the initial position is not set in the diagram. It can be set
  // using `set_initial_position()`.
  DRAKE_THROW_UNLESS(position.allFinite());

  plant().SetPositions(plant_context, position);
}

void DifferentialInverseKinematicsSystem::PrepareCartesianDesires(
    const Context<double>& context, CartesianDesires* cartesian_desires) const {
  cartesian_desires->frame_list.clear();
  cartesian_desires->X_TGlist.clear();
  cartesian_desires->Vd_TGlist.clear();

  // Check that exactly one of the two cartesian desire ports is connected.
  const bool has_cartesian_positions_input =
      get_input_port_desired_cartesian_poses().HasValue(context);
  const bool has_cartesian_velocities_input =
      get_input_port_desired_cartesian_velocities().HasValue(context);
  DRAKE_THROW_UNLESS(has_cartesian_positions_input !=
                     has_cartesian_velocities_input);

  // Evaluate the connected input.
  const BusValue& desired = (has_cartesian_velocities_input
                                 ? get_input_port_desired_cartesian_velocities()
                                 : get_input_port_desired_cartesian_poses())
                                .Eval<BusValue>(context);

  // Grab the plant context so that we can compute kinematics.
  const auto& plant_context = this->get_cache_entry(plant_context_cache_index_)
                                  .template Eval<Context<double>>(context);

  // Loop over all controlled frames requested on the input port.
  for (const auto&& [frame_name, abstract_value] : desired) {
    const Frame<double>* frame_i = &GetScopedFrameByName(plant(), frame_name);
    const RigidTransformd X_TGi =
        plant().CalcRelativeTransform(plant_context, *task_frame_, *frame_i);
    SpatialVelocity<double> Vd_TGi;
    if (has_cartesian_velocities_input) {
      Vd_TGi = abstract_value.template get_value<SpatialVelocity<double>>();
      // TODO(jeremy.nimmer): Per #lbm-platform slack thread, velocity limit
      // clamping should also happen for desired velocities; move the limit
      // code from the `else` block below to run outside (after) the if-else.
    } else {
      const auto& desired_pose =
          abstract_value.template get_value<RigidTransformd>();
      const Vector6d dX_TGi = ComputePoseDiffInCommonFrame(X_TGi, desired_pose);
      Vd_TGi.get_coeffs() = K_VX_ * dX_TGi / time_step_;
      const VectorXd& limit = Vd_TG_limit_.get_coeffs();
      for (int i = 0; i < 6; ++i) {
        Vd_TGi.get_coeffs()(i) =
            std::clamp(Vd_TGi.get_coeffs()(i), -limit(i), limit(i));
      }
    }
    cartesian_desires->frame_list.push_back(frame_i);
    cartesian_desires->X_TGlist.push_back(X_TGi);
    cartesian_desires->Vd_TGlist.push_back(Vd_TGi);
  }
}

void DifferentialInverseKinematicsSystem::CalcCommandedVelocity(
    const Context<double>& context, BasicVector<double>* output) const {
  // Get the multibody plant context and cartesian desires.
  const auto& plant_context = this->get_cache_entry(plant_context_cache_index_)
                                  .template Eval<Context<double>>(context);
  const auto& cartesian_desires =
      this->get_cache_entry(cartesian_desires_cache_index_)
          .template Eval<CartesianDesires>(context);
  const int num_desires = ssize(cartesian_desires.frame_list);

  MatrixXd Jv_TGs(kCartesianSpaceJacobianRows * num_desires,
                  active_dof_.count());
  int point_index = 0;

  const int v_dim = plant().num_velocities();
  for (int i = 0; i < num_desires; ++i) {
    MatrixXd jacobian_matrix(kCartesianSpaceJacobianRows, v_dim);
    plant().CalcJacobianSpatialVelocity(plant_context, JacobianWrtVariable::kV,
                                        *cartesian_desires.frame_list[i],
                                        Vector3d::Zero(), *task_frame_,
                                        *task_frame_, &jacobian_matrix);

    MatrixXd active_jacobian_matrix(kCartesianSpaceJacobianRows,
                                    active_dof_.count());
    active_dof_.GetColumnsFromMatrix(jacobian_matrix, &active_jacobian_matrix);
    Jv_TGs.block(kCartesianSpaceJacobianRows * point_index, 0,
                 kCartesianSpaceJacobianRows, active_dof_.count()) =
        active_jacobian_matrix;

    point_index += 1;
  }

  const VectorXd& nominal_posture =
      get_input_port_nominal_posture().Eval(context);

  MathematicalProgram prog;
  VectorXDecisionVariable v_next =
      prog.NewContinuousVariables(active_dof_.count(), "v_next");
  CallbackDetails details{
      .mathematical_program = prog,
      .v_next = v_next,
      .plant_context = plant_context,
      .collision_checker = *collision_checker_,
      .active_dof = active_dof_,
      .time_step = time_step_,
      .nominal_posture = nominal_posture,
      .frame_list = cartesian_desires.frame_list,
      .X_TGlist = cartesian_desires.X_TGlist,
      .Vd_TGlist = cartesian_desires.Vd_TGlist,
      .Jv_TGs = Jv_TGs,
  };
  recipe_->AddToProgram(&details);

  const VectorXd commanded_velocity = TrySolveQPAndFallbackToZero(prog, v_next);
  output->set_value(commanded_velocity);
}

std::vector<Binding<EvaluatorBase>>
DifferentialInverseKinematicsSystem::Recipe::AddToProgram(
    CallbackDetails* details) const {
  std::vector<Binding<EvaluatorBase>> result;
  for (const auto& ingredient : ingredients_) {
    std::vector<Binding<EvaluatorBase>> added =
        ingredient->AddToProgram(details);
    result.insert(result.end(), std::make_move_iterator(added.begin()),
                  std::make_move_iterator(added.end()));
  }
  return result;
}

}  // namespace operational_space_control
}  // namespace anzu
