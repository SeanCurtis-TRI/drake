#pragma once

#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "drake/common/fmt.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/hydroelastic_mesh_stats.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/contact_solvers/icf/icf_builder.h"
#include "drake/multibody/contact_solvers/icf/icf_external_systems_linearizer.h"
#include "drake/multibody/contact_solvers/icf/icf_model.h"
#include "drake/multibody/contact_solvers/icf/icf_solver.h"
#include "drake/multibody/contact_solvers/icf/icf_solver_parameters.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/integrator_base.h"

namespace drake {
namespace multibody {
namespace internal {
/* A sequence of subsystem indices that allows finding an arbitrary
nested-diagram asset from the root. */
using SubsystemPath = std::vector<systems::SubsystemIndex>;

/* Facts of nested diagram structure used at run-time by CenicIntegrator. */
template <typename T>
struct CenicDiagramStructure {
  /* The plant used as the basis of the convex optimization problem. */
  const MultibodyPlant<T>* plant{};
  /* The path to the plant from the root system. An empty path means the root
  system *is* the plant. */
  SubsystemPath plant_path;
  /* Paths to subsystems, other than the targeted plant, that have continuous
  state. */
  std::vector<SubsystemPath> non_plant_xc_paths;
};
}  // namespace internal

/* Per-step statistics collected when IcfSolverParameters::collect_heavy_stats
is enabled. Preserved from the thin-objects / barrier CENIC research branch; the
rolling-sphere and related plotting scripts consume this record. This is
supplementary to the formal statistics reported by DoGetStatisticsSummary(). */
struct CenicStepStatistics {
  std::string step_type;
  double time;
  double step_size;
  int num_solver_iterations;
  int total_linesearch_iterations;
  int max_linesearch_iterations;
  double mean_linesearch_iterations;
  double max_condition_number;
  double last_condition_number;
  double max_e0;
  double mean_e0;
  int total_num_constraint_pairs;

  std::string to_string() const {
    return fmt::format("{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}",
                       step_type, time, step_size, num_solver_iterations,
                       total_linesearch_iterations, max_linesearch_iterations,
                       mean_linesearch_iterations, max_condition_number,
                       last_condition_number, max_e0, mean_e0,
                       total_num_constraint_pairs);
  }
};

// TODO(#23767): Consider applying SIMD optimizations to integrator hot spots,
// under benchmarking and profiling guidance.

/** Convex Error-controlled Numerical Integration for Contact (CENIC) is a
specialized error-controlled implicit integrator for contact-rich robotics
simulations [Kurtz and Castro, 2025].

CENIC provides variable-step error-controlled integration for multibody systems
with stiff contact interactions, while maintaining the high speeds
characteristic of discrete-time solvers required for modern robotics workflows.

Benefits of CENIC include:

- **Guaranteed convergence**. Unlike traditional implicit integrators that rely
  on non-convex Newton-Raphson solves, CENIC's convex formulation eliminates
  step rejections due to convergence failures.

- **Guaranteed accuracy**. CENIC inherits the well-studied accuracy guarantees
  associated with error-controlled integration [Hairer and Wanner, 1996],
  avoiding discretization artifacts common in fixed-step discrete-time methods.

- **Automatic time step selection**. Users specify a desired accuracy rather
  than a fixed time step, eliminating a common pain point in authoring multibody
  simulations.

- **Implicit treatment of external systems**. This means that users can connect
  arbitrary stiff controllers (e.g., a custom `LeafSystem`) to the
  `MultibodyPlant` and have them treated implicitly in CENIC's convex
  formulation. This allows for larger time steps, leading to faster and more
  stable simulations.

- **Principled static/dynamic friction modeling**. Unlike discrete solvers,
  CENIC can simulate frictional contact with different static and dynamic
  friction coefficients.

- **Speed**. CENIC consistently outperforms general-purpose integrators by
  orders of magnitude on contact-rich problems. Error-controlled CENIC is often
  (but not always) faster than discrete-time simulation, depending on the
  simulation in question and the requested accuracy.

CENIC works by solving a convex Irrotational Contact Fields (ICF) optimization
problem [Castro et al., 2023] to advance the system state at each time step. A
simple half-stepping strategy provides a second-order error estimate for
automatic step-size selection.

Because CENIC is specific to multibody systems, the system it's asked to
integrate must either be a MultibodyPlant system or a Diagram that contains a
MultibodyPlant subsystem, at any level of Diagram nesting.

Running CENIC in fixed-step mode (with error-control disabled) recovers the
"Lagged" variant of discrete-time ICF simulation from [Castro et al., 2023].

This branch additionally supports a non-penetration ("thin objects" / barrier)
contact model: when error control is enabled, each candidate full and half step
is checked for a feasible (penetration-free) linear trajectory via a
continuous-collision-detection (CCD) query, and infeasible steps are rejected
and the step size shrunk (optionally to a computed time-of-impact; see
IcfSolverParameters::use_toi). This treatment is currently rigid-bodies-only.

Implementation notes:

@warning CENIC's error control implementation is not sensitive to continuous
         state of systems other than the plant(). See issue #23921.

@warning When asked to perform 0-sized integration steps, CENIC executes a
         special case that does no integration or state updates, but does reset
         the error estimate to all 0, and always succeeds. This is in contrast
         to other integrator implementations in Drake. See DoStep() in the
         implementation.

References:

  [Castro et al., 2023] Castro A., Han X., and Masterjohn J., 2023. Irrotational
  Contact Fields. https://arxiv.org/abs/2312.03908.

  [Hairer and Wanner, 1996] Hairer E. and Wanner G., 1996. Solving Ordinary
  Differential Equations II: Stiff and Differential-Algebraic Problems. Springer
  Series in Computational Mathematics, Vol. 14. Springer-Verlag, Berlin, 2nd
  edition.

  [Kurtz and Castro, 2025] Kurtz V. and Castro A., 2025. CENIC: Convex
  Error-controlled Numerical Integration for Contact.
  https://arxiv.org/abs/2511.08771.

@tparam_nonsymbolic_scalar */
template <class T>
class CenicIntegrator final : public systems::IntegratorBase<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CenicIntegrator);

  /** This target accuracy is established in the constructor, but may be
  changed by @ref integrator-accuracy methods. CENIC works best at loose
  accuracy. */
  static constexpr double kDefaultAccuracy = 1e-3;

  /** Constructs the integrator.
  @param system The overall system to simulate. Must either be a MultibodyPlant
                or a Diagram that contains exactly one continuous-time
                MultibodyPlant. Other (discrete-time) plants are allowed in a
                diagram. This `system` is aliased by this object so must remain
                alive longer than the integrator.
  @param context context for the overall system.  */
  explicit CenicIntegrator(const systems::System<T>& system,
                           systems::Context<T>* context = nullptr);

  ~CenicIntegrator() final;

  /** Gets a reference to the MultibodyPlant used to formulate the convex
  optimization problem. */
  const MultibodyPlant<T>& plant() const { return *structure_.plant; }

  /** Gets the current convex solver tolerances and iteration limits. */
  const contact_solvers::icf::IcfSolverParameters& get_solver_parameters()
      const {
    return solver_.get_parameters();
  }

  /** Sets the convex solver tolerances and iteration limits. */
  void SetSolverParameters(
      const contact_solvers::icf::IcfSolverParameters& parameters);

  /** Gets the current total number of solver iterations across all time steps.
   */
  int get_total_solver_iterations() const {
    return stats_.total_solver_iterations;
  }

  /** Gets the current total number of linesearch iterations, across all time
  steps and solver iterations. */
  int get_total_ls_iterations() const { return stats_.total_ls_iterations; }

  /** Gets the current total number of Hessian factorizations performed, across
  all time steps and solver iterations. */
  int get_total_hessian_factorizations() const {
    return stats_.total_hessian_factorizations;
  }

  /** Gets the per-step statistics collected when
  IcfSolverParameters::collect_heavy_stats is enabled. */
  const std::vector<CenicStepStatistics>& get_step_statistics() const {
    return step_statistics_;
  }

  bool supports_error_estimation() const final;

  int get_error_estimate_order() const final;

  /** When the barrier/CCD non-penetration model rejects a step, this returns
  the computed time-of-impact-based step size (if IcfSolverParameters::use_toi
  is set), otherwise bisects the step. */
  T ComputeAdjustedStepSize(const T& h) const final {
    if (this->get_solver_parameters().use_toi &&
        time_of_impact_ < std::numeric_limits<T>::infinity()) {
      return time_of_impact_;
    }
    // Use a subdivision factor of 0.5 for halving the step size on failure.
    return 0.5 * h;
  }

 private:
  /* Preallocated scratch space. */
  struct Scratch {
    /* Resizes scratch space to accommodate the given plant and root system. */
    void Resize(const MultibodyPlant<T>& plant,
                const systems::System<T>& system);

    /* State-sized variables, x = [q; v] for the plant only. When the root
    system is a Diagram, any non-plant continuous state will not be stored
    here. */
    VectorX<T> v_guess;
    VectorX<T> q;

    /* Linearized external system gains (sized to the plant's num_velocities):
    Torque-limited actuation, τᵤ(v) ≈ clamp(-Kᵤ⋅v + bᵤ, e). */
    contact_solvers::icf::internal::IcfLinearFeedbackGains<T>
        actuation_feedback_storage;
    /* Non-limited external forces, τₑ(v) ≈ −Kₑ⋅v + bₑ. */
    contact_solvers::icf::internal::IcfLinearFeedbackGains<T>
        external_feedback_storage;

    /* Intermediate states for error control, which compares a single large
    step (x_next_full_) to the result of two smaller steps (x_next_half_2_).
    These states include continuous state for the entire root system. */
    /* x_{t+h}. */
    std::unique_ptr<systems::ContinuousState<T>> x_next_full;
    /* x_{t+h/2}. */
    std::unique_ptr<systems::ContinuousState<T>> x_next_half_1;
    /* x_{t+h/2+h/2}. */
    std::unique_ptr<systems::ContinuousState<T>> x_next_half_2;
    /* x_{t}, snapshot used to restore state when the barrier/CCD model rejects
    a step. */
    std::unique_ptr<systems::ContinuousState<T>> x_prev;

    /* Trajectory-start poses for the barrier/CCD feasibility checks,
    restricted to the CCD participant geometries (design-doc X6: previously
    every check harvested a full copy of *all* geometry poses). Values are
    overwritten in place each (sub)step; the key set only grows. */
    std::unordered_map<geometry::GeometryId, math::RigidTransform<T>>
        X_WGs_ccd_prev;
  };

  /* Data for PrintSimulatorStatistics(). */
  struct Stats {
    int total_solver_iterations{0};
    int total_hessian_factorizations{0};
    int total_ls_iterations{0};
    /* Rejections issued by the barrier/CCD feasibility check, per check site
    in DoStep(). Disambiguates CCD rejections from error-control shrinkages in
    the collected data. */
    int num_feasibility_rejections_full{0};
    int num_feasibility_rejections_half1{0};
    int num_feasibility_rejections_half2{0};
    /* Total number of calls, INCLUDING calls made on behalf of steps that
    were later rejected (by the feasibility check or by error control):
    convex solves = IcfSolver::SolveWithGuess invocations; feasibility calls =
    barrier/CCD IsFeasibleTrajectory checks (the rejection counters above are
    the failing subset). Geometry-query counts/time live in IcfBuilder and are
    reported alongside these in the statistics summary. */
    int64_t num_convex_solves{0};
    int64_t num_feasibility_calls{0};
    /* Accumulated wall-clock runtime breakdown of DoStep() [seconds]:
    model_update = IcfBuilder::UpdateModel (geometry queries + constraint
    assembly); solve = convex solves (ComputeNextContinuousState); feasibility
    = CCD feasibility checks incl. broadphase and pose harvesting; linearize =
    external-system linearization. */
    double time_model_update{0.0};
    double time_solve{0.0};
    double time_feasibility{0.0};
    double time_linearize{0.0};
  };

  void DoResetStatistics() final;

  std::vector<systems::NamedStatistic> DoGetStatisticsSummary() const final;

  T CalcStateChangeNorm(
      const systems::ContinuousState<T>& dx_state) const final;

  void DoInitialize() final;

  /* @warning For `h` == 0, CENIC DoStep() executes a special case that does no
  integration or state updates, but does reset the error estimate to all 0,
  and always succeeds.

  See IntegratorBase::DoStep() for full implementation requirements. */
  bool DoStep(const T& h) final;

  /* Solves the ICF problem to compute x_{t+h}.

  @param model The ICF model for the convex problem min_v ℓ(v; q₀, v₀, h).
  @param v_guess The initial guess for the MbP plant velocities.
  @param[out] x_next The output continuous state, includes state for both the
                     plant and any external systems. */
  void ComputeNextContinuousState(
      const contact_solvers::icf::internal::IcfModel<T>& model,
      const VectorX<T>& v_guess, systems::ContinuousState<T>* x_next);

  /* Advances the plant's generalized positions, q = q₀ + h N(q₀) v, taking care
  to handle quaternion DoFs properly.

  @param h The time step size.
  @param v The next-step generalized velocities v.
  @param[out] q The output next-step generalized positions q.

  N.B. q₀ is stored in this->get_context(). */
  void AdvancePlantConfiguration(const T& h, const VectorX<T>& v,
                                 VectorX<T>* q) const;

  /* Overwrites `out` with the current world poses of the CCD participant
  geometries only (evaluated from the plant's geometry query input port at the
  current context state). The participant id set is discovered on first use
  and cached. */
  void SnapshotCcdPoses(
      std::unordered_map<geometry::GeometryId, math::RigidTransform<T>>* out);

  /* Returns true if the trajectory from geometry poses X_WGs_prev to the
  *current* context poses is feasible (penetration-free). When
  IcfSolverParameters::use_toi is set, an infeasible result additionally
  records the fractional time-of-impact in time_of_impact_factor_. */
  bool IsFeasibleTrajectory(
      const std::unordered_map<geometry::GeometryId, math::RigidTransform<T>>&
          X_WGs_prev);

  /* Appends a CenicStepStatistics record for the given (sub)step to
  step_statistics_. Only called when collect_heavy_stats is enabled. */
  void LogStepStatistics(
      const T& t, const T& h, const std::string& step_type,
      const contact_solvers::icf::internal::IcfModel<T>& model);

  /* Locations of plant and non-plant continuous state. Note that the contained
  `plant` pointer is guaranteed to be non-null by the CenicIntegrator
  constructor .*/
  const internal::CenicDiagramStructure<T> structure_;

  /* Helper class that linearizes torques dτ/dv from plant input ports. */
  const contact_solvers::icf::internal::IcfExternalSystemsLinearizer<T>
      external_systems_linearizer_;

  /* ICF integrator state and storage. */
  std::unique_ptr<contact_solvers::icf::internal::IcfBuilder<T>> builder_;
  contact_solvers::icf::internal::IcfSolver solver_;
  /* For the full step and first half-step. */
  contact_solvers::icf::internal::IcfModel<T> model_at_x0_;
  /* For the second half-step (at t + h/2). */
  contact_solvers::icf::internal::IcfModel<T> model_at_xh_;
  /* Reduced-problem data for joint locking. */
  contact_solvers::icf::internal::IcfModel<T> reduced_model_;
  contact_solvers::icf::internal::ReducedMapping mapping_;
  /* Data used with any/all of the above models. */
  contact_solvers::icf::internal::IcfData<T> data_;

  /* Track whether solves are initialized at the same time as a previous
  rejected step, to enable model (e.g., constraints, geometry) reuse. */
  T time_at_last_solve_{NAN};

  /* Preallocated scratch space for intermediate calculations. */
  Scratch scratch_;

  /* Data for PrintSimulatorStatistics(). */
  Stats stats_;

  /* Per-step statistics for the research plotting pipeline (opt-in via
  IcfSolverParameters::collect_heavy_stats). */
  std::vector<CenicStepStatistics> step_statistics_;

  /* Barrier/CCD non-penetration bookkeeping. time_of_impact_factor_ is the
  fractional time-of-impact in [0, 1] within the most recently rejected
  (sub)step; time_of_impact_ is the resulting absolute step size fed back to
  ComputeAdjustedStepSize(). Both are reset to infinity at the start of every
  DoStep() (design-doc X5: values no longer leak across steps). */
  T time_of_impact_factor_{std::numeric_limits<T>::infinity()};
  T time_of_impact_{std::numeric_limits<T>::infinity()};

  /* Cached ids of the CCD participant geometries (see SnapshotCcdPoses()).
  Discovered once on first use; geometry added after the first step with a
  connected query port is not picked up. */
  std::vector<geometry::GeometryId> ccd_participants_;
  bool ccd_participants_initialized_{false};

  /* Scene mesh-size statistics (surface triangles / tetrahedra over all
  hydroelastic geometries), computed lazily on the first DoStep with a
  connected geometry query port and reported by DoGetStatisticsSummary(). */
  geometry::internal::HydroelasticMeshStats scene_mesh_stats_;
  bool scene_mesh_stats_initialized_{false};
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::multibody::CenicIntegrator);
