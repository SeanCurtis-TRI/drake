#pragma once

#include "drake/common/drake_copyable.h"

namespace drake {
namespace systems {

/** The set of per-object compliant material parameters. The material properties
 include:
   - stiffness with units of pascals (i.e.,N/m²). This should be thought of as
     the Young's modulus of the material. The default value is that of a soft
     rubber: 1e8 pascals (a medium rubber).
   - dissipation with units of s/m. Its default value is 0.32, drawn from
     the Hunt-Crossly 1975 paper representing the dissipation for ivory.
   - coefficients of friction (static and dynamic). Unitless values with
     default values of 0.9 for the static coefficient of friction and 0.5 for
     the dynamic coefficient.
 See @ref drake_contacts for details on these parameters. */
class CompliantMaterialParameters {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CompliantMaterialParameters)

  /** Constructs instance with the documented, hard-coded default values. */
  CompliantMaterialParameters() = default;

  void set_stiffness(double value);
  double stiffness() const { return youngs_modulus_; }
  void set_dissipation(double value);
  double dissipation() const { return dissipation_; }

  /** Sets *both* coefficients of friction to the same value.
   @throws std::runtime_exception if the value is negative. */
  void set_friction(double value);

  /** Sets the two coefficients of friction. The `dynamic_friction` values must
   be less than or equal to the `static_friction`.
   @throws std::runtime_exception if values dynamic_friction > static_friction
                                  or if either value is negative. */
  void set_friction(double static_friction, double dynamic_friction);

  double static_friction() const { return static_friction_; }
  double dynamic_friction() const { return dynamic_friction_; }

 private:
  // Confirms two properties on the coefficient pair:
  //  1. Both values non-negative.
  //  2. static_friction >= dynamic_friction.
  // Throws std::runtime_error on failure of these tests.
  static void ThrowForBadFriction(double static_friction,
                                  double dynamic_friction);

  // NOTE: If *any* of these values get changed, the class doxygen should echo
  // the new values as well as modifying the corresponding unit test.
  // Stiffness
  double youngs_modulus_{1e8};
  // Dissipation in s/m from [Hunt 1975] -- dissipation for ivory.
  double dissipation_{0.32};
  double static_friction_{0.9};
  double dynamic_friction_{0.5};
};

}  // namespace systems
}  // namespace drake
