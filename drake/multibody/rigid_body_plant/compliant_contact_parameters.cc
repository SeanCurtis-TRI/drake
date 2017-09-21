#include "drake/multibody/rigid_body_plant/compliant_contact_parameters.h"

#include <stdexcept>
#include <string>

namespace drake {
namespace systems {

using std::to_string;
using std::runtime_error;

void CompliantContactParameters::set_stiffness(double value) {
  if (value <= 0) {
    throw runtime_error(
        "Stiffness value must be non-negative. Given " + to_string(value));
  }
  stiffness_ = value;
}

void CompliantContactParameters::set_dissipation(double value) {
  if (value < 0) {
    throw runtime_error(
        "Dissipation value must be non-negative. Given " + to_string(value));
  }
  dissipation_ = value;
}

void CompliantContactParameters::set_friction(double value) {
  ThrowForBadFriction(value, value);
  static_friction_ = dynamic_friction_ = value;
}

void CompliantContactParameters::set_friction(double static_friction,
                                              double dynamic_friction) {
  ThrowForBadFriction(static_friction, dynamic_friction);
  static_friction_ = static_friction;
  dynamic_friction_ = dynamic_friction;
}

void CompliantContactParameters::ThrowForBadFriction(double static_friction,
                                                     double dynamic_friction) {
  if (dynamic_friction < 0) {
    throw runtime_error("Given dynamic friction is negative: " +
                        to_string(dynamic_friction));
  }
  if (static_friction < 0) {
    throw runtime_error("Given static friction is negative: " +
                        to_string(dynamic_friction));
  }
  if (dynamic_friction > static_friction) {
    throw std::runtime_error("Given dynamic friction (" +
                             to_string(dynamic_friction) +
                             ") is greater than given static friction (" +
                             to_string(static_friction) +
                             "). Must be less or equal.");
  }
}

}  // namespace systems
}  // namespace drake
