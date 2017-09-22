#include "drake/multibody/rigid_body_plant/compliant_material_parameters.h"

#include <gtest/gtest.h>

namespace drake {
namespace systems {
namespace {

// Utility function to create a "baked" version of a parameter set. The new
// instance reports the same values as the input, but no longer depends
// on or changes with the default values.
CompliantMaterialParameters bake(const CompliantMaterialParameters& input) {
  CompliantMaterialParameters parameters;
  parameters.set_stiffness(input.stiffness());
  parameters.set_dissipation(input.dissipation());
  parameters.set_friction(input.static_friction(),
                           input.dynamic_friction());

  EXPECT_EQ(parameters.stiffness(), input.stiffness());
  EXPECT_EQ(parameters.dissipation(), input.dissipation());
  EXPECT_EQ(parameters.static_friction(), input.static_friction());
  EXPECT_EQ(parameters.dynamic_friction(), input.dynamic_friction());
  return parameters;
}

// NOTE: this does *not* test the parsing functionality. See
// drake/multibody/parsers/test/compliant_parameters_parse_test.cc for the
// testing of *that* functionality.

// Test the setting of *valid* values for the various parameters. Confirm they
// change.
GTEST_TEST(CompliantMaterialParameters, SetValidValues) {
  CompliantMaterialParameters params;
  CompliantMaterialParameters original = bake(params);

  params.set_stiffness(params.stiffness() + 1);
  EXPECT_EQ(params.stiffness(), original.stiffness() + 1);
  params.set_dissipation(params.dissipation() + 1);
  EXPECT_EQ(params.dissipation(), original.dissipation() + 1);
  params.set_friction(params.static_friction() + 1,
                      params.dynamic_friction() + 1);
  EXPECT_EQ(params.static_friction(), original.static_friction() + 1);
  EXPECT_EQ(params.dynamic_friction(), original.dynamic_friction() + 1);
}

// Confirms exceptions are thrown for bad parameter values.
GTEST_TEST(CompliantMaterialParameters, SetInvalidValues) {
  CompliantMaterialParameters params;
  EXPECT_THROW(params.set_stiffness(0), std::runtime_error);
  EXPECT_THROW(params.set_stiffness(-1), std::runtime_error);
  EXPECT_THROW(params.set_dissipation(-1), std::runtime_error);
  EXPECT_THROW(params.set_friction(-1), std::runtime_error);
  EXPECT_THROW(params.set_friction(-1, -2), std::runtime_error);
  EXPECT_THROW(params.set_friction(1, -2), std::runtime_error);
  EXPECT_THROW(params.set_friction(-1, 2), std::runtime_error);
  EXPECT_THROW(params.set_friction(1, 2), std::runtime_error);
}

// Simply tests that the hard-coded default values haven't changed. This is
// obviously brittle, these literal values should appear in both the initializer
// list for each parameter as well as the doxygen for
// CompliantMaterialParameters.
GTEST_TEST(CompliantContactParamters, DefaultValues) {
  CompliantMaterialParameters defaults;
  EXPECT_EQ(defaults.stiffness(), 1e5) << "Have you changed the doxygen?";
  EXPECT_EQ(defaults.dissipation(), 0.32) << "Have you changed the doxygen?";
  EXPECT_EQ(defaults.static_friction(), 0.9) << "Have you changed the doxygen?";
  EXPECT_EQ(defaults.dynamic_friction(), 0.5)
            << "Have you changed the doxygen?";
}

}  // namespace
}  // namespace systems
}  // namespace drake
