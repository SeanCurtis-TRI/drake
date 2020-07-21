#include "drake/multibody/parsing/detail_common.h"

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace internal {
namespace {

using geometry::GeometryProperties;
using geometry::PropertyName;
using geometry::ProximityProperties;
using geometry::internal::HydroelasticType;
using std::optional;

using ReadDoubleFunc = std::function<optional<double>(const char*)>;
const bool rigid{true};
const bool soft{true};

// A read_double implementation that always returns nullopt.
optional<double> empty_read_double(const char*) { return {}; }

// Creates a read_double implementation that returns nullopt for every tag name
// expect for the given name (which returns the given value).
ReadDoubleFunc param_read_double(
    const std::string& tag, double value) {
  return [&tag, value](const char* name) -> optional<double> {
    return (tag == name) ? optional<double>(value) : std::nullopt;
  };
}

// Tests for a particular value in the given properties.
::testing::AssertionResult ExpectScalar(const PropertyName& property,
                                        double expected,
                                        const ProximityProperties& p) {
  ::testing::AssertionResult failure = ::testing::AssertionFailure();
  const bool has_value = p.HasProperty(property);
  if (!has_value) {
    return failure << "Expected the " << property << " property; not found";
  }
  const double value = p.Get<double>(property);
  if (value != expected) {
    return failure << "Wrong value for the " << property << "property:"
                   << "\n  Expected: " << expected << "\n  Found: " << value;
  }
  return ::testing::AssertionSuccess();
}

// Confirms that an "empty" <drake:proximity_properties> tag produces an empty
// instance of ProximityProperties.
GTEST_TEST(ParseProximityPropertiesTest, NoProperties) {
  ProximityProperties properties =
      ParseProximityProperties(empty_read_double, !rigid, !soft);
  // It is empty if there is a single group: the default group with no
  // properties.
  ASSERT_EQ(properties.num_groups(), 1);
  ASSERT_TRUE(properties.HasGroup(GeometryProperties::default_group_name()));
  ASSERT_EQ(
      properties.GetPropertiesInGroup(GeometryProperties::default_group_name())
          .size(),
      0u);
}

// Confirms successful parsing of hydroelastic properties.
GTEST_TEST(ParseProximityPropertiesTest, HydroelasticProperties) {
  const char* kTag = "drake:mesh_resolution_hint";
  const double kRezHintValue{0.25};

  auto expect_compliance =
      [](bool is_rigid, bool is_soft,
         const ProximityProperties& p) -> ::testing::AssertionResult {
    DRAKE_DEMAND(!(is_rigid && is_soft));
    ::testing::AssertionResult failure = ::testing::AssertionFailure();
    const bool has_compliance_type =
        p.HasProperty(p.hydroelastic_compliance_type());
    if (is_rigid || is_soft) {
      if (!has_compliance_type) {
        return failure << "Expected compliance; found none";
      }
      auto compliance =
          p.Get<HydroelasticType>(p.hydroelastic_compliance_type());
      if (is_rigid && compliance != HydroelasticType::kRigid) {
        return failure << "Expected rigid compliance; found " << compliance;
      } else if (is_soft && compliance != HydroelasticType::kSoft) {
        return failure << "Expected soft compliance; found " << compliance;
      }
    } else {
      if (has_compliance_type) {
        return failure << "Expected no compliance; compliance found";
      }
    }
    return ::testing::AssertionSuccess();
  };

  const PropertyName& rez_hint_prop =
      ProximityProperties::hydroelastic_resolution_hint();
  // Case: Declared rigid without a resolution hint.
  {
    ProximityProperties properties =
        ParseProximityProperties(empty_read_double, rigid, !soft);
    EXPECT_TRUE(expect_compliance(rigid, !soft, properties));
    // Compliance is the only property.
    EXPECT_EQ(properties.GetPropertiesInGroup(rez_hint_prop.group()).size(),
              1u);
  }

  // Case: Declared rigid with a resolution hint.
  {
    ProximityProperties properties = ParseProximityProperties(
        param_read_double(kTag, kRezHintValue), rigid, !soft);
    EXPECT_TRUE(expect_compliance(rigid, !soft, properties));
    // Should have compliance and resolution.
    EXPECT_EQ(properties.GetPropertiesInGroup(rez_hint_prop.group()).size(),
              2u);
    EXPECT_TRUE(ExpectScalar(rez_hint_prop, kRezHintValue, properties));
  }

  // Case: Declared soft without a resolution hint.
  {
    ProximityProperties properties =
        ParseProximityProperties(empty_read_double, !rigid, soft);
    EXPECT_TRUE(expect_compliance(!rigid, soft, properties));
    // Compliance is the only property.
    EXPECT_EQ(properties.GetPropertiesInGroup(rez_hint_prop.group()).size(),
              1u);
  }

  // Case: Declared soft with a resolution hint.
  {
    ProximityProperties properties = ParseProximityProperties(
        param_read_double(kTag, kRezHintValue), !rigid, soft);
    EXPECT_TRUE(expect_compliance(!rigid, soft, properties));
    // Should have compliance and resolution.
    EXPECT_EQ(properties.GetPropertiesInGroup(rez_hint_prop.group()).size(),
              2u);
    EXPECT_TRUE(
        ExpectScalar(rez_hint_prop, kRezHintValue, properties));
  }

  // Case: Resolution without any hydroelastic declaration.
  {
    ProximityProperties properties = ParseProximityProperties(
        param_read_double(kTag, kRezHintValue), !rigid, !soft);
    EXPECT_TRUE(expect_compliance(!rigid, !soft, properties));
    // Resolution should be the only property.
    EXPECT_EQ(properties.GetPropertiesInGroup(rez_hint_prop.group()).size(),
              1u);
    EXPECT_TRUE(
        ExpectScalar(rez_hint_prop, kRezHintValue, properties));
  }
}

// Confirms successful parsing of elastic modulus.
GTEST_TEST(ParseProximityPropertiesTest, ElasticModulus) {
  const double kValue = 1.75;
  ProximityProperties properties = ParseProximityProperties(
      param_read_double("drake:elastic_modulus", kValue), !rigid, !soft);
  const PropertyName& property = properties.material_elastic_modulus();
  EXPECT_TRUE(ExpectScalar(property, kValue, properties));
  // Elastic modulus is the only property.
  EXPECT_EQ(properties.GetPropertiesInGroup(property.group()).size(), 1u);
  EXPECT_EQ(properties.num_groups(), 2);  // Material and default groups.
}

// Confirms successful parsing of dissipation.
GTEST_TEST(ParseProximityPropertiesTest, Dissipation) {
  const double kValue = 1.25;
  ProximityProperties properties = ParseProximityProperties(
      param_read_double("drake:hunt_crossley_dissipation", kValue), !rigid,
      !soft);
  const PropertyName& property =
      properties.material_hunt_crossley_dissipation();
  EXPECT_TRUE(ExpectScalar(property, kValue, properties));
  // Dissipation is the only property.
  EXPECT_EQ(properties.GetPropertiesInGroup(property.group()).size(), 1u);
  EXPECT_EQ(properties.num_groups(), 2);  // Material and default groups.
}

// Confirms successful parsing of stiffness.
GTEST_TEST(ParseProximityPropertiesTest, Stiffness) {
  const double kValue = 300.0;
  ProximityProperties properties = ParseProximityProperties(
      param_read_double("drake:point_contact_stiffness", kValue), !rigid,
      !soft);
  const PropertyName& property = properties.material_point_contact_stiffness();
  EXPECT_TRUE(ExpectScalar(property, kValue, properties));
  // Stiffness is the only property.
  EXPECT_EQ(properties.GetPropertiesInGroup(property.group()).size(), 1u);
  EXPECT_EQ(properties.num_groups(), 2);  // Material and default groups.
}

// Confirms successful parsing of friction.
GTEST_TEST(ParseProximityPropertiesTest, Friction) {
  // We're not testing the case where *no* coefficients are provided; that's
  // covered in the NoProperties test.
  auto friction_read_double = [](optional<double> mu_d,
      optional<double> mu_s) -> ReadDoubleFunc {
    return [mu_d, mu_s](const char* name) -> optional<double> {
      optional<double> result;
      if (mu_d.has_value() && std::string("drake:mu_dynamic") == name) {
        result = *mu_d;
      } else if (mu_s.has_value() && std::string("drake:mu_static") == name) {
        result = *mu_s;
      }
      return result;
    };
  };

  auto expect_friction =
      [](double mu_d, double mu_s,
         const ProximityProperties& p) -> ::testing::AssertionResult {
    ::testing::AssertionResult failure = ::testing::AssertionFailure();
    const PropertyName& property = p.material_coulomb_friction();
    const bool has_value = p.HasProperty(property);
    if (!has_value) {
      return failure << "Expected " << property << "; not found";
    }
    const auto& friction = p.Get<CoulombFriction<double>>(property);
    if (friction.dynamic_friction() != mu_d ||
        friction.static_friction() != mu_s) {
      return failure << "Wrong value for " << property << ":"
                     << "\n  Expected mu_d: " << mu_d << ", mu_s: " << mu_s
                     << "\n  Found mu_d " << friction.dynamic_friction()
                     << ", mu_s: " << friction.static_friction();
    }
    return ::testing::AssertionSuccess();
  };

  const PropertyName& property =
      ProximityProperties::material_coulomb_friction();
  // Case: Only dynamic -- both coefficients match dynamic coefficient.
  {
    const double kValue = 1.25;
    ProximityProperties properties = ParseProximityProperties(
        friction_read_double(kValue, {}), !rigid,
        !soft);
    EXPECT_TRUE(expect_friction(kValue, kValue, properties));
    // Friction is the only property.
    EXPECT_EQ(properties.GetPropertiesInGroup(property.group()).size(), 1u);
    EXPECT_EQ(properties.num_groups(), 2);  // Material and default groups.
  }

  // Case: Only static -- both coefficients match static coefficient.
  {
    const double kValue = 1.5;
    ProximityProperties properties = ParseProximityProperties(
        friction_read_double({}, kValue), !rigid,
        !soft);
    EXPECT_TRUE(expect_friction(kValue, kValue, properties));
    // Friction is the only property.
    EXPECT_EQ(properties.GetPropertiesInGroup(property.group()).size(), 1u);
    EXPECT_EQ(properties.num_groups(), 2);  // Material and default groups.
  }

  // Case: Both defined -- resulting coefficients match.
  {
    const double kMuD = 1.5;
    const double kMuS = 2.25;
    ProximityProperties properties = ParseProximityProperties(
        friction_read_double(kMuD, kMuS), !rigid,
        !soft);
    EXPECT_TRUE(expect_friction(kMuD, kMuS, properties));
    // Friction is the only property.
    EXPECT_EQ(properties.GetPropertiesInGroup(property.group()).size(), 1u);
    EXPECT_EQ(properties.num_groups(), 2);  // Material and default groups.
  }
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
