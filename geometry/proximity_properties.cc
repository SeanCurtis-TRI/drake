#include "drake/geometry/proximity_properties.h"

namespace drake {
namespace geometry {
namespace internal {

const char* const kMaterialGroup = "material";
const char* const kElastic = "elastic_modulus";
const char* const kFriction = "coulomb_friction";
const char* const kHcDissipation = "hunt_crossley_dissipation";

const char* const kHydroGroup = "hydroelastic";
const char* const kRezHint = "resolution_hint";
const char* const kComplianceType = "compliance_type";
const char* const kThickness = "slab_thickness";

std::ostream& operator<<(std::ostream& out, const HydroelasticType& type) {
  switch (type) {
    case HydroelasticType::kUndefined:
      out << "undefined";
      break;
    case HydroelasticType::kRigid:
      out << "rigid";
      break;
    case HydroelasticType::kSoft:
      out << "soft";
      break;
    default:
      DRAKE_UNREACHABLE();
  }
  return out;
}

}  // namespace internal

void AddContactMaterial(
    const std::optional<double>& elastic_modulus,
    const std::optional<double>& dissipation,
    const std::optional<multibody::CoulombFriction<double>>& friction,
    ProximityProperties* properties) {
  if (elastic_modulus.has_value()) {
    if (*elastic_modulus <= 0) {
      throw std::logic_error(fmt::format(
          "The elastic modulus must be positive; given {}", *elastic_modulus));
    }
    properties->AddProperty(internal::kMaterialGroup, internal::kElastic,
                            *elastic_modulus);
  }
  if (dissipation.has_value()) {
    if (*dissipation < 0) {
      throw std::logic_error(fmt::format(
          "The dissipation can't be negative; given {}", *dissipation));
    }
    properties->AddProperty(internal::kMaterialGroup, internal::kHcDissipation,
                            *dissipation);
  }

  if (friction.has_value()) {
    properties->AddProperty(internal::kMaterialGroup, internal::kFriction,
                            *friction);
  }
}

void AddResolutionHint(double resolution_hint,
                       ProximityProperties* properties) {
  DRAKE_DEMAND(properties);
  if (resolution_hint <= 0.0) {
    throw std::logic_error(
        fmt::format("The resolution hint must be greater than zero; given {}",
                    resolution_hint));
  }

  properties->AddProperty(internal::kHydroGroup, internal::kRezHint,
                          resolution_hint);
}

void AddRigidHydroelasticProperties(
    const std::optional<double>& resolution_hint,
    ProximityProperties* properties) {
  DRAKE_DEMAND(properties);
  if (resolution_hint.has_value()) {
    AddResolutionHint(*resolution_hint, properties);
  }
  AddRigidHydroelasticProperties(properties);
}

void AddRigidHydroelasticProperties(ProximityProperties* properties) {
  DRAKE_DEMAND(properties);
  // The bare minimum of defining a rigid geometry is to declare its compliance
  // type. Downstream consumers (ProximityEngine) will determine if this is
  // sufficient.
  properties->AddProperty(internal::kHydroGroup, internal::kComplianceType,
                          internal::HydroelasticType::kRigid);
}

void AddSlabThickness(double slab_thickness, ProximityProperties* properties) {
  DRAKE_DEMAND(properties);
  if (slab_thickness <= 0.0) {
    throw std::logic_error(fmt::format(
        "The slab thickness value must be greater than zero; given {}",
        slab_thickness));
  }

  properties->AddProperty(internal::kHydroGroup, internal::kThickness,
                          slab_thickness);
}

void AddSoftHydroelasticProperties(const std::optional<double> resolution_hint,
                                   const std::optional<double>& slab_thickness,
                                   ProximityProperties* properties) {
  DRAKE_DEMAND(properties);
  if (resolution_hint.has_value()) {
    AddResolutionHint(*resolution_hint, properties);
  }
  if (slab_thickness.has_value()) {
    AddSlabThickness(*slab_thickness, properties);
  }
  AddSoftHydroelasticProperties(properties);
}

void AddSoftHydroelasticProperties(double resolution_hint,
                                   ProximityProperties* properties) {
  DRAKE_DEMAND(properties);
  AddResolutionHint(resolution_hint, properties);
  AddSoftHydroelasticProperties(properties);
}

void AddSoftHydroelasticProperties(ProximityProperties* properties) {
  DRAKE_DEMAND(properties);
  // The bare minimum of defining a soft geometry is to declare its compliance
  // type. Downstream consumers (ProximityEngine) will determine if this is
  // sufficient.
  properties->AddProperty(internal::kHydroGroup, internal::kComplianceType,
                          internal::HydroelasticType::kSoft);
}

void AddSoftHydroelasticPropertiesForHalfSpace(
    double slab_thickness, ProximityProperties* properties) {
  DRAKE_DEMAND(properties);
  AddSlabThickness(slab_thickness, properties);
  AddSoftHydroelasticProperties(properties);
}

}  // namespace geometry
}  // namespace drake
