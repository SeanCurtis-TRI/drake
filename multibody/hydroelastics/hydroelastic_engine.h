#pragma once

#include <vector>

#include "drake/geometry/query_object.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/geometry/scene_graph_inspector.h"

namespace drake {
namespace multibody {
namespace hydroelastics {
namespace internal {

/// The class that provides some of the encoding of the hydroelastic model.
/// It doesn't handle the geometric work or the integration work, but it
/// contains the logic for combining quantities (e.g., effective elastic
/// modulus). The hydroelastic model is described in:
/// [Elandt, 2019]  R. Elandt, E. Drumwright, M. Sherman, and A. Ruina.
/// A pressure field model for fast, robust approximation of net contact force
/// and moment between nominally rigid objects. Proc. IEEE/RSJ Intl. Conf. on
/// Intelligent Robots and Systems (IROS), 2019.
template <typename T>
class HydroelasticEngine  {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(HydroelasticEngine)

  HydroelasticEngine() = default;

  ~HydroelasticEngine() = default;

  /// Computes the combined elastic modulus for geometries with ids `id_A` and
  /// `id_B`.
  /// Refer to @ref mbp_hydroelastic_materials_properties "Hydroelastic model
  /// material properties" for further details.
  double CalcCombinedElasticModulus(
      geometry::GeometryId id_A, geometry::GeometryId id_B,
      const geometry::SceneGraphInspector<T>& inspector) const;

  /// Computes the combined Hunt & Crossley dissipation for geometries with ids
  /// `id_A` and `id_B`. Refer to @ref mbp_hydroelastic_materials_properties
  /// "Hydroelastic model material properties" for further details.
  double CalcCombinedDissipation(
      geometry::GeometryId id_A, geometry::GeometryId id_B,
      const geometry::SceneGraphInspector<T>& inspector) const;

  /// For a given state of `query_object`, this method computes the contact
  /// surfaces for all geometries in contact.
  /// @warning Unsupported geometries are ignored unless the broadphase pass
  /// detects that they are possibly in contact. In this case an exception is
  /// thrown.
  std::vector<geometry::ContactSurface<T>> ComputeContactSurfaces(
      const geometry::QueryObject<T>& query_object) const;
};

// Specialization to support compilation and linking with T =
// symbolic::Expression. Even though the Clang compiler does not need this, the
// GCC compiler does need it at linking time.
template <>
class HydroelasticEngine<symbolic::Expression> {
 public:
  using T = symbolic::Expression;

  std::vector<geometry::ContactSurface<T>> ComputeContactSurfaces(
      const geometry::QueryObject<T>&) const {
    Throw("ComputeContactSurfaces");
    return std::vector<geometry::ContactSurface<T>>();
  }

  // TODO(amcastro-tri): Update this list of methods if the API in
  // HydroelasticEngine grows.

 private:
  static void Throw(const char* operation_name) {
    throw std::logic_error(fmt::format(
        "Cannot call `{}` on a HydroelasticEngine<symbolic::Expression>",
        operation_name));
  }
};

}  // namespace internal
}  // namespace hydroelastics
}  // namespace multibody
}  // namespace drake
