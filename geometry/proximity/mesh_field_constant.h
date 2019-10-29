#pragma once

#include "drake/geometry/proximity/mesh_field.h"

namespace drake {
namespace geometry {

template <typename FieldValue, typename MeshType>
class MeshFieldConstant final : public MeshField<FieldValue, MeshType> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MeshFieldConstant)

  // TODO(DamrongGuoy): Consider passing a function to evaluate the field.
  /** Constructs a MeshFieldConstant.
   @param name    The name of the field variable.
   @param values  The field value at each element of the mesh.
   @param mesh    The mesh M to which this MeshField refers.
   @pre   The `mesh` is non-null, and the number of entries in `values` is the
          same as the number of elements in the mesh.
   */
  MeshFieldConstant(std::string name, std::vector<FieldValue>&& values,
  const MeshType* mesh)
  : MeshField<FieldValue, MeshType>(mesh),
  name_(std::move(name)), values_(std::move(values)) {
    DRAKE_DEMAND(static_cast<int>(values_.size()) ==
        this->mesh().num_elements());
  }

  FieldValue EvaluateAtVertex(typename MeshType::VertexIndex v) const final {
    throw std::logic_error(
        "Evaluating at a vertex for a constant mesh field is ill-defined");
  }

  FieldValue Evaluate(typename MeshType::ElementIndex e,
                      const typename MeshType::Barycentric&) const final {
    return values_[e];
  }

  FieldValue EvaluateCartesian(
      typename MeshType::ElementIndex e,
      const typename MeshType::Cartesian& p_MQ) const final {
    return values_[e];
  }

  const std::string& name() const { return name_; }
  const std::vector<FieldValue>& values() const { return values_; }
  std::vector<FieldValue>& mutable_values() { return values_; }

  // TODO(#12173): Consider NaN==NaN to be true in equality tests.
  // TODO(#12173): Support the VolumeMesh MeshType by implementing the Equal
  //               function in VolumeMesh.
  /** Checks to see whether the given MeshFieldLinear object is equal via deep
   exact comparison. The name of the objects are exempt from this comparison.
   NaNs are treated as not equal as per the IEEE standard.
   @note Using a MeshType of VolumeMesh is not yet supported.
   @param field The field for comparison.
   @returns `true` if the given field is equal.
   */
  bool Equal(const MeshFieldConstant<FieldValue, MeshType>& field) const {
    if (this == &field) return true;

    if (!this->mesh().Equal(field.mesh())) return false;

    const typename MeshType::Cartesian dummy;
    for (typename MeshType::ElementIndex i(0); i < this->mesh().num_elements();
         ++i) {
      if (this->Evaluate(i) != field.Evaluate(i, dummy))
        return false;
    }

    // All checks passed.
    return true;
  }

 private:
  // Clones MeshFieldLinear data under the assumption that the mesh
  // pointer is null.
  DRAKE_NODISCARD std::unique_ptr<MeshField<FieldValue, MeshType>>
  DoCloneWithNullMesh() const final {
    return std::make_unique<MeshFieldConstant>(*this);
  }
  std::string name_;
  // The field values are indexed in the same way as elements, i.e.,
  // values_[i] is the field value for the mesh elements_[i].
  std::vector<FieldValue> values_;
};

/**
 @tparam FieldValue  a valid Eigen scalar or vector of valid Eigen scalars for
                     the field value.
 @tparam T  a valid Eigen scalar for coordinates.
 */
template <typename FieldValue, typename T>
using SurfaceMeshFieldConstant = MeshFieldConstant<FieldValue, SurfaceMesh<T>>;

/**@}*/

}  // namespace geometry
}  // namespace drake