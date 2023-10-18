#pragma once

#include <optional>

#include <Eigen/Dense>

#include "drake/geometry/render/render_material.h"

namespace drake {
namespace geometry {
namespace internal {

/* The data representing a mesh with a single material. The triangle mesh is
 defined by `indices`. Row t represents a triangle by the triplet of vertex
 indices: tᵥ₀, tᵥ₁, tᵥ₂. The indices map into the rows of `positions`,
 `normals`, and `uvs`. I.e., for vertex index v, the position of that vertex is
 at `positions.row(v)`, its corresponding normal is at `normals.row(v)`, and its
 texture coordinates are at `uvs.row(v)`.

 For now, all vertex quantities (`positions`, `normals` and `uvs`) are
 guaranteed (as well as the `indices` data). In the future, `uvs` may become
 optional. */
struct RenderMesh {
  Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> positions;
  Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> normals;
  Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor> uvs;
  Eigen::Matrix<unsigned int, Eigen::Dynamic, 3, Eigen::RowMajor> indices;

  /* Indicates the degree that UV coordinates have been assigned to the mesh.
   Only UvState::kFull supports texture images. No matter what, the `uvs` matrix
   will be appropriately sized. But only for kFull will the values be
   meaningful. */
  UvState uv_state{UvState::kNone};

  /* The specification of the material associated with this mesh data.
   `material` may be undefined (`std::nullopt`). Why it is undefined depends on
   the origin of the `RenderMesh`. The consumer of the mesh is free to define
   the material as they see fit by defining an arbitrary bespoke material or
   using a utility like MakeDiffuseMaterial(). */
  std::optional<RenderMaterial> material;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
