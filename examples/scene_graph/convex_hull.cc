#include <map>
#include <string>
#include <vector>

#include <fmt/format.h>
#include <libqhullcpp/Qhull.h>
#include <libqhullcpp/QhullFacet.h>
#include <libqhullcpp/QhullVertexSet.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/fmt_eigen.h"
#include "drake/common/fmt_ostream.h"
#include "drake/common/ssize.h"
#include "drake/geometry/read_obj.h"

namespace drake {
namespace example {
namespace {

using Eigen::Vector3d;

/* Used for ordering polygon vertices associating an ordering "score" with
 the index associated with the scored vertex. See OrderPolyVertices(). */
struct VertexScore {
  double score;
  int index;
};

/* Orders the vertices of a convex, planar polygon into a right-handed winding
 w.r.t. the plane's normal `n`.

 The polygon vertices are drawn from the set of `vertices` and indicated by the
 indices in `v_indices`.

 @param vertices  The set of vertices including the polygon's vertices.
 @param v_indices The indices of the vertices that bound the polygon.
 @param n         The plane normal.

 @pre The polygon has at least three vertices.
 @pre The polygon is convex.
 @pre The vertices and normals are expressed in the same basis.
 @pre the indexed vertices are truly all planar. */
std::vector<int> OrderPolyVertices(const std::vector<Vector3d>& vertices,
                                   const std::vector<int>& v_indices,
                                   const Vector3d& n) {
  DRAKE_DEMAND(vertices.size() >= 3);
  DRAKE_DEMAND(v_indices.size() >= 3);

  /* We arbitrarily define v0 to be the first vertex in the polygon. We will
   then order the other vertices such that they wind around the normal starting
   from v0. */

  /* Define a reference direction from v0 to v1, v_01. For all other vertices,
   we'll determine how far to each side of v_01 they lie, sorting them from
   "right most" to "left most". This is the polygon ordering. */
  const Vector3d& p_M0 = vertices[v_indices[0]];
  const Vector3d& p_M1 = vertices[v_indices[1]];
  const Vector3d v_01 = (p_M1 - p_M0).normalized();
  /* Vertex 1 has a score of 0, by construction. */
  std::vector<VertexScore> scored_vertices{{0, v_indices[1]}};

  /* Now score the remaining vertices. */
  for (int i = 2; i < ssize(v_indices); ++i) {
    const Vector3d& p_MI = vertices[v_indices[i]];
    const Vector3d v_0I = (p_MI - p_M0).normalized();
    // Vertices to the "right" of v1 will get negative scores.
    scored_vertices.emplace_back(v_01.cross(v_0I).dot(n), v_indices[i]);
  }

  /* Sort by increasing score. */
  std::sort(scored_vertices.begin(), scored_vertices.end(),
            [](const VertexScore& a, const VertexScore& b) {
              return a.score < b.score;
            });

  /* Construct the ordered polygon. */
  std::vector<int> ordered{{v_indices[0]}};
  ordered.reserve(v_indices.size());
  for (const VertexScore& vs : scored_vertices) {
    ordered.push_back(vs.index);
  }

  return ordered;
}

int do_main() {
  const std::string filename =
      FindResourceOrThrow("drake/geometry/render/test/meshes/box.obj");

  const auto [tinyobj_vertices, _1, _2] =
      geometry::internal::ReadObjFile(filename, 1.0, /* triangulate = */ false);

  orgQhull::Qhull qhull;
  const int dim = 3;
#if 0
  // This copying is what you have to do if you don't like reinterpret casting.
  std::vector<double> tinyobj_vertices_flat(tinyobj_vertices->size() * dim);
  for (int i = 0; i < ssize(*tinyobj_vertices); ++i) {
    for (int j = 0; j < dim; ++j) {
      tinyobj_vertices_flat[dim * i + j] = (*tinyobj_vertices)[i](j);
    }
  }
  qhull.runQhull("", dim, tinyobj_vertices->size(),
                 tinyobj_vertices_flat.data(), "");
#else
  // This is a cheat that relies on the fact that a vector of Vector3d really
  // is just a vector<double> with three times as many entries. It *does*
  // eliminate a copy.
  qhull.runQhull("", dim, tinyobj_vertices->size(),
                 reinterpret_cast<double*>(tinyobj_vertices->data()), "");
#endif
  if (qhull.qhullStatus() != 0) {
    throw std::runtime_error(
        fmt::format("Qhull terminated with status {} and  message:\n{}",
                    qhull.qhullStatus(), qhull.qhullMessage()));
  }
  // A mapping from qhull vertex ids to indices in the final mesh (vertices_M).
  std::map<countT, int> id_to_mesh_index;
  std::vector<Vector3d> vertices_M;
  vertices_M.reserve(qhull.vertexCount());
  for (auto& vertex : qhull.vertexList()) {
    const int index = ssize(vertices_M);
    id_to_mesh_index[vertex.id()] = index;
    const auto* p_MV = vertex.point().coordinates();
    vertices_M.emplace_back(p_MV[0], p_MV[1], p_MV[2]);
    fmt::print("{}\n", fmt_eigen(vertices_M.back().transpose()));
  }

  std::vector<int> face_data;
  face_data.reserve(qhull.facetCount() * 3);
  for (auto& facet : qhull.facetList()) {
    std::vector<int> mesh_indices;
    auto vertices = facet.vertices().toStdVector();
    std::transform(vertices.cbegin(), vertices.cend(),
                   std::back_inserter(mesh_indices),
                   [&id_to_mesh_index](auto v) {
                     return id_to_mesh_index[v.id()];
                   });
    const Vector3d normal(facet.hyperplane().coordinates());
    fmt::print("QhulFacet has {} vertices:", vertices.size());
    for (const auto& vertex : vertices) {
      fmt::print(" {}({})", vertex.id(), id_to_mesh_index[vertex.id()]);
    }
    fmt::print("\n");
    std::vector<int> ordered_vertices =
        OrderPolyVertices(vertices_M, mesh_indices, normal);
    fmt::print("  Ordered vertices:");
    for (const auto& index : ordered_vertices) {
      fmt::print(" {}", index);
    }
    fmt::print("\n");
  }

  return 0;
}
}  // namespace
}  // namespace example
}  // namespace drake

int main() {
  return drake::example::do_main();
}
