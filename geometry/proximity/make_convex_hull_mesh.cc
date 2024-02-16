#include "drake/geometry/proximity/make_convex_hull_mesh.h"

#include <map>
#include <string>
#include <vector>

#include <fmt/format.h>
#include <libqhullcpp/Qhull.h>
#include <libqhullcpp/QhullFacet.h>
#include <libqhullcpp/QhullVertexSet.h>

#include "drake/common/find_resource.h"
#include "drake/common/fmt_eigen.h"
#include "drake/common/fmt_ostream.h"
#include "drake/common/ssize.h"
#include "drake/geometry/read_obj.h"

namespace drake {
namespace geometry {
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

// Simply reads the vertices from the mesh file referred to by either a Mesh or
// Convex shape. Supports getting the vertices for MakeConvexHull.
class ObjVertexReader final : public ShapeReifier {
 public:
  ObjVertexReader() = default;
  ~ObjVertexReader() final = default;

  const std::vector<Vector3d>& vertices() const { return vertices_; }

 private:
  using ShapeReifier::ImplementGeometry;

  void ImplementGeometry(const Mesh& mesh, void*) {
    ReadVertices(mesh.filename(), mesh.extension(), mesh.scale());
  }

  void ImplementGeometry(const Convex& convex, void*) {
    ReadVertices(convex.filename(), convex.extension(), convex.scale());
  }

  void ReadVertices(std::string_view filename, std::string_view extension,
                    double scale) {
    if (extension != ".obj") {
      throw std::runtime_error(fmt::format(
          "MakeConvexHull only applies to obj meshes; given file: {}\n",
          filename));
    }

    const auto [tinyobj_vertices, _1, _2] = geometry::internal::ReadObjFile(
        std::string(filename), scale, /* triangulate = */ false);
    vertices_ = std::move(*tinyobj_vertices);
  }

  std::vector<Vector3d> vertices_;
};

}  // namespace

PolygonSurfaceMesh<double> MakeConvexHull(const Shape& shape) {
  if (shape.type_name() != "Mesh" && shape.type_name() != "Convex") {
    throw std::runtime_error(fmt::format(
        "MakeConvexHull only applies to Mesh and Convex types, given {}.",
        shape.type_name()));
  }

  ObjVertexReader v_reader;
  shape.Reify(&v_reader);

  orgQhull::Qhull qhull;
  const int dim = 3;

  // This is a cheat that relies on the fact that a vector of Vector3d really
  // is just a vector<double> with three times as many entries. It *does*
  // eliminate a copy.
  qhull.runQhull("", dim, v_reader.vertices().size(),
                 reinterpret_cast<const double*>(v_reader.vertices().data()),
                 "");

  if (qhull.qhullStatus() != 0) {
    throw std::runtime_error(
        fmt::format("MakeConvexHull failed. Qhull terminated with status {} "
                    "and  message:\n{}",
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
  }

  std::vector<int> face_data;
  // Note: 4 * facet count will not generally be enough, but it's a safe
  // starting size.
  face_data.reserve(qhull.facetCount() * 4);
  for (auto& facet : qhull.facetList()) {
    auto qhull_vertices = facet.vertices().toStdVector();
    std::vector<int> mesh_indices;
    std::transform(qhull_vertices.cbegin(), qhull_vertices.cend(),
                   std::back_inserter(mesh_indices),
                   [&id_to_mesh_index](auto v) {
                     return id_to_mesh_index[v.id()];
                   });
    // QHull doesn't necessarily order the vertices in the winding we want.
    const Vector3d normal(facet.hyperplane().coordinates());
    std::vector<int> ordered_vertices =
        OrderPolyVertices(vertices_M, mesh_indices, normal);

    // Now populate the face data.
    face_data.push_back(ssize(ordered_vertices));
    face_data.insert(face_data.end(), ordered_vertices.begin(),
                     ordered_vertices.end());
  }
  return PolygonSurfaceMesh(std::move(face_data), std::move(vertices_M));
}

}  // namespace geometry
}  // namespace drake
