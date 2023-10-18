#include "drake/geometry/render/render_mesh_functions.h"

#include "drake/common/drake_assert.h"
#include "drake/geometry/render/internal_obj_render_parser.h"
// #include "drake/geometry/render/internal_gltf_parser.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {

using drake::internal::DiagnosticPolicy;
using Eigen::Vector3d;
using std::map;
using std::pair;
using std::string;
using std::vector;

vector<RenderMesh> LoadRenderMeshesFromObj(
    const std::filesystem::path& obj_path, const GeometryProperties& properties,
    const std::optional<Rgba>& default_diffuse,
    const DiagnosticPolicy& policy) {
  return ObjRenderParser().ExtractRenderData(obj_path, properties,
                                             default_diffuse, policy);
}

// pair<vector<RenderMesh>, map<string, RenderTexture>>
// LoadRenderMeshesFromGltf(
//     const std::filesystem::path& gltf_path,
//     const GeometryProperties& properties, const Rgba& default_diffuse,
//     const DiagnosticPolicy& policy) {
//   GltfParser parser(gltf_path, &policy);
//   return parser.ExtractRenderData(properties, default_diffuse);
// }

pair<vector<RenderMesh>, map<string, RenderTexture>> LoadRenderMeshesFromFile(
    const std::filesystem::path& mesh_path,
    const GeometryProperties& properties, const Rgba& default_diffuse,
    const DiagnosticPolicy& policy) {
  const string extension = Mesh(mesh_path).extension();
  if (extension == ".obj") {
    vector<RenderMesh> meshes =
        LoadRenderMeshesFromObj(mesh_path, properties, default_diffuse, policy);
    return {std::move(meshes), {}};
    // } else if (extension == ".gltf") {
    //   return LoadRenderMeshesFromGltf(mesh_path, properties, default_diffuse,
    //                                   policy);
  }
  DRAKE_UNREACHABLE();
}

RenderMesh MakeRenderMeshFromTriangleSurfaceMesh(
    const TriangleSurfaceMesh<double>& mesh,
    const GeometryProperties& properties, const DiagnosticPolicy& policy) {
  RenderMesh result;
  result.material =
      MaybeMakeMeshFallbackMaterial(properties, "", {}, policy, UvState::kNone);
  const int vertex_count = mesh.num_vertices();
  const int triangle_count = mesh.num_triangles();
  result.positions.resize(vertex_count, 3);
  result.normals.resize(vertex_count, 3);
  /* Normals need to be zero initialized because we will accumulate into them.
   */
  result.normals.setZero();
  /* uv values are unused but need to be properly sized. We arbitrarily set them
   to all zeros. */
  result.uvs.resize(vertex_count, 2);
  result.uvs.setZero();
  result.indices.resize(triangle_count, 3);
  for (int i = 0; i < triangle_count; ++i) {
    const SurfaceTriangle& t = mesh.element(i);
    result.indices.row(i) << t.vertex(0), t.vertex(1), t.vertex(2);
    const double area = mesh.area(i);
    const Vector3<double> weighted_normal = area * mesh.face_normal(i);
    for (int j = 0; j < 3; ++j) {
      result.normals.row(t.vertex(j)) += weighted_normal;
    }
  }
  for (int i = 0; i < vertex_count; ++i) {
    result.positions.row(i) = mesh.vertex(i);
    result.normals.row(i).normalize();
  }
  return result;
}

RenderMesh MakeFacetedRenderMeshFromTriangleSurfaceMesh(
    const TriangleSurfaceMesh<double>& mesh,
    const GeometryProperties& properties, const DiagnosticPolicy& policy) {
  // The simple solution is to create a *new* mesh where every triangle has its
  // own vertices and then pass to MakeRenderMeshFromTriangleSurfaceMesh().
  // If this ever becomes an onerous burden, we can do that directly into the
  // RenderMesh.
  std::vector<Vector3d> vertices;
  vertices.reserve(mesh.num_triangles() * 3);
  std::vector<SurfaceTriangle> triangles;
  vertices.reserve(mesh.num_triangles());
  for (const SurfaceTriangle& t_in : mesh.triangles()) {
    const int v_index = ssize(vertices);
    triangles.emplace_back(v_index, v_index + 1, v_index + 2);
    vertices.push_back(mesh.vertex(t_in.vertex(0)));
    vertices.push_back(mesh.vertex(t_in.vertex(1)));
    vertices.push_back(mesh.vertex(t_in.vertex(2)));
  }
  const TriangleSurfaceMesh<double> faceted(std::move(triangles),
                                            std::move(vertices));
  return MakeRenderMeshFromTriangleSurfaceMesh(faceted, properties, policy);
}

TriangleSurfaceMesh<double> MakeTriangleSurfaceMesh(
    const RenderMesh& render_mesh) {
  const int num_vertices = render_mesh.positions.rows();
  const int num_triangles = render_mesh.indices.rows();
  std::vector<Vector3<double>> vertices;
  vertices.reserve(num_vertices);
  std::vector<SurfaceTriangle> triangles;
  triangles.reserve(num_triangles);
  for (int v = 0; v < num_vertices; ++v) {
    vertices.emplace_back(render_mesh.positions.row(v));
  }
  for (int t = 0; t < num_triangles; ++t) {
    triangles.emplace_back(render_mesh.indices(t, 0), render_mesh.indices(t, 1),
                           render_mesh.indices(t, 2));
  }
  return TriangleSurfaceMesh<double>(
      TriangleSurfaceMesh(std::move(triangles), std::move(vertices)));
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
