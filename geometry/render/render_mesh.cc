#include "drake/geometry/render/render_mesh.h"

#include <algorithm>
#include <fstream>
#include <tuple>

#include <fmt/format.h>
#include <tiny_gltf.h>
#include <tiny_obj_loader.h>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/ssize.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using drake::internal::DiagnosticPolicy;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Matrix4d;
using std::make_tuple;
using std::map;
using std::pair;
using std::string;
using std::tuple;
using std::vector;

/* Constructs a RenderMaterial from a single tinyobj material specification. */
RenderMaterial MakeMaterialFromMtl(const tinyobj::material_t& mat,
                                   const std::filesystem::path& obj_path,
                                   const GeometryProperties& properties,
                                   const DiagnosticPolicy& policy,
                                   UvState uv_state) {
  RenderMaterial result;

  result.from_mesh_file = true;
  result.diffuse.set(mat.diffuse[0], mat.diffuse[1], mat.diffuse[2],
                     mat.dissolve);
  result.diffuse_map = [&mat, &obj_path]() -> string {
    if (mat.diffuse_texname.empty()) {
      return mat.diffuse_texname;
    }
    std::filesystem::path tex_path(mat.diffuse_texname);
    if (tex_path.is_absolute()) {
      return mat.diffuse_texname;
    }
    // There are three potential paths: path to obj, path to mtl, and path to
    // texture image (png). We have the path to obj. The mtl is defined relative
    // to the obj (inside the obj) and the png is defined relative to the
    // mtl (defined in the mtl). However, tinyobj doesn't give us access to
    // the mtl path to resolve image paths. For now, we're making the
    // simplifying assumption that obj and mtl are in the same directory.
    //
    // What if the OBJ references multiple mtl files in disparate locations?
    // Ideally, `material_t` should  come with a string indicating either the
    // the mtl file it came from or the directory of that mtl file. Then
    // relative paths of the referenced images can be properly interpreted.
    // Or what is stored in material_t should be relative to the obj.
    std::filesystem::path obj_dir = obj_path.parent_path();
    return (obj_dir / tex_path).lexically_normal().string();
  }();

  // If a diffuse map is specified, it must be as a path, and the indicated
  // file must be available and applicable.
  if (!result.diffuse_map.empty()) {
    std::ifstream tex_file(result.diffuse_map);
    if (!tex_file.is_open()) {
      policy.Warning(fmt::format(
          "The OBJ file's material requested an unavailable diffuse texture "
          "image: {}. The image will be omitted.",
          result.diffuse_map));
      result.diffuse_map.clear();
    } else if (uv_state != UvState::kFull) {
      policy.Warning(fmt::format(
          "The OBJ file's material requested a diffuse texture image: {}. "
          "However the mesh doesn't define {} texture coordinates. The image "
          "will be omitted.",
          result.diffuse_map,
          uv_state == UvState::kNone ? "any" : "a complete set of"));
      result.diffuse_map.clear();
    }
  }

  MaybeWarnForRedundantMaterial(properties, obj_path.string(), policy);
  return result;
}

}  // namespace

// TODO(SeanCurtis-TRI): Add troubleshooting entry on OBJ support and
// reference it in these errors/warnings.

vector<RenderMesh> LoadRenderMeshesFromObj(
    const std::filesystem::path& obj_path, const GeometryProperties& properties,
    const std::optional<Rgba>& default_diffuse,
    const DiagnosticPolicy& policy) {
  tinyobj::ObjReaderConfig config;
  config.triangulate = true;
  config.vertex_color = false;
  tinyobj::ObjReader reader;
  const bool valid_parse = reader.ParseFromFile(obj_path.string(), config);

  if (!valid_parse) {
    throw std::runtime_error(fmt::format("Failed parsing the obj file: {}: {}",
                                         obj_path.string(), reader.Error()));
  }

  // We better not get any errors if we have a valid parse.
  DRAKE_DEMAND(reader.Error().empty());

  const vector<tinyobj::shape_t>& shapes = reader.GetShapes();

  bool faces_found = false;
  for (const auto& shape : shapes) {
    faces_found = shape.mesh.indices.size() > 0;
    if (faces_found) break;
  }
  if (!faces_found) {
    throw std::runtime_error(fmt::format(
        "The OBJ data appears to have no faces; it could be missing faces or "
        "might not be an OBJ file: {}",
        obj_path.string()));
  }

  if (!reader.Warning().empty()) {
    policy.Warning(reader.Warning());
  }

  const tinyobj::attrib_t& attrib = reader.GetAttrib();

  /* The parsed product needs to be further processed. The RenderMesh assumes
   that all vertex quantities (positions, normals, texture coordinates) are
   indexed with a common index; a face that references vertex i, will get its
   position from positions[i], its normal from normals[i], and its texture
   coordinate from uvs[i]. However, we _cannot_ assume that each vertex
   position is associated with a single per-vertex quantity (normal, uv) in
   the OBJ file. OBJ allows a vertex position to be associated with arbitrary
   per-vertex quantities in each face definition independently. So, we need to
   create the unique association here.

   To accomplish this:
    1. Every vertex referenced by a face in the parsed OBJ is a "meta"
       vertex consisting of a tuple of indices: (p, n, uv), the index in
       vertex positions, normals, and texture coordinates. For example,
       imagine one face refers to meta index (p, n₀, uv) and another face
       refers to index (p, n₁, uv). Although the two faces appear to share a
       single vertex (and a common texture coordinate), those vertices have
       different normals which require two different vertices in the mesh
       data. We copy the vertex position and texture coordinate and then
       associate one copy with each normal. A similar operation would apply if
       they only differed in texture coordinate (or in both).
    2. Given a mapping (p, n, uv) --> i (a mapping from the meta vertex in the
       parsed OBJ data to the unique index in the resultant mesh data), we
       can build the faces in the final mesh data by mapping the (p, n, uv)
       tuple in the OBJ face specification to the final mesh data vertex
       index i.
    3. When done, we should have an equal number of vertex positions as
       normals and texture coordinates. And all indices in the faces should be
       valid indices into all three vectors of data.
   NOTE: In the case of meta vertices (p, n₀, uv) and (p, n₁, uv) we are not
   confirming that normals[n₀] and normals[n₁] are actually different normals;
   we're assuming different indices implies different values. Again, the same
   applies to different texture coordinate indices.  */

  /* The map from (p, n, uv) --> i.  */
  map<tuple<int, int, int>, int> obj_vertex_to_new_vertex;
  /* Accumulators for vertex positions, normals, and triangles.  */
  vector<Vector3d> positions;
  vector<Vector3d> normals;
  vector<Vector2d> uvs;
  vector<Vector3<int>> triangles;

  /* A map from the index of a material that was referenced to the indices of
   the faces to which it is applied. The face indices are not indices into
   the original obj, but into the `triangles` data. */
  map<int, vector<int>> material_triangles;

  // TODO(SeanCurtis-TRI) Revisit how we handle normals:
  //   1. If normals are absent, generate normals so that we get faceted meshes.
  //   2. Make use of smoothing groups.
  if (attrib.normals.size() == 0) {
    throw std::runtime_error(
        fmt::format("OBJ has no normals: {}", obj_path.string()));
  }

  /* Each triangle consists of three vertices. Any of those vertices may be
   associated with a corresponding UV value. For each material, we'll count the
   number of vertices with associated UVs. If the value is zero, no UVs were
   assigned at all. If it is equal to 3T (T = num triangles), UVs are assigned
   to all vertices. Any number in between means incomplete assignment. This
   will result in the following behavior:

     - fewer than 3T UVs: if the material references texture maps
                            - the map will be omitted,
                            - a warning will be dispatched.
                            - the RenderMesh will report no UVs (although it
                              will hold a full complement of zero-valued UVs).
     - 3T UVs: The RenderMesh will report it has UVs and the material will not
               be hampered in any way. */
  map<int, int> material_uvs;

  for (const auto& shape : shapes) {
    /* Each face is a sequence of indices. All of the face indices are
     concatenated together in one long sequence: [i1, i2, ..., iN]. Because
     we have nothing but triangles, that sequence can be partitioned into
     triples, each representing one triangle:
       [(i1, i2, i3), (i4, i5, i6), ..., (iN-2, iN-1, iN)].
     We maintain an index into that long sequence (v_index) and simply
     increment it knowing that every three increments takes us to a new face. */
    int v_index = 0;
    const auto& shape_mesh = shape.mesh;
    const int num_faces = static_cast<int>(shape_mesh.num_face_vertices.size());
    for (int f = 0; f < num_faces; ++f) {
      DRAKE_DEMAND(shape_mesh.num_face_vertices[f] == 3);
      const int mat_index = shape_mesh.material_ids[f];
      /* Captures the [i0, i1, i2] new index values for the face.  */
      int face_vertices[3] = {-1, -1, -1};
      for (int i = 0; i < 3; ++i) {
        const int position_index = shape_mesh.indices[v_index].vertex_index;
        const int norm_index = shape_mesh.indices[v_index].normal_index;
        const int uv_index = shape_mesh.indices[v_index].texcoord_index;
        if (norm_index < 0) {
          throw std::runtime_error(fmt::format(
              "Not all faces reference normals: {}", obj_path.string()));
        }
        const auto obj_indices =
            make_tuple(position_index, norm_index, uv_index);
        if (uv_index >= 0) {
          // If this vertex has a UV coordinate, we need to count it, even if
          // the "meta" vertex isn't unique.
          // Note: the map's value gets zero initialized so we can blindly
          // increment it without worry.
          ++material_uvs[mat_index];
        }
        if (!obj_vertex_to_new_vertex.contains(obj_indices)) {
          obj_vertex_to_new_vertex[obj_indices] =
              static_cast<int>(positions.size());
          /* Guarantee that the positions.size() == normals.size() == uvs.size()
           by always growing them in lock step.  */
          positions.emplace_back(
              Vector3d{attrib.vertices[3 * position_index],
                       attrib.vertices[3 * position_index + 1],
                       attrib.vertices[3 * position_index + 2]});
          normals.emplace_back(attrib.normals[3 * norm_index],
                               attrib.normals[3 * norm_index + 1],
                               attrib.normals[3 * norm_index + 2]);
          if (uv_index >= 0) {
            uvs.emplace_back(attrib.texcoords[2 * uv_index],
                             attrib.texcoords[2 * uv_index + 1]);
          } else {
            uvs.emplace_back(0.0, 0.0);
          }
        }
        face_vertices[i] = obj_vertex_to_new_vertex[obj_indices];
        ++v_index;
      }
      material_triangles[mat_index].push_back(ssize(triangles));
      triangles.emplace_back(&face_vertices[0]);
    }
  }

  DRAKE_DEMAND(positions.size() == normals.size());
  DRAKE_DEMAND(positions.size() == uvs.size());

  /* Now we need to partition the prepped geometry data. Each material used
   will lead to a unique `RenderMesh` and possibly a `RenderMaterial`. Note: the
   obj may have declared distinct *objects*. We are erasing that distinction as
   irrelevant for rendering the mesh as a rigid structure. */
  vector<RenderMesh> meshes;
  for (const auto& [mat_index, tri_indices] : material_triangles) {
    RenderMesh mesh_data;

    /* Create the material for set of triangles. */
    mesh_data.uv_state = material_uvs[mat_index] == 0 ? UvState::kNone
                         : material_uvs[mat_index] == ssize(tri_indices) * 3
                             ? UvState::kFull
                             : UvState::kPartial;
    if (mat_index == -1) {
      /* This is the default material. No material was assigned to the faces.
       We'll apply the fallback logic. */
      mesh_data.material = MaybeMakeMeshFallbackMaterial(
          properties, obj_path, default_diffuse, policy, mesh_data.uv_state);
    } else {
      mesh_data.material =
          MakeMaterialFromMtl(reader.GetMaterials().at(mat_index), obj_path,
                              properties, policy, mesh_data.uv_state);
    }

    /* Partition the data into distinct RenderMeshes. The triangles in one
     render mesh can reference vertex indices from all over the original vertex
     buffer. However, in the new render mesh, they must be compactly enumerated
     from [0, N-1]. This map provides the mapping from the original index value
     in the full buffer, to the vertex buffer in this RenderMesh. At the same
     time, convert them to the unsigned type required by RenderMesh. */
    using indices_uint_t = decltype(mesh_data.indices)::Scalar;
    map<int, indices_uint_t> vertex_index_full_to_part;
    for (const auto& t : tri_indices) {
      const auto& tri = triangles[t];
      for (int i = 0; i < 3; ++i) {
        if (!vertex_index_full_to_part.contains(tri[i])) {
          vertex_index_full_to_part[tri(i)] =
              static_cast<indices_uint_t>(vertex_index_full_to_part.size());
        }
      }
    }
    mesh_data.indices.resize(ssize(tri_indices), 3);
    int row = -1;
    for (const int t : tri_indices) {
      const Vector3<int>& source_tri = triangles[t];
      auto mapped_tri = mesh_data.indices.row(++row);
      /* Each vertex maps independently. */
      for (int i = 0; i < 3; ++i) {
        mapped_tri[i] = vertex_index_full_to_part[source_tri[i]];
      }
    }

    const int vertex_count = ssize(vertex_index_full_to_part);
    mesh_data.positions.resize(vertex_count, 3);
    mesh_data.normals.resize(vertex_count, 3);
    mesh_data.uvs.resize(vertex_count, 2);
    for (const auto& [full_index, part_index] : vertex_index_full_to_part) {
      mesh_data.positions.row(part_index) = positions[full_index];
      mesh_data.normals.row(part_index) = normals[full_index];
      mesh_data.uvs.row(part_index) = uvs[full_index];
    }
    meshes.push_back(std::move(mesh_data));
  }

  return meshes;
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

namespace {

/* The size of the component type in bytes. */
int ComponentSize(int component_type) {
  switch (component_type) {
    case TINYGLTF_COMPONENT_TYPE_BYTE:
      return 1;
    case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:
      return 1;
    case TINYGLTF_COMPONENT_TYPE_SHORT:
      return 2;
    case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT:
      return 2;
    case TINYGLTF_COMPONENT_TYPE_INT:
      return 4;
    case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT:
      return 4;
    case TINYGLTF_COMPONENT_TYPE_FLOAT:
      return 4;
    case TINYGLTF_COMPONENT_TYPE_DOUBLE:
      return 8;
    default:
      throw std::runtime_error(
          fmt::format("{} (invalid component type)", component_type));
  }
}

/* TODO
  - Only read in the default scene
    - Walk the node tree in the default scene
      - For each node collect:
        - its mesh primitives (group them by material)
        - its pose w.r.t. parent (probably the world at the same time with
          depth first traversal).
    - For each material
      - Create RenderMaterial
      - Create vertex data
        - This will likely require extracting data from various buffers.
        - We're going to report the rigid pose in the *file* frame. Make sure
          that vertices *and normals* are expressed in that frame.

  - Loading semantics (this will, ultimately, have to go into file format
    module and be clarified on all of the consumers).
    - What gets loaded
      - Case 1: Scenes defined
        - Case a: Valid default scene specified
          - Only nodes in default scene used.
        - Case b: No default scene specified
          - Nodes in the zeroth scene used.  Warning?
        - Case c: Invalid default scene specified
          - Throws!
      - Case 2: No Scenes
        - Case a: Nodes present
          - All nodes used
        - Case b: No Nodes present
          - Throws!

  - Mesh primitive attributes can have arbitrary indices; I need to make sure
    they get swizzled according to my needs.
  - y-up to z-up.
  - triangle fan vs triangle strip vs triangles
*/

// TODO: Lots of glTF stuff should all go into its own file.

/* This class is responsible for parsing the glTF file and creating the set of
 RenderMesh instances and the image cache. */
class GltfParser {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GltfParser);

  explicit GltfParser(std::filesystem::path gltf_path,
                      const DiagnosticPolicy* policy)
      : path_(std::move(gltf_path)), policy_(*policy) {
    DRAKE_DEMAND(policy != nullptr);
    tinygltf::TinyGLTF loader;
    string error;
    string warn;

    const bool valid_parse =
        loader.LoadASCIIFromFile(&model_, &error, &warn, path_.string());

    if (!valid_parse) {
      throw std::runtime_error(fmt::format(
          "Failed parsing the glTF file: {}: {}", path_.string(), error));
    }

    // We better not get any errors if we have a valid parse.
    DRAKE_DEMAND(error.empty());
    if (!warn.empty()) {
      policy_.Warning(warn);
    }
  }

  /* Creates the RenderMesh instances and image cache for this parser's gltf
   path. */
  pair<vector<RenderMesh>, map<string, RenderTexture>> ExtractRenderData(
      const GeometryProperties& properties, const Rgba& default_diffuse) {
    // The root nodes of all the hierarchies that will be instantiated (by
    // index).
    vector<int> root_indices = FindTargetRootNodes();

    // Walk the forest of node trees rooted in the target root nodes. Create a
    // RenderMesh for each unique material applied to a glTF primitive in the
    // forest.
    return MakeRenderMeshesFromNodes(root_indices, properties, default_diffuse);
  }

 private:
  // Identifies the source scene from the glTF file and returns the indices of
  // that scene's root nodes. If no default scene can be identified, then
  // all root nodes in the file are returned.
  vector<int> FindTargetRootNodes() {
    // The root nodes of all the hierarchies that will be instantiated (by
    // index).
    vector<int> root_indices;
    if (model_.scenes.size() > 0) {
      if (model_.defaultScene >= ssize(model_.scenes)) {
        policy_.Error(fmt::format(
            "Error parsing a glTF file; it defines {} scenes but has an "
            "invalid value for the \"scene\" property: {}. '{}'. No geometry "
            "will be added.",
            model_.scenes.size(), model_.defaultScene, path_.string()));
        return root_indices;
      }
      // TODO(SeanCurtis-TRI): I need to decide if this *deserves* a warning. It
      // is not clear if blindly picking the zeroth scene is consistent with the
      // glTF spec, which states (EMPHASIS mine):
      //
      //   When scene is undefined, client implementations MAY delay rendering
      //   until a particular scene is requested.
      //
      // See: https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html#scenes
      if (model_.defaultScene < 0) {
        policy_.Warning(fmt::format(
            "Parsing a glTF file with multiple scene and no explicit "
            "default scene; using the zeroth scene: '{}'.",
            path_.string()));
      }
      // tinygltf initializes defaultScene to -1 to indicate an undefined value
      const int scene_index = std::max(model_.defaultScene, 0);
      root_indices = model_.scenes[scene_index].nodes;
    } else {
      if (model_.nodes.size() == 0) {
        policy_.Error(fmt::format(
            "Error parsing a glTF file; it has no scenes and no nodes. '{}'.",
            path_.string()));
      }
      root_indices = FindAllRootNodes();
      if (root_indices.empty() && model_.nodes.size() > 0) {
        policy_.Error(
            fmt::format("Error parsing a glTF file; none of its {} nodes are "
                        "root nodes. '{}'.",
                        model_.nodes.size(), path_.string()));
      }
    }
    return root_indices;
  }

  /* Simply returns the indices of all nodes that have no parents (are root
   nodes). It searches *all* the nodes, unconstrained by what may or may not be
   indicated by the model's scenes. */
  vector<int> FindAllRootNodes() {
    vector<bool> has_parent(model_.nodes.size(), false);
    for (const auto& node : model_.nodes) {
      for (int child_index : node.children) {
        has_parent[child_index] = true;
      }
    }
    vector<int> roots;
    for (int n = 0; n < ssize(has_parent); ++n) {
      if (!has_parent[n]) {
        roots.push_back(n);
      }
    }
    return roots;
  }

  /* Given the indices of the root nodes, builds the render data based on the
   hierarchies rooted at those nodes. */
  pair<vector<RenderMesh>, map<string, RenderTexture>>
  MakeRenderMeshesFromNodes(const vector<int> root_nodes,
                            const GeometryProperties& properties,
                            const Rgba& default_diffuse) {
    vector<RenderMesh> meshes;
    map<string, RenderTexture> image_cache;

    map<int, vector<PosedPrimitive>> primitives_by_material =
        FindPrimitivesInForest(root_nodes);
    for (const auto& [mat_index, primitives] : primitives_by_material) {
      RenderMesh render_mesh;

      // TODO(SeanCurtis-TRI): Perhaps it would be better to partition the
      // primitives in those that *do* have texture coordinates and those that
      // don't. Then I could create two variants of the material, one with
      // textures for the meshes with UVs, and one without for those that don't.

      int uv_count = 0;
      for (const PosedPrimitive& posed_prim : primitives) {
        if (posed_prim.primitive->attributes.count("TEXCOORD_0") > 0) {
          uv_count += 1;
        }
      }
      /* Create the material for set of triangles. */
      render_mesh.uv_state = uv_count == 0 ? UvState::kNone
                             : uv_count == ssize(primitives)
                                 ? UvState::kFull
                                 : UvState::kPartial;
      // Build the material.
      if (mat_index < 0 || mat_index >= ssize(model_.materials) ||
          render_mesh.uv_state != UvState::kFull) {
        if (mat_index >= ssize(model_.materials)) {
          policy_.Warning(fmt::format(
              "A glTF file specifies a material with a bad index ({}) in {}. "
              "Using the fallback material.",
              mat_index, path_));
        }
        // Warnings for uv_state are issued by MaybeMakeMeshFallbackMaterial().
        render_mesh.material = *MaybeMakeMeshFallbackMaterial(
            properties, path_, default_diffuse, policy_, render_mesh.uv_state);
      } else {
        render_mesh.material = MakeGltfMaterial(mat_index, &image_cache);
      }

      for (const PosedPrimitive& posed_prim : primitives) {
        AccumulateMeshData(posed_prim, &render_mesh);
      }
      meshes.push_back(std::move(render_mesh));
    }
    return {std::move(meshes), std::move(image_cache)};
  }

  /* Creates a RenderMaterial for the glTF material indicated by the given
   index. Currently, we produce the most rudimentary of phong materials; we
   extract the diffuse color (aka baseColorFactor) and (possibly) diffuse color
   map. The diffuse map is added to the image_cache and the material's
   diffuse_map value contains the key for the image in the cache. */
  RenderMaterial MakeGltfMaterial(int mat_index,
                                  map<string, RenderTexture>* image_cache) {
    RenderMaterial material;
    const tinygltf::Material gltf_mat = model_.materials.at(mat_index);
    const tinygltf::PbrMetallicRoughness& gltf_pbr =
        gltf_mat.pbrMetallicRoughness;
    const vector<double>& gltf_rgba = gltf_pbr.baseColorFactor;
    material.diffuse =
        Rgba(gltf_rgba[0], gltf_rgba[1], gltf_rgba[2], gltf_rgba[3]);
    const int diffuse_index = gltf_pbr.baseColorTexture.index;
    if (diffuse_index >= 0) {
      if (gltf_pbr.baseColorTexture.texCoord != 0) {
        // TODO(SeanCurtis-TRI) Would this be better as a one-time warning?
        log()->debug(
            "Drake's native support of glTF files only includes zero-indexed "
            "texture coordinates. The material '{}' specifies texture "
            "coordinates {} in {}.",
            gltf_mat.name.empty() ? string("<unnamed>") : gltf_mat.name,
            gltf_pbr.baseColorTexture.texCoord, path_.string());
      }
      const string image_key =
          fmt::format("{}?image={}", path_.string(), diffuse_index);
      material.diffuse_map = image_key;
      if (image_cache->count(image_key) == 0) {
        const tinygltf::Image& image = model_.images.at(diffuse_index);
        // The image is not "as is" encoded; it has been decoded into pixels.
        DRAKE_DEMAND(image.as_is == false);
        RenderTexture render_image{.width = image.width,
                                   .height = image.height,
                                   .channels = image.component,
                                   .bits = image.bits,
                                   .pixel_type = image.pixel_type};
        render_image.pixel_data.reserve(image.image.size());
        std::copy(image.image.begin(), image.image.end(),
                  std::back_inserter(render_image.pixel_data));
        (*image_cache)[image_key] = std::move(render_image);
      }
    }
    // TODO(SeanCurtis-TRI): Debug messages for any of the other textures that
    // we're currently not supporting.
    return material;
  }

  /* A glTF primitive and its transformed pose in the file (the pose may not be
   a rigid transform; it can include scale). A primitive's pose is the same as
   its node's (N) pose (as only nodes get posed). This flattens the node
   hierarchy but as we are rendering it as a rigid assembly, we lose nothing. */
  struct PosedPrimitive {
    Matrix4d T_FN;
    const tinygltf::Primitive* primitive{};
  };

  /* Recursively traverses the node hierarchy rooted at the node with the given
   index. For each *mesh* node, it adds the mesh posed mesh primitives to the
   given material-to-primitive map.

   @param node_index   The index of the node N at the root of the sub tree.
   @param T_FP         The transform for N's parent node P relative to the file
                       frame F.
   @param primitives   A map from material index to the primitives that use it.
   */
  void FindPrimitivesInTree(int node_index,
                            const Matrix4d& T_FP,
                            map<int, vector<PosedPrimitive>>* primitives) {
    const tinygltf::Node& node = model_.nodes.at(node_index);

    const Matrix4d T_PN = EigenMatrixFromNode(node);
    const Matrix4d T_FN = T_FP * T_PN;

    if (node.mesh >= 0) {
      const tinygltf::Mesh& mesh = model_.meshes.at(node.mesh);
      for (const tinygltf::Primitive& prim : mesh.primitives) {
        (*primitives)[prim.material].emplace_back(T_FN, &prim);
      }
    }
    for (int child_index : node.children) {
      FindPrimitivesInTree(child_index, T_FN, primitives);
    }
  }

  /* Creates a map from material index to all mesh primitives in the hierarchy
   forest whose trees are rooted at the nodes with the given root indices. */
  map<int, vector<PosedPrimitive>> FindPrimitivesInForest(
      const vector<int>& root_indices) {
    map<int, vector<PosedPrimitive>> primitives_by_material;
    const Matrix4d I = Matrix4d::Identity();
    for (int node_index : root_indices) {
      FindPrimitivesInTree(node_index, I, &primitives_by_material);
    }
    return primitives_by_material;
  }

  template <typename Element>
  static void ThrowIfInvalidIndex(int index, const vector<Element>& array,
                                  std::string_view container_name,
                                  std::string_view element_name,
                                  std::string_view elaboration,
                                  const std::string& file_name) {
    if (index < 0 || index >= ssize(array)) {
      throw std::runtime_error(
          fmt::format("A {} uses an invalid {} index{} in {}.", container_name,
                      element_name, elaboration, file_name));
    }
  }

  /* Convenience class for reading entries from a glTF buffer *for meshes*.*/
  class BufferReader {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BufferReader);

    /* Creates a zero-filled dummy reader that can write out zeros for
     `count` elements each consisting of `byte_size` number of bytes. */
    BufferReader(int count, int byte_size) {
      const int total_bytes = count * byte_size;
      owned_buffer_ = vector<unsigned char>(total_bytes, 0);
      buffer_ = owned_buffer_.data();
      end_ = buffer_ + total_bytes;
      element_count_ = count;
      byte_size_ = byte_size;
      byte_stride_ = 0;
    }

    /* Constructs the reader for triangle indices. */
    BufferReader(int tri_accessor_index, const tinygltf::Model& model,
                 const std::string& file_name) {
      GltfParser::ThrowIfInvalidIndex(tri_accessor_index, model.accessors,
                                      "mesh primitive", "accessor",
                                      " for its indices", file_name);
      Initialize(model.accessors.at(tri_accessor_index),
                 "mesh primitive's indices", model, file_name);
    }

    /* Constructs the reader on the model for the indexed accessor. The `model`
     parameter is aliased by this instance and must outlive it. */
    BufferReader(const tinygltf::Primitive& primitive,
                 const std::string& attribute, const tinygltf::Model& model,
                 const std::string& file_name) {
      const auto iter = primitive.attributes.find(attribute);
      if (iter != primitive.attributes.end()) {
        const int accessor_index = iter->second;
        GltfParser::ThrowIfInvalidIndex(
            accessor_index, model.accessors, "mesh primitive", "accessor",
            fmt::format(" for the {} attribute", attribute), file_name);
        const tinygltf::Accessor& accessor = model.accessors.at(accessor_index);
        Initialize(accessor,
                   fmt::format("mesh primitive's {} attribute", attribute),
                   model, file_name);
      }
    }

    /* Returns the number of elements this reader can produce. */
    int count() const { return element_count_; }

    /* Writes a contiguous block of data from the ith element in the buffer to
     the given pointer. */
    void WriteElement(int i, void* data) {
      DRAKE_DEMAND(i >= 0 && i < element_count_);
      std::memcpy(data, buffer_ + (i * byte_stride_), byte_size_);
    }

   private:
    /* Initializes the buffer reader for the given accessor. */
    void Initialize(const tinygltf::Accessor& accessor,
                    std::string_view accessor_owner,
                    const tinygltf::Model& model,
                    const std::string& file_name) {
      element_count_ = accessor.count;
      const int component_count = ComponentSize(accessor.componentType);
      /* Tinygltf swaps the glTF strings with encoded ints in accessor.type. The
       number of components in the accessor type are encoded in the lowest
       *five* bits (because the largest possible value is 16). */
      const int component_size = (accessor.type & 0x1f);
      byte_size_ = component_count * component_size;

      const int bufferView_index = accessor.bufferView;
      GltfParser::ThrowIfInvalidIndex(bufferView_index, model.bufferViews,
                                      accessor_owner, "bufferView", "",
                                      file_name);
      const tinygltf::BufferView& buffer_view =
          model.bufferViews.at(bufferView_index);
      const int buffer_index = buffer_view.buffer;
      GltfParser::ThrowIfInvalidIndex(buffer_index, model.buffers,
                                      accessor_owner, "buffer", "", file_name);
      const tinygltf::Buffer& buffer = model.buffers.at(buffer_index);
      if (!buffer.uri.empty()) {
        throw std::runtime_error(fmt::format(
            "Drake's native glTF support requires embedded glTFs. Buffer {} "
            "has a non-empty URI in {}.",
            buffer_index, file_name));
      }
      byte_stride_ = buffer_view.byteStride;
      buffer_ =
          buffer.data.data() + accessor.byteOffset + buffer_view.byteOffset;
    }

    const unsigned char* buffer_{};
    const unsigned char* end_{};
    /* The number of elements. */
    int element_count_{};
    /* The size of a single element (e.g., 1-byte for scalar byte, 12 bytes for
     Vec3 of floats). */
    int byte_size_{};
    /* The distance (in bytes) between elements in the buffer. */
    int byte_stride_{};
    /* For the dummy reader, a vector of zeros. */
    vector<unsigned char> owned_buffer_{};
  };

  void AccumulateMeshData(const PosedPrimitive& prim, RenderMesh* render_mesh) {
    if (prim.primitive->mode != TINYGLTF_MODE_TRIANGLES) {
      log()->debug(fmt::format(
          "Drake's native support of glTF files requires that all primitives "
          "use the triangle mode ({}). At least one primitive has an "
          "unsupported  mode: {}. The primitive will be ignored. {}.",
          TINYGLTF_MODE_TRIANGLES, prim.primitive->mode, path_.string()));
      return;
    }

    BufferReader tri_data(prim.primitive->indices, model_, path_.string());
    BufferReader pos_data(*prim.primitive, "POSITION", model_, path_.string());
    DRAKE_DEMAND(pos_data.count() > 0);
    // No normals is a failure.
    BufferReader norm_data(*prim.primitive, "NORMAL", model_, path_.string());
    if (norm_data.count() == 0) {
      // The current practice for parsing an OBJ without normals is to simply
      // throw, so we'll treat glTF the same.
      throw std::runtime_error(fmt::format(
          "Drake's native support of glTF files requires that all primitives "
          "define normals. At least one primitive is missing normals. {}",
          path_.string()));
    }

    // When parsing an OBJ, if there are no texture coordinates, we set the
    // primitives to all zeros and mark the uv state appropriately. To maintain
    // compatibility, we'll do the same here.
    BufferReader uv_data(*prim.primitive, "TEXCOORD_0", model_, path_.string());
    if (uv_data.count() == 0) {
      // Replace the buffer reader with a zero-valued reader. A single texture
      // coordinate is 2 floats, for a size of 8 bytes.
      uv_data = BufferReader(pos_data.count(), 8);
      if (render_mesh->uv_state == UvState::kFull) {
        render_mesh->uv_state = UvState::kPartial;
      }
    } else {
      if (render_mesh->uv_state == UvState::kNone) {
        // If the render mesh hasn't accumulated any geometry yet, the current
        // uv state is meaningless and we can simply set to full. Otherwise,
        // "none" means previous primitives lacked UVs, so it becomes partial.
        if (render_mesh->positions.rows() == 0) {
          render_mesh->uv_state = UvState::kFull;
        } else {
          render_mesh->uv_state = UvState::kPartial;
        }
      }
    }

    DRAKE_DEMAND(pos_data.count() == norm_data.count());
    DRAKE_DEMAND(pos_data.count() == uv_data.count());

    /* The map from indices in the glTF file to indices in the RenderMesh.  */
    map<int, int> gltf_vertex_to_new_vertex;
    /* Accumulators for vertex positions, normals, and uvs. We can't write */
    vector<Vector3d> positions(pos_data.count());
    vector<Vector3d> normals(pos_data.count());
    vector<Vector2d> uvs(pos_data.count());

    /* Note: tri_data should have 3*N entries; each entry is an index into the
     vertices. */
    DRAKE_DEMAND(tri_data.count() % 3 == 0);
    const int tri_count = tri_data.count() / 3;
    const int tri_offset = render_mesh->indices.rows();
    render_mesh->indices.resize(tri_offset + tri_count, 3);

    /* The number of pre-existing vertices in the render_mesh. */
    const int v_offset = render_mesh->positions.size();
    int local_index = 0;
    for (int i = 0; i < tri_data.count(); ++i) {
      int v_index = 0;
      // N.B. If the glTF has stored indices as shorts, we're assuming
      // appropriate endianness, such that the two bytes written to this four
      // byte int will report the right value.
      tri_data.WriteElement(i, &v_index);
      if (gltf_vertex_to_new_vertex.count(v_index) > 0) {
        continue;
      }
      const int new_index = v_offset + local_index;
      gltf_vertex_to_new_vertex[v_index] = new_index;
      pos_data.WriteElement(i, positions[local_index].data());
      norm_data.WriteElement(i, normals[local_index].data());
      uv_data.WriteElement(i, uvs[local_index].data());
      ++local_index;

      const int tri_index = i / 3;
      const int tri_vert_index = i % 3;
      render_mesh->indices(tri_index, tri_vert_index) = new_index;
    }

    // Now I need to concatenate the vectors I've made to render mesh's
    // data.
    //  1. Resize each of the arrays by the number of vertices I have.
    //  2. Write.
    render_mesh->positions.resize(v_offset + ssize(positions), 3);
    render_mesh->normals.resize(v_offset + ssize(positions), 3);
    render_mesh->uvs.resize(v_offset + ssize(positions), 2);
    for (int v = 0; v < ssize(positions); ++v) {
      render_mesh->positions.row(v + v_offset) = positions[v];
      render_mesh->normals.row(v + v_offset) = normals[v];
      render_mesh->uvs.row(v + v_offset) = uvs[v];
    }
  }

  // Creates a transform from the given `nodes` data.
  Matrix4d EigenMatrixFromNode(const tinygltf::Node& node) {
    Matrix4d T;
    if (node.matrix.size() == 16) {
      // For glTF, transform matrix is a *column-major* matrix.
      int i = -1;
      for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r) {
          T(r, c) = node.matrix.at(++i);
        }
      }
    } else {
      T = Matrix4d::Identity();
      if (node.translation.size() > 0) {
        DRAKE_DEMAND(node.translation.size() == 3);
        const Vector3d p(node.translation.at(0), node.translation.at(1),
                         node.translation.at(2));
        T.block<3, 1>(0, 3) = p.transpose();
      }
      if (node.rotation.size() > 0) {
        DRAKE_DEMAND(node.rotation.size() == 4);
        const Quaternion<double> quat(node.rotation[3], node.rotation[0],
                                      node.rotation[1], node.rotation[2]);
        T.block<3, 3>(0, 0) = math::RotationMatrixd(quat).matrix();
      }
      if (node.scale.size() > 0) {
        DRAKE_DEMAND(node.scale.size() == 3);
        for (int i = 0; i < 3; ++i) {
          T.block<3, 1>(0, i) *= node.scale.at(i);
        }
      }
    }
    return T;
  }

  // The path to the glTF being parsed.
  std::filesystem::path path_;

  const DiagnosticPolicy& policy_;

  // The parsed glTF file.
  tinygltf::Model model_;
};

}  // namespace

pair<vector<RenderMesh>, map<string, RenderTexture>> LoadRenderMeshesFromGltf(
    const std::filesystem::path& gltf_path,
    const GeometryProperties& properties, const Rgba& default_diffuse,
    const DiagnosticPolicy& policy) {
  GltfParser parser(gltf_path, &policy);
  return parser.ExtractRenderData(properties, default_diffuse);
}

pair<vector<RenderMesh>, map<string, RenderTexture>> LoadRenderMeshesFromFile(
    const std::filesystem::path& mesh_path,
    const GeometryProperties& properties, const Rgba& default_diffuse,
    const DiagnosticPolicy& policy) {
  const string extension = Mesh(mesh_path).extension();
  if (extension == ".obj") {
    vector<RenderMesh> meshes =
        LoadRenderMeshesFromObj(mesh_path, properties, default_diffuse, policy);
    return {std::move(meshes), {}};
  } else if (extension == ".gltf") {
    return LoadRenderMeshesFromGltf(mesh_path, properties, default_diffuse,
                                    policy);
  }
  DRAKE_UNREACHABLE();
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
