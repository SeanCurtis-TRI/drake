#pragma once

#include <filesystem>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/diagnostic_policy.h"
#include "drake/geometry/geometry_properties.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/render/render_material.h"
#include "drake/geometry/render/render_mesh.h"
#include "drake/geometry/rgba.h"

namespace drake {
namespace geometry {
namespace internal {

// TODO: Does this actually have any value? Will RenderEngineGl want to blindly
// call this? Or does it makes more sense to split it out?

/* Reads a supported mesh file and returns the corresponding render data. The
 render data consists of one or more RenderMesh instances and a possibly empty
 cache of in-memory RenderImages.

 Some mesh formats handle texture maps by providing file paths to images. Some
 can embed the images directly in the mesh file. When a texture has been used,
 the RenderMaterial map parameter will contain a non-empty string. That string
 may be a file path to an image, or it may be a key into the returned
 RenderTexture cache. There are no tokens in the string to identify which it is.
 Instead, the string should be tested against the image cache; if it is accepted
 as a key, use the in-memory image. Otherwise, it should be used as a file path
 to a supported image file.

 @pre `mesh_path` has a supported extension.
 @returns A collection of meshes and the image cache. */
std::pair<std::vector<RenderMesh>, std::map<std::string, RenderTexture>>
LoadRenderMeshesFromFile(const std::filesystem::path& mesh_path,
                         const GeometryProperties& properties,
                         const Rgba& default_diffuse,
                         const drake::internal::DiagnosticPolicy& policy = {});

// TODO(SeanCurtis-TRI): All of this explanation, and general guidance for what
// meshes (and which features) are supported, needs to go into the trouble-
// shooting guide.
/* Returns a set of RenderMesh instances based on the objects and materials
 defined in the indicated obj file.

 For each unique material referenced in the obj file, a RenderMesh will be
 created. Even if there are errors in the material specification, the algorithm
 does its best to provide the "best" approximation of the declared material. The
 fact that it was declared and applied is respected. For example, the following
 are specification defects that nevertheless result in a RenderMaterial _based_
 on the mtl material and _not_ on the fallback logic:

   - Referencing a non-existent or unavailable texture.
   - Failing to specify *any* material properties at all beyond its name.
   - Specifying a texture but failing to provide texture coordinates.
   - etc.

 Note: This API is similar to ReadObjToTriangleSurfaceMesh, but it differs in
 several ways:

    - Support of per-vertex data that TriangleSurfaceMesh doesn't support
      (e.g., vertex normals, texture coordinates).
    - Material semantics.
    - The geometric data is reconditioned to be compatible with "geometry
      buffer" applications. (See RenderMesh for details.)
    - It supports multiple objects in the file. In fact, there may be more
      RenderMesh instances than objects defined because a single object with
      multiple materials will likewise be partitioned into separate RenderMesh
      instances.

 If texture coordinates are assigned to vertices, it will be indicated in
 the returned RenderMesh. See RenderMesh::uv_state for more detail.

 If the material includes texture paths, they will have been confirmed to both
 exist and be accessible.

 TODO(SeanCurtis-TRI): all throwing conditions should be converted to messages
 on the policy. Problems that would produce an "empty" mesh (e.g., no
 faces/normals) would log errors on the policy (and return the empty mesh).
 Others, like no uvs referenced, should emit a warning and return a non-empty
 mesh, representing the best approximation of what was parsed.

 @throws std::exception if a) tinyobj::LoadObj() fails, (b) there are no faces
                           or normals, c) faces fail to reference normals, or d)
                           faces fail to reference the texture coordinates if
                           they are present. */
std::vector<RenderMesh> LoadRenderMeshesFromObj(
    const std::filesystem::path& obj_path, const GeometryProperties& properties,
    const std::optional<Rgba>& default_diffuse,
    const drake::internal::DiagnosticPolicy& policy = {});

/* Returns a set of RenderMesh instances based on the objects and materials
 defined in the indicated glTF file.

 Several notes on how the in-memory representation will be different from the
 in-file representation of the data:

   - Only the nodes in the default scene are used (or the zeroth scene if the
     "scene" element isn't defined).
   - PBR materials are projected into the materials supported by RenderMaterial.
   - We treat the glTF as a rigid object, as such, we flatten the hierarchy
     such that all vertex data is expressed in the frame of the file.
   - If two glTF "primitives" reference the same material, they get merged into
     a single RenderMesh. Generally, each RenderMesh will have a unique
     material. (Unique in the sense that it had a different index in the glTF
     file; we don't test for duplicate materials in the glTF file).

 Finally, the image_map includes all of the images used by the returned meshes.
 As documented above, the URIs contained in the RenderMaterial map members serve
 as keys into the map. */
std::pair<std::vector<RenderMesh>, std::map<std::string, RenderTexture>>
LoadRenderMeshesFromGltf(const std::filesystem::path& gltf_path,
                         const GeometryProperties& properties,
                         const Rgba& default_diffuse,
                         const drake::internal::DiagnosticPolicy& policy = {});

/* Constructs a render mesh (without material) from a triangle surface mesh.

 The normal of a vertex v is computed using area-weighted normals of the
 triangles containing vertex v. In particular, for a watertight mesh, this will
 result in a smoothed geometry. On the other hand, if the mesh consists of
 triangles with duplicated and collocated vertices, this will result in a
 faceted geometry.

 UvState will be set to UvState::kNone and all uv coordinates are set to zero.
 The material of the resulting render mesh is created using the protocol in
 MaybeMakeMeshFallbackMaterial() with the given `properties`.

 The returned RenderMesh has the same number of positions, vertex normals, and
 uv coordinates as `mesh.num_vertices()`. */
RenderMesh MakeRenderMeshFromTriangleSurfaceMesh(
    const TriangleSurfaceMesh<double>& mesh,
    const GeometryProperties& properties,
    const drake::internal::DiagnosticPolicy& policy = {});

/* A variant of MakeRenderMeshFromTriangleSurfaceMesh(). In this case, the
 RenderMesh is guaranteed to effectively have per-face normals so it renders
 as a faceted mesh. */
RenderMesh MakeFacetedRenderMeshFromTriangleSurfaceMesh(
    const TriangleSurfaceMesh<double>& mesh,
    const GeometryProperties& properties,
    const drake::internal::DiagnosticPolicy& policy = {});

/* Converts from RenderMesh to TriangleSurfaceMesh. Only connectivity and
 vertex positions information are retained. */
TriangleSurfaceMesh<double> MakeTriangleSurfaceMesh(
    const RenderMesh& render_mesh);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
