#pragma once

#include <filesystem>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/diagnostic_policy.h"
#include "drake/geometry/geometry_properties.h"
#include "drake/geometry/render/render_material.h"
#include "drake/geometry/render/render_mesh.h"
#include "drake/geometry/rgba.h"

namespace drake {
namespace geometry {
namespace internal {

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

/* Parses the indicated glTF file, returning the set of RenderMeshes found. For
 any embedded, base64-encoded image contains within the glTF, the bytes of the
 image (as a png/jpg/whatever) will be returned in map of MemoryImageFile. The
 key is the name that would appear as a texture map name in the RenderMesh's
 RenderMaterial and can be used as a viable texture name in the TextureLibrary.

 There may be content in the glTF file that does not get included in the
 returned RenderMesh instances. This includes:

   - Any texture types that RenderMaterial does not support.
   - Any nodes not included in the "active" nodes set (see below for more detail).

 Active nodes

 The glTF may contain nodes that don't contribute to the parsed result. It all
 comes down to how the glTF defines its scenes. The following rules are applied
 in selecting the "active" nodes.

  - If a default scene is indicated, the nodes in that default scene are used.
  - If no default scene is indicated, but there are scenes, we assume the 0ᵗʰ
    scene is the default scene.
  - If there are no scenes, all root nodes (and nodes reachable from root nodes
    will be included).

  Generally, an empty set of active nodes is considered an error condition.

 Material definitions

 The glTF specification supports physically-based rendered (PBR) materials.
 RenderEngineGl supports a primitive Phong illumination model. This means the
 materials defined in the glTF file are not faithfully reproduced. Instead,
 they'll be converted to a RenderEngineGl-compatible approximation. Currently,
 the following glTF properties get mapped as follows (anything not explicitly
 listed is ignored):

   |            glTF material property              |     RenderMaterial value    |
   | :--------------------------------------------: | :-------------------------: |
   | material.pbrMetallicRoughness.baseColorFactor  | RenderMaterial::diffuse     |
   | material.pbrMetallicRoughness.baseColorTexture | RenderMaterial::diffuse_map |

 Where no material is defined, Drake's material heuristic is defined. See
 @ref geometry_materials.

 Mesh definitions

 The minimum number of RenderMesh instances get returned. They are derived from
 the active set of glTF nodes, but some nodes may get merged if they share the
 same material. Furthermore, while nodes can be posed arbitrarily in the frame
 of the glTF file (which transforms a referenced mesh's vertices), the vertex
 positions in the returned RenderMesh will have the transform applied and the
 vertex positions will be expressed in the *file's* frame.

 Drake currently only supports a subset of the features.

   - The meshes must be articulated as "triangle soups". Or, in OpenGL parlance,
     GL_TRIANGLES. No strips, fans, lines, points, etc. Mesh primitives using
     these types will be ignored.
   - The meshes *must* have normals defined. Missing normals is an error
     condition.

 @param gltf_path        The path to a glTF file.
 @param properties       The geometry properties associated with the glTF file
                         -- used to define materials (if the glTF hasn't defined
                         its own).
 @param default_diffuse  The caller's default_diffuse color, used as part of
                         Drake's material heuristic.
 @param policy           Used for reporting errors and warnings.

 @throws if there's a read/parse error (including I/O errors, json errors, glTF
            logic errors, etc.) for the indicated file. */
std::pair<std::vector<RenderMesh>, std::map<std::string, MemoryImageFile>>
GetRenderMeshesFromGltf(std::filesystem::path gltf_path,
                        const GeometryProperties& properties,
                        // TODO: Should this be optional now?
                        const Rgba& default_diffuse,
                        const drake::internal::DiagnosticPolicy& policy);

/* (internal) An entry point to facilitate testing. Generally, we assume that
 the glTF files will be consumed from file paths. */
std::pair<std::vector<RenderMesh>, std::map<std::string, MemoryImageFile>>
GetRenderMeshesFromGltfFromString(
    std::string_view gltf_contents, const GeometryProperties& properties,
    const Rgba& default_diffuse,
    const drake::internal::DiagnosticPolicy& policy,
    std::filesystem::path gltf_path = "<memory>");

}  // namespace internal
}  // namespace geometry
}  // namespace drake
