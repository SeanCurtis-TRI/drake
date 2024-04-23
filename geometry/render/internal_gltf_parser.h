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
 */
std::pair<std::vector<RenderMesh>, std::map<std::string, MemoryImageFile>>
GetRenderMeshesFromGltf(std::filesystem::path gltf_path,
                        const GeometryProperties& properties,
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
