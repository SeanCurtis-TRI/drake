#include "drake/geometry/render/internal_gltf_parser.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_roles.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using drake::internal::DiagnosticPolicy;
using std::string;

/* glTF tests
 - Scene logic - confirm I'm getting the set of root indices I expect.
    - use specified scene (if given)
    - use zero scene if not given
    - use everything if there are no scenes
    - Weird but not problems
      - No nodes (nothing created).
      - negative default index (but scenes defined) defaults to zero.
    - Error conditions
      - default scene index too big
      - default scene index too small is *not* an error; it defaults to zero.
      - no scenes and no nodes
      - cycle in the graph (no root nodes).
 - reading data
   - materials
     - baseColorFactor comes through as diffuse color
     - baseColorTexture is included
       - It exists in the cache and the diffuse_map contains a key.
     - If multiple meshes include the same texture, the texture is only in the
       cache once and they all share the same key.
     - For RenderTexture, the data is all correct.
     - Missing material uses default.
     - Negative material index uses fallback.
     - Too big material index uses fallback.
     - Bad uv state; warning and fallback material
     - Errors
       - Unsupported uv channels logs debug. (Untestable)
   - geometry
    - primitives align correctly; the transforms are properly combined.
      - various spellings of transforms (matrices, transforms, scales, etc.)
    - Primitives given as anything but triangle soups are ignored.
    - RenderMesh::uv_state
      - Multiple meshes combine as full, partial, or none correctly.
    - We have the right geometry.
      - This one is *tricky*. How to confirm the mesh is right?
        - Lots of hard-coding and mixture. I'll have to figure this one out.
      - I should get the faces, vertices, normals, and uvs I expect.
      - I should be able to encode my rainbow_box.gltf into a RenderMesh by hand.
    - Errors
      - No normals throws
      - No UVs produce mesh with all zero UVs.   
 - glTF that doesn't define materials!
   - parser with deferred material.   
*/

GTEST_TEST(GltfParserTest, Pyramid) {
  string gltf_path = FindResourceOrThrow(
      "drake/geometry/render/test/meshes/fully_textured_pyramid.gltf");

  DiagnosticPolicy policy;
  const PerceptionProperties properties;
  const Rgba default_diffuse(0.25, 0.5, 0.75, 0.25);
  const auto [meshes, image_cache] =
      GltfParser(gltf_path, &policy)
          .ExtractRenderData(properties, default_diffuse);
  ASSERT_EQ(meshes.size(), 1);
  ASSERT_TRUE(image_cache.empty());
}
}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
