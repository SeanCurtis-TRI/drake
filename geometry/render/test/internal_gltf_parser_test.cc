#include "drake/geometry/render/internal_gltf_parser.h"

#include <gtest/gtest.h>
#include <gmock/gmock.h>

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
 - embedded texture vs external texture
*/

class GltfParserTest : public testing::Test {

};

/* Confirms scene selection logic:
   - default scene specified
     - throw for invalid index
     - loads only scene indicated by valid index.
   - No default scene specified
     - No scenes, no problem.
     - One scene, no problem.
     - Multiple scenes, loads the zeroth scene (also makes a warning).
   - No scenes but nodes
     - load all nodes.
   - No nodes, but default scene
     - No meshes
   - No nodes, no scenes, no returned meshes.
   - Nodes have cycle - throw
     - Will this be caught by the parser?
     - What if the node listed in a scene isn't a root node?

  Make sure that when a scene gets loaded it comprises multiple root nodes and
  multiple non-root nodes.
*/
TEST_F(GltfParserTest, SceneSelection) {}

/* Confirm that the parser successfully identifies root nodes when there are
 no scenes. */
// TEST_F(GltfParserTest, RootNodeIdentification) {}
// TEST_F(GltfParserTest, XXX) {}
// TEST_F(GltfParserTest, XXX) {}
// TEST_F(GltfParserTest, XXX) {}
// TEST_F(GltfParserTest, XXX) {}
// TEST_F(GltfParserTest, XXX) {}
// TEST_F(GltfParserTest, XXX) {}
// TEST_F(GltfParserTest, XXX) {}
// TEST_F(GltfParserTest, XXX) {}
// TEST_F(GltfParserTest, XXX) {}
// TEST_F(GltfParserTest, XXX) {}
// TEST_F(GltfParserTest, XXX) {}
// TEST_F(GltfParserTest, XXX) {}
// TEST_F(GltfParserTest, XXX) {}
// TEST_F(GltfParserTest, XXX) {}

/* A holistic test to give the sense that things are actually happening. */
TEST_F(GltfParserTest, Pyramid) {
  string gltf_path = FindResourceOrThrow(
      "drake/geometry/render/test/meshes/fully_textured_pyramid.gltf");

  DiagnosticPolicy policy;
  const PerceptionProperties properties;
  const Rgba default_diffuse(0.25, 0.5, 0.75, 0.25);
  const auto [meshes, image_cache] =
      GltfParser(gltf_path, &policy)
          .ExtractRenderData(properties, default_diffuse);

  /* One mesh with a single material. */
  ASSERT_EQ(meshes.size(), 1);
  const auto& mesh = meshes[0];
  /* Pyramid with square base is made up of six triangles. */
  EXPECT_EQ(mesh.indices.rows(), 6);
  EXPECT_TRUE(mesh.material.has_value());
  EXPECT_EQ(mesh.material->diffuse, Rgba(1, 1, 1, 1));
  EXPECT_FALSE(mesh.material->diffuse_map.empty());
  EXPECT_THAT(mesh.material->diffuse_map,
              testing::EndsWith("fully_textured_pyramid_base_color.png"));
  EXPECT_TRUE(std::filesystem::exists(mesh.material->diffuse_map))
      << mesh.material->diffuse_map;

  ASSERT_TRUE(image_cache.empty());
}
}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
