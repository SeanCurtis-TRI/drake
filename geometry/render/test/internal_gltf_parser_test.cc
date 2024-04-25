#include "drake/geometry/render/internal_gltf_parser.h"

#include <fstream>

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <nlohmann/json.hpp>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using drake::internal::DiagnosticPolicy;
using drake::math::RotationMatrixd;
using std::string;
using json = nlohmann::json;

/* Testing strategy (broken down by function):
 - GetRenderMeshesFromGltfFromString
   - tinygltf parser errors get reported as runtime error.
   - Embedded textures get returned in the image cache.
   - tinygltf warnings get reported as policy warnings.
 - FindTargetRootNodes
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
     - Error
   - No nodes, no scenes, no returned meshes.
   - Nodes have cycle - throw
     - Will this be caught by the parser?
     - What if the node listed in a scene isn't a root node?
   - Make sure that when a scene gets loaded it comprises multiple root nodes
     and multiple non-root nodes.
  - FindAllRootNodes
    - no specific test required (covered) by FindTargetRootNode()
  - MakeGltfMaterial
    - No material defintion does what?
    - Values not consumed are safely ignored.
    - diffuse color with no map produces a material with color.
    - diffuse map without color does...what?
    - invalid material indices (too big or too small) use fallback.
    - no uvs go to fallback material
    - Embedded textures
      - prefixed with `embedded:`
      - The diffuse_map is included in the image cache.
      - The embedded texture has appropriate mimetype.
      - The embedded texture can be decoded.
      - If an embedded texture is referenced multiple times, it appears in the
        cache once, and all materials use the same URI.
    - Unsupported texture coordinates get a warning.
  - AccumulateMeshData
    - Error conditions
      - Wrong primitive type
      - No normals
    - For RenderMesh from a single primitive
      - with uvs -> uv_state full
      - w/o uvs -> uv_state none
    - Vertices get transformed appropriately.
      - handle the various spellings of transforms.
    - Primitives given as anything but triangle soups are ignored.
    - Merging primitives
      - permutations of the presence of uvs resulting in uv_state.
        - uv values get reported as expected.
      - Total number of vertices and triangles are as expected.
        - triangle indices are offset appropriately.
        - Two primitives which reference the same buffer are *not* sufficiently
          smart.
*/

/* This test harness works by exercising the GetRenderMeshesFromGltfFromString()
 API. We'll mutate an in-memory glTF file (json) and pass it into the function
 with expectations on the output. */
class RenderMeshFromGltfContentsTest : public testing::Test {
 public:
  RenderMeshFromGltfContentsTest() {
    const string gltf_path = FindResourceOrThrow(
        "drake/geometry/render/test/meshes/fully_textured_pyramid.gltf");
    {
      std::ifstream f(gltf_path);
      pyramid_json_ = json::parse(f);
    }

    /* Three instances of each vertex at the base (for three adjacent faces)
     plus four more instances of the top vertex (four incident faces). */
    constexpr int kVertexCount = 16; 
    constexpr int kTriCount = 6;

    /* Reality check that .bin is organized basically as we expect. Vertex
     attributes are 2 or 3 floats, triangle vertex indices are 16-bit. */
    DRAKE_DEMAND(pyramid_json_["bufferViews"][0]["byteLength"] ==
                 kVertexCount * 3 * sizeof(float));
    DRAKE_DEMAND(pyramid_json_["bufferViews"][1]["byteLength"] ==
                 kVertexCount * 3 * sizeof(float));
    DRAKE_DEMAND(pyramid_json_["bufferViews"][2]["byteLength"] ==
                 kVertexCount * 2 * sizeof(float));
    DRAKE_DEMAND(pyramid_json_["bufferViews"][3]["byteLength"] ==
                 kTriCount * 3 * sizeof(unsigned short));

    /* Rotate from glTF's y-up to Drake's z-up. */
    const RotationMatrixd R_DG = RotationMatrixd::MakeFromOrthonormalRows(
        {1, 0, 0}, {0, 0, -1}, {0, 1, 0});
    {
      const string bin_path = FindResourceOrThrow(
          "drake/geometry/render/test/meshes/fully_textured_pyramid.bin");
      std::ifstream f(bin_path, std::ios::binary);
      float values[48];

      auto& pos = pyramid_render_mesh_.positions;
      f.read(reinterpret_cast<char*>(values), 192 /* bytes */);
      pos =
          Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>>(
              &values[0], kVertexCount, 3)
              .cast<double>();
      for (int v = 0; v < kVertexCount; ++v) {
        pos.row(v) = (R_DG.matrix() * pos.row(v).transpose()).transpose();
      }

      auto& norm = pyramid_render_mesh_.normals;
      f.read(reinterpret_cast<char*>(values), 192 /* bytes */);
      norm =
          Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>>(
              &values[0], kVertexCount, 3)
              .cast<double>();
      for (int n = 0; n < kVertexCount; ++n) {
        norm.row(n) = (R_DG.matrix() * norm.row(n).transpose()).transpose();
      }

      f.read(reinterpret_cast<char*>(values), 128 /* bytes */);
      pyramid_render_mesh_.uvs =
          Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor>>(
              &values[0], kVertexCount, 2)
              .cast<double>();

      unsigned short index_values[36];
      f.read(reinterpret_cast<char*>(index_values), 36 /* bytes */);
      pyramid_render_mesh_.indices =
          Eigen::Map<Eigen::Matrix<unsigned short, Eigen::Dynamic, 3,
                                   Eigen::RowMajor>>(index_values, kTriCount, 3)
              .cast<unsigned int>();
    }
    pyramid_render_mesh_.uv_state = UvState::kFull;
    pyramid_render_mesh_.material = RenderMaterial{
        .diffuse = Rgba(1, 1, 1, 1),
        .diffuse_map =
            FindResourceOrThrow("drake/geometry/render/test/meshes/"
                                "fully_textured_pyramid_base_color.png"),
        .from_mesh_file = true};
  }

 protected:
  /* The json for the fully_texture_pyramid.gltf. */
  json pyramid_json_;
  /* The expected render mesh for the pyramid. Hand-crafted. */
  RenderMesh pyramid_render_mesh_;
};

/* Confirms scene selection logic:
*/
TEST_F(RenderMeshFromGltfContentsTest, SceneSelection) {}

/* Confirms that the parser successfully identifies root nodes when there are
 no scenes. */
// TEST_F(RenderMeshFromGltfContentsTest, RootNodeIdentification) {}

/* Confirms that embedded textures are reported properly in the image cache. */
// TEST_F(RenderMeshFromGltfContentsTest, EmbeddedTextures) {}

/* Confirms that images with URIs pointing to external files git mapped into
 the render material. */
// TEST_F(RenderMeshFromGltfContentsTest, ExternalTextures) {}

// TEST_F(RenderMeshFromGltfContentsTest, XXX) {}
// TEST_F(RenderMeshFromGltfContentsTest, XXX) {}
// TEST_F(RenderMeshFromGltfContentsTest, XXX) {}
// TEST_F(RenderMeshFromGltfContentsTest, XXX) {}
// TEST_F(RenderMeshFromGltfContentsTest, XXX) {}
// TEST_F(RenderMeshFromGltfContentsTest, XXX) {}
// TEST_F(RenderMeshFromGltfContentsTest, XXX) {}
// TEST_F(RenderMeshFromGltfContentsTest, XXX) {}
// TEST_F(RenderMeshFromGltfContentsTest, XXX) {}
// TEST_F(RenderMeshFromGltfContentsTest, XXX) {}
// TEST_F(RenderMeshFromGltfContentsTest, XXX) {}
// TEST_F(RenderMeshFromGltfContentsTest, XXX) {}

/* Invoke the file_path variant and do a basic test that the output is as
 expected. This is an indication that the file-variant is invoking the actual
 implementaiton correctly. */
TEST_F(RenderMeshFromGltfContentsTest, ParseFromFile) {
  string gltf_path = FindResourceOrThrow(
      "drake/geometry/render/test/meshes/fully_textured_pyramid.gltf");

  DiagnosticPolicy policy;
  const PerceptionProperties properties;
  const Rgba default_diffuse(0.25, 0.5, 0.75, 0.25);
  const auto [meshes, image_cache] =
      GetRenderMeshesFromGltf(gltf_path, properties, default_diffuse, policy);

  /* One mesh with a single material. */
  ASSERT_EQ(meshes.size(), 1);
  const auto& mesh = meshes[0];

  /* The mesh data matches our hand-crafted version. */
  EXPECT_TRUE(CompareMatrices(mesh.positions, pyramid_render_mesh_.positions));
  EXPECT_TRUE(CompareMatrices(mesh.normals, pyramid_render_mesh_.normals));
  EXPECT_TRUE(CompareMatrices(mesh.uvs, pyramid_render_mesh_.uvs));
  /* Note: We can't use CompareMatrices on an array of unsigned int. So, we cast
   them to double. */
  EXPECT_TRUE(CompareMatrices(mesh.indices.cast<double>(),
                              pyramid_render_mesh_.indices.cast<double>()));

  /* The material matches our hand-crafted version. */
  EXPECT_TRUE(mesh.material.has_value());
  EXPECT_EQ(mesh.material->diffuse, Rgba(1, 1, 1, 1));
  EXPECT_FALSE(mesh.material->diffuse_map.empty());
  EXPECT_THAT(mesh.material->diffuse_map,
              testing::EndsWith("fully_textured_pyramid_base_color.png"));
  /* The name points to a file that actually exists. */
  EXPECT_TRUE(std::filesystem::exists(mesh.material->diffuse_map))
      << mesh.material->diffuse_map;

  /* There were no embedded textures. */
  ASSERT_TRUE(image_cache.empty());
}

/* A second invocation of the file_path variant to confirm it handles the
 error condition properly. */
TEST_F(RenderMeshFromGltfContentsTest, PaseFromFileError) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      GetRenderMeshesFromGltf("bad_path.gltf", properties, default_diffuse,
                              policy),
      ".*Unable to read.*bad_path.gltf.*");
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
