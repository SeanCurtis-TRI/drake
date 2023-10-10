#include "drake/geometry/render_vtk/render_engine_vtk_params.h"

#include <variant>

#include <gtest/gtest.h>

#include "drake/common/yaml/yaml_io.h"

namespace drake {
namespace geometry {
namespace {

GTEST_TEST(RenderEngineVtkParams, BasicSerialize) {
  using Params = RenderEngineVtkParams;
  const Params original{
      .default_diffuse = Eigen::Vector4d{1.0, 0.5, 0.25, 1.0},
      .default_clear_color = Eigen::Vector3d{0.25, 0.5, 1.0},
      .lights = {{.type = "point"}}};
  const std::string yaml = yaml::SaveYamlString<Params>(original);
  const Params dut = yaml::LoadYamlString<Params>(yaml);
  EXPECT_EQ(dut.default_diffuse, original.default_diffuse);
  EXPECT_EQ(dut.default_clear_color, original.default_clear_color);
  ASSERT_EQ(dut.lights.size(), 1);
  EXPECT_EQ(dut.lights.at(0).type, "point");
  ASSERT_FALSE(dut.environment_map.has_value());
}

GTEST_TEST(RenderEngineVtkParams, SerializeWithEquirectangularMap) {
  using Params = RenderEngineVtkParams;
  // We'll assume all of the other fields still work and focus on env map.
  // We won't specify the `skybox` value in the environment_map to confirm that
  // it defaults to `true`. We'll explicitly set it to false while testing the
  // cube map.
  const Params original{
      .environment_map = EnvironmentMap{
          .texture = EquirectangularMap{.path = "local.hdr"}}};
  const std::string yaml = yaml::SaveYamlString<Params>(original);
  const Params dut = yaml::LoadYamlString<Params>(yaml);
  ASSERT_TRUE(dut.environment_map.has_value());
  EXPECT_TRUE(dut.environment_map->skybox);
  ASSERT_TRUE(
      std::holds_alternative<EquirectangularMap>(dut.environment_map->texture));
  ASSERT_EQ(std::get<EquirectangularMap>(dut.environment_map->texture).path,
            "local.hdr");
}

GTEST_TEST(RenderEngineVtkParams, SerializeWithCubeMap) {
  using Params = RenderEngineVtkParams;
  // We'll assume all of the other fields still work and focus on env map.
  const Params original{.environment_map = EnvironmentMap{
                            .skybox = false,
                            .texture = CubeMap{.right = "right.hdr",
                                               .left = "left.hdr",
                                               .top = "top.hdr",
                                               .bottom = "bottom.hdr",
                                               .front = "front.hdr",
                                               .back = "back.hdr"}}};
  const std::string yaml = yaml::SaveYamlString<Params>(original);
  const Params dut = yaml::LoadYamlString<Params>(yaml);
  ASSERT_TRUE(dut.environment_map.has_value());
  EXPECT_FALSE(dut.environment_map->skybox);
  ASSERT_TRUE(std::holds_alternative<CubeMap>(dut.environment_map->texture));
  const CubeMap& map = std::get<CubeMap>(dut.environment_map->texture);
  EXPECT_EQ(map.right, "right.hdr");
  EXPECT_EQ(map.left, "left.hdr");
  EXPECT_EQ(map.top, "top.hdr");
  EXPECT_EQ(map.bottom, "bottom.hdr");
  EXPECT_EQ(map.front, "front.hdr");
  EXPECT_EQ(map.back, "back.hdr");
}

}  // namespace
}  // namespace geometry
}  // namespace drake
