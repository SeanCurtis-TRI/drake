#include <filesystem>
#include <string>

// #include <benchmark/benchmark.h>
#include <fmt/format.h>
#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/name_value.h"
#include "drake/common/profiler.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/render/light_parameter.h"
#include "drake/geometry/render_gl/render_engine_gl_params.h"
#include "drake/geometry/render_vtk/render_engine_vtk_params.h"
#include "drake/geometry/scene_graph_config.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/package_map.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/sensors/camera_config.h"
#include "drake/systems/sensors/camera_config_functions.h"
#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/image_io.h"
#include "drake/systems/sensors/rgbd_sensor.h"
#include "drake/visualization/visualization_config.h"
#include "drake/visualization/visualization_config_functions.h"

namespace drake {
namespace benchmarking {
namespace {

namespace fs = std::filesystem;

using Eigen::Vector3d;
using geometry::EnvironmentMap;
using geometry::EquirectangularMap;
using geometry::RenderEngineGlParams;
using geometry::RenderEngineVtkParams;
using geometry::render::LightParameter;
using geometry::Rgba;
using geometry::SceneGraphConfig;
using multibody::AddMultibodyPlant;
using multibody::MultibodyPlant;
using multibody::MultibodyPlantConfig;
using multibody::PackageMap;
using multibody::Parser;
using systems::Context;
using systems::DiagramBuilder;
using systems::sensors::ApplyCameraConfig;
using systems::sensors::CameraConfig;
using systems::sensors::ImageIo;
using systems::sensors::ImageRgba8U;
using systems::sensors::RgbdSensor;
using visualization::ApplyVisualizationConfig;
using yaml::LoadYamlFile;

DEFINE_string(lbm_package_dir, "", "Path to the lbm_eval_models package.");
DEFINE_int32(N, 500, "The number of rendering iterations to perform.");
DEFINE_string(save_image_path, "",
              "Enables saving rendered images in the given location");
DEFINE_string(engine, "vtk", "'vtk' or 'gl'");

std::string_view robot_sdf =
    "package://lbm_eval_models/stations/cabot/add_cabot_simulation.dmd.yaml";

// class ScenarioRenderBenchmark : public benchmark::Fixture {
//  public:
// 	void SetUp(const ::benchmark::State& state) override {
// 		// Setup code (if needed)
// 	}
// 	void TearDown(const ::benchmark::State& state) override {
// 		// Teardown code (if needed)
// 	}
// };

// BENCHMARK_F(ScenarioRenderBenchmark, Foo)(benchmark::State& state) {
// 	for (auto _ : state) {
// 		// Benchmark code goes here
// 	}
// }

/* A convenient struct for parsing a slice of lbm_eval's scenario yaml. */
struct SceneConfig {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(cameras));
  }

  std::vector<CameraConfig> cameras;
};

/* Return a set of render engine parameters of the requested type.
 @param package_map  Used to resolve resources that are only named via urls. */
template <typename RenderEngineParams>
RenderEngineParams GetRenderEngineParams(const PackageMap& package_map) {
  // This is the light originally specified in cabot_box_scenario.yaml.
  std::vector<LightParameter> lights{{
      .type = "directional",
      .color = Rgba(0.95, 1.0, 0.76),
      .attenuation_values = Vector3d{1, 0, 0},
      .frame = "world",
      .intensity = 3.0,
      .direction = Vector3d(.25, 0, -1),
      .cone_angle = 20,
  }};

  if constexpr (std::is_same_v<RenderEngineParams, RenderEngineGlParams>) {
    return RenderEngineGlParams{
        .lights = lights,
    };
  } else if constexpr (std::is_same_v<RenderEngineParams,
                                      RenderEngineVtkParams>) {
    const std::string env_map_url =
        "package://lbm_eval_models/environment_maps/poly_haven_studio_4k.hdr";
    const fs::path env_map_path = package_map.ResolveUrl(env_map_url);
    return RenderEngineVtkParams{
        .lights = lights,
        .environment_map =
            EnvironmentMap{.skybox = true,
                           .texture = EquirectangularMap{.path = env_map_path}},
        .exposure = 0.4,
        .cast_shadows = true,
        .shadow_map_size = 1024,
    };
  } else {
      DRAKE_UNREACHABLE();
  }
}

/* Updates the SceneConfig's cameras to use the RenderEngine type specified.
 The engine parameters are hard-coded, but derived from the original parameters
 specified in lbm_eval's cabot_box_scenario.xml. */
template <typename RenderEngineParams>
void ConfigureCameras(SceneConfig* config, const PackageMap& package_map) {
  for (int c = 0; c < ssize(config->cameras); ++c) {
    CameraConfig& camera = config->cameras[c];
    if (camera.renderer_class.index() == 0) {
      // The first camera gets the full parameter spec.
      camera.renderer_class =
          GetRenderEngineParams<RenderEngineParams>(package_map);
    } else {
      // The remaining cameras simply get the matching class name.
      if constexpr (std::is_same_v<RenderEngineParams,
                                   RenderEngineGlParams>) {
        camera.renderer_class = "RenderEngineGl";
      } else if constexpr (std::is_same_v<RenderEngineParams,
                                         RenderEngineVtkParams>) {
        camera.renderer_class = "RenderEngineVtk";
      } else {
        DRAKE_UNREACHABLE();
      }
    }
  }
}

void ConfigureSimulation(std::string_view lbm_package_dir) {
  if (lbm_package_dir.empty()) {
    throw std::runtime_error(
        "The --lbm_package_dir flag must be set to the path of the "
        "lbm_eval_models package.\n");
  }

  DiagramBuilder<double> builder;

  auto [plant, scene_graph] = AddMultibodyPlant({}, {}, &builder);
  Parser parser(&builder);
  parser.package_map().Add("lbm_eval_models", lbm_package_dir);
  parser.AddModelsFromUrl(std::string(robot_sdf));
  // TODO: Load manipulands on table.
  parser.AddModels(FindResourceOrThrow(
      "drake/geometry/benchmarking/manipulands.dmd.yaml"));

  plant.Finalize();

  // Note: we're opting-out of LCM by passing lcm_buses = nullptr and a
  // throw-away DrakeLcm instance.
  lcm::DrakeLcm lcm(systems::lcm::LcmBuses::kLcmUrlMemqNull);
  ApplyVisualizationConfig({}, &builder, nullptr, nullptr, nullptr, nullptr,
                           &lcm);

  const std::string config_path = "drake/geometry/benchmarking/lbm_config.yaml";
  auto config = LoadYamlFile<SceneConfig>(FindResourceOrThrow(config_path),
                                          std::nullopt, SceneConfig{});
  if (FLAGS_engine == "vtk") {
    ConfigureCameras<RenderEngineVtkParams>(&config, parser.package_map());
  } else if (FLAGS_engine == "gl") {
    ConfigureCameras<RenderEngineGlParams>(&config, parser.package_map());
  } else {
    throw std::runtime_error(
        "The --engine flag must be set to either 'vtk' or 'gl'.");
  }
  std::vector<const RgbdSensor*> cameras;
  for (const auto& camera_config : config.cameras) {
    ApplyCameraConfig(camera_config, &builder, nullptr, nullptr, nullptr, &lcm);
    cameras.push_back(&builder.GetDowncastSubsystemByName<RgbdSensor>(
        fmt::format("rgbd_sensor_{}", camera_config.name)));
  }
  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();
  diagram->ForcedPublish(*context);

  fs::path out_path = FLAGS_save_image_path;
  auto request_timer = addTimer("Image Request");
  for (const auto* camera : cameras) {
    // Let VTK render engine get through a couple of iterations to prime itself.
    for (int i = 0; i < 3; ++i) {
      context->SetTime(i * 0.01);
      const auto& cam_context = camera->GetMyContextFromRoot(*context);
      (void)camera->color_image_output_port().Eval<ImageRgba8U>(cam_context);
    }
    reset();
    const auto& cam_context = camera->GetMyContextFromRoot(*context);
    ImageRgba8U color_image;
    for (int i = 0; i < FLAGS_N; ++i) {
      context->SetTime(i);
      startTimer(request_timer);
      color_image =
          camera->color_image_output_port().Eval<ImageRgba8U>(cam_context);
      stopTimer(request_timer);
    }
    fmt::print("Camera {}: ", camera->get_name());
    fmt::print("{}\n", TableOfAverages());
    if (FLAGS_save_image_path.empty()) continue;
    ImageIo().Save(
        color_image,
        out_path / fmt::format("{}_{}.png", FLAGS_engine, camera->get_name()));
  }

  fmt::print("There are {} perception geometries in the scene\n",
             scene_graph.model_inspector().NumGeometriesWithRole(
                 geometry::Role::kPerception));
}

#ifdef RUN_AS_BINARY

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  ConfigureSimulation(FLAGS_lbm_package_dir);
  return 0;
}
#endif

}  // namespace
}  // namespace benchmarking
}  // namespace drake

#ifdef RUN_AS_BINARY

int main(int argc, char* argv[]) {
  drake::benchmarking::do_main(argc, argv);
}

#endif  // RUN_AS_BINARY
