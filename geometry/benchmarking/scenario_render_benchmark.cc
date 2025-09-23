#include <filesystem>
#include <iostream>
#include <optional>
#include <string>

#ifndef RUN_AS_BINARY
#include <benchmark/benchmark.h>
#endif
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
using geometry::Obb;
using geometry::RenderEngineGlParams;
using geometry::RenderEngineVtkParams;
using geometry::Rgba;
using geometry::Role;
using geometry::SceneGraphConfig;
using geometry::render::LightParameter;
using math::RigidTransformd;
using multibody::AddMultibodyPlant;
using multibody::ModelInstanceIndex;
using multibody::MultibodyPlant;
using multibody::MultibodyPlantConfig;
using multibody::PackageMap;
using multibody::Parser;
using multibody::RigidBody;
using systems::Context;
using systems::Diagram;
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
DEFINE_bool(visualize, false, "If true, meshcat visualization is enabled.");
DEFINE_bool(shadows_off, false, "If true, shadows are disabled.");
DEFINE_bool(all_spheres, false, "If true, replaces *all* geometries with spheres.");

std::string_view robot_sdf =
    "package://lbm_eval_models/stations/cabot/add_cabot_simulation.dmd.yaml";


/* A convenient struct for parsing a slice of lbm_eval's scenario yaml. */
struct SceneConfig {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(cameras));
  }

  std::vector<CameraConfig> cameras;
};

/* The data necessary to run a benchmark test. */
struct BenchmarkData {
  std::unique_ptr<Diagram<double>> diagram;
  std::vector<const RgbdSensor*> cameras;
  int num_geometries = 0;
  std::unique_ptr<Context<double>> context{};
  const RigidBody<double>* manipuland{};
  RigidTransformd X_PB_reference;
  MultibodyPlant<double>* plant{};
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
        .cast_shadows = !FLAGS_shadows_off,
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

BenchmarkData ConfigureSimulation(std::string_view lbm_package_dir,
                                  const std::string& engine) {
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
  parser.AddModels(
      FindResourceOrThrow("drake/geometry/benchmarking/manipulands.dmd.yaml"));

  plant.Finalize();

  if (FLAGS_all_spheres) {
    const fs::path sphere_path = FindResourceOrThrow(
        "drake/geometry/benchmarking/multi_texture_sphere.gltf");
    const auto& inspector = scene_graph.model_inspector();
    for (const auto& id : inspector.GetAllGeometryIds(Role::kPerception)) {
      const std::optional<Obb> maybe_obb = inspector.GetObbInGeometryFrame(id);
      DRAKE_DEMAND(maybe_obb.has_value());
      RigidTransformd X_PG = inspector.GetPoseInFrame(id) * maybe_obb->pose();
      scene_graph.ChangeShape(
          *plant.get_source_id(), id,
          geometry::Mesh(sphere_path, maybe_obb->half_width()), X_PG);
    }
  }

  // We'll use the first manipuland for benchmarking.
  ModelInstanceIndex model_instance = plant.GetModelInstanceByName("orange");
  const auto& body =
      plant.GetRigidBodyByName("fake_ycb_orange", model_instance);
  const auto X_PB = plant.GetDefaultFreeBodyPose(body);
  BenchmarkData data{
    .num_geometries = scene_graph.model_inspector().NumGeometriesWithRole(
        geometry::Role::kPerception),
    .manipuland = &body,
    .X_PB_reference = X_PB,
    .plant = &plant,
  };

  lcm::DrakeLcm lcm(systems::lcm::LcmBuses::kLcmUrlMemqNull);
  if (FLAGS_visualize) {
    // Note: we're opting-out of LCM by passing lcm_buses = nullptr and a
    // throw-away DrakeLcm instance.
    ApplyVisualizationConfig({}, &builder, nullptr, nullptr, nullptr, nullptr,
                             &lcm);
  }

  const std::string config_path = "drake/geometry/benchmarking/lbm_config.yaml";
  auto config = LoadYamlFile<SceneConfig>(FindResourceOrThrow(config_path),
                                          std::nullopt, SceneConfig{});
  if (engine == "vtk") {
    ConfigureCameras<RenderEngineVtkParams>(&config, parser.package_map());
  } else if (engine == "gl") {
    ConfigureCameras<RenderEngineGlParams>(&config, parser.package_map());
  } else {
    throw std::runtime_error(
        "The --engine flag must be set to either 'vtk' or 'gl'.");
  }
  for (const auto& camera_config : config.cameras) {
    ApplyCameraConfig(camera_config, &builder, nullptr, nullptr, nullptr, &lcm);
    data.cameras.push_back(&builder.GetDowncastSubsystemByName<RgbdSensor>(
        fmt::format("rgbd_sensor_{}", camera_config.name)));
  }

  data.diagram = builder.Build();
  data.context = data.diagram->CreateDefaultContext();
  data.context->DisableCaching();
  return data;
}

BenchmarkData& GetVtkData() {
  // Note: all benchmarks use the same diagram. We only want to build the
  // diagram once and use it for all benchmarks.
  static BenchmarkData data = ConfigureSimulation(
      FLAGS_lbm_package_dir, "vtk");
  return data;
}

BenchmarkData& GetGlData() {
  // Note: all benchmarks use the same diagram. We only want to build the
  // diagram once and use it for all benchmarks.
  static BenchmarkData data = ConfigureSimulation(
      FLAGS_lbm_package_dir, "gl");
  return data;
}

BenchmarkData* GetSimulationData(int engine_id) {
  if (engine_id == 0) {
    return &GetVtkData();
  } else {
    return &GetGlData();
  }
}

/* With a newly built diagram and its allocated context, do some preliminary
 work to prime the context -- warm starting any render engine that requires it.
 Return the context for the sensor being evaluated. */
const Context<double>& WarmUpSensor(const Diagram<double>& diagram,
                                    Context<double>* context,
                                    const RgbdSensor& sensor) {
  if (FLAGS_visualize) {
    diagram.ForcedPublish(*context);
  }

  const auto& sensor_context = sensor.GetMyContextFromRoot(*context);

  // Cycle through a couple renderings untimed just in case the underlying
  // engine needs to be warm started.
  for (int i = 0; i < 3; ++i) {
    sensor.color_image_output_port().Eval<ImageRgba8U>(sensor_context);
  }
  reset();  // The timers.
  return sensor_context;
}

#ifdef RUN_AS_BINARY

/* This function would serve as the benchmark core test. Instead of FLAGS_N,
 we simply iterate through the states. */
void RunBenchmarkOnCamera(BenchmarkData* data, const RgbdSensor& sensor) {
  auto& context = *data->context;
  const auto& diagram = *data->diagram;
  const auto& sensor_context = WarmUpSensor(diagram, &context, sensor);
  auto& plant_context =
      data->plant->GetMyMutableContextFromRoot(&context);

  // Now do the work. In the benchmark, this'll be in the benchmark loop.
  auto request_timer = addTimer("Image Request");
  ImageRgba8U color_image;
  for (int i = 0; i < FLAGS_N; ++i) {
    startTimer(request_timer);
    RigidTransformd X_PB =
        data->X_PB_reference * RigidTransformd(Vector3d(0.001 * i, 0, 0));
    data->plant->SetFreeBodyPose(&plant_context, *data->manipuland, X_PB);
    color_image =
        sensor.color_image_output_port().Eval<ImageRgba8U>(sensor_context);
    stopTimer(request_timer);
  }
  fmt::print("Camera {}: ", sensor.get_name());
  fmt::print("{}\n", TableOfAverages());
  if (!FLAGS_save_image_path.empty()) {
    fs::path out_path = FLAGS_save_image_path;
    ImageIo().Save(color_image,
                    out_path / fmt::format("{}_{}.png", FLAGS_engine,
                                          sensor.get_name()));
  }
}

void RunAllCameras() {
  int engine = -1;
  if (FLAGS_engine == "vtk") {
    engine = 0;
  } else if (FLAGS_engine == "gl") {
    engine = 1;
  } else {
    throw std::runtime_error(
        "The --engine flag must be set to either 'vtk' or 'gl'.");
  }
  BenchmarkData* data = GetSimulationData(engine);
  for (const RgbdSensor* camera : data->cameras) {
    RunBenchmarkOnCamera(data, *camera);
  }

  fmt::print("There are {} perception geometries in the scene\n",
             data->num_geometries);
}

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  RunAllCameras();
  return 0;
}
#else

class LbmBenchmark : public benchmark::Fixture {
 public:
  LbmBenchmark() {
    request_timer_ = addTimer("Image Request");
  }
  ~LbmBenchmark() override {}

  void SetUp(::benchmark::State& state) {
    data_ = GetSimulationData(state.range(0));
    if (data_->cameras.empty()) {
      throw std::runtime_error("No cameras were configured.");
    }
    const int sensor_index = state.range(1);
    if (sensor_index < 0 || sensor_index >= ssize(data_->cameras)) {
      throw std::runtime_error(
          fmt::format("The camera index ({}) is out of range [0, {}).",
                      sensor_index, data_->cameras.size()));
    }
    sensor_ = data_->cameras[sensor_index];
    sensor_context_ =
        &WarmUpSensor(*data_->diagram, data_->context.get(), *sensor_);

    const auto& camera = sensor_->GetColorRenderCamera(*sensor_context_);

    image_.resize(camera.core().intrinsics().width(),
                  camera.core().intrinsics().height());
  }

 protected:
  void RenderImage(::benchmark::State& state) {
    reset();  // The timers.
    for (auto _ : state) {
      startTimer(request_timer_);
      image_ = sensor_->color_image_output_port().Eval<ImageRgba8U>(
          *sensor_context_);
      stopTimer(request_timer_);
    }
  }

  struct ArgProfiler {
    std::string name;
    benchmark::IterationCount iterations;
    std::string table;
  };

  const BenchmarkData* data_;
  const RgbdSensor* sensor_{nullptr};
  const Context<double>* sensor_context_{nullptr};
  ImageRgba8U image_;
  common::TimerIndex request_timer_{};
  std::vector<ArgProfiler> profilers_;
};

BENCHMARK_DEFINE_F(LbmBenchmark, RenderTest)(benchmark::State& state) {
  RenderImage(state);
}
BENCHMARK_REGISTER_F(LbmBenchmark, RenderTest)
    ->Unit(benchmark::kMillisecond)
    ->ArgsProduct({
        {0 /*, 1*/},    // 0: vtk, 1: gl
        {0, 1, 2, 3, 4, 5}, // camera index
    });

#endif

}  // namespace
}  // namespace benchmarking
}  // namespace drake

#ifdef RUN_AS_BINARY

int main(int argc, char* argv[]) {
  drake::benchmarking::do_main(argc, argv);
}

#endif  // RUN_AS_BINARY
