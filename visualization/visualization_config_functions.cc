#include "drake/visualization/visualization_config_functions.h"

#include <memory>
#include <stdexcept>
#include <string>

#include "drake/geometry/drake_visualizer.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/systems/primitives/shared_pointer_system.h"

namespace drake {
namespace visualization {
namespace {

using geometry::DrakeVisualizer;
using geometry::DrakeVisualizerParams;
using geometry::Rgba;
using geometry::Role;
using geometry::SceneGraph;
using lcm::DrakeLcm;
using lcm::DrakeLcmInterface;
using multibody::ConnectContactResultsToDrakeVisualizer;
using multibody::MultibodyPlant;
using systems::DiagramBuilder;
using systems::SharedPointerSystem;
using systems::System;
using systems::lcm::LcmBuses;

template <template <typename> class ChildSystem>
const ChildSystem<double>* DowncastSubsystem(
    const DiagramBuilder<double>* builder, std::string_view name) {
  DRAKE_DEMAND(builder != nullptr);
  for (const System<double>* system : builder->GetSystems()) {
    if (system->get_name() == name) {
      const auto* child = dynamic_cast<const ChildSystem<double>*>(system);
      if (child == nullptr) {
        throw std::logic_error(fmt::format(
            "ApplyVisualizationConfig: the DiagramBuilder contains a system"
            " named '{}' but of the wrong type (expected: {}, actual {}).",
            name, NiceTypeName::Get<ChildSystem<double>>(),
            NiceTypeName::Get(*system)));
      }
      return child;
    }
  }
  throw std::logic_error(fmt::format(
      "ApplyVisualizationConfig: the DiagramBuilder does not contain a system"
      " named '{}'.",  name));
}

// N.B. `scene_graph` is always required, but `plant` is only necessary if we
// have requested contact visualization.
void ApplyVisualizationConfigImpl(
    const VisualizationConfig& config,
    DrakeLcmInterface* lcm,
    const MultibodyPlant<double>* plant,
    const SceneGraph<double>& scene_graph,
    DiagramBuilder<double>* builder) {
  const std::vector<DrakeVisualizerParams> all_params =
      internal::ConvertVisualizationConfigToParams(config);
  for (const DrakeVisualizerParams& params : all_params) {
    // TODO(jwnimmer-tri) At the moment, meldis cannot yet display hydroelastic
    // geometry. So long as that's true, we should not enable it.
    DrakeVisualizerParams oopsie = params;
    oopsie.show_hydroelastic = false;
    DrakeVisualizer<double>::AddToBuilder(builder, scene_graph, lcm, oopsie);
  }
  if (config.publish_contacts) {
    if (plant == nullptr) {
      plant = DowncastSubsystem<MultibodyPlant>(builder, "plant");
    }
    DRAKE_THROW_UNLESS(plant->is_finalized());
    ConnectContactResultsToDrakeVisualizer(builder, *plant, scene_graph, lcm);
  }
}

}  // namespace

void ApplyVisualizationConfig(
    const VisualizationConfig& config,
    systems::DiagramBuilder<double>* builder,
    const systems::lcm::LcmBuses* lcm_buses,
    const multibody::MultibodyPlant<double>* plant,
    const geometry::SceneGraph<double>* scene_graph,
    lcm::DrakeLcmInterface* lcm) {
  DRAKE_THROW_UNLESS(builder != nullptr);
  if (lcm == nullptr) {
    if (lcm_buses != nullptr) {
      lcm = lcm_buses->Find("ApplyVisualizationConfig", config.lcm_bus);
    } else {
      DRAKE_THROW_UNLESS(config.lcm_bus == "default");
      auto* owner_system = builder->AddSystem<SharedPointerSystem<double>>(
          std::make_shared<DrakeLcm>());
      lcm = owner_system->get<DrakeLcm>();
    }
  }
  DRAKE_DEMAND(lcm != nullptr);
  if (scene_graph == nullptr) {
    scene_graph = DowncastSubsystem<SceneGraph>(builder, "scene_graph");
  }
  ApplyVisualizationConfigImpl(config, lcm, plant, *scene_graph, builder);
}

void AddDefaultVisualization(DiagramBuilder<double>* builder) {
  ApplyVisualizationConfig(VisualizationConfig{}, builder);
}

namespace internal {

std::vector<DrakeVisualizerParams>
ConvertVisualizationConfigToParams(
    const VisualizationConfig& config) {
  std::vector<DrakeVisualizerParams> result;

  if (config.publish_illustration) {
    DrakeVisualizerParams illustration;
    illustration.role = Role::kIllustration;
    illustration.publish_period = config.publish_period;
    illustration.default_color = config.default_illustration_color;
    result.push_back(illustration);
  }

  if (config.publish_proximity) {
    DrakeVisualizerParams proximity;
    proximity.role = Role::kProximity;
    proximity.publish_period = config.publish_period;
    proximity.default_color = config.default_proximity_color;
    proximity.show_hydroelastic = true;
    proximity.use_role_channel_suffix = true;
    result.push_back(proximity);
  }

  return result;
}

}  // namespace internal
}  // namespace visualization
}  // namespace drake
