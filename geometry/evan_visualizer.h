#pragma once

#include <DR/common/exception.h>
#include <DR/common/real_time_string.h>
#include <DR/common/real_time_vector.h>
#include <drake/geometry/drake_visualizer.h>
#include <drake/lcm/drake_lcm.h>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/tree/multibody_tree_indexes.h>
#include <drake/systems/framework/leaf_system.h>

#include <drake/lcmt_viewer_geometry_data.hpp>
#include <memory>
#include <optional>
#include <utility>

namespace drake { namespace multibody { template <typename> class MultibodyPlant; }}

namespace DR {

using VisualizerIndex = drake::TypeSafeIndex<class VisualizerIndexTag>;

/// A visualization system that can be added to a Diagram to provide visualization. Other systems can use the static
/// functions provided in this class to add data to be visualized between publishes. Only one such instance can be
/// "alive" per program instance.
template <typename T>
class Visualizer : public drake::systems::LeafSystem<T> {
 public:
  Visualizer(const Visualizer&) = delete;
  Visualizer(Visualizer&&) = delete;
  Visualizer& operator=(const Visualizer&) = delete;
  Visualizer& operator=(Visualizer&&) = delete;
  explicit Visualizer(double publish_freq);
  ~Visualizer();

  /// Returns the QueryObject-valued input port. It should be connected to SceneGraph's QueryObject-valued output port.
  /// Failure to do so will cause a runtime error when attempting to broadcast messages.
  const drake::systems::InputPort<T>& query_object_input_port() const {
    return this->get_input_port(query_object_input_port_index_);
  }

  /// The frequency (in Hz) at which the (single) system publishes. Only valid when a system has been constructed.
  static double publish_freq() { return publish_freq_.value(); }

  /// Constructs a visualizer in the given Diagram and connects it to the provided scene graph (which must also live in
  /// the same Diagram).
  static const Visualizer<T>& AddToBuilder(drake::systems::DiagramBuilder<T>* builder,
                                           const drake::geometry::SceneGraph<T>& scene_graph,
                                           double publish_freq = 30.0);

  /// Constructs a visualizer in the given Diagram and connects it to the given query object input port.
  static const Visualizer<T>& AddToBuilder(drake::systems::DiagramBuilder<T>* builder,
                                           const drake::systems::OutputPort<T>& query_object_port,
                                           double publish_freq = 30.0);

  /// Maximum number of geometries supported per body.
  static constexpr int kMaxGeometries = 58;  // Necessary for the Pandas.

  /// Static function for adding data to be visualized.
  /// @param name the name of the visualized data (to be displayed in the visualizer).
  /// @param geometries a vector of geometries to be visualized.
  /// @returns an VisualizerIndexeger index used to update the pose of the geometry.
  static VisualizerIndex AddVisualization(
      const std::string_view& name,
      RealTimeVector<drake::lcmt_viewer_geometry_data, kMaxGeometries>&& geometries) {
    return AddVisualization(name, -1, std::move(geometries));
  }

  /// Static function for adding the bodies and geometries of a MultibodyPlant for visualization.
  /// @tparam kVectorSize the maximum number of bodies that can be added; in general, you want to tie this number to
  ///         the number of bodies in `plant`.
  /// @param plant the plant to be visualized.
  /// @param scene_graph the scene graph that the plant is connected to.
  /// @param scene_graph_context the Context for the scene graph.
  /// @param color_transform_fn a transformation function that describes how the colors in the visualization geometry
  ///        should be transformed, thereby permitting the same plant to be visualized simultaneously with reduced
  ///        visual conflict.
  /// @param visualize_fixed_bodies if `false`, does not add fixed bodies to the visualization.
  template <int kVectorSize>
  static RealTimeVector<std::pair<drake::multibody::BodyIndex, VisualizerIndex>, kVectorSize> AddVisualization(
      const drake::multibody::MultibodyPlant<T>& plant, const drake::geometry::SceneGraph<T>& scene_graph,
      const drake::systems::Context<T>& scene_graph_context,
      const std::function<drake::geometry::Rgba(const drake::geometry::Rgba&)>& color_transform_fn,
      bool visualize_fixed_bodies = true);

  /// Updates the pose of the geometries associated with the given index.
  static void UpdatePose(const VisualizerIndex index, const drake::math::RigidTransformd& X_WF);

  /// Updates the poses of the geometries associated with the given indices built for a MultibodyPlant.
  template <int kVectorSize>
  static void UpdatePoses(const drake::multibody::MultibodyPlant<T>& plant,
                          const drake::geometry::SceneGraph<T>& scene_graph,
                          const drake::systems::Context<T>& scene_graph_context,
                          const RealTimeVector<std::pair<drake::multibody::BodyIndex, VisualizerIndex>, kVectorSize>&
                              body_indices_to_visualizer_indices);

  /// Removes the geometries associated with the given index from the visualizer.
  static void RemoveVisualization(VisualizerIndex i);

 private:
  static VisualizerIndex AddVisualization(
      const std::string_view& name, int robot_number,
      RealTimeVector<drake::lcmt_viewer_geometry_data, kMaxGeometries>&& geometries);

  struct NonPortBasedData {
    RealTimeString<> name;
    int robot_number;
    bool active;
    RealTimeVector<drake::lcmt_viewer_geometry_data, kMaxGeometries> geometries;
    drake::math::RigidTransform<double> X_WF;
  };

  // The maximum number of "frames" (in the SceneGraphInspector sense of the word) that may be visualized.
  constexpr static int kMaxFrames{1024};

  // The maximum number of groups of geometries that can be added statically (i.e., not over the
  // query_object_input_port()).
  constexpr static int kMaxNonPortBasedData{1024};

  drake::systems::EventStatus SendVisualizationMessages(const drake::systems::Context<T>& context) const;
  void SendLoadRobotVisualizationMessage(
      const drake::systems::Context<T>& context,
      const RealTimeVector<drake::geometry::internal::DynamicFrameData, kMaxFrames>& query_port_frame_data,
      const RealTimeVector<NonPortBasedData, kMaxNonPortBasedData>& non_port_based_data) const;
  void SendDrawRobotVisualizationMessage(
      const drake::systems::Context<T>& context,
      const RealTimeVector<drake::geometry::internal::DynamicFrameData, kMaxFrames>& query_port_frame_data,
      const RealTimeVector<NonPortBasedData, kMaxNonPortBasedData>& non_port_based_data) const;

  // Port indices.
  drake::systems::InputPortIndex query_object_input_port_index_{};

  static bool constructed_;
  static bool need_to_send_load_message_;
  static int next_non_port_robot_index_;
  static RealTimeVector<drake::geometry::internal::DynamicFrameData, kMaxFrames> query_port_frame_data_;
  static RealTimeVector<NonPortBasedData, kMaxNonPortBasedData> non_port_based_data_;
  static drake::lcm::DrakeLcm visualization_lcm_;
  static std::optional<double> publish_freq_;
};

extern template class Visualizer<double>;

}  // namespace DR
