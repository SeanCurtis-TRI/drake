#include <DR/common_systems/visualizer.h>
#include <DR/tools/shape_to_lcm.h>
#include <drake/geometry/drake_visualizer.h>
#include <drake/multibody/plant/multibody_plant.h>

#include <algorithm>  // std::max
#include <drake/lcmt_viewer_draw.hpp>
#include <drake/lcmt_viewer_load_robot.hpp>

using drake::AbstractValue;
using drake::geometry::GeometryId;
using drake::geometry::QueryObject;
using drake::geometry::Rgba;
using drake::geometry::Role;
using drake::geometry::SceneGraph;
using drake::geometry::Shape;
using drake::geometry::internal::DynamicFrameData;
using drake::math::RigidTransform;
using drake::multibody::Body;
using drake::multibody::BodyIndex;
using drake::multibody::Frame;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyForces;
using drake::multibody::MultibodyPlant;
using drake::multibody::SpatialAcceleration;
using drake::multibody::SpatialVelocity;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::EventStatus;
using drake::systems::OutputPort;
using drake::systems::State;

namespace DR {

// clang-format off
template <typename T> bool Visualizer<T>::constructed_ = false;
template <typename T> int Visualizer<T>::next_non_port_robot_index_ = 1000;
template <typename T> bool Visualizer<T>::need_to_send_load_message_;       // Note: both of these variables are...
template <typename T> std::optional<double> Visualizer<T>::publish_freq_;   // ...initialized upon object construction.
template <typename T> RealTimeVector<DynamicFrameData, Visualizer<T>::kMaxFrames> Visualizer<T>::query_port_frame_data_;
template <typename T>
RealTimeVector<typename Visualizer<T>::NonPortBasedData, Visualizer<T>::kMaxNonPortBasedData>
    Visualizer<T>::non_port_based_data_;
template <typename T> drake::lcm::DrakeLcm Visualizer<T>::visualization_lcm_;
// clang-format on

namespace {

// Helper utility to create lcm geometry description from geometry id.
template <typename T>
drake::lcmt_viewer_geometry_data MakeGeometry(
    const drake::geometry::SceneGraphInspector<T>& inspector, GeometryId g_id,
    const std::function<drake::geometry::Rgba(const drake::geometry::Rgba&)>& color_transform_fn) {
  const drake::geometry::GeometryProperties* props = inspector.GetProperties(g_id, Role::kIllustration);
  // We assume that the g_id was obtained by asking for geometries with the illustration role. So, by definition, the
  // properties should be non-null.
  DR_DEMAND(props != nullptr);
  const Shape& shape = inspector.GetShape(g_id);
  const Rgba kRedDefaultColor(1, 0, 0);
  const Rgba& color = props->GetPropertyOrDefault("phong", "diffuse", kRedDefaultColor);
  return ShapeToLcm().Convert(shape, inspector.GetPoseInFrame(g_id),
                              (color_transform_fn) ? color_transform_fn(color) : color);
}
}  // namespace

template <typename T>
const Visualizer<T>& Visualizer<T>::AddToBuilder(DiagramBuilder<T>* builder, const SceneGraph<T>& scene_graph,
                                                 double publish_freq) {
  return AddToBuilder(builder, scene_graph.get_query_output_port(), publish_freq);
}

template <typename T>
const Visualizer<T>& Visualizer<T>::AddToBuilder(DiagramBuilder<T>* builder, const OutputPort<T>& query_object_port,
                                                 double publish_freq) {
  auto& visualizer = *builder->template AddSystem<Visualizer<T>>(publish_freq);
  builder->Connect(query_object_port, visualizer.query_object_input_port());
  return visualizer;
}

template <typename T>
Visualizer<T>::Visualizer(double publish_freq) {
  // Note: it really only makes sense for one visualizer to be constructed at a time, unless they were to be constructed
  // to send data over different LCM ports. Adding static data would obviously complicate that situation, so we
  // just disallow constructing more than one object at a time.
  if (constructed_)
    throw std::runtime_error("Only one visualizer can be constructed at a time.");
  constructed_ = true;

  // Save the publish freq.
  publish_freq_ = publish_freq;

  // Load message must be sent at least once.
  need_to_send_load_message_ = true;

  query_object_input_port_index_ =
      this->DeclareAbstractInputPort("query_object", drake::Value<QueryObject<T>>()).get_index();

  // Declare publish mechanism.
  const double kOffset = 0.0;
  this->DeclarePeriodicPublishEvent(1.0 / publish_freq, kOffset, &Visualizer<T>::SendVisualizationMessages);
}

template <typename T>
Visualizer<T>::~Visualizer() {
  // Allow visualizer to be constructed again.
  constructed_ = false;

  // Remove the publish frequency.
  publish_freq_ = std::optional<double>();
}

template <typename T>
EventStatus Visualizer<T>::SendVisualizationMessages(const Context<T>& context) const {
  // Re-create the frame data. We do this every step since we have no indication of when it has changed over the input
  // port.
  query_port_frame_data_.clear();

  if (query_object_input_port().HasValue(context)) {
    const drake::geometry::SceneGraphInspector<T>& inspector =
        query_object_input_port().template Eval<QueryObject<T>>(context).inspector();

    // First, add all of the query port data.
    for (const drake::geometry::FrameId& frame_id : inspector.all_frame_ids()) {
      if (frame_id == inspector.world_frame_id()) continue;
      const int count = inspector.NumGeometriesForFrameWithRole(frame_id, Role::kIllustration);
      if (count > 0) {
        query_port_frame_data_.push_back(
            {frame_id, count, inspector.GetOwningSourceName(frame_id) + "::" + inspector.GetName(frame_id)});
      }
    }
  }

  // If the load message hasn't been sent, send it.
  if (need_to_send_load_message_) {
    SendLoadRobotVisualizationMessage(context, query_port_frame_data_, non_port_based_data_);
    need_to_send_load_message_ = false;
  }

  // Send the draw message.
  SendDrawRobotVisualizationMessage(context, query_port_frame_data_, non_port_based_data_);

  return EventStatus::Succeeded();
}

template <typename T>
void Visualizer<T>::SendLoadRobotVisualizationMessage(
    const Context<T>& context, const RealTimeVector<DynamicFrameData, kMaxFrames>& query_port_frame_data,
    const RealTimeVector<NonPortBasedData, kMaxNonPortBasedData>& non_port_based_data) const {
  drake::lcmt_viewer_load_robot message{};

  message.num_links = 0;
  message.link.resize(non_port_based_data.size());

  if (query_object_input_port().HasValue(context)) {
    const drake::geometry::SceneGraphInspector<T>& inspector =
        query_object_input_port().template Eval<QueryObject<T>>(context).inspector();

    // Add the world frame if it has geometries with the specified role.
    const int anchored_count =
        inspector.NumGeometriesForFrameWithRole(inspector.world_frame_id(), Role::kIllustration);
    const int frame_count = static_cast<int>(query_port_frame_data.size()) + (anchored_count > 0 ? 1 : 0);
    message.link.resize(frame_count + non_port_based_data.size());

    // Load anchored geometry into the world frame.
    if (anchored_count > 0) {
      message.link[message.num_links].name = "world";
      message.link[message.num_links].robot_num = 0;
      message.link[message.num_links].num_geom = anchored_count;
      message.link[message.num_links].geom.resize(anchored_count);
      int geom_index = -1;  // We'll pre-increment before using.
      for (const GeometryId& g_id : inspector.GetGeometries(inspector.world_frame_id(), Role::kIllustration))
        message.link[message.num_links].geom[++geom_index] = MakeGeometry(inspector, g_id, {});
      ++message.num_links;
    }

    // Load dynamic geometry into their own frames.
    for (const auto& [frame_id, geometry_count, name] : query_port_frame_data) {
      message.link[message.num_links].name = name;
      message.link[message.num_links].robot_num = inspector.GetFrameGroup(frame_id);
      message.link[message.num_links].num_geom = geometry_count;
      message.link[message.num_links].geom.resize(geometry_count);
      int geom_index = -1;  // We'll pre-increment before using.
      for (const GeometryId& g_id : inspector.GetGeometries(frame_id, Role::kIllustration))
        message.link[message.num_links].geom[++geom_index] = MakeGeometry(inspector, g_id, {});
      ++message.num_links;
    }
  }

  // TODO(drum): It's hard to ensure that the MakeGeometry(.) function is real-time-safe. Think of a workaround.

  // Load non-port-based geometries.
  for (const NonPortBasedData& non_port_data : non_port_based_data) {
    message.link[message.num_links].name = non_port_data.name;
    message.link[message.num_links].robot_num = non_port_data.robot_number;
    message.link[message.num_links].num_geom = non_port_data.geometries.size();
    message.link[message.num_links].geom =
        std::vector<drake::lcmt_viewer_geometry_data>(non_port_data.geometries.begin(), non_port_data.geometries.end());
    ++message.num_links;
  }

  drake::lcm::Publish(&visualization_lcm_, "DRAKE_VIEWER_LOAD_ROBOT", message, context.get_time());
}

template <typename T>
void Visualizer<T>::SendDrawRobotVisualizationMessage(
    const Context<T>& context, const RealTimeVector<DynamicFrameData, kMaxFrames>& query_port_frame_data,
    const RealTimeVector<NonPortBasedData, kMaxNonPortBasedData>& non_port_based_data) const {
  drake::lcmt_viewer_draw message{};

  const int frame_count = static_cast<int>(query_port_frame_data.size());

  message.num_links = 0;
  message.timestamp = static_cast<int64_t>(context.get_time() * 1000.0);

  auto increment_size_fn = [&message]() {
    ++message.num_links;
    message.link_name.resize(message.num_links);
    message.robot_num.resize(message.num_links);
    message.position.resize(message.num_links);
    message.quaternion.resize(message.num_links);
  };

  auto set_pose_fn = [](const RigidTransform<T>& X_WF, int i, drake::lcmt_viewer_draw* msg) {
    msg->position[i].resize(3);
    msg->position[i][0] = X_WF.translation()[0];
    msg->position[i][1] = X_WF.translation()[1];
    msg->position[i][2] = X_WF.translation()[2];

    const Eigen::Quaternion<double> q = X_WF.rotation().ToQuaternion();
    msg->quaternion[i].resize(4);
    msg->quaternion[i][0] = q.w();
    msg->quaternion[i][1] = q.x();
    msg->quaternion[i][2] = q.y();
    msg->quaternion[i][3] = q.z();
  };

  int i = 0;

  // Update the pose of every object coming in over the QueryObject input port.
  if (query_object_input_port().HasValue(context)) {
    const auto& query_object = query_object_input_port().template Eval<QueryObject<T>>(context);
    const drake::geometry::SceneGraphInspector<T>& inspector = query_object.inspector();
    for (; i < frame_count; ++i) {
      increment_size_fn();
      const drake::geometry::FrameId frame_id = query_port_frame_data[i].frame_id;
      message.robot_num[i] = inspector.GetFrameGroup(frame_id);
      message.link_name[i] = query_port_frame_data[i].name;

      const drake::math::RigidTransform<T>& X_WF = query_object.GetPoseInWorld(frame_id);
      set_pose_fn(X_WF, i, &message);
    }
  }

  // Update the pose of all non-port-based objects.
  for (const NonPortBasedData& non_port_data : non_port_based_data) {
    if (!non_port_data.active)
      continue;
    increment_size_fn();
    message.link_name[i] = non_port_data.name;
    message.robot_num[i] = non_port_data.robot_number;
    set_pose_fn(non_port_data.X_WF, i, &message);
    ++i;
  }

  drake::lcm::Publish(&visualization_lcm_, "DRAKE_VIEWER_DRAW", message, context.get_time());
}

template <typename T>
void Visualizer<T>::UpdatePose(const VisualizerIndex index, const drake::math::RigidTransformd& X_WF) {
  DR_DEMAND(non_port_based_data_[index].active);
  non_port_based_data_[index].X_WF = X_WF;
}

template <typename T>
template <int kVectorSize>
void Visualizer<T>::UpdatePoses(
    const MultibodyPlant<T>& plant, const SceneGraph<T>& scene_graph,
    const Context<T>& scene_graph_context,
    const RealTimeVector<std::pair<BodyIndex, VisualizerIndex>, kVectorSize>& body_indices_to_visualizer_indices) {
  const auto& query_object = scene_graph.get_query_output_port().template Eval<QueryObject<T>>(scene_graph_context);
  const drake::geometry::SceneGraphInspector<T>& inspector = query_object.inspector();

  for (const auto& [body_index, visualization_index] : body_indices_to_visualizer_indices) {
    const std::vector<GeometryId>& visual_geometries = plant.GetVisualGeometriesForBody(plant.get_body(body_index));
    if (visual_geometries.empty()) continue;
    const drake::geometry::FrameId frame_id = inspector.GetFrameId(visual_geometries.front());
    const drake::math::RigidTransformd& X_WF = query_object.GetPoseInWorld(frame_id);
    UpdatePose(visualization_index, X_WF);
  }
}

template <typename T>
template <int kVectorSize>
RealTimeVector<std::pair<BodyIndex, VisualizerIndex>, kVectorSize> Visualizer<T>::AddVisualization(
    const MultibodyPlant<T>& plant, const SceneGraph<T>& scene_graph, const Context<T>& scene_graph_context,
    const std::function<drake::geometry::Rgba(const drake::geometry::Rgba&)>& color_transform_fn,
    bool visualize_fixed_bodies) {
  need_to_send_load_message_ = true;

  const drake::geometry::SceneGraphInspector<T>& inspector =
      scene_graph.get_query_output_port().template Eval<QueryObject<T>>(scene_graph_context).inspector();

  RealTimeVector<int, kVectorSize> model_instance_to_robot_number_mapping;
  model_instance_to_robot_number_mapping.resize(plant.num_model_instances());
  std::fill_n(model_instance_to_robot_number_mapping.begin(), plant.num_model_instances(), -1);

  // Loop through all bodies in all model instances.
  RealTimeVector<std::pair<BodyIndex, VisualizerIndex>, kVectorSize> mapping;
  for (BodyIndex i(0); i < plant.num_bodies(); ++i) {
    RealTimeVector<drake::lcmt_viewer_geometry_data, kMaxGeometries> lcm_geometric_data;
    const Body<T>& body = plant.get_body(i);
    if (!body.is_floating() && !visualize_fixed_bodies) continue;
    const std::vector<GeometryId>& visual_geometries = plant.GetVisualGeometriesForBody(plant.get_body(i));
    DR_DEMAND(visual_geometries.size() <= kMaxGeometries);
    for (const GeometryId g_id : visual_geometries)
      lcm_geometric_data.push_back(MakeGeometry(inspector, g_id, color_transform_fn));

    // See whether a new robot number is needed.
    int& robot_number = model_instance_to_robot_number_mapping[body.model_instance()];
    if (robot_number == -1) robot_number = next_non_port_robot_index_++;

    mapping.push_back(
        std::make_pair(i, AddVisualization(plant.get_body(i).name(), robot_number, std::move(lcm_geometric_data))));
  }

  next_non_port_robot_index_ += plant.num_model_instances();

  return mapping;
}

template <typename T>
VisualizerIndex Visualizer<T>::AddVisualization(
    const std::string_view& name,
    int robot_number,
    RealTimeVector<drake::lcmt_viewer_geometry_data, kMaxGeometries>&& geometries) {
  need_to_send_load_message_ = true;

  // Try to find an empty element to slot this data into.
  VisualizerIndex added_index;
  for (int i = 0; i < non_port_based_data_.size(); ++i) {
    if (!non_port_based_data_[i].active) {
      added_index = VisualizerIndex(i);
      break;
    }
  }

  if (!added_index.is_valid()) {
    added_index = VisualizerIndex(non_port_based_data_.size());
    non_port_based_data_.AddUninitializedElement();
  }

  if (robot_number < 0)
    robot_number = next_non_port_robot_index_++;

  NonPortBasedData& data = non_port_based_data_[added_index];
  data.name = name;
  data.robot_number = robot_number;
  data.active = true;
  data.geometries = std::move(geometries);

  return added_index;
}

template <typename T>
void Visualizer<T>::RemoveVisualization(VisualizerIndex i) {
  DR_DEMAND(!non_port_based_data_[i].active);
  non_port_based_data_[i].active = false;
}

// Instantiate templates.
constexpr int kVectorSize = 128;
template class Visualizer<double>;
template RealTimeVector<std::pair<BodyIndex, VisualizerIndex>, kVectorSize> Visualizer<double>::AddVisualization(
    const MultibodyPlant<double>&, const SceneGraph<double>&, const Context<double>&,
    const std::function<drake::geometry::Rgba(const drake::geometry::Rgba&)>&, bool);
template void Visualizer<double>::UpdatePoses(
    const MultibodyPlant<double>&, const SceneGraph<double>&, const Context<double>&,
    const RealTimeVector<std::pair<BodyIndex, VisualizerIndex>, kVectorSize>&);

}  // namespace DR

