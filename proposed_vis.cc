// NOTE: {Geometry|Frame}RangeIterators may simply be std::vector of things.

namespace geometry {
/** A class for walking the SceneGraph tree.  */
class TreeWalker {
 public:
  /** Provides a range iterator over all geometries that are rigidly
  affixed to the world frame with the given role.  */
  const GeometryRangeIterator GetAnchoredGeometry(Role role) const;

  /** Provides a range iterator over all frames (except the world frame)
  that have geometry rigidly affixed to them with the given role. The
  range iterator guarantees that a frame's parent will always be visited
  before the frame itself.  */
  const FrameRangeIterator GetFrames(Role role) const;
};

// The following classes are not to be taken literally - this may be classes that
// already exist repurposed for this practice. But to simplify things, I've
// defined unique structs here.

struct Geometry {
  std::string name;
  Shape* shape;
  RigidTransform X_FG;
  IllustrationProperties properties;
};

struct Frame {
  Frame* parent;                      // Parent frame
  std::string name                    // name of the frame.
  std::string group_name;             // The name assocaited with the `group_num`.
  int group_num;                      // Note: drake visualizer LCM message consumes *number*.
  RigidTransform X_PF;
  std::vector<Geometry> geometries;
};
}

/**
 The initialization call back for a Visualization system.
   @param context      The context for the SceneGraph.
   @param scene_graph  The scene graph system to visualize from.
 */
template <typename T>
void InitCallback(const systems::Context<T>& context) {
  lcmt_viewer_load_robot message{};

  const geometry::TreeWalker<T>& tree_walker =
      this->EvalAbstractInput(context, tree_walker_port_)
          ->template GetValue<geometry::TreeWalker<T>>();

  const auto& anchored_geometries =
      tree_walker.GetAnchoredGeometry(geometry::Role::kIllustration);
  const auto& frames = tree_walker.frames(geometry::Role::kIllustration);

  const int frame_count = frames.size() +
      anchored_geometries.size() > 0 ? 1 : 0;

  message.num_links = frame_count;
  message.link.reserve(frame_count);

  if (anchored_geometries.size()) {
    message.link.push_back();
    message.link[0].name = "world";
    message.link[0].robot_num = 0;
    message.link[0].num_geom = anchored_geometries.size();
    message.link[0].geom.reserve(anchored_geometries.size());

    for (const geometry::Geometry& geometry : anchored_geometries) {
      const IllustrationProperties& props = geometry.properties();
      const Eigen::Vector4d& color =
          properties.GetPropertOrDefault("phong", "diffuse", kDefaultDiffuse);
      // NOTE: MakeGeometryData() is defined as in geometry_visualization.cc.
      message.link[0].geom.push_back(
          MakeGeometryData(geometry.shape, geometry.X_FG, color));
    }
  }

  for (const geometry::Frame& frame : frames) {
    // Add this frame.
    message.link.push_back();
    drake::lcmt_viewer_link_data& link = message.link.back();
    link.name = frame.name;
    link.robot_num = frame.group_num;
    link.num_geom = frame.geometries.size();
    link.geom.resize(frame.geometries.size());
    for (const geometry::Geometry& geometry : frame.geometries) {
      // Add this geometry
      const IllustrationProperties& props =
          geometry.properties();
      const Eigen::Vector4d& color =
          properties.GetPropertOrDefault("phong", "diffuse", kDefaultDiffuse);
      link.geom.push_back(
          MakeGeometryData(geometry.shape, geometry.X_FG, color));
    }
  }
}

template <typename T>
void UpdateCallback(const systems::Context<T>& context,
                    MyPoseFormat<T>* poses) {
  const geometry::TreeWalker<T>& tree_walker =
      this->EvalAbstractInput(context, tree_walker_port_)
          ->template GetValue<geometry::TreeWalker<T>>();

  lcmt_viwer_draw message;
  const int n = static_cast<int>(tree_walker.frames().size());

  message.timestamp = static_cast<int64_t>(context.get_time() * 1000.0);
  message.num_links = n;
  message.link_name.resize(n);
  message.robot_num.resize(n);
  message.position.resize(n);
  message.quaternion.resize(n);

  int i = 0;
  for (const geometry::Frame& frame : frames) {
    message.robot_num[i] = frame.group_num;
    message.link_name[i] = frame.name;

    Eigen::Translation<double, 3> t(frame.X_PF.translation());
    message.position[i].resize(3);
    message.position[i][0] = t.x();
    message.position[i][1] = t.y();
    message.position[i][2] = t.z();

    Eigen::Quaternion<double> q(frame.X_PF.linear());
    message.quaternion[i].resize(4);
    message.quaternion[i][0] = q.w();
    message.quaternion[i][1] = q.x();
    message.quaternion[i][2] = q.y();
    message.quaternion[i][3] = q.z();
    ++i;
  }
}
