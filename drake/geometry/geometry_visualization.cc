#include "drake/geometry/geometry_visualization.h"

#include "drake/geometry/geometry_state.h"
#include "drake/geometry/geometry_system.h"
#include "drake/geometry/shapes.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmtypes/drake/lcmt_viewer_geometry_data.hpp"
#include "drake/lcmtypes/drake/lcmt_viewer_load_robot.hpp"
#include "drake/math/rotation_matrix.h"
#include "geometry_system.h"

namespace drake {
namespace geometry {

// Computes a full axis whose z-axis is the given z_axis, and x- and y-axes are
// arbitrarily select to create an orthonormal basis. The basis is measured and
// expressed in the same frame that z_axis is.
Matrix3<double> ComputeBasisFromZ(const Vector3<double> z_axis) {
  // Projects the z-axis into the first quadrant in order to identify the
  // *smallest* component of the normal.
  const Vector3<double> u(z_axis.cwiseAbs());
  int minAxis;
  u.minCoeff(&minAxis);
  // The world axis corresponding to the smallest component of the local
  // z-axis will be *most* perpendicular.
  Vector3<double> perpAxis;
  perpAxis << (minAxis == 0 ? 1 : 0), (minAxis == 1 ? 1 : 0),
      (minAxis == 2 ? 1 : 0);
  // Now define x- and y-axes.
  Vector3<double> x_axis = z_axis.cross(perpAxis).normalized();
  Vector3<double> y_axis = z_axis.cross(x_axis);
  // Transformation from world frame to local frame.
  Matrix3<double> R_WL;
  R_WL.col(0) = x_axis;
  R_WL.col(1) = y_axis;
  R_WL.col(2) = z_axis;
  return R_WL;
}

lcmt_viewer_geometry_data MakeGeometryData(const Shape& shape,
                                           const Isometry3<double>& X_GP) {
  lcmt_viewer_geometry_data geometry_data;
  Eigen::Isometry3d transform; // extract X_FG
  switch (shape.get_type()) {
    case Shape::SPHERE: {
      geometry_data.type = geometry_data.SPHERE;
      geometry_data.num_float_data = 1;
      auto sphere = static_cast<const Sphere&>(shape);
      geometry_data.float_data.push_back(static_cast<float>(sphere.get_radius()));
      transform = X_GP;
      break;
    }
    case Shape::HALF_SPACE: {
      // TODO(SeanCurtis-TRI): Modify visualization to support half spaces.
      // Translate it into a box. Assuming it's "centered" at the origin.
      geometry_data.type = geometry_data.BOX;
      geometry_data.num_float_data = 3;
      // Re-align the box so that the normal points in the z-direction. It also
      // needs to be offset so that the "top" of the box aligns with the half
      // space.
      geometry_data.float_data.push_back(50);
      geometry_data.float_data.push_back(50);
      const double thickness = 1;
      geometry_data.float_data.push_back(float(thickness));
      // Set transform based on this voodoo.
      auto half_space = static_cast<const HalfSpace&>(shape);
      Isometry3<double> box_xform = Isometry3<double>::Identity();
      // Shift it down so that the origin lies on the top surface.
      box_xform.translation() << 0, 0, -thickness / 2;
      Isometry3<double> plane_xform = Isometry3<double>::Identity();
      // Reposition it so that the top surface is oriented perpendicular to the
      // normal and positioned at the user-specified point.
      plane_xform.translation() = half_space.get_point_on_plane();
      plane_xform.linear() = ComputeBasisFromZ(half_space.get_normal());
      transform = plane_xform *box_xform;
      break;
    }
    case Shape::UNKNOWN:
      // Intentionally doing nothing -- copied form drake_visualizer_client.cc.
      break;
  }
  // Saves the location and orientation of the visualization geometry in the
  // `lcmt_viewer_geometry_data` object. The location and orientation are
  // specified in the body's frame.
  Eigen::Map<Eigen::Vector3f> position(geometry_data.position);
  position = transform.translation().cast<float>();
  // LCM quaternion must be w, x, y, z.
  Eigen::Quaternion<double> q(transform.rotation());
  geometry_data.quaternion[0] = q.w();
  geometry_data.quaternion[1] = q.x();
  geometry_data.quaternion[2] = q.y();
  geometry_data.quaternion[3] = q.z();

  Eigen::Map<Eigen::Vector4f> color(geometry_data.color);
  Eigen::Vector4d default_color(0.8, 0.8, 0.8, 1.0);
  color = default_color.template cast<float>();

  return geometry_data;
}

void DispatchLoadMessage(const GeometryState<double>& state) {
  using lcm::DrakeLcm;

  lcmt_viewer_load_robot message;
  // Populate the message.
  const int frame_count = state.get_num_frames();
  const int anchored_count =
      static_cast<int>(state.anchored_geometry_index_id_map_.size());

  // Include the world frame as one of the frames (if there are anchored
  // geometries).
  int total_link_count = frame_count + (anchored_count > 0 ? 1 : 0);
  message.num_links = total_link_count;
  message.link.resize(total_link_count);

  int link_index = 0;
  // Load anchored geometry into the world frame.
  {
    if (anchored_count) {
      message.link[0].name = "world";
      message.link[0].robot_num = 0;
      message.link[0].num_geom = anchored_count;
      message.link[0].geom.resize(anchored_count);
      int geom_index = 0;
      for (const auto &pair : state.anchored_geometries_) {
        AnchoredGeometryIndex index = pair.second;
        const Shape &shape = state.geometry_engine_->get_anchored_shape(index);
        // TODO(SeanCurtis-TRI): Fix this when anchored geometry has pose.
        message.link[0].geom[geom_index] = MakeGeometryData(
            shape, Isometry3<double>::Identity());
        ++geom_index;
      }
      link_index = 1;
    }
  }

  // Load dynamic geometry into their own frames.
  for (const auto& pair : state.frames_) {
    const internal::InternalFrame& frame = pair.second;
    // TODO: This frame has to have the same name as when loaded.
    SourceId s_id = state.GetSourceId(frame.get_id());
    const std::string& src_name = state.get_source_name(s_id);
    // This name should be well correlated with the GeometrySystem output.
    message.link[link_index].name = src_name + "::" + frame.get_name();
    message.link[link_index].robot_num = frame.get_frame_group();
    const int geom_count = static_cast<int>(frame.get_child_geometries().size());
    message.link[link_index].num_geom = geom_count;
    message.link[link_index].geom.resize(geom_count);
    int geom_index = 0;
    for (GeometryId geom_id : frame.get_child_geometries()) {
      GeometryIndex index = state.geometries_.at(geom_id).get_engine_index();
      const Shape& shape = state.geometry_engine_->get_shape(index);
      const Isometry3<double> X_WG = state.X_FG_.at(index);
      message.link[link_index].geom[geom_index] = MakeGeometryData(shape, X_WG);
      ++geom_index;
    }
    ++link_index;
  }

  // Send a load message.
  const int message_length = message.getEncodedSize();
  std::vector<uint8_t> message_bytes(message_length);
  message.encode(message_bytes.data(), 0, message_length);
  DrakeLcm lcm;
  lcm.Publish("DRAKE_VIEWER_LOAD_ROBOT", message_bytes.data(),
              message_bytes.size());
}

void DispatchLoadMessage(const GeometrySystem<double>& system) {
  system.ThrowIfContextAllocated();
  DispatchLoadMessage(*system.initial_state_);
}

}  // namespace geometry
}  // namespace drake
