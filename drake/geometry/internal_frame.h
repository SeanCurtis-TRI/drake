#pragma once

#include <unordered_set>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_index.h"

namespace drake {
namespace geometry {
namespace internal {

/** This class represents the internal representation of a GeometryFrame. It
 includes the user-specified data (name and frame group), excludes the pose
 data, and then includes topology data.

 It is not intended to be used outside of the geometry library. To instantiate
 frames in GeometryWorld, use the drake::geometry::GeometryFrame class in
 conjunction with the GeometryWorld::RegisterFrame() methods.
 */
class InternalFrame {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(InternalFrame)

  /** Default constructor. The parent identifier and pose index will be
   invalid. */
  InternalFrame() {}

  /** Full constructor.
   @param source_id     The identifier of the source this belongs to.
   @param frame_id      The identifier of _this_ frame.
   @param name          The name of the frame.
   @param frame_group   The frame's frame group membership.
   @param pose_index    The position in the pose vector of this frame's last
                        known pose.
   @param parent_id     The id of the parent frame.
   */
  InternalFrame(SourceId source_id, FrameId frame_id, const std::string &name,
                int frame_group, PoseIndex pose_index, FrameId parent_id) :
      source_id_(source_id),
      id_(frame_id),
      name_(name),
      frame_group_(frame_group),
      pose_index_(pose_index),
      parent_id_(parent_id) {}

  /** Compares two %InternalFrame instances for "equality". Two internal frames
   are considered equal if they have the same frame identifier. */
  bool operator==(const InternalFrame &other) const {
    return id_ == other.id_;
  }

  /** Compares two %InternalFrame instances for inequality. See operator==()
   for the definition of equality. */
  bool operator!=(const InternalFrame &other) const {
    return !(*this == other);
  }

  SourceId get_source_id() const { return source_id_; }
  FrameId get_id() const { return id_; }
  const std::string &get_name() const { return name_; }
  int get_frame_group() const { return frame_group_; }
  PoseIndex get_pose_index() const { return pose_index_; }
  /** Returns true if this frame is the child of the identified frame. */
  bool has_parent(FrameId parent) const { return parent_id_ == parent; }
  FrameId get_parent_frame_id() const { return parent_id_; }
  const std::unordered_set<FrameId>& get_child_frames() const {
    return child_frames_;
  }
  std::unordered_set<FrameId>* get_mutable_child_frames() {
    return &child_frames_;
  }
  const std::unordered_set<GeometryId>& get_child_geometries() const {
    return child_geometries_;
  }
  std::unordered_set<GeometryId>* get_mutable_child_geometries() {
    return &child_geometries_;
  }
  bool has_child(FrameId frame_id) const {
    return child_frames_.find(frame_id) != child_frames_.end();
  }
  void add_child(FrameId frame_id) {
    child_frames_.insert(frame_id);
  }
  void remove_child(FrameId frame_id) {
    child_frames_.erase(frame_id);
  }
  bool has_child(GeometryId geometry_id) const {
    return child_geometries_.find(geometry_id) != child_geometries_.end();
  }
  void add_child(GeometryId geometry_id) {
    child_geometries_.insert(geometry_id);
  }
  void remove_child(GeometryId geometry_id) {
    child_geometries_.erase(geometry_id);
  }

 private:
  // The identifier of the source, to which this frame belongs.
  SourceId source_id_;

  // The identifier of this frame.
  FrameId id_;

  // The name of the frame. Must be unique across frames from the same
  // geometry source.
  std::string name_;

  // TODO(SeanCurtis-TRI): Consider whether this should be an Identifier or
  // TypeSafeIndex type.
  // The frame group to which this frame belongs.
  int frame_group_{0};

  // TODO(SeanCurtis-TRI): Use default constructor when the type safe index
  // default value PR lands.
  // The index in the pose vector where this frame's pose lives.
  PoseIndex pose_index_{0};

  // The identifier of this frame's parent frame.
  FrameId parent_id_;

  // The identifiers of the frames, who have this frame as parent.
  std::unordered_set<FrameId> child_frames_;

  // The identifiers of the geometries that are rigidly affixed to this frame.
  // This includes geometries that were hung directly on the frame and those
  // that were hung on geometries that were already rigidly affixed.
  // It does *not* include geometries hung on child frames.
  std::unordered_set<GeometryId> child_geometries_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake

namespace std {
/** Enables use of the %InternalFrame to serve as a key in STL containers.
 @relates InternalFrame
 */
template <>
struct hash<drake::geometry::internal::InternalFrame> {
  size_t operator()(
      const drake::geometry::internal::InternalFrame& frame) const {
    return hash<drake::geometry::FrameId>()(frame.get_id());
  }
};
}  // namespace std
