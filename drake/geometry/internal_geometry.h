#pragma once

#include <unordered_set>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_index.h"

namespace drake {
namespace geometry {
namespace internal {

/** This class represents the internal representation of registered geometry.
 It includes the user-specified data (?? and ??, excluding pose data) and
 includes internal topology representations.
 */
class InternalGeometry {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(InternalGeometry)

  /** Default constructor. The parent identifier and pose index will be
   invalid. */
  InternalGeometry() {}

  /** Full constructor.
   @param frame_id      The identifier of the frame this belongs to.
   @param geometry_id   The identifier for _this_ geometry.
   @param name          The name of the geometry.
   @param engine_index  The position in the geometry engine of this geometry.
   @param parent_id     The optional id of the parent geometry.
   */
  InternalGeometry(FrameId frame_id, GeometryId geometry_id,
                   const std::string &name, GeometryIndex engine_index,
                   const optional<GeometryId>& parent_id = {}) :
  frame_id_(frame_id),
  id_(geometry_id),
  name_(name),
  engine_index_(engine_index),
  parent_id_(parent_id) {}

  /** Compares two %InternalGeometry instances for "equality". Two internal
   frames are considered equal if they have the same geometry identifier. */
  bool operator==(const InternalGeometry &other) const {
    return id_ == other.id_;
  }

  /** Compares two %InternalGeometry instances for inequality. See operator==()
   for the definition of equality. */
  bool operator!=(const InternalGeometry &other) const {
    return !(*this == other);
  }

  FrameId get_frame_id() const { return frame_id_; }
  GeometryId get_id() const { return id_; }
  const std::string& get_name() const { return name_; }
  GeometryIndex get_engine_index() const { return engine_index_; }
  void set_engine_index(GeometryIndex index) { engine_index_ = index; }
  optional<GeometryId> get_parent() const { return parent_id_; }
  bool has_parent(GeometryId geom_id) const {
    return parent_id_ && *parent_id_ == geom_id;
  }
  bool has_frame(FrameId frame_id) const { return frame_id == frame_id_; }
  const std::unordered_set<GeometryId>& get_child_geometries() const {
    return child_geometries_;
  }
  std::unordered_set<GeometryId>* get_mutable_child_geometries() {
    return &child_geometries_;
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
  // The identifier of the frame to which this geometry belongs.
  FrameId frame_id_;

  // The identifier for this frame.
  GeometryId id_;

  // The name of the frame. Must be unique across frames from the same
  // geometry source.
  std::string name_;

  // TODO(SeanCurtis-TRI): Use default constructor when the type safe index
  // default value PR lands.
  // The index in the pose vector where this frame's pose lives.
  GeometryIndex engine_index_{0};

  // The identifier for this frame's parent frame.
  optional<GeometryId> parent_id_;

  // TODO(SeanCurtis-TRI): Should this be a set?
  // The identifiers for the geometry hung on this frame.
  std::unordered_set<GeometryId> child_geometries_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake

namespace std {
/** Enables use of the %InternalGeometry to serve as a key in STL containers.
 @relates InternalGeometry
 */
template <>
struct hash<drake::geometry::internal::InternalGeometry> {
  size_t operator()(
      const drake::geometry::internal::InternalGeometry& geometry) const {
    return hash<drake::geometry::GeometryId>()(geometry.get_id());
  }
};
}  // namespace std
