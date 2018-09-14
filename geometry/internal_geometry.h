#pragma once

#include <memory>
#include <string>
#include <unordered_set>
#include <utility>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_index.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {

// TODO(SeanCurtis-TRI): Include additional user-specified payload metadata when
// added to the declaration of GeometryInstance.
/** Base class for the internal representation of registered geometry. It
 includes the data common to both anchored and dynamic geometry. */
class InternalGeometryBase {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(InternalGeometryBase)

  /** Default constructor. The geometry id will be invalid, the shape will
   be nullptr, and the pose will be uninitialized. */
  InternalGeometryBase() {}

  virtual ~InternalGeometryBase() {}

  /** Constructs the base internal geometry without any assigned roles.
   @param shape         The shape specification for this instance.
   @param geometry_id   The identifier for _this_ geometry.
   @param name          The name of the geometry.
   @param X_PG          The pose of the geometry G in the parent frame P. P may
                        be the frame of another geometry and not the dynamic
                        frame this geometry is ultimately rigidly affixed to.
                        It simply refers to whatever parent with respect to
                        which the instance was originally specified.
   @param index         The internal index of this internal geometry (w.r.t.
                        its anchored/dynamic status).  */
  InternalGeometryBase(std::unique_ptr<Shape> shape, GeometryId geometry_id,
                       const std::string& name, const Isometry3<double>& X_PG,
                       InternalIndex index)
      : shape_spec_(std::move(shape)),
        id_(geometry_id),
        name_(name),
        X_PG_(X_PG),
        internal_index_(index) {}

  /** Compares two %InternalGeometryBase instances for "equality". Two internal
   geometries are considered equal if they have the same geometry identifier. */
  bool operator==(const InternalGeometryBase &other) const {
    return id_ == other.id_;
  }

  /** Compares two %InternalGeometry instances for inequality. See operator==()
   for the definition of equality. */
  bool operator!=(const InternalGeometryBase &other) const {
    return !(*this == other);
  }

  const Shape& shape() const { return *shape_spec_; }

  GeometryId id() const { return id_; }

  const std::string& name() const { return name_; }

  const Isometry3<double>& pose_in_parent() const { return X_PG_; }

  InternalIndex internal_index() const { return internal_index_; }

  void SetRole(ProximityProperties properties) {
    if (proximity_props_) {
      throw std::logic_error("Geometry already has proximity role assigned");
    }
    proximity_props_ = std::move(properties);
  }

  void SetRole(IllustrationProperties properties) {
    if (illustration_props_) {
      throw std::logic_error("Geometry already has illustration role assigned");
    }
    illustration_props_ = std::move(properties);
  }

  void SetRole(PerceptionProperties properties) {
    if (perception_props_) {
      throw std::logic_error("Geometry already has perception role assigned");
    }
    perception_props_ = std::move(properties);
  }

  /** Reports if the geometry has the indicated `role`.  */
  bool has_role(Role role) const;

  /** Reports if the geometry has a proximity role. */
  bool has_proximity_role() const { return proximity_props_ != nullopt; }

  /** Returns a pointer to the geometry's proximity properties (if they are
   defined. Nullptr otherwise.  */
  const ProximityProperties* proximity_properties() const {
    if (proximity_props_) return &*proximity_props_;
    return nullptr;
  }

  /** Reports if the geometry has a illustration role. */
  bool has_illustration_role() const { return illustration_props_ != nullopt; }

  /** Returns a pointer to the geometry's illustration properties (if they are
   defined. Nullptr otherwise.  */
  const IllustrationProperties* illustration_properties() const {
    if (illustration_props_) return &*illustration_props_;
    return nullptr;
  }

  /** Reports if the geometry has a perception role. */
  bool has_perception_role() const { return perception_props_ != nullopt; }

  /** Returns a pointer to the geometry's perception properties (if they are
   defined. Nullptr otherwise.  */
  const PerceptionProperties* perception_properties() const {
    if (perception_props_) return &*perception_props_;
    return nullptr;
  }

  RenderIndex render_index() const { return render_index_; }
  void set_render_index(RenderIndex index) { render_index_ = index; }

 private:
  // The specification for this instance's shape.
  copyable_unique_ptr<Shape> shape_spec_;

  // The identifier for this frame.
  GeometryId id_;

  // The name of the geometry. Must be unique among geometries attached to the
  // same frame.
  std::string name_;

  // The pose of this geometry in the parent frame. The parent may be a frame or
  // another registered geometry.
  Isometry3<double> X_PG_;

  // The index of this geometry in the "full" set of geometries (regardless of
  // role). However, its scope is limited to either the dynamic set or the
  // anchored set.
  InternalIndex internal_index_{};

  // TODO(SeanCurtis-TRI): Consider introducing a mechanism where these are
  // defined at the frame level, and all child geometries inherit.

  // The optional property sets tied to the roles that the geometry plays.
  optional<ProximityProperties> proximity_props_{nullopt};
  optional<IllustrationProperties> illustration_props_{nullopt};
  optional<PerceptionProperties> perception_props_{nullopt};
  RenderIndex render_index_{};
};

/** This class represents the internal representation of registered _dynamic_
 geometry. It includes the user-specified meta data (e.g., name) and internal
 topology representations. */
class InternalGeometry final : public InternalGeometryBase {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(InternalGeometry)

  /** Default constructor. The parent and frame ids will be invalid as well as
   the state documented in InternalGeometryBase(). */
  InternalGeometry();

  /** Constructs internal geometry without any assigned role.
   @param shape           The shape specification for this instance.
   @param frame_id        The identifier of the frame this belongs to.
   @param geometry_id     The identifier for _this_ geometry.
   @param name            The name of the geometry.
   @param X_PG            The pose of the geometry G in the parent frame P. The
                          parent may be a frame, or another registered geometry.
   @param internal_index  The internal index of this geometry (in the dynamic
                          set).
   @param parent_id       The optional id of the parent geometry. */
  InternalGeometry(std::unique_ptr<Shape> shape, FrameId frame_id,
                   GeometryId geometry_id, const std::string& name,
                   const Isometry3<double>& X_PG, InternalIndex internal_index,
                   const optional<GeometryId>& parent_id = {});

  FrameId frame_id() const { return frame_id_; }

  optional<GeometryId> parent_id() const { return parent_id_; }

  void set_parent_id(GeometryId id) { parent_id_ = id; }

  DynamicProximityIndex proximity_index() const { return proximity_index_; }

  void set_proximity_index(DynamicProximityIndex index) {
    proximity_index_ = index;
  }

  /** Returns true if this geometry has a geometry parent and the parent has the
   given `geometry_id`. */
  bool is_child_of_geometry(GeometryId geometry_id) const {
    return parent_id_ && *parent_id_ == geometry_id;
  }

  /** Returns true if the geometry is affixed to the frame with the given
   `frame_id`. */
  bool is_child_of_frame(FrameId frame_id) const {
    return frame_id == frame_id_;
  }

  const std::unordered_set<GeometryId>& child_geometry_ids() const {
    return child_geometry_ids_;
  }
  std::unordered_set<GeometryId>* mutable_child_geometry_ids() {
    return &child_geometry_ids_;
  }

  /** Returns true if this geometry has a child geometry with the given
   `geometry_id`. */
  bool has_child(GeometryId geometry_id) const {
    return child_geometry_ids_.find(geometry_id) != child_geometry_ids_.end();
  }

  /** Adds a geometry with the given `geometry_id` to this geometry's set of
   children. */
  void add_child(GeometryId geometry_id) {
    child_geometry_ids_.insert(geometry_id);
  }

  /** Removes the given `geometry_id` from this geometry's set of children. If
   the id is not in the set, nothing changes. */
  void remove_child(GeometryId geometry_id) {
    child_geometry_ids_.erase(geometry_id);
  }

 private:
  // The identifier of the frame to which this geometry belongs.
  FrameId frame_id_;

  // The index of the geometry in the engine.
  DynamicProximityIndex proximity_index_{};

  // The identifier for this frame's parent frame.
  optional<GeometryId> parent_id_;

  // The identifiers for the geometry hung on this frame.
  std::unordered_set<GeometryId> child_geometry_ids_;
};

/** This class represents the internal representation of registered _anchored_
 geometry. It includes the user-specified meta data (e.g., name) and internal
 topology representations. */
class InternalAnchoredGeometry final : public InternalGeometryBase {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(InternalAnchoredGeometry)

  /** Default constructor. State will be as documented in
   InternalGeometryBase(). */
  InternalAnchoredGeometry();

  /** Constructs anchored internal geometry without any role assigned.
   @param shape           The shape specification for this instance.
   @param geometry_id     The identifier for _this_ geometry.
   @param name            The name of the geometry.
   @param X_WG            The pose of the geometry G in the world frame W.
   @param internal_index  The internal index of this geometry (in the dynamic
                          set). */
  InternalAnchoredGeometry(std::unique_ptr<Shape> shape, GeometryId geometry_id,
                           const std::string& name,
                           const Isometry3<double> X_WG,
                           InternalIndex internal_index);

  /** Sets the geometry's proximity index -- if it has a geometry role. This
   should always be called in conjunction with assigning the proximity
   properties.  */
  void set_proximity_index(AnchoredProximityIndex index) {
    proximity_index_ = index;
  }

  AnchoredProximityIndex proximity_index() const { return proximity_index_; }

 private:
  // The index of the geometry in the engine.
  AnchoredProximityIndex proximity_index_{};
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
