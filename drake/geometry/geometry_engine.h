#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_index.h"

namespace drake {
namespace geometry {

// forward declaration
class Geometry;
template <typename T> class GeometryInstance;

// TODO(SeanCurtis-TRI): Move this somewhere appropriate.
template <typename T>
struct InstanceData {
  /** The name of the instance. */
  std::string name;

  /** The identifier for the instance. */
  GeometryId id;

  // TODO(SeanCurtis-TRI): Materials, etc.

  // TODO(SeanCurtis-TRI): Should this be a void pointer, or should I do some
  // CRTP so that I can stash typed artifacts and get them back out typed?
  // This arbitrary user data allows drivers of the frame to associate
  // information with results from the result of geometric queries.
  void* user_data;
};

/**
 A geometry engine is the underlying engine for computing the results of
 geometric queries.

 It owns the geometry instances and, once it has been provided with the poses
 of the geometry, it provides geometric queries on that geometry. This
 serves as the abstract interface for all engines. Specific implementations
 will implement the query evaluations.

 @internal Historically, this replaces the DrakeCollision::Model class.

 @tparam T The underlying scalar type. Must be a valid Eigen scalar. */
template <typename T>
class GeometryEngine {
 public:
  virtual ~GeometryEngine() {}

  std::unique_ptr<GeometryEngine> Clone() const {
    return std::unique_ptr<GeometryEngine>(DoClone());
  }

  /** @name                    Geometry Management
   @{ */

  /** Reports the _minimum_ vector size required for a successful invocation of
   UpdateWorldPoses(). A vector of this size is guaranteed to span all of the
   active geometry indices. */
  int get_update_input_size() const;

  // TODO(SeanCurtis-TRI): I suspect GeometryInstance should *not* be passed.
  //  1. GeometryWorld is going to handle the computation of world pose. (So the
  //     GeometryInstance::X_PG transform is unused.
  //  2. The result of the geometry query needs access to any meta data that has
  //     been associated with the original instance:
  //        - materials, user data, the original GeometryId so it can be part
  //          of query results.
  //  In fact, GeometryInstance may be correct, but I'm wrong in embedding the
  //  pose *inside* the instance. It should only really contain the Geometry
  //  data. For now, I'll call it InstanceData.
  /** Add movable geometry to the engine. The engine takes ownership of the
   instance. The instances transform is a fixed pose relative to an unknown
   parent frame (which will be determined later.
   @param geometry    The geometry instance to add to the engine.
   @return  An index by which the geometry can be referenced later. */
  // TODO(SeanCurtis-TRI): Include the invocation of the geometry.
  virtual GeometryIndex AddDynamicGeometry(
      std::unique_ptr<GeometryInstance<T>> data) = 0;

  /** Add anchored geometry to the engine. The engine takes ownership of the
   instance. The instance's pose is a fixed pose relative to the _world_ frame
   `W`.
   @param geometry    The geometry instance to add to the engine.
   @return  An index by which the geometry can be referenced later. */
  virtual GeometryIndex AddAnchoredGeometry(
      std::unique_ptr<GeometryInstance<T>> data) = 0;

  /** Provides the poses for all of the geometries in the engine. This vector
   should span the full range of active GeometryIndex values provided by the
   engine. The iᵗʰ entry contains the pose for the frame whose GeometryIndex
   value is `i`.
   @param X_WG     The poses of each geometry `G` measured and expressed in the
                   world frame `W`. */
  virtual void UpdateWorldPoses(const std::vector<Isometry3<T>>& X_WG) = 0;

  //@}

  /** @name                    Geometry Queries
   @{ */
  // TODO(SeanCurtis-TRI): Provide the interface that GeometryQuery invokes.
  //@}
 protected:
  /*! NVI implementation for cloning GeometryEngine instances.
   @return A _raw_ pointers to the newly cloned GeometryEngine instance.
   */
  virtual GeometryEngine* DoClone() const = 0;

 private:
};
}  // namespace geometry
}  // namespace drake
