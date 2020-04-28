#pragma once

#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {

/** The %QueryObject serves as a mechanism to perform geometry queries on the
 world's geometry. The SceneGraph has an abstract-valued port that contains
 a  %QueryObject (i.e., a %QueryObject-valued output port).

 To perform geometry queries on SceneGraph:
   - a LeafSystem must have a %QueryObject-valued input port and connect it to
     the corresponding query output port on SceneGraph,
   - the querying LeafSystem can evaluate the input port, retrieving a `const
     QueryObject&` in return, and, finally,
   - invoke the appropriate method on the %QueryObject.

 The const reference returned by the input port is considered "live" - it is
 linked to the context, system, and cache (making full use of all of those
 mechanisms). This const reference should _never_ be persisted; doing so can
 lead to erroneous query results. It is simpler and more advisable to acquire it
 for evaluation in a limited scope (e.g., CalcTimeDerivatives()) and then
 discard it. If a %QueryObject is needed for many separate functions in a
 LeafSystem, each should re-evaluate the input port. The underlying caching
 mechanism should make the cost of this negligible.

 The %QueryObject _can_ be copied. The copied instance is no longer "live"; it
 is now "baked". Essentially, it freezes the state of the live scene graph in
 its current configuration and disconnects it from the system and context. This
 means, even if the original context changes values, the copied/baked instance
 will always reproduce the same query results. This baking process is not cheap
 and should not be done without consideration.

 <h2>Queries and scalar type</h2>

 A %QueryObject _cannot_ be converted to a different scalar type. A %QueryObject
 of scalar type T can only be acquired from the output port of a SceneGraph
 of type T evaluated on a corresponding Context, also of type T.

 %QueryObject's support for arbitrary scalar type is incomplete. Not all queries
 support all scalar types to the same degree. In some cases the level of support
 is obvious (such as when the query is declared *explicitly* in terms of a
 double-valued scalar -- see ComputePointPairPenetration()). In other cases,
 where the query is expressed in terms of scalar `T`, the query may have
 restrictions. If a query has restricted scalar support, it is included in
 the query's documentation.

 @tparam_nonsymbolic_scalar
*/
template <typename T>
class QueryObject {
 public:
  /**
   @anchor render_queries
   @name                Render Queries

   The methods support queries along the lines of "What do I see?" They support
   simulation of sensors. External entities define a sensor camera -- its
   extrinsic and intrinsic properties and %QueryObject renders into the
   provided image.

   <!-- TODO(SeanCurtis-TRI): Currently, pose is requested as a transform of
   double. This puts the burden on the caller to be compatible. Provide
   specializations for AutoDiff and symbolic (the former extracts a
   double-valued transform and the latter throws). -->
   */
  //@{

  /** Renders an RGB image for the given `camera` posed with respect to the
   indicated parent frame P.

   @param camera                The intrinsic properties of the camera.
   @param parent_frame          The id for the camera's parent frame.
   @param X_PC                  The pose of the camera body in the world frame.
   @param show_window           If true, the render window will be displayed.
   @param[out] color_image_out  The rendered color image. */
  void RenderColorImage(const render::CameraProperties& camera,
                        FrameId parent_frame,
                        const math::RigidTransformd& X_PC,
                        bool show_window,
                        systems::sensors::ImageRgba8U* color_image_out) const;

  /** Renders a depth image for the given `camera` posed with respect to the
   indicated parent frame P.

   In contrast to the other rendering methods, rendering depth images doesn't
   provide the option to display the window; generally, basic depth images are
   not readily communicative to humans.

   @param camera                The intrinsic properties of the camera.
   @param parent_frame          The id for the camera's parent frame.
   @param X_PC                  The pose of the camera body in the world frame.
   @param[out] depth_image_out  The rendered depth image. */
  void RenderDepthImage(const render::DepthCameraProperties& camera,
                        FrameId parent_frame,
                        const math::RigidTransformd& X_PC,
                        systems::sensors::ImageDepth32F* depth_image_out) const;

  /** Renders a label image for the given `camera` posed with respect to the
   indicated parent frame P.

   @param camera                The intrinsic properties of the camera.
   @param parent_frame          The id for the camera's parent frame.
   @param X_PC                  The pose of the camera body in the world frame.
   @param show_window           If true, the render window will be displayed.
   @param[out] label_image_out  The rendered label image. */
  void RenderLabelImage(const render::CameraProperties& camera,
                        FrameId parent_frame,
                        const math::RigidTransformd& X_PC,
                        bool show_window,
                        systems::sensors::ImageLabel16I* label_image_out) const;

  //@}

};

}  // namespace geometry
}  // namespace drake
