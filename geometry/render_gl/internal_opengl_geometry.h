#pragma once

#include <array>
#include <limits>
#include <utility>

#include "drake/geometry/render/render_label.h"
#include "drake/geometry/render_gl/internal_opengl_includes.h"
#include "drake/geometry/render_gl/internal_shader_program_data.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace render_gl {
namespace internal {

// TODO(SeanCurtis-TRI): Consider moving this up to RenderEngine; it's useful
//  for multiple RenderEngine types.
/* Rendering types available. Used to index into render-type-dependent data
 structures. Because it serves as an index, we use kTypeCount to declare the
 *number* of index values available (relying on C++'s default behavior of
 assigning sequential values in enumerations).  */
enum RenderType { kColor = 0, kLabel, kDepth, kTypeCount };

/* For a fixed OpenGL context, defines the definition of a mesh geometry. The
 geometry is defined by the handles to various objects in the OpenGL context.
 If the context is changed or otherwise invalidated, these handles will no
 longer be valid.

 The code that constructs instances is completely responsible for guaranteeing
 that the array and buffer values are valid in the OpenGl context and that the
 index buffer size is likewise sized correctly.  */
struct OpenGlGeometry {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(OpenGlGeometry)

  /* Default constructor; the resultant instance is considered "undefined".  */
  OpenGlGeometry() = default;

  /* Constructs an %OpenGlGeometry from the given "object names" OpenGl objects.
   (See e.g.,
   https://www.khronos.org/registry/OpenGL-Refpages/gl4/html/glGenFramebuffers.xhtml
   for an example of where such an "object name" would come from.)

   @param vertex_array_in       The handle to the OpenGl vertex array object
                                containing the mesh's data.
   @param vertex_buffer_in      The handle to the OpenGl vertex buffer
                                containing mesh per-vertex data.
   @param index_buffer_in       The handle to the OpenGl index buffer defining a
                                set of triangles.
   @param index_count_in        The number of indices in the index buffer.
   @pre `index_count_in >= 0`.  */
  OpenGlGeometry(GLuint vertex_array_in, GLuint vertex_buffer_in,
                 GLuint index_buffer_in, int index_count_in)
      : vertex_array{vertex_array_in},
        vertex_buffer{vertex_buffer_in},
        index_buffer{index_buffer_in},
        index_count{index_count_in} {
    if (index_count < 0) {
      throw std::logic_error("Index buffer size must be non-negative");
    }
  }

  /* Reports true if `this` has been defined with "meaningful" values. In this
   case, "meaningful" is limited to "not default initialized". It can't know
   if the values are actually object identifiers in the current OpenGl context.
   */
  bool is_defined() const {
    return vertex_array != kInvalid && vertex_buffer != kInvalid &&
           index_buffer != kInvalid;
  }

  /* Throws an exception with the given `message` if `this` hasn't been
   populated with meaningful values.
   @see if_defined().  */
  void throw_if_undefined(const char* message) const {
    if (!is_defined()) throw std::logic_error(message);
  }

  // TODO(SeanCurtis-TRI): This can't really be a struct; there are invariants
  // that need to be maintained: vertex_array depends on vertex_buffer,
  // and index_count needs to be the actual number of indices stored in
  // index_buffer.
  GLuint vertex_array{kInvalid};
  GLuint vertex_buffer{kInvalid};
  GLuint index_buffer{kInvalid};

  // Parameters for glDrawElements(). See
  // https://registry.khronos.org/OpenGL-Refpages/gl4/html/glDrawElements.xhtml
  int index_count{0};
  GLenum type;
  GLenum mode;

  /* The value of an object (array, buffer) that should be considered invalid.
   */
  static constexpr GLuint kInvalid = std::numeric_limits<GLuint>::max();
};

/* An instance of a geometry in the renderer - a reference to the underlying
 OpenGl geometry definition in Frame G, its pose in the world frame W, and scale
 factors. The scale factors are not required to be uniform. They _can_ be
 negative, but that is not recommended; in addition to mirroring the geometry it
 will also turn the geometry "inside out".

 When rendering, the visual geometry will be scaled around G's origin and
 subsequently posed relative to W.

 There are a number of frames relating to how geometries are handled in
 RenderEngineGl. What they are and how they relate can be confusing. This
 discussion defines the frames, their relationships, and looks at the provenance
 of the values in those relationships (with particular focus on what values are
 stored in each instance and how they are used).

   W: Drake's world frame.
   G: Drake's geometry frame (this is what gets posed by SceneGraph) and is
      what we should think of as the frame of the instance.
   M: An OpenGl "model" frame. This is a conceptual frame relating the parts of
      a prop. It is neither the instance frame nor the frame that OpenGlGeometry
      vertex positions or normals are measured and expressed in. For example,
      the model can be scaled (as with Mesh or Convex shapes) to create the
      effective geometry instance. Also, the instance can be built of multiple
      parts (the constituent pieces of a "prop") and those parts can themselves
      be transformed relative to each other to build the model.
   N: The frame in which the vertex positions and normals of an OpenGlGeometry
      are measured and expressed.

 Relationships between these frames:

   While Drake mostly focuses on rigid transforms between frames (X_AB) and
   relative orientations (R_AB), rendering needs to describe additional
   relationships. So, in addition to the X_ and R_ prefixes, we also use the
   P_, S_, T_, and N_ prefixes:

     P_: A homogenous matrix representing translation. The same as a
         RigidTransform consisting of only a translation.
     S_: A *scale* matrix. A diagonal matrix with the Sx, Sy, and Sz values on
         the diagonal.
     T_: A general affine transform which can include translation, rotation, and
         anisotropic scale. T_AB = P_AB * R_AB * S_AB.
     N_: The transform to apply to a mesh's normals to re-express them in
         another frame. Given T_AB = P_AB * R_AB * S_AB, we can derive the
         corresponding normal transform: N_AB = R_AB * S⁻¹_AB.

   Immutable relationships defined when a Shape is registered with the engine:

      S_GM: The scaling applied to the underlying "model" data to create the
            requested Drake-specified geometry (e.g., usually the scale factor
            associated with a Mesh or Convex).
      S_MN: The scale to apply to the mesh data associated with an
            OpenGlGeometry to create the requested model. For example, we
            store the mesh of a unit sphere, but apply an arbitrary scale to
            model an ellipsoid; this scale matrix contains that transformation.
      T_MN: The pose (not necessarily RigidTransform) of the vertex position
            data associated with a single part in the conceptual frame of the
            prop.
      T_GN: The transform for measuring and expressing the vertex position data
            associated with an OpenGlGeometry in the instance's frame. Defined
            as T_GN = S_GM * T_MN
      N_GN: The transform for expressing the vertex normal data associated with
            an OpenGlGeometry in the instance's frame.

   Relationships that depend on the pose of the instance (as defined by
   SceneGraph):

      X_WG: The pose (RigidTransform) of the Drake geometry frame in Drake's
            world frame. This is provided by SceneGraph and provided to
            RenderEngineGl via UpdateVisualPose() and updates frequently.
      T_WN: T_WG = X_WG * S_GN.
      N_WN: N_WN = R_WG * N_GN.

   With each instance, we store the necessary immutable transforms and, when
   updating the pose of the geometry (X_WG), update the final transforms that
   depend on the instantaneous pose and immutable transforms (see below). */
struct OpenGlInstance {
  /* Previously this struct provided a constructor that would validate these
   quantities. To facilitate designated initializers, we've killed the
   constructor but moved the demanded properties into its on validator. */
  void DemandValid() const {
    DRAKE_DEMAND(shader_data[RenderType::kColor].shader_id().is_valid());
    DRAKE_DEMAND(shader_data[RenderType::kDepth].shader_id().is_valid());
    DRAKE_DEMAND(shader_data[RenderType::kLabel].shader_id().is_valid());
    DRAKE_DEMAND(geometry >= 0);
  }

  /* This is the index to the OpenGlGeometry stored by RenderEngineGl. */
  int geometry{};

  /* The immutable transform mapping vertex position to the instance frame. */
  Eigen::Matrix4f T_GN{Eigen::Matrix3f::Identity()};
  /* The pose-dependent transform mapping vertex position to Drake World. */
  Eigen::Matrix4f T_WN;

  /* The immutable transform mapping vertex normals to the instance frame. */
  Eigen::Matrix3f N_GN{Eigen::Matrix3f::Identity()};
  /* The pose-dependent transform mapping vertex normals to Drake World. */
  Eigen::Matrix3f N_WN;

  std::array<ShaderProgramData, RenderType::kTypeCount> shader_data;
};

}  // namespace internal
}  // namespace render_gl
}  // namespace geometry
}  // namespace drake
