#pragma once

#include <string>
#include <vector>

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/scene_graph_inspector.h"
#include "drake/multibody/tree/rotational_inertia.h"
#include "drake/systems/framework/context.h"

// TODO(SeanCurtis-TRI): Eventually move this into Anzu. For now, this will be
//  more convenient to edit/test here.

namespace drake {
namespace examples {
namespace slicing {
/** Attempts to write the given mesh formatted as Wavefront Obj to a file with
 the given name.
 @throws std::exception if the file cannot be written. */
template <typename T>
void WriteSurfaceMeshToObj(const geometry::SurfaceMesh<T>& mesh,
                           const std::string& file_name);

/** The result of cutting a mesh. It reports the id of the mesh that this was
 cut from, the path to the file to which it was saved, and the mass properties
 of the mesh. */
template <typename T>
struct MeshCut {
  /** The mesh representing the cut portion. The vertices of the mesh are
   measured and expressed in the Frame C, such that the mesh's center of mass
   is located at Co.*/
  geometry::SurfaceMesh<T> mesh;
  /** The pose of the cut geometry (defined in Frame C) relative to the source
   geometry's Frame G: X_GC.  */
  math::RigidTransform<T> X_SourceCut;
  /** The mass of the cut geometry.  */
  T mass;
  /** The inertia tensor of this cut geometry around the cut geometry's center
   of mass and expressed in the geometry's frame: I_CCcm_C.  */
  multibody::RotationalInertia<T> inertia;
};

/** Slices a mesh into two meshes using a plane. In the simplest usage, a mesh
 (indicated by the given identifier, `mesh_id`) will be cut into two disjoint
 pieces by a plane passing through point P with normal n (both measured and
 expressed in the geometry's frame G). The resulting meshes and correspoding
 data are returned (see MeshCut for details).

 The plane can have non-zero thickness. In this case, the volume of the
 intersecting mesh will be lost. Intuitively, think of it like a laser beam
 cutting the geometry and burning up the material in contact.

 There are several atypical cases, however:
  
   - If `mesh_id` doesn't refer to a Mesh or Convex shape, we throw.
   - If the plane doesn't actually intersect the shape, no meshes are generated.
   - Based on the plane thickness, it is possible to "shave" the mesh such that
     the result is a single mesh with volume. Only that mesh will be returned.

 <!-- TODO(SeanCurtis-TRI) Consider storing the density in the proximity
  properties. -->

 @param mesh_id       The identifier for the mesh to cut.
 @param density       The uniform density of the mesh (kg/m^3) of the mesh being
                      cut.
 @param p_GP          A point P on the cutting plane's surface (measured and
                      expressed in the geometry's frame G).
 @param n_G           The cutting plane's normal. Need not be normalized, but
                      must have "non-trivial" length (i.e., greater than 1e-10).
 @param scene_graph   The SceneGraph instance that owns the geometry.
 @param sg_context    The SceneGraph's context.
 @param thickness     The (optional) plane thickness; defaults to zero.
 @returns Zero or more cut meshes (see comments above).
 @pre n_G.norm() > 1e-10. */
template <typename T>
std::vector<MeshCut<T>> SliceMesh(geometry::GeometryId mesh_id, double density,
                                  const Vector3<T>& p_GP, const Vector3<T> n_G,
                                  const geometry::SceneGraph<T>& scene_graph,
                                  const systems::Context<T>& sg_context,
                                  double thickness = 0.0);

namespace internal {
/* The internal functions are declared here so that they can be effectively
 unit tested. */

/* Creates the final cut mesh (including mass properties and transforms). This
assumes that the mesh implied by the given `vertices_S` and `faces` is a
*closed* mesh (the results will largely be meaningless without that).

The resulting MeshCut will have its mesh with vertices defined in frame G and
the mesh's center of mass is coincident with Go.

@param vertices_S    The cut mesh's vertices measured and expressed in the
                    source geometry's frame S.
@param faces         The triangles of the cut mesh.
@returns The fully specified cut mesh. */
template <typename T>
MeshCut<T> ComputeCutWithMass(
    const std::vector<geometry::SurfaceVertex<T>>& vertices_S,
    const std::vector<geometry::SurfaceFace>& faces, double density);

}  // namespace internal

}  // namespace slicing
}  // namespace examples
}  // namespace drake
