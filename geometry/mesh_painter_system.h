#pragma once

#include <memory>

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/bounding_volume_hierarchy.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/scene_graph.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace geometry {

/** The definition of a canvas mesh: a 3D surface mesh with per-vertex texture
 coordinates. Generally, the domain of the texture coordinates would lie in
 [0, 1] X [0, 1]. However, values outside of that domain are valid -- it leads
 to tiled textures in the rendered mesh.
 */
struct TexturedCanvasMesh {
  TexturedCanvasMesh(
      std::unique_ptr<SurfaceMesh<double>> mesh_in,
      std::vector<Vector2<double>> uvs_in,
      std::unique_ptr<internal::BoundingVolumeHierarchy<SurfaceMesh<double>>>
          bvh_in)
      : mesh(std::move(mesh_in)),
        uvs(std::move(uvs_in)),
        bvh(std::move(bvh_in)) {}

  std::unique_ptr<SurfaceMesh<double>> mesh;
  std::vector<Vector2<double>> uvs;
  std::unique_ptr<internal::BoundingVolumeHierarchy<SurfaceMesh<double>>> bvh;
};

/** The definition of a pinter mesh; a 3D volume mesh. A canvas mesh is painted
 in its UV space based on the intersection with this volume mesh. */
struct PainterMesh {
  PainterMesh(
      std::unique_ptr<VolumeMesh<double>> mesh_in,
      std::unique_ptr<internal::BoundingVolumeHierarchy<VolumeMesh<double>>>
          bvh_in)
      : mesh(std::move(mesh_in)),
        bvh(std::move(bvh_in)) {}

  std::unique_ptr<VolumeMesh<double>> mesh;
  std::unique_ptr<internal::BoundingVolumeHierarchy<VolumeMesh<double>>> bvh;
};

// TODO(SeanCurtis-TRI): Things to consider:
//
//  1. I'm currently hard-coding a black image that gets painted white. Worth
//     generalizing that? Clear color and paint color?
//  2. If this gets too expensive, it can be combined with a zero-order hold.
//     Might be worth instantiating a MeshPainterSystemDiscrete (akin to
//     RgbdSensorDiscrete.

/** System for painting on a mesh. The painter system is associated with two
 geometries in a related SceneGraph instance: a canvas object and a painter
 object. As the painter object makes contact with the canvas, a texture map
 associated with the mesh gets painted based on the intersection. The texture
 is available on the output port. The poses of the canvas and painter are read
 from the QueryObject-valued input port.

 @system{MeshPainterSystem,
   @input_port{geometry_query},
   @output_port{texture}
 }
 */
class MeshPainterSystem final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MeshPainterSystem)

  /** Constructs a %MeshPainterSystem.

   @param canvas_id     The id of the shape that is to be painted.
   @param painter_id    The id of the geometry that paints on the mesh.
   @param scene_graph   The SceneGraph instances that contains the given ids.
   @param width         The width of the texture.
   @param height        The height of the texture.
   */
  MeshPainterSystem(GeometryId canvas_id, GeometryId painter_id,
                    const SceneGraph<double>& scene_graph, int width,
                    int height);

  const systems::InputPort<double>& geometry_query_input_port() const;
  const systems::OutputPort<double>& texture_output_port() const;

 private:
  /* Update function for the texture map. This will paint onto the given texture
   based on overlap between the painter object and the mesh. It only accumulates
   and makes no assumptions about the previous state of the texture.  */
  void CalcTextureImage(const systems::Context<double>& context,
                        systems::sensors::ImageRgba8U* texture) const;

  const GeometryId canvas_id_;
  const GeometryId painter_id_;
  const std::unique_ptr<PainterMesh> painter_mesh_;
  const std::unique_ptr<TexturedCanvasMesh> canvas_mesh_;
  const systems::InputPort<double>* geometry_query_input_port_{};
  const systems::OutputPort<double>* texture_output_port_{};
};

}  // namespace geometry
}  // namespace drake
