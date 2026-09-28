#pragma once

#include <cstdint>

namespace drake {
namespace geometry {
namespace internal {

/* Aggregate mesh-size statistics over every hydroelastic geometry in a scene
 (see QueryObject::ComputeHydroelasticMeshStats()). Used by experiment
 instrumentation to report problem size independent of run-time behavior. */
struct HydroelasticMeshStats {
  /* Total surface triangles over all hydroelastic geometries. For a compliant
   geometry carrying a rigid-core collision mesh (the thin-object barrier
   representation), its collision mesh is counted; for any other compliant
   geometry (e.g. the volumetric barrier == 0 representation), the boundary
   surface of its volume mesh is counted; for a rigid (non half-space)
   geometry, its surface mesh is counted. Half spaces contribute nothing. */
  int64_t num_surface_triangles{0};
  /* Total tetrahedra over all compliant volume meshes. In the thin-object
   barrier representation these are exactly the extruded barrier-layer
   meshes; with barrier == 0 they are the volumetric hydroelastic meshes. */
  int64_t num_tetrahedra{0};
  /* Number of geometries contributing to the counts above. */
  int num_geometries{0};
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
