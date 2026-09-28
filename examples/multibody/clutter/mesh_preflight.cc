#include <exception>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <gflags/gflags.h>

#include "drake/common/unused.h"
#include "drake/geometry/proximity/inflate_mesh.h"
#include "drake/geometry/proximity/obj_to_surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh_field.h"
#include "drake/geometry/shape_specification.h"
#include "drake/multibody/tree/geometry_spatial_inertia.h"

DEFINE_double(margin, 1e-4, "Hydroelastic margin [m].");
DEFINE_double(barrier, 1e-4, "LogBarrier layer thickness [m].");
DEFINE_double(density, 1000.0, "Density used for the inertia check [kg/m^3].");

namespace drake {
namespace examples {
namespace {

using geometry::TriangleSurfaceMesh;
using geometry::VolumeMesh;
using geometry::VolumeMeshFieldLinear;
using geometry::internal::MakeExtrudedMesh;

/* Exercises the same code path used to build the compliant hydroelastic
representation of a Mesh (see MakeCompliantRepresentation() in
geometry/proximity/hydroelastic_internal.cc) plus the inertia computation
performed when the mesh is added as a rigid body. Prints one TSV line per
input file:

  OK|FAIL <path> <num_vertices> <num_faces> <message>

Always exits with status 0; callers parse the per-file lines. */
int do_main(int argc, char* argv[]) {
  for (int i = 1; i < argc; ++i) {
    const std::string path(argv[i]);
    int num_vertices = 0;
    int num_faces = 0;
    try {
      const TriangleSurfaceMesh<double> surface =
          geometry::ReadObjToTriangleSurfaceMesh(path);
      num_vertices = surface.num_vertices();
      num_faces = surface.num_triangles();

      auto extruded = std::make_unique<VolumeMesh<double>>(
          MakeExtrudedMesh(surface, FLAGS_margin + FLAGS_barrier));

      // Mirror the extruded pressure field construction, including the
      // gradient computation (MeshGradientMode::kOkOrThrow) that throws on
      // degenerate tetrahedra.
      const double surface_epsilon = -2 * FLAGS_margin / FLAGS_barrier;
      std::vector<double> extruded_values(extruded->num_vertices(),
                                          surface_epsilon);
      for (int v = 0; v < surface.num_vertices(); ++v) {
        extruded_values[v] = 2.0;
      }
      const VolumeMeshFieldLinear<double, double> field(
          std::move(extruded_values), extruded.get());

      const geometry::Mesh shape(path);
      const multibody::SpatialInertia<double> M =
          multibody::CalcSpatialInertia(shape, FLAGS_density);
      unused(M);

      fmt::print("OK\t{}\t{}\t{}\t\n", path, num_vertices, num_faces);
    } catch (const std::exception& e) {
      std::string message(e.what());
      for (char& c : message) {
        if (c == '\n' || c == '\t') c = ' ';
      }
      fmt::print("FAIL\t{}\t{}\t{}\t{}\n", path, num_vertices, num_faces,
                 message);
    }
  }
  return 0;
}

}  // namespace
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "Checks OBJ meshes against the compliant hydroelastic (barrier) and "
      "spatial inertia pipelines. Usage: mesh_preflight [flags] a.obj b.obj");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::do_main(argc, argv);
}
