#include <fmt/format.h>

#include "drake/geometry/shape_specification.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"
#include "drake/geometry/proximity/tessellation_strategy.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"

using drake::geometry::Sphere;
using drake::geometry::TriangleSurfaceMesh;
using drake::geometry::internal::MakeSphereSurfaceMesh;

int main() {
  const Sphere s(1.0);
  const TriangleSurfaceMesh<double> mesh_W =
      MakeSphereSurfaceMesh<double>(Sphere(1.0), 0.01);

  fmt::print("# Mesh has {} vertices and {} faces\n", mesh_W.num_vertices(),
             mesh_W.num_elements());

  for (const auto& v : mesh_W.vertices()) {
    fmt::print("v {} {} {}\n", v.x(), v.y(), v.z());
  }
  for (const auto& tri : mesh_W.triangles()) {
    fmt::print("f {} {} {}\n", tri.vertex(0) + 1, tri.vertex(1) + 1,
               tri.vertex(2) + 1);
  }
  return 0;
}