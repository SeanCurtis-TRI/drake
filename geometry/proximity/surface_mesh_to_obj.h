#pragma once

#include <filesystem>

#include "drake/geometry/proximity/polygon_surface_mesh.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"

namespace drake {
namespace geometry {

void WriteToObj(const std::filesystem::path& out_path,
                const TriangleSurfaceMesh<double>& mesh);

void WriteToObj(const std::filesystem::path& out_path,
                const PolygonSurfaceMesh<double>& mesh);

}  // namespace geometry
}  // namespace drake
