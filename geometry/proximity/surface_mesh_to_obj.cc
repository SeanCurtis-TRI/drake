#include "drake/geometry/proximity/surface_mesh_to_obj.h"

#include <fstream>

#include <fmt/format.h>

namespace drake {
namespace geometry {
namespace {

using Eigen::Vector3d;

template <typename MeshType>
void _WriteToObj(const std::filesystem::path& out_path, const MeshType& mesh) {
  std::ofstream fstream(out_path);
  if (!fstream) {
    throw std::runtime_error(
        fmt::format("WriteToObj(): Error opening file for writing: '{}'",
                    out_path.string()));
  }

  fstream << "# Obj written by Drake\n";
  for (int vi = 0; vi < mesh.num_vertices(); ++vi) {
    const Vector3d& v = mesh.vertex(vi);
    fstream << "v " << v.x() << " " << v.y() << " " << v.z() << "\n";
  }
  // TODO(SeanCurtis-TRI): Consider writing out normals.
  for (int ei = 0; ei < mesh.num_elements(); ++ei) {
    const auto& face = mesh.element(ei);
    fstream << "f";
    for (int vi = 0; vi < face.num_vertices(); ++vi) {
        fstream << " " << (face.vertex(vi) + 1);
    }
    fstream << "\n";
  }

  fstream.close();
}

}  // namespace

void WriteToObj(const std::filesystem::path& out_path,
                const TriangleSurfaceMesh<double>& mesh) {
  _WriteToObj(out_path, mesh);
}

void WriteToObj(const std::filesystem::path& out_path,
                const PolygonSurfaceMesh<double>& mesh) {
  _WriteToObj(out_path, mesh);
}

}  // namespace geometry
}  // namespace drake
