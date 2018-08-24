#include "drake/geometry/render_material.h"

namespace drake {
namespace geometry {

RenderMaterial::RenderMaterial(const render::RenderLabel& label)
    : label_(label) {
  validate_label();
}

RenderMaterial::RenderMaterial(const render::RenderLabel& label,
                               const Eigen::Vector4d& diffuse)
    : label_(label), diffuse_(diffuse) {
  validate_label();
}
}  // namespace geometry
}  // namespace drake
