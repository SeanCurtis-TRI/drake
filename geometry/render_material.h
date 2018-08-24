#pragma once

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/render/render_label.h"

namespace drake {
namespace geometry {

/** Definition of material for rendering. Default materials are a light grey. */
class RenderMaterial final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RenderMaterial)

  /** Constructs a material with the default color properties, but with the
   given label. The label *cannot* be an empty label. */
  explicit RenderMaterial(const render::RenderLabel& label);

  /** Full constructor with color and class label. The label *cannot* be an
   empty label. */
  RenderMaterial(const render::RenderLabel& label,
                 const Eigen::Vector4d& diffuse);

  /** Returns the material's diffuse color. */
  const Eigen::Vector4d& diffuse() const { return diffuse_; }

  /** Returns the label for this material. */
  const render::RenderLabel& label() const { return label_; }

 private:
  void validate_label() const {
    if (label_.is_empty())
      throw std::runtime_error("Label cannot be the empty label");
  }

  // Class label - defaults to empty.
  render::RenderLabel label_{};

  // The diffuse color.
  Eigen::Vector4d diffuse_{0.9, 0.9, 0.9, 1.0};
};

}  // namespace geometry
}  // namespace drake
