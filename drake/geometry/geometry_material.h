#pragma once

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace geometry {

/** Definition of visual material -- it describes the optical properties for
 visual render. */
class VisualMaterial {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(VisualMaterial)

  /** Constructs a material with the default material properties. */
  VisualMaterial();

  /** Constructs a material with the given diffuse color and default values for
   all other properties. */
  explicit VisualMaterial(const Eigen::Vector4d& diffuse);

  /** Returns the diffuse color of the material. */
  const Eigen::Vector4d get_diffuse() const { return diffuse_; }

 private:
  // The diffuse color.
  Eigen::Vector4d diffuse_{0.9, 0.9, 0.9, 1.0};
};
}  // namespace geometry
}  // namespace drake
