#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/shapes/shape.h"

namespace drake {
namespace geometry {

/** Definition of sphere. It is centered in its canonical frame but with the
 given radius. */
class Sphere final : public Shape {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Sphere)

  explicit Sphere(double radius) : Shape(kSphere), radius_(radius) {}

  double get_radius() const { return radius_; }

 protected:
  Sphere* DoClone() const override {
    return new Sphere(radius_);
  }

 private:
  double radius_;
};

}  // namespace geometry
}  // namespace drake
