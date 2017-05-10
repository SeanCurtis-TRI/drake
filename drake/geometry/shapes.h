#pragma once

#include <memory>
#include "drake/common/drake_copyable.h"

namespace drake {
namespace geometry {

/** Abstract class defining a geometrical shape. */
class Shape {
 public:
  /** Specification of shape type. */
  enum Type {
    UNKNOWN = 0,
    SPHERE,
  };

  /** Constructor.
   @param type  The type of the particular shape. */
  explicit Shape(Type type) : type_(type) {}

  virtual ~Shape() {}

  std::unique_ptr<Shape> Clone() const {
    return std::unique_ptr<Shape>(DoClone());
  }

  Type get_type() const { return type_; }

 protected:

  virtual Shape* DoClone() const = 0;

  // TODO(SeanCurtis-TRI): THe geometry engine is going to need access to type.

 private:
  // The type of the shape.
  Type type_;
};

/** Definition of sphere. It is centered in its canonical frame but with the
 given radius. */
class Sphere final : public Shape {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Sphere)

  explicit Sphere(double radius) : Shape(SPHERE), radius_(radius) {}

  double get_radius() const { return radius_; }

 protected:
  Sphere* DoClone() const {
    return new Sphere(radius_);
  }

 private:
  double radius_;
};

}  // namespace geometry
}  // namespace drake
