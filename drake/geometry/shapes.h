#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {

/** Abstract class defining a geometrical shape. */
class Shape {
 public:
  /** Specification of shape type. */
  enum Type {
    UNKNOWN = 0,
    SPHERE,
    HALF_SPACE,
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
  Sphere* DoClone() const override {
    return new Sphere(radius_);
  }

 private:
  double radius_;
};

/** Definition of a half space. It is defined by a normal to the plane and a
 point on the plane. Other shapes are considered to be in penetration with the
 half space if there exists any point on the object that lies on the side of
 the plane opposite the normal.*/
class HalfSpace final : public Shape {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(HalfSpace)

  /** Constructor.
   @param normal    A vector normal to the plane. It points in the "outside"
                    direction. It is _not_ assumed to be unit length. It is
                    measured and expressed in the _world_ frame.
   @param point     A point that lies on the plane, measured and expressed in
                    the _world_ frame.  */
  HalfSpace(const Vector3<double>& normal, const Vector3<double>& point);

  /** Reports the signed distance from the given point `p` to the half-spaces
   plane boundary. Positive values indicate *outside* the half-space. */
  double get_signed_distance(const Vector3<double>& p) const {
    return normal_.dot(p) + d_;
  }

  /** Reports the half-space's outward-pointing normal. */
  const Vector3<double> get_normal() const { return normal_; }

 protected:
  Shape* DoClone() const override {
    return new HalfSpace(*this);
  }

 private:
  // Defines the implicit equation of the plane: P(x) = dot(N, x) + d
  Vector3<double> normal_;
  double d_;
};

}  // namespace geometry
}  // namespace drake
