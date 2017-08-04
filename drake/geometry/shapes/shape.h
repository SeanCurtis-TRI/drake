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
    kUnknown = 0,
    kSphere,
    kHalfSpace,
  };

  /** Constructor.
   @param type  The type of the particular shape. */
  explicit Shape(Type type) : type_(type) {}

  virtual ~Shape() {}

  /** Creates a unique copy of this shape. Invokes the protected DoClone(). */
  std::unique_ptr<Shape> Clone() const {
    return std::unique_ptr<Shape>(DoClone());
  }

  Type get_type() const { return type_; }

 protected:
  /** Performs the work of cloning a concrete shape. */
  virtual Shape* DoClone() const = 0;

 private:
  // The type of the shape.
  Type type_;
};

}  // namespace geometry
}  // namespace drake
