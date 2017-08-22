// Simple example to illustrate gcc issue -- this is gcc compatible.
// See gcc_bad for the *bad* version of this.

#include <memory>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_copyable.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {

class Base {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Base)

  Base(std::unique_ptr<Shape> shape, GeometryId id)
      : shape_spec_(std::move(shape)), id_(id) {}

  const Shape& shape() const { return *shape_spec_; }
  GeometryId id() const { return id_; }

 private:
  copyable_unique_ptr<Shape> shape_spec_;
  GeometryId id_;
};

class Derived : public Base {
 public:
  Derived(std::unique_ptr<Shape> shape, GeometryId id)
  : Base(std::move(shape), id) {}
};

void main() {
  std::unique_ptr<Shape> sphere(new Sphere(1.0));
  Derived d{std::move(sphere), GeometryId::get_new_id()};
  Derived copy{d};
}
}  // namespace drake
}  // namespace geometry

int main() {
  drake::geometry::main();
  return 0;
}