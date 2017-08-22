// Simple example to illustrate gcc issue -- this is gcc adversarial.
// See gcc_good for the *good* version of this.

#include <memory>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_copyable.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {

template<class IdType>
class Base {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Base)

  Base(std::unique_ptr<Shape> shape, IdType id)
      : shape_spec_(std::move(shape)), id_(id) {}

  const Shape& shape() const { return *shape_spec_; }
  IdType id() const { return id_; }

 private:
  copyable_unique_ptr<Shape> shape_spec_;
  IdType id_;
};

class Derived : public Base<GeometryId> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Derived)

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