#include "drake/geometry/proximity/plane.h"

#include "drake/common/drake_assert.h"
#include "drake/common/fmt_eigen.h"

namespace drake {
namespace geometry {

using Eigen::Vector3d;

template <typename T>
Plane<T>::Plane(const Vector3<T>& normal, const Vector3<T>& point_on_plane,
                bool already_normalized) {
  // A couple of convenient notational aliases.
  const Vector3<T>& n_F = normal;
  const Vector3<T>& p_FP = point_on_plane;

  if (!already_normalized) {
    const T magnitude = n_F.norm();
    // NOTE: This threshold is arbitrary. Given Drake uses mks and generally
    // works on problems at a human scale, the assumption is that if someone
    // passes in an incredibly small normal (picometers), it is probably an
    // error.
    if (magnitude < 1e-10) {
      throw std::runtime_error(fmt::format(
          "Cannot instantiate plane from normal n_F = [{}]; its magnitude is "
          "too small: {}",
          fmt_eigen(n_F.transpose()), magnitude));
    }
    nhat_F_ = n_F / magnitude;
  } else {
    DRAKE_ASSERT_VOID(ThrowIfInsufficientlyNormal(n_F));
    nhat_F_ = n_F;
  }
  displacement_ = nhat_F_.dot(p_FP);
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class Plane);

}  // namespace geometry
}  // namespace drake
