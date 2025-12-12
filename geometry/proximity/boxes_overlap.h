#pragma once

#include <Eigen/Core>

#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

/* Helper function for detecting overlap between two boxes A and B. Each box is
 defined in its own canonical frame. The box is aligned with the frame and
 described by a vector describing its "half size" (half its extent along each of
 its frame's axes). Another way to think about the half size is that it's the
 position vector from the box's center to its most positive corner (expressed
 in the box's canonical frame).

 @param half_size_a   The half size of box A expressed in A's canonical frame.
 @param half_size_b   The half size of box B expressed in B's canonical frame.
 @param X_AB          The relative pose between boxes A and B. */
bool BoxesOverlap(const Vector3<double>& half_size_a,
                  const Vector3<double>& half_size_b,
                  const math::RigidTransformd& X_AB);

/* Reports if a box overlaps with a half space. The box is specified in
 generic terms. It is an axis-aligned box, centered on a point with _half_
 measures along the axis directions.

 Because the parameters define the box in the half space's frame H, there is no
 need to pass the half space; by construction, its normal likes in the Hz
 direction and the origin Ho lies on the half space's boundary.

 @param half_width                The half-width extents of the box along its
                                  local axes.
 @param box_center_in_plane       The center of the box measured and expressed
                                  in the plane's frame.
 @param box_orientation_in_plane  The orientation of the box expressed in the
                                  plane's frame. The ith column goes with the
                                  ith half width.
*/
bool BoxOverlapsHalfspace(
    const Vector3<double>& half_size,
    const Vector3<double>& box_center_in_halfspace,
    const math::RotationMatrixd& box_orientation_in_halfspace);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
