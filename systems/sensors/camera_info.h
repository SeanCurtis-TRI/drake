#pragma once

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace systems {
namespace sensors {

// TODO(kunimatsu-tri) Add camera distortion parameters and other parameters as
// needed.
/**
Simple data structure for camera information that includes the image size
and camera intrinsic parameters.

Please note the following terms:

- frame, pose: denotes an origin and a space-spanning basis in 2D/3D, as in
  @ref multibody_frames_and_bodies.
- video frame: an image in a video sequence.
- camera: an image sensor with lens; in Drake, this will be modeled as a
  pinhole cameras unless stated otherwise. Because of this, the camera
  aperature is a point in space.
- image plane: defines how the 3D world is projected to a 2D image.

The camera frame `C` is comprised of the basis [Cx Cy Cz] and origin point Co,
which are described as follows:

- Co at camera aperature.
- Cz aligned with the viewing direction, orthogonal to the image plane.
- Cx aligned with the right direction of the image plane.
- Cy aligned with the downwards direction of the image plane.

This can be summarized as `X-right`, `Y-down`, and `Z-forward` with respect to
the image plane.

The image plane can be described in the following coordinate systems:

- normalized coordinate system `(X/Z, Y/Z)`, with its origin at the principal
  point.
- pixel coordinate system `(u, v)` using the following transformation:
  <pre>
    u = focal_x * (X/Z) + center_x
    v = focal_y * (Y/Z) + center_y
  </pre>
  where `focal_x, focal_y` are focal lengths, and `center_x, center_y` describe
  the principal point.

For more detail including an explanation of the focal lengths, refer to:
- https://en.wikipedia.org/wiki/Pinhole_camera_model
- http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
*/
class CameraInfo final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CameraInfo)

  /**
   Constructor that directly sets the image size, center, and focal lengths.

   @param width The image width in pixels, must be greater than zero.
   @param height The image height in pixels, must be greater than zero.
   @param focal_x The focal length x in pixels.
   @param focal_y The focal length y in pixels.
   @param center_x The x coordinate of the image center in the pixel
   coordinate system in pixels.
   @param center_y The y coordinate of the image center in the pixel
   coordinate system in pixels.
  */
  CameraInfo(int width, int height, double focal_x, double focal_y,
             double center_x, double center_y);

  /**
   Constructor that sets the image size and vertical field of view `fov_y`.
   We assume there is no image offset, so the image center `(center_x,`
   `center_y)` is equal to `(width / 2, height / 2)`.  We also assume the
   focal lengths `focal_x` and `focal_y` are identical.  The horizontal
   field of view `fov_x` is calculated using the aspect ratio of the image
   width and height together with the vertical field of view:
   <pre>
     fov_x = 2 * atan(width / height * tan(fov_y / 2)).
   </pre>
   This can be derived from the equations of the focal lengths:
   <pre>
     focal_x = width / 2 / tan(fov_x / 2)
     focal_y = height / 2 / tan(fov_y / 2)
   </pre>
   where `focal_x / focal_y = 1`.

   @param width The image width in pixels, must be greater than zero.
   @param height The image height in pixels, must be greater than zero.
   @param fov_y The vertical field of view.
  */
  CameraInfo(int width, int height, double fov_y);

  /** Returns the width of the image in pixels. */
  int width() const { return width_; }

  /** Returns the height of the image in pixels. */
  int height() const { return height_; }

  /** Returns the focal length x in pixels. */
  double focal_x() const { return intrinsic_matrix_(0, 0); }

  /** Returns the focal length y in pixels. */
  double focal_y() const { return intrinsic_matrix_(1, 1); }

  /** Returns the image center x value in pixels. */
  double center_x() const { return intrinsic_matrix_(0, 2); }

  /** Returns the image center y value in pixels. */
  double center_y() const { return intrinsic_matrix_(1, 2); }

  /** Returns the camera intrinsic matrix. */
  const Eigen::Matrix3d& intrinsic_matrix() const {
    return intrinsic_matrix_;
  }

 private:
  int width_{};
  int height_{};
  // Camera intrinsic parameter matrix. For the detail, see
  // http://docs.opencv.org/2.4/modules/calib3d/doc/
  // camera_calibration_and_3d_reconstruction.html
  Eigen::Matrix3d intrinsic_matrix_;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
