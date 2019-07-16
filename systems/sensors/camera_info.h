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
- video frame: an image in a video sequence. This is *not* an extension of
  "frame" as a general term.
- sensor: a device used to take and report measurements.
- image: an array of measurements. Coordinates are in pixels.
- imager: a sensor that measures images.
- camera: an image sensor.
- aperature: a hole through which light travels through.
- image plane: defines how the 3D world is projected to a 2D image. In
  accordance with the OpenCV documentation below, the image plane will be
  presented as though it were "in front" of the aperature, rather than behind
  (which requires accounting for the camera obscura effect).
- pinhole camera: a physical camera with no lens and a tiny aperature.
- pinhole model: modeling a camera as though it were a pinhole camera, with
  parameters that account for the camera lens and image plane. In Drake, all
  cameras are assumed to use the pinhole model unless otherwise stated. The
  images captured by these cameras are 2D, and their pixel coordinates are
  described as `(u, v)`.
- viewing direction / optical axis: direction from aperature towards scene being
  captured by the camera (orthogonal to the image plane for the pinhole model).
- principal point, camera center: the intersection of the viewing ray with the
  image plane, typically measured in pixels.

The camera frame C is comprised of the basis [Cx Cy Cz] and origin point Co,
which are described as follows:

- Co at camera aperature (per the pinhole model)
- Cz aligned with the optical axis (orthogonal to the image plane).
- Cx aligned with the `u` axis of the captured image.
- Cy aligned with the `v` axis of the captured image.

This can be summarized as `X-right`, `Y-down`, and `Z-forward` with respect to
the image plane.

The projection from coordinates `(X, Y, Z)` in the camera frame C to image coordoinates `(u, v)` is:
<pre>
  u = focal_x * (X/Z) + center_x
  v = focal_y * (Y/Z) + center_y
</pre>
where `focal_x, focal_y` are focal lengths, and `center_x, center_y` describe
the camera center (or principal point).

For more detail including an explanation of the focal lengths, refer to:
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
