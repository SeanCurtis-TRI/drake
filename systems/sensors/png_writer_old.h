#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace systems {
namespace sensors {

/** A system for periodically writing images to the file system. This is a
 convenience mechanism for capturing images from (typically) an RgbdCamera.
 However, any system that outputs a color image (8-bit RGBA) is compatible.

 The writer writes images based on a periodic event. For a given period `P`
 images will be written at times `t = [0, P, 2P, ...]`.

 The output frames are enumerated as 0, 1, 2, etc. The enumeration can be
 zero-padded (by providing a positive padding value) or not.
 */
class PngWriterOld : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PngWriterOld)

  /** Constructs the image-to-file system.
   @param image_name  The base name of the files to write. Images will be
                      written as `image_name_%0Xd.png` where X is `padding`.
                      The path *can* contain directories (relative or absolute).
   @param start_time  The earliest simulation time at which an image will be
                      written. NOTE: it is *not* guaranteed that a frame will be
                      written at start time.
   @param padding     The minimum number of digits in the output file (with
                      the total value padded by zeros).
   */
  PngWriterOld(const std::string& image_name, double start_time = 0,
            int padding = 4);

  /** The period at which images are written. The images will be written at
   times `[t₀, t₁, ..., tₙ]` where `t₀ = minᵢ(i * period)`, such that
   `t₀ > start_time_`.  */
  void set_publish_period(double period);

  const InputPort<double>& color_image_input_port() const;

 private:
  // Do the image writing.
  void DoPublish(
      const systems::Context<double>& context,
      const std::vector<const PublishEvent<double>*>&) const override;

  // Output parameters.
  const std::string image_name_base_;
  const double start_time_;
  const int padding_{4};

  int image_port_index_{-1};

  // This is a *hack*. This should be as part of discrete state.
  mutable int frame_number_{-1};
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
