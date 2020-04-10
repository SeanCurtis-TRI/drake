#pragma once

#include <utility>

#include "drake/geometry/geometry_ids.h"
#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/pixel_types.h"

namespace drake {
namespace geometry {

/** Struct for supplying an externally controlled image to SceneGraph as an
 input.

 SceneGraph can take as an input an image. The use of the image depends on the
 configuration of the various components of SceneGraph. One example would be
 to control the appearance of an object in render queries. Some external system
 updates the texture for one or more objects, controlling its time-varying
 appearance. The %InputImage type is passed from that system's output port into
 the corresponding input port of SceneGraph.

 Operations involving images may be very expensive. It would make sense for
 SceneGraph to only respond to actual changes in the image. Downstream consumers
 _could_ simply compare the previous image data with the current image data to
 determine if there is a difference, but this would likewise be an expensive
 operation.

 Instead, an %InputImage has a serial number. Each time the image source makes a
 change to the image that it wants visible in the downstream consumer, it should
 increment the serial number. This is a very efficient mechanism -- the image
 source is in a unique position to know if things have changed and the serial
 number compactly communicates that. However, if the image source _forgets_ to
 increment the serial number, downstream consumers are not obliged to update
 their internal representation of the provided image.

 Currently only RGBA-channel, unsigned-byte valued images are supported.  */
// TODO(SeanCurtis-TRI): I'm losing flexibility of passing in masks as single
//  channel images. Investigate making this more flexible.
class InputImage {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(InputImage)

  using ImageType = systems::sensors::ImageRgba8U;

  /** Default constructor for model instantiation of port types; it produces
   an otherwise invalid instance with an invalid id and a zero-area image. */
  InputImage() = default;

  /**
   @param id    The ImageId provided by SceneGraph when declaring an image
                input.
   @param image The image.
   */
  InputImage(ImageId id, ImageType&& image)
      : id_(id), image_(std::move(image)) {}

  /** The image data.  */
  const ImageType& image() const { return image_; }

  // TODO(SeanCurtis-TRI): Consider burying the serial number increment in the
  //  access to the mutable image. Then simply warning that accessing the
  //  mutable data will definitely cause downstream consumers to perform work,
  //  even if the image hasn't actually changed. So, only access the image data
  //  before an actual write operation.

  /** Access to a _mutable_ version of the image data.  */
  ImageType& mutable_image() { return image_; }

  /** The image id.  */
  ImageId id() const { return id_; }

  /** The current value of the serial number.  */
  int64_t serial_number() const { return serial_number_; }

  /** Increment the serial number. This should be done every time the image is
   actually written to. This is the only circumstance in which downstream
   consumers _must_ modify their representation of the image data.  */
  void increment_serial_number() { ++serial_number_; }

 private:
  // TODO(SeanCurtis-TRI): This should have a SourceId to confirm ownership.
  ImageId id_{};
  ImageType image_;
  int64_t serial_number_{0};
};

}
}
