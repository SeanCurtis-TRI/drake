#include "drake/systems/sensors/png_writer_old.h"

#include "fmt/ostream.h"

#include <vtkImageData.h>
#include <vtkNew.h>
#include <vtkPNGWriter.h>

#include "drake/systems/sensors/image.h"

namespace drake {
namespace systems {
namespace sensors {

template <PixelType kPixelType>
void SaveToFile(const std::string& filepath, const Image<kPixelType>& image) {
  const int width = image.width();
  const int height = image.height();
  const int num_channels = Image<kPixelType>::kNumChannels;

  vtkNew<vtkImageData> vtk_image;
  vtk_image->SetDimensions(width, height, 1);

  switch (kPixelType) {
    case PixelType::kRgba8U:
      vtk_image->AllocateScalars(VTK_UNSIGNED_CHAR, num_channels);
      break;
    case PixelType::kDepth32F:
      vtk_image->AllocateScalars(VTK_FLOAT, num_channels);
      break;
    case PixelType::kLabel16I:
      vtk_image->AllocateScalars(VTK_UNSIGNED_SHORT, num_channels);
      break;
  }

  auto image_ptr = reinterpret_cast<typename Image<kPixelType>::T*>(
      vtk_image->GetScalarPointer());
  const int num_scalar_components = vtk_image->GetNumberOfScalarComponents();

  for (int v = height - 1; v >= 0; --v) {
    for (int u = 0; u < width; ++u) {
      for (int c = 0; c < num_channels; ++c) {
        image_ptr[c] =
            static_cast<typename Image<kPixelType>::T>(image.at(u, v)[c]);
      }
      image_ptr += num_scalar_components;
    }
  }

  vtkNew<vtkPNGWriter> writer;
  writer->SetFileName(filepath.c_str());
  writer->SetInputData(vtk_image.GetPointer());
  writer->Write();
};

PngWriterOld::PngWriterOld(const std::string& image_name, double start_time,
                     int padding)
    : image_name_base_(image_name), start_time_(start_time), padding_(padding) {
  image_port_index_ =
      DeclareAbstractInputPort(systems::Value<ImageRgba8U>()).get_index();
}

void PngWriterOld::set_publish_period(double period) {
  LeafSystem<double>::DeclarePeriodicPublish(period);
}

const InputPort<double>& PngWriterOld::color_image_input_port() const {
  return this->get_input_port(image_port_index_);
}

void PngWriterOld::DoPublish(
    const Context<double>& context,
    const std::vector<const PublishEvent<double>*>&) const {
  if (context.get_time() >= start_time_) {
    const AbstractValue* image_value =
        this->EvalAbstractInput(context, image_port_index_);
    if (image_value) {
      const ImageRgba8U& image = image_value->GetValue<ImageRgba8U>();
      // TODO(SeanCurtis-TRI): Write image.
      std::string image_name = fmt::format(
          "{0}_{1:0{2}d}.png", image_name_base_, ++frame_number_, padding_);
      SaveToFile(image_name, image);
    }
  }
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
