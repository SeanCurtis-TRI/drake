// Test baking shadow maps.

#include <vtkActor.h>
#include <vtkAutoInit.h>    
#include <vtkCamera.h>
#include <vtkCameraPass.h>
#include <vtkCubeSource.h>
#include <vtkHDRReader.h>
#include <vtkImageData.h>
#include <vtkImageIterator.h>
#include <vtkImageReader2.h>
#include <vtkImageReader2Factory.h>
#include <vtkLight.h>
#include <vtkMatrix4x4.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkOpenGLRenderer.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderPassCollection.h>
#include <vtkRenderStepsPass.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSequencePass.h>
#include <vtkShadowMapBakerPass.h>
#include <vtkShadowMapPass.h>
#include <vtkSimpleImageToImageFilter.h>
#include <vtkSmartPointer.h>
#include <vtkToneMappingPass.h>
#include <vtkVersion.h>

#include <vtkSphereSource.h>
#include <vtkTexture.h>

#include <vtksys/SystemTools.hxx>

#include <array>
#include <filesystem>
#include <iostream>
#include <string>

#include "drake/common/nice_type_name.h"

VTK_AUTOINIT_DECLARE(vtkRenderingOpenGL2);

namespace {

vtkSmartPointer<vtkPolyData> ReadPolyData();
// Reports the texture to use for the environment map with some important
// details regarding how the environment map should be configured.
struct EnvironmentTexture {
  vtkSmartPointer<vtkTexture> texture;
  bool is_hdr{};
};

// Taken from: https://examples.vtk.org/site/Cxx/Rendering/PBR_HDR_Environment/
EnvironmentTexture ReadEquirectangularFile(std::string const& fileName) {
  vtkNew<vtkTexture> texture;

  std::string extension =
      std::filesystem::path(fileName).extension().generic_string();
  std::transform(extension.cbegin(), extension.cend(), extension.begin(),
                 [](char c) {
                   return std::tolower(c);
                 });

  bool is_hdr = false;
  if (std::string(".jpeg .jpg .png").find(extension, 0) != std::string::npos) {
    vtkNew<vtkImageReader2Factory> readerFactory;
    vtkSmartPointer<vtkImageReader2> imgReader;
    imgReader.TakeReference(
        readerFactory->CreateImageReader2(fileName.c_str()));
    imgReader->SetFileName(fileName.c_str());
    texture->SetInputConnection(imgReader->GetOutputPort());
  } else {
    vtkNew<vtkHDRReader> reader;
    auto extensions = reader->GetFileExtensions();
    if (std::string(extensions).find(extension, 0) != std::string::npos) {
      if (reader->CanReadFile(fileName.c_str())) {
        reader->SetFileName(fileName.c_str());
        texture->SetInputConnection(reader->GetOutputPort());
        texture->SetColorModeToDirectScalars();
        is_hdr = true;
      } else {
        throw std::runtime_error(
            "Unable to instantiate environment map for RenderEngineVtk.");
      }
    }
  }

  texture->MipmapOn();
  texture->InterpolateOn();

  return {texture, is_hdr};
}

}  // namespace


//----------------------------------------------------------------------------
int main() {
  VTK_AUTOINIT_CONSTRUCT(vtkRenderingOpenGL2);
  std::cout << "VTK version: " << vtkVersion::GetVTKVersion() << "\n";
  // Read the polyData.
  auto polyData = ReadPolyData();


  vtkNew<vtkNamedColors> colors;
  colors->SetColor("HighNoonSun", 1.0, 1.0, .9843, 1.0); // Color temp. 5400k.
  colors->SetColor("100W Tungsten", 1.0, .8392, .6667,
                   1.0); // Color temp. 2850k.

  vtkNew<vtkRenderer> renderer;
  renderer->SetBackground(colors->GetColor3d("Silver").GetData());
  renderer->SetBackgroundAlpha(1.0);

  const bool kUseEnvMap = true;
  const bool kAddMap = true;
  const bool kUsePbr = true;
  auto *gl_renderer = vtkOpenGLRenderer::SafeDownCast(renderer);
  if (gl_renderer == nullptr) {
    std::cerr << "The renderer wasn't opengl!!!\n";
    renderer->PrintSelf(std::cerr, vtkIndent(0));
    return 1;
  }
  EnvironmentTexture env_map = ReadEquirectangularFile(
      // "/home/seancurtis/Downloads/poly_haven_studio_2k.hdr");
      "/home/seancurtis/code/drake/geometry/test/env_256_six_color_room.hdr");
  if (kUseEnvMap)
  {
    gl_renderer->UseImageBasedLightingOn();
    gl_renderer->SetUseSphericalHarmonics(env_map.is_hdr);
  }
  if (kAddMap)
  {
    gl_renderer->SetEnvironmentTexture(env_map.texture,
                                       /* isSRGB = */ !env_map.is_hdr);
  }

  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->SetSize(640, 640);
  renderWindow->AddRenderer(renderer);

  vtkNew<vtkRenderWindowInteractor> interactor;
  interactor->SetRenderWindow(renderWindow);

  vtkNew<vtkLight> light1;
  light1->SetPositional(true);
  light1->SetFocalPoint(0, 0, 0);
  light1->SetPosition(0, 1, 0.2);
  light1->SetColor(colors->GetColor3d("HighNoonSun").GetData());
  light1->SetIntensity(2.3);
  light1->SetConeAngle(40);
  renderer->AddLight(light1);
  // std::cout << "Light 1\n";
  // light1->PrintSelf(std::cout, vtkIndent(1));

  vtkNew<vtkLight> light2;
  light2->SetPositional(false);
  light2->SetFocalPoint(0, 0, 0);
  light2->SetPosition(1.0, 1.0, 1.0);
  light2->SetColor(colors->GetColor3d("100W Tungsten").GetData());
  light2->SetIntensity(2.8);
  light2->SetConeAngle(30);
  renderer->AddLight(light2);
  // std::cout << "Light 2\n";
  // light2->PrintSelf(std::cout, vtkIndent(1));

  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputData(polyData);

  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(colors->GetColor3d("White").GetData());
  actor->GetProperty()->SetOpacity(1.0);
  if (kUsePbr) {
    actor->GetProperty()->SetInterpolationToPBR();
  }
  renderer->AddActor(actor);

  // Add a plane.
  std::array<double, 6> bounds;
  polyData->GetBounds(bounds.data());

  std::array<double, 3> range;
  range[0] = bounds[1] - bounds[0];
  range[1] = bounds[3] - bounds[2];
  range[2] = bounds[5] - bounds[4];
  double expand = 1.0;
  auto thickness = range[2] * 0.1;
  vtkNew<vtkCubeSource> plane;
  plane->SetCenter((bounds[1] + bounds[0]) / 2.0, bounds[2] - thickness / 2.0,
                   (bounds[5] + bounds[4]) / 2.0);
  plane->SetXLength(bounds[1] - bounds[0] + (range[0] * expand));
  plane->SetYLength(thickness);
  plane->SetZLength(bounds[5] - bounds[4] + (range[2] * expand));

  vtkNew<vtkPolyDataMapper> planeMapper;
  planeMapper->SetInputConnection(plane->GetOutputPort());

  vtkNew<vtkActor> planeActor;
  planeActor->SetMapper(planeMapper);
  if (kUsePbr) {
    planeActor->GetProperty()->SetInterpolationToPBR();
  }
  renderer->AddActor(planeActor);

  renderWindow->SetMultiSamples(0);

  vtkNew<vtkSequencePass> seq;
  vtkNew<vtkRenderPassCollection> passes;

  vtkNew<vtkShadowMapPass> shadows;
  passes->AddItem(shadows->GetShadowMapBakerPass());
  passes->AddItem(shadows);
  seq->SetPasses(passes);
  vtkNew<vtkCameraPass> cameraP;
  cameraP->SetDelegatePass(seq);

  vtkNew<vtkToneMappingPass> toneMappingP;
  toneMappingP->SetToneMappingType(vtkToneMappingPass::GenericFilmic);
  toneMappingP->SetGenericFilmicUncharted2Presets();
  toneMappingP->SetExposure(.25);
  toneMappingP->SetDelegatePass(cameraP);


  // Tell the renderer to use our render pass pipeline.
  vtkOpenGLRenderer* glrenderer =
      dynamic_cast<vtkOpenGLRenderer*>(renderer.GetPointer());
  glrenderer->SetPass(toneMappingP);

  renderer->GetActiveCamera()->SetPosition(-0.2, 0.2, 1);
  renderer->GetActiveCamera()->SetFocalPoint(0, 0, 0);
  renderer->GetActiveCamera()->SetViewUp(0, 1, 0);
  renderer->ResetCamera();
  renderer->GetActiveCamera()->Dolly(2.25);
  
  renderer->ResetCameraClippingRange();
  // renderer->GetActiveCamera()->PrintSelf(std::cout, vtkIndent(1));
  renderWindow->Render();
  renderWindow->SetWindowName("Shadows");

  std::cout << "Starting interactor\n";
  interactor->Start();
  std::cout << "Interactor done\n";

  char junk;
  std::cin >> junk;

  return EXIT_SUCCESS;
}

namespace {
vtkSmartPointer<vtkPolyData> ReadPolyData()
{
  vtkSmartPointer<vtkPolyData> polyData;
    vtkNew<vtkSphereSource> source;
    source->SetThetaResolution(100);
    source->SetPhiResolution(100);
    source->Update();
    polyData = source->GetOutput();
  return polyData;
}
} // namespace
