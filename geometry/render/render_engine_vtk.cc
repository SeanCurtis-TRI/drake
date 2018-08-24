#include "drake/geometry/render/render_engine_vtk.h"

#include <vtkCamera.h>
#include <vtkCubeSource.h>
#include <vtkCylinderSource.h>
#include <vtkOBJReader.h>
#include <vtkPlaneSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkSphereSource.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>

#include "drake/geometry/render/color_palette.h"
#include "drake/geometry/render/vtk_util.h"

namespace drake {
namespace geometry {
namespace render {

using std::make_unique;
using vtk_util::ConvertToVtkTransform;
using vtk_util::MakeVtkPointerArray;

namespace {

const int kNumMaxLabel = 256;

// TODO(kunimatsu-tri) Add support for the arbitrary clipping planes.
// TODO(SeanCurtis-TRI): For depth cameras, the clipping planes should be a
// function of the reportable z_near and z_far values; if the clipping planes
// are *beyond* those values, we've wasted precision.
const double kClippingPlaneNear = 0.01;
const double kClippingPlaneFar = 100.;

// For Z-buffer value conversion.
const double kA = kClippingPlaneFar / (kClippingPlaneFar - kClippingPlaneNear);
const double kB = -kA * kClippingPlaneNear;

// Updates VTK rendering related objects including vtkRenderWindow,
// vtkWindowToImageFilter and vtkImageExporter, so that VTK reflects
// vtkActors' pose update for rendering.
void PerformVTKUpdate(
    const vtkNew<vtkRenderWindow>& window,
    const vtkNew<vtkWindowToImageFilter>& filter,
    const vtkNew<vtkImageExport>& exporter) {
  window->Render();
  filter->Modified();
  filter->Update();
  exporter->Update();
}

void UpdateCamera(const CameraProperties& camera,
                  bool show_window,
                  vtkRenderWindow* window,
                  vtkRenderer* renderer) {
  // NOTE: This is a horrible hack for modifying what otherwise looks like
  // const entities.
  window->SetSize(camera.width, camera.height);
  window->SetOffScreenRendering(!show_window);
  renderer->GetActiveCamera()->SetViewAngle(camera.fov_y * 180 / M_PI);
}

void SetModelTransformMatrixToVtkCamera(
    vtkCamera* camera, const vtkSmartPointer<vtkTransform>& X_WC) {
  // vtkCamera contains a transformation as the internal state and
  // ApplyTransform multiplies a given transformation on top of the internal
  // transformation. Thus, resetting 'Set{Position, FocalPoint, ViewUp}' is
  // needed here.
  camera->SetPosition(0., 0., 0.);
  camera->SetFocalPoint(0., 0., 1.);  // Sets z-forward.
  camera->SetViewUp(0., -1, 0.);  // Sets y-down. For the detail, please refer
  // to CameraInfo's document.
  camera->ApplyTransform(X_WC);
}

float CheckRangeAndConvertToMeters(float z_buffer_value, double z_near,
                                   double z_far) {
  // Initialize with the assumption that the buffer value is outside the range
  // [kClippingPlaneNear, kClippingPlaneFar]. If the buffer value is *not* 1,
  // then it lies inside the range.
  float z = std::numeric_limits<float>::quiet_NaN();
  if (z_buffer_value != 1.f) {
    z = static_cast<float>(kB / (z_buffer_value - kA));
    if (z > z_far) {
      z = InvalidDepth::kTooFar;
    } else if (z < z_near) {
      z = InvalidDepth::kTooClose;
    }
  }
  return z;
}

}  // namespace

RenderEngineVtk::RenderEngineVtk()
    : color_palette_(kNumMaxLabel, RenderLabel::terrain_label(),
                     RenderLabel::empty_label()) {
  ColorD sky_color =
      ColorPalette::Normalize(color_palette_.get_sky_color());
  const vtkSmartPointer<vtkTransform> vtk_identity =
      ConvertToVtkTransform(Eigen::Isometry3d::Identity());

  for (auto& renderer : MakeVtkPointerArray(rgbd_renderer_, label_renderer_)) {
    renderer->SetBackground(sky_color.r, sky_color.g, sky_color.b);
    auto camera = renderer->GetActiveCamera();
    camera->SetViewAngle(90.0);  // Default to a 90° field of view.
    camera->SetClippingRange(kClippingPlaneNear, kClippingPlaneFar);
    SetModelTransformMatrixToVtkCamera(camera, vtk_identity);
  }

  rgbd_renderer_->SetUseDepthPeeling(1);
  rgbd_renderer_->UseFXAAOn();

  rgbd_window_->SetSize(0, 0);
  rgbd_window_->AddRenderer(rgbd_renderer_.Get());

  rgb_filter_->SetInputBufferTypeToRGBA();
  rgb_filter_->SetInput(rgbd_window_.GetPointer());
  d_filter_->SetInputBufferTypeToZBuffer();
  d_filter_->SetInput(rgbd_window_.GetPointer());

  label_window_->SetSize(0, 0);
  label_window_->AddRenderer(label_renderer_.Get());
  label_window_->SetMultiSamples(0);
  label_filter_->SetInputBufferTypeToRGB();
  label_filter_->SetInput(label_window_.GetPointer());

  auto exporters = MakeVtkPointerArray(
      rgb_exporter_, d_exporter_, label_exporter_);

  auto filters = MakeVtkPointerArray(rgb_filter_, d_filter_, label_filter_);

  for (int i = 0; i < 3; ++i) {
    filters[i]->SetScale(1);
    filters[i]->ReadFrontBufferOff();
    exporters[i]->SetInputData(filters[i]->GetOutput());
    exporters[i]->ImageLowerLeftOff();
  }
}

std::unique_ptr<RenderEngineVtk> RenderEngineVtk::Clone() const {
  auto clone = make_unique<RenderEngineVtk>();

  // Utility function for creating a cloned actor which *shares* the same
  // underlying polygonal data.
  auto clone_actor = [&clone](auto actor, auto renderer) {
    auto new_actor = vtkSmartPointer<vtkActor>::New();
    new_actor->GetProperty()->SetColor(actor->GetProperty()->GetColor());
    // NOTE: The clone renderer and original renderer *share* polygon data.
    // If the meshes were *deformable* this would be invalid. Furthermore,
    // even if dynamic adding/removing of geometry were valid, VTK's
    // reference counting preserves the underlying geometry in the
    // copy that still references it.
    new_actor->SetMapper(actor->GetMapper());
    new_actor->SetUserTransform(actor->GetUserTransform());
    // This is necessary because *terrain* has its lighting turned off. To
    // blindly handle arbitrary actors being flagged as terrain, we need to
    // treat all actors this way.
    new_actor->GetProperty()->SetLighting(
        actor->GetProperty()->GetLighting());
    renderer->AddActor(new_actor);
    return new_actor;
  };

  DRAKE_DEMAND(rgbd_actors_.size() == label_actors_.size());
  const int actor_count = static_cast<int>(rgbd_actors_.size());
  for (int i = 0; i < actor_count; ++i) {
    // Color actor
    auto color_actor = clone_actor(rgbd_actors_.at(i).Get(),
                                   clone->rgbd_renderer_.Get());
    clone->rgbd_actors_.push_back(color_actor);

    // Label actor
    auto label_actor = clone_actor(label_actors_.at(i).Get(),
                                   clone->label_renderer_.Get());
    clone->label_actors_.push_back(label_actor);
  }

  // Copy camera properties
  auto copy_cameras = [](auto src_renderer, auto dst_renderer) {
    dst_renderer->GetActiveCamera()->DeepCopy(
        src_renderer->GetActiveCamera());
  };
  copy_cameras(rgbd_renderer_.Get(), clone->rgbd_renderer_.Get());
  copy_cameras(label_renderer_.Get(), clone->label_renderer_.Get());

  return clone;
}

void RenderEngineVtk::AddFlatTerrain() {
  ColorD terrain_color =
      color_palette_.get_normalized_color(RenderLabel::terrain_label());
  RenderMaterial material(RenderLabel::terrain_label(),
                          Eigen::Vector4d{terrain_color.r, terrain_color.g,
                                          terrain_color.b, 1.0});
  // TODO(SeanCurtis-TRI): This is bad; I'm consuming an index for something
  // that isn't stored in SceneGraph. This should be killed in favor of actually
  // introducing managed geometry (and appropriate materials).
  RegisterVisual(HalfSpace(), material);
}

RenderIndex RenderEngineVtk::RegisterVisual(const Shape& shape,
                                            const RenderMaterial& material) {
  DRAKE_DEMAND(rgbd_actors_.size() == label_actors_.size());
  // The next index is just the next available slot; assuming rgbd and label are
  // kept in sync.
  RenderIndex index{static_cast<int>(rgbd_actors_.size())};
  // Note: the user_data interface on reification requires a non-const pointer
  shape.Reify(this, const_cast<RenderMaterial*>(&material));
  return index;
}

void RenderEngineVtk::UpdateVisualPose(const Eigen::Isometry3d& X_WG,
                                       RenderIndex geometry_id) const {
  vtkSmartPointer<vtkTransform> vtk_X_WG = ConvertToVtkTransform(X_WG);
  // TODO(SeanCurtis-TRI): Provide the ability to specify one or both sets of
  // actors.
  rgbd_actors_[geometry_id]->SetUserTransform(vtk_X_WG);
  label_actors_[geometry_id]->SetUserTransform(vtk_X_WG);
}

void RenderEngineVtk::UpdateViewpoint(const Eigen::Isometry3d& X_WR) const {
  vtkSmartPointer<vtkTransform> vtk_X_WR = ConvertToVtkTransform(X_WR);

  for (auto& renderer : MakeVtkPointerArray(rgbd_renderer_, label_renderer_)) {
    auto camera = renderer->GetActiveCamera();
    SetModelTransformMatrixToVtkCamera(camera, vtk_X_WR);
  }
}

void RenderEngineVtk::RenderColorImage(const CameraProperties& camera,
                                       ImageRgba8U* color_image_out,
                                       bool show_window) const {
  // TODO(sherm1) Should evaluate VTK cache entry.
  UpdateCamera(camera, show_window, rgbd_window_.GetPointer(),
               rgbd_renderer_.GetPointer());
  PerformVTKUpdate(rgbd_window_, rgb_filter_, rgb_exporter_);
  rgb_exporter_->Export(color_image_out->at(0, 0));
}

void RenderEngineVtk::RenderDepthImage(const DepthCameraProperties& camera,
                                       ImageDepth32F* depth_image_out,
                                       bool show_window) const {
  // TODO(sherm1) Should evaluate VTK cache entry.
  UpdateCamera(camera, show_window, rgbd_window_.GetPointer(),
               rgbd_renderer_.GetPointer());
  PerformVTKUpdate(rgbd_window_, d_filter_, d_exporter_);
  d_exporter_->Export(depth_image_out->at(0, 0));

  // TODO(kunimatsu-tri) Calculate this in a vertex shader.
  for (int v = 0; v < camera.height; ++v) {
    for (int u = 0; u < camera.width; ++u) {
      depth_image_out->at(u, v)[0] = CheckRangeAndConvertToMeters(
          depth_image_out->at(u, v)[0], camera.z_near, camera.z_far);
    }
  }
}

void RenderEngineVtk::RenderLabelImage(const CameraProperties& camera,
                                       ImageLabel16I* label_image_out,
                                       bool show_window) const {
  // TODO(SeanCurtis-TRI): Rather than rendering human readable palette
  // colors, the labels should be encoded as colors *directly* and only
  // re-encoded into colors when visualization is required. In other words,
  // we should render directly to the computationally meaningful format and
  // leave the human filter as a post-processing affect.
  // TODO(sherm1) Should evaluate VTK cache entry.
  UpdateCamera(camera, show_window, label_window_.GetPointer(),
               label_renderer_.GetPointer());
  PerformVTKUpdate(label_window_, label_filter_, label_exporter_);

  ImageRgb8U image(camera.width, camera.height);
  label_exporter_->Export(image.at(0, 0));

  ColorI color;
  for (int v = 0; v < camera.height; ++v) {
    for (int u = 0; u < camera.width; ++u) {
      color.r = image.at(u, v)[0];
      color.g = image.at(u, v)[1];
      color.b = image.at(u, v)[2];
      label_image_out->at(u, v)[0] = color_palette_.LookUpId(color);
    }
  }
}

void RenderEngineVtk::ImplementGeometry(const Sphere& sphere, void* user_data) {
  vtkNew<vtkSphereSource> vtk_sphere;
  vtk_sphere->SetRadius(sphere.get_radius());
  // TODO(SeanCurtis-TRI): Provide control for smoothness/tessellation.
  vtk_sphere->SetThetaResolution(50);
  vtk_sphere->SetPhiResolution(50);
  ImplementGeometry(vtk_sphere.GetPointer(), user_data);
}

void RenderEngineVtk::ImplementGeometry(const Cylinder& cylinder,
                                        void* user_data) {
  vtkNew<vtkCylinderSource> vtk_cylinder;
  vtk_cylinder->SetHeight(cylinder.get_length());
  vtk_cylinder->SetRadius(cylinder.get_radius());
  // TODO(SeanCurtis-TRI): Provide control for smoothness/tessellation.
  vtk_cylinder->SetResolution(50);

  // Since the cylinder in vtkCylinderSource is y-axis aligned, we need
  // to rotate it to be z-axis aligned because that is what Drake uses.
  vtkNew<vtkTransform> transform;
  transform->RotateX(90);
  vtkNew<vtkTransformPolyDataFilter> transform_filter;
  transform_filter->SetInputConnection(vtk_cylinder->GetOutputPort());
  transform_filter->SetTransform(transform.GetPointer());
  transform_filter->Update();

  ImplementGeometry(transform_filter.GetPointer(), user_data);
}

void RenderEngineVtk::ImplementGeometry(const HalfSpace&,
                                        void* user_data) {
  vtkNew<vtkPlaneSource> vtk_plane;
  // TODO(SeanCurtis-TRI): Provide control for plane resolution and size.
  vtk_plane->SetResolution(50, 50);

  // The vtk plane is a 1 x 1 plane. This scales it up to 100 x 100 plane
  // (centered on the origin).
  vtkNew<vtkTransform> transform;
  transform->Scale(100, 100, 100);
  vtkNew<vtkTransformPolyDataFilter> transform_filter;
  transform_filter->SetInputConnection(vtk_plane->GetOutputPort());
  transform_filter->SetTransform(transform.GetPointer());
  transform_filter->Update();

  ImplementGeometry(transform_filter.GetPointer(), user_data);
}

void RenderEngineVtk::ImplementGeometry(const Box& box, void* user_data) {
  vtkNew<vtkCubeSource> cube;
  cube->SetXLength(box.width());
  cube->SetYLength(box.depth());
  cube->SetZLength(box.height());
  ImplementGeometry(cube.GetPointer(), user_data);
}

void RenderEngineVtk::ImplementGeometry(const Mesh& mesh, void* user_data) {
  vtkNew<vtkOBJReader> mesh_reader;
  mesh_reader->SetFileName(mesh.filename().c_str());
  mesh_reader->Update();

  vtkNew<vtkTransform> transform;
  // TODO(SeanCurtis-TRI): Should I be allowing only isotropic scale.
  // TODO(SeanCurtis-TRI): Only add the transform filter if scale is not all 1.
  const double scale = mesh.scale();
  transform->Scale(scale, scale, scale);
  vtkNew<vtkTransformPolyDataFilter> transform_filter;
  transform_filter->SetInputConnection(mesh_reader->GetOutputPort());
  transform_filter->SetTransform(transform.GetPointer());
  transform_filter->Update();

  ImplementGeometry(transform_filter.GetPointer(), user_data);
}

/** Returns the sky's color in an RGB image. */
const ColorI& RenderEngineVtk::get_sky_color() const {
  return color_palette_.get_sky_color();
}

/** Returns flat terrain's color in an RGB image. */
const ColorI& RenderEngineVtk::get_flat_terrain_color() const {
  return color_palette_.get_terrain_color();
}

void RenderEngineVtk::ImplementGeometry(vtkPolyDataAlgorithm* source,
                                        void* user_data) {
  DRAKE_DEMAND(rgbd_actors_.size() == label_actors_.size());
  // By the time this is invoked, a material must be given.
  DRAKE_DEMAND(user_data != nullptr);

  auto color_actor = vtkSmartPointer<vtkActor>::New();
  rgbd_actors_.push_back(color_actor);

  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputConnection(source->GetOutputPort());

  // TODO(SeanCurtis-TRI): Process the texture, if and when it exists.
  const RenderMaterial& material = *static_cast<RenderMaterial*>(user_data);
  const auto diffuse = material.diffuse();
  color_actor->GetProperty()->SetColor(diffuse(0), diffuse(1), diffuse(2));
  RenderLabel render_label = material.label();

  // TODO(SeanCurtis-TRI): Swap this for ignored.
  // Anything marked terrain has its lighting turned off.
  // NOTE: In talking to Kuni (8/23/18), the feeling is that this was done to
  // facilitate testing. It would be better if this were a material property.
  if (render_label == RenderLabel::terrain_label())
    color_actor->GetProperty()->LightingOff();

  auto label_actor = vtkSmartPointer<vtkActor>::New();
  label_actors_.push_back(label_actor);

  const auto& label_color = color_palette_.get_normalized_color(render_label);
  label_actor->GetProperty()->SetColor(label_color.r, label_color.g,
                                       label_color.b);

  // Can this not be done better at the renderer level? Turn off automatic
  // lights and make sure there are *no* lights in the renderer. Surely
  // that would have the same result?
  // This is to disable shadows and to get an object painted with a single
  // color.
  label_actor->GetProperty()->LightingOff();

  vtkSmartPointer<vtkTransform> vtk_identity =
      ConvertToVtkTransform(Isometry3<double>::Identity());

  auto wire_actor = [&mapper, &vtk_identity](vtkActor* actor,
                                             vtkRenderer* renderer) {
    actor->SetMapper(mapper.GetPointer());
    actor->SetUserTransform(vtk_identity);
    renderer->AddActor(actor);
  };
  wire_actor(color_actor.GetPointer(), rgbd_renderer_.GetPointer());
  wire_actor(label_actor.GetPointer(), label_renderer_.GetPointer());
}

}  // namespace render
}  // namespace geometry
}  // namespace drake
