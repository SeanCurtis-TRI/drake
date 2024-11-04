"""Tinkering with lighting"""

import math
import sys
import time

import matplotlib.pyplot as plt


from pydrake.common.value import Value
from pydrake.geometry import (
    Box,
    ClippingRange,
    ColorRenderCamera,
    EnvironmentMap,
    EquirectangularMap,
    FramePoseVector,
    GeometryFrame,
    GeometryId,
    GeometryInstance,
    IllustrationProperties,
    LightParameter,
    MakeRenderEngineVtk,
    Meshcat,
    MeshcatVisualizer,
    PerceptionProperties,
    QueryObject,
    RenderCameraCore,
    RenderEngineVtkParams,
    Rgba,
    SceneGraph,
    Sphere,
)
from pydrake.math import (
    RigidTransform,
    RotationMatrix,
)
from pydrake.systems.framework import (
    DiagramBuilder,
    LeafSystem,
)
from pydrake.systems.sensors import (
    CameraInfo,
    ImageRgba8U,
)

class CameraController(LeafSystem):
    def __init__(self, scene_graph, meshcat):
        """The single output port provides a RigidTransform defining the
        pose of the camera body in the world frame: X_WC. The value is defined
        by polling the given Meshcat instance. If the meshcat url includes the
        argument `tracked_camera=on` we'll get the pose of the visualizer's
        camera. Otherwise, we simply return some generic camera position.
        """
        LeafSystem.__init__(self)
        self._meshcat = meshcat
        self._source_id = scene_graph.RegisterSource("camera_controller")
        self._frame_id = scene_graph.RegisterFrame(
            source_id=self._source_id, frame=GeometryFrame("camera"))
        output_cls = Value[FramePoseVector]
        self.DeclareAbstractOutputPort("camera_pose",
                                       lambda: output_cls(),
                                       self.CalcPose)
        # Looking down from above.
        self._default_X_WC = RigidTransform(
            R=RotationMatrix.MakeXRotation(math.pi), p=[0, 0, 5])

    def frame_id(self):
        return self._frame_id

    def source_id(self):
        return self._source_id

    def CalcPose(self, context, pose_vector):
        X_WC = self._meshcat.GetTrackedCameraPose()
        if X_WC is None:
            X_WC = self._default_X_WC
        poses = FramePoseVector()
        poses.set_value(id=self._frame_id, value=X_WC)
        pose_vector.set_value(poses)

def assign_materials(geometry, diffuse_rgba):
    illustration = IllustrationProperties()
    perception = PerceptionProperties()
    illustration.AddProperty("phong", "diffuse", diffuse_rgba)
    perception.AddProperty("phong", "diffuse", diffuse_rgba)
    geometry.set_perception_properties(perception)
    geometry.set_illustration_properties(illustration)

def add_ground(context, scene_graph, source_id):
    ground = GeometryInstance(X_PG=RigidTransform([0, 0, -0.5]),
                              shape=Box(20, 20, 1), name="ground")
    red = Rgba(0.8, 0.2, 0.2)
    assign_materials(ground, red)
    scene_graph.RegisterGeometry(context=context, source_id=source_id,
                                 frame_id=scene_graph.world_frame_id(),
                                 geometry=ground)

def add_ball(context, scene_graph, source_id):
    ball = GeometryInstance(X_PG=RigidTransform([0, 0, 1]),
                            shape=Sphere(1), name="ball")
    blue = Rgba(0.2, 0.2, 0.8)
    assign_materials(ball, blue)
    scene_graph.RegisterGeometry(context=context, source_id=source_id,
                                 frame_id=scene_graph.world_frame_id(),
                                 geometry=ball)

def add_ground_and_sphere(scene_graph):
    """We'll simply populate the scene with a number of static objects that
    we can render.
    """
    s_id = scene_graph.RegisterSource("main")

    ground = GeometryInstance(X_PG=RigidTransform([0, 0, -0.5]),
                              shape=Box(20, 20, 1), name="ground")
    red = Rgba(0.8, 0.2, 0.2)
    assign_materials(ground, red)
    scene_graph.RegisterAnchoredGeometry(source_id=s_id, geometry=ground)

    ball = GeometryInstance(X_PG=RigidTransform([0, 0, 1]),
                            shape=Sphere(1), name="ball")
    blue = Rgba(0.2, 0.2, 0.8)
    assign_materials(ball, blue)
    scene_graph.RegisterAnchoredGeometry(source_id=s_id, geometry=ball)

class VtkParams:
    """Builder for RenderEngineVtkParams.
    
    We can build a dictionary of parameters as we go and then dump it out into
    an instance of RenderEngineVtkParams.
    
    The intention is to chain them together. E.g.:
    
        VtkParams().shadows_on().shadow_res(64).params()
        
    Creates an instance of RenderEngineVtkParams with shadows configured to be
    on and the shadow map resolution set to 64 pixels.
    """
    def __init__(self):
        self._params = {}

    def shadows_on(self):
        self._params["cast_shadows"] = True
        return self

    def shadows_off(self):
        self._params["cast_shadows"] = False
        return self

    def shadow_res(self, resolution):
        self._params["shadow_map_size"] = resolution
        return self

    def add_light(self, **light_parameters):
        """Adds a light to the parameters based on a dictionary of
        LightParameter parameters.
        """
        lights = self._params.setdefault("lights", [])
        lights.append(LightParameter(**light_parameters))
        return self

    def env_map(self, path, skybox=True):
        self_params["environment_map"] = EnvironmentMap(
            skybox=skybox, texture=EquirectangularMap(path=path))
        return self

    def exposure(self, value):
        self._params["exposure"] = value
        return self

    def params(self):
        return RenderEngineVtkParams(**self._params)


class Config:
    def __init__(self, description: str, vtk_params: VtkParams):
        self.description = description
        self.vtk_params = vtk_params

class Compare:
    """The pair of configured renderings that we want to compare along with
    some text noting what is of significance.
    """
    def __init__(self, a: Config, b: Config, text: str):
        self.a = a
        self.b = b
        self.text = text

def render_comparisons():
    """Returns a tuple of rendering comparisons.
    """
    # Lights
    L_dir_down = {"frame": "world", "direction": [0, 0, -1]}
    L_spot = {"type": "spot"}
    L_point = {"type": "point",
               "frame": "world",
               "position": [0, 0, 5]}
    L_w_spot_down = L_spot | {
                     "frame": "world",
                     "direction": [0, 0, -1],
                     "cone_angle": 50,
                     "position": [0, 0, 5]}
    L_n_spot_down = L_w_spot_down | {"cone_angle": 20}


    configs = {
        "default": Config("Default parameters", VtkParams()),
        "down_d_light": Config("Downward directional light in world",
                               VtkParams().add_light(**L_dir_down)),
        "down_spot_wide": Config("Downward wide spot light",
                                 VtkParams().add_light(**L_w_spot_down)),
        "down_spot_narrow": Config("Downward narrow spot light",
                                   VtkParams().add_light(**L_n_spot_down))
    }

    return (Compare(configs["default"], configs["down_d_light"],
                    "A: The default light is a directional light mounted to "
                    "the camera, pointing in the camera's view direction. "
                    "Objects are always illuminated from the camera's view "
                    "point.\n"
                    "B: The same directional light is posed w.r.t. the world "
                    "frame: pointing straight down. Only when the camera is "
                    "looking in the same direction as the light will objects "
                    "appear illuminated.\n"
                    "Move the camera to look down at the scene, and the two "
                    "renderings will look the same."),
            Compare(configs["down_d_light"],
                    Config("Point light above the scene",
                           VtkParams().add_light(**L_point)),
                    "A point light placed above the scene likewise casts "
                    "light downward. However, the point light illuminates the "
                    "ground differently. The directional light illuminates "
                    "the entire ground uniformly. The point light shines the "
                    "most light at the closest point, and points farther from "
                    "the light get less light. (It's not actually *distance* "
                    "that creates this affect, but the direction of the "
                    "surface normal at a point relative to the direction of "
                    "the light.)"),
            Compare(configs["down_d_light"], configs["down_spot_wide"],
                    "A: The downward directional light\n"
                    "B: A spot light with a full span of 100-degrees.\n"),
            Compare(configs["down_spot_wide"], configs["down_spot_narrow"],
                    "A: A spot light with a full span of 100-degrees.\n"
                    "B: The same spot light with a 40-degree span.\n"
                    "The narrower spot shows produces a much smaller circle "
                    "of illumination"),
            Compare(Config("Wide spot with shadows",
                           VtkParams().add_light(**L_w_spot_down).shadows_on()),
                    Config("Narrow spot with shadows",
                           VtkParams().add_light(**L_n_spot_down).shadows_on()),
                    "Shadows have been turned on. The angle of the cone "
                    "doesn't affect the size of the shadow; only the "
                    "distances between light source, object, and shadow "
                    "receiver"),
            Compare(Config("Wide spot with high resolution shadows",
                           VtkParams().shadows_on().shadow_res(512).
                           add_light(**L_w_spot_down)),
                    Config("Narrow spot with low resolution shadows",
                           VtkParams().add_light(**L_n_spot_down).shadows_on()),
                    "By increasing the shadow map resolution of the "
                    "wide-angle spot light, the fidelity of the shadow "
                    "increases."),
           )

def make_cameras(name1, name2):
    """Makes two cameras which exploit different renderers but are otherwise
    identical.
    """
    # This matches the default Meshcat vertical field of view.
    fov_y = 75 / 180 * math.pi
    core1 = RenderCameraCore(
        renderer_name=name1,
        intrinsics=CameraInfo(width=640, height=480, fov_y=fov_y),
        clipping=ClippingRange(0.1, 100.0),
        X_BS=RigidTransform())
    core2 = RenderCameraCore(
        renderer_name=name2,
        intrinsics=core1.intrinsics(),
        clipping=core1.clipping(),
        X_BS=core1.sensor_pose_in_camera_body())
    return (ColorRenderCamera(core=core1), ColorRenderCamera(core=core2))


class RenderEngineSpec:
    def __init__(self, name, spec):
        """Specification for a render engine.
        `name` will be used as the renderer name and be displayed with the
        image.
        `spec` is a dictionary that can be used as
        `RenderEngineVtkParams(**spec).
        """
        self.name = name
        self.spec = spec


class RenderComparator:
    """Configures a diagram that can produce render comparisons between
    multiple engines. The camera poses will be driven by a provided Meshcat
    instance. Each time the rendering is made, the camera pose is pulled from
    `meshcat`.
    """
    def __init__(self, meshcat):
        self._meshcat = meshcat
        builder = DiagramBuilder()
        self._scene_graph = builder.AddSystem(SceneGraph())
        # Note: we'll be changing render engines a lot. So, rather than declare
        # them in SceneGraph's model, we'll simply wait and add them to the
        # context.
        self._source_id = self._scene_graph.RegisterSource("main")

        MeshcatVisualizer.AddToBuilder(builder, self._scene_graph, self._meshcat)

        self._cam_controller = builder.AddSystem(
            CameraController(self._scene_graph, self._meshcat))
        builder.Connect(self._cam_controller.GetOutputPort("camera_pose"),
                        self._scene_graph.get_source_pose_port(
                            self._cam_controller.source_id()))

        builder.ExportOutput(self._scene_graph.get_query_output_port(),
                            "query_object")

        self._diagram = builder.Build()
        self._context = self._diagram.CreateDefaultContext()
        # Make sure MeshVisualizer publishes content to meshcat.
        self._diagram.ForcedPublish(self._context)

        self._sg_context = self._scene_graph.GetMyContextFromRoot(self._context)

        self._cameras = []

    @staticmethod
    def _make_camera(renderer_name):
        """Makes a camera referencing the given `renderer_name`."""
         # This matches the default Meshcat vertical field of view.
        fov_y = 75 / 180 * math.pi
        core = RenderCameraCore(
            renderer_name=renderer_name,
            intrinsics=CameraInfo(width=640, height=480, fov_y=fov_y),
            clipping=ClippingRange(0.1, 100.0),
            X_BS=RigidTransform())
        return ColorRenderCamera(core=core)

    def clear(self):
        """Clear's the geometry."""
        raise ValueError("I haven't implemented clear yet.")

    def visualize(self):
        # Make sure MeshVisualizer publishes content to meshcat.
        self._diagram.ForcedPublish(self._context)

    @staticmethod
    def _assign_materials(geometry, diffuse_rgba):
        illustration = IllustrationProperties()
        illustration.AddProperty("phong", "diffuse", diffuse_rgba)
        geometry.set_illustration_properties(illustration)

        perception = PerceptionProperties()
        perception.AddProperty("phong", "diffuse", diffuse_rgba)
        geometry.set_perception_properties(perception)

    def add_ground(self):
        ground = GeometryInstance(X_PG=RigidTransform([0, 0, -0.5]),
                                  shape=Box(20, 20, 1), name="ground")
        self._add_geometry(ground, Rgba(0.8, 0.2, 0.2))

    def add_ball(self):
        ball = GeometryInstance(X_PG=RigidTransform([0, 0, 1]),
                                shape=Sphere(1), name="ball")
        self._add_geometry(ball, Rgba(0.2, 0.2, 0.8))

    def _add_geometry(self, geometry, diffuse_color):
        self._assign_materials(geometry, diffuse_color)
        self._scene_graph.RegisterGeometry(
            context=self._sg_context,
            source_id=self._source_id,
            frame_id=self._scene_graph.world_frame_id(),
            geometry=geometry)

    def add_geometry(self, register_geo):
        """Calls register_geo(context, scene_graph, source_id) so that
        register_geo can add geometries to the scene. register_geo should use
        the context-variant registration methods.
        """
        register_geo(self._sg_context, self._scene_graph, self._source_id)

    def set_render_engines(self, renderer_specs):
        """Clears all render engines from the diagram, adding the engines
        implied by the `renderer_specs`.
        """
        for camera in self._cameras:
            self._scene_graph.RemoveRenderer(self._sg_context,
                                             camera.core().renderer_name())
        assert self._scene_graph.RendererCount(self._sg_context) == 0
        self._cameras.clear()

        for renderer in renderer_specs:
            self._cameras.append(self._make_camera(renderer.name))
            self._scene_graph.AddRenderer(
                context=self._sg_context, name=renderer.name,
                renderer=MakeRenderEngineVtk(renderer.spec.params()))

    def render(self):
        """Renders an image for each of the """
        # Bumping the time dirties the dependency tree; CameraController will
        # query for a new camera pose.
        self._context.SetTime(self._context.get_time() + 1)
        query_object = self._diagram.GetOutputPort("query_object").Eval(
            self._context)

        fig, ax = plt.subplots(1, len(self._cameras))
        if not isinstance(ax, list):
            ax = [ax]
        fig.subplots_adjust(wspace=0.01, hspace=0, left=0.05, right=0.95)

        images = []
        for i, camera in enumerate(self._cameras):
            image = query_object.RenderColorImage(
                camera=camera, parent_frame=self._cam_controller.frame_id(),
                X_PC=RigidTransform())
            ax[i].imshow(image.data)
            ax[i].set_title(camera.core().renderer_name())
        plt.pause(0.5)


def main():
    meshcat = Meshcat()
    dut = RenderComparator(meshcat)
    dut.add_ground()
    dut.add_ball()
    dut.visualize()

    for compare in render_comparisons():
        engine_specs = [RenderEngineSpec("A", compare.a.vtk_params),
                        RenderEngineSpec("B", compare.b.vtk_params)]
        dut.set_render_engines(engine_specs)

        running = True
        while running:
            dut.render()
            x = input("Hit Enter to re-render. Type x to go to the next scenario: ").lower()
            if x == 'x': break

def main2():
    builder = DiagramBuilder()

    scene_graph = builder.AddSystem(SceneGraph())
    # Note: we'll defer adding RenderEngines until after we've allocated the
    # context.

    add_ground_and_sphere(scene_graph)

    meshcat = Meshcat()
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    cam_controller = builder.AddSystem(CameraController(scene_graph, meshcat))
    builder.Connect(cam_controller.GetOutputPort("camera_pose"),
                    scene_graph.get_source_pose_port(
                        cam_controller.source_id()))

    builder.ExportOutput(scene_graph.get_query_output_port(), "query_object")
    
    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    # Make sure MeshVisualizer publishes content to meshcat.
    diagram.ForcedPublish(context)

    def render():
        context.SetTime(context.get_time() + 1)
        query_object = diagram.GetOutputPort("query_object").Eval(context)
        image1 = query_object.RenderColorImage(
            camera=camera1, parent_frame=cam_controller.frame_id(),
            X_PC=RigidTransform())
        ax[0].imshow(image1.data)
        image2 = query_object.RenderColorImage(
            camera=camera2, parent_frame=cam_controller.frame_id(),
            X_PC=RigidTransform())
        ax[1].imshow(image2.data)
        plt.pause(0.5)

    sg_context = scene_graph.GetMyContextFromRoot(context)

    def set_renderer(name, params: RenderEngineVtkParams):
        if scene_graph.HasRenderer(sg_context, name):
            scene_graph.RemoveRenderer(sg_context, name)
        scene_graph.AddRenderer(context=sg_context, name=name,
                                renderer=MakeRenderEngineVtk(params))

    # Prepare for rendering.
    name1 = "renderer1"
    name2 = "renderer2"
    camera1, camera2 = make_cameras(name1, name2)

    for compare in render_comparisons():
        set_renderer(name1, compare.a.vtk_params.params())
        set_renderer(name2, compare.b.vtk_params.params())

        fig, ax = plt.subplots(1, 2)
        # End the program when the figure gets closed.
        running = True
        def handle_close(evt):
            nonlocal running
            running = False
        fig.canvas.mpl_connect('close_event', handle_close)

        fig.subplots_adjust(wspace=0.01, hspace=0, left=0.05, right=0.95)
        ax[0].set_title(compare.a.description)
        ax[0].set_axis_off()
        ax[1].set_title(compare.b.description)
        ax[1].set_axis_off()
        print()
        print(compare.text)
        print("\nClose the window to proceed to the next example\n")
        while running:
            render()

    return 0


if __name__ == "__main__":
    sys.exit(main())
