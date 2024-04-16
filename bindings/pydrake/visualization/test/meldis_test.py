"""Unit tests for MeLDiS.

You can also visualize the LCM test data like so:

1. In a separate terminal,
    bazel run //tools:meldis -- -w

2. In another terminal, pass the name of a specific test to bazel. For example,
    bazel run //bindings/pydrake/visualization:py/meldis_test \\
        TestMeldis.test_contact_applet_hydroelastic
"""

import pydrake.visualization as mut

import functools
import hashlib
import json
import os
from pathlib import Path
import tempfile
import unittest

import numpy as np
import umsgpack

from drake import (
    lcmt_point_cloud,
    lcmt_viewer_draw,
    lcmt_viewer_geometry_data,
    lcmt_viewer_link_data,
    lcmt_viewer_load_robot,
)

from pydrake.geometry import (
    DrakeVisualizer,
    DrakeVisualizerParams,
    Meshcat,
    MeshcatParams,
    Role,
)
from pydrake.lcm import (
    DrakeLcm,
)
from pydrake.multibody.parsing import (
    Parser,
)
from pydrake.multibody.plant import (
    AddMultibodyPlantSceneGraph,
    ConnectContactResultsToDrakeVisualizer,
)
from pydrake.multibody.tree import (
    PrismaticJoint,
)
from pydrake.perception import (
    BaseField,
    Fields,
    PointCloud,
    PointCloudToLcm,
)
from pydrake.systems.analysis import (
    Simulator,
)
from pydrake.systems.framework import (
    DiagramBuilder,
)
from pydrake.systems.lcm import (
    LcmPublisherSystem,
)
import pydrake.visualization.meldis

# https://bugs.launchpad.net/ubuntu/+source/u-msgpack-python/+bug/1979549
#
# Jammy shipped with python3-u-msgpack 2.3.0, which tries to use
# `collections.Hashable`, which was removed in Python 3.10. Work around this by
# monkey-patching `Hashable` into `umsgpack.collections`.
#
# TODO(mwoehlke-kitware): Remove this when Jammy's python3-u-msgpack has been
# updated to 2.5.2 or later.
if not hasattr(umsgpack, 'Hashable'):
    import collections
    setattr(umsgpack.collections, 'Hashable', collections.abc.Hashable)


class TestMeldis(unittest.TestCase):

    def _make_diagram(self, *, resource, visualizer_params, lcm):
        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
        parser = Parser(plant=plant)
        parser.AddModels(url=f"package://{resource}")
        plant.Finalize()
        DrakeVisualizer.AddToBuilder(builder=builder, scene_graph=scene_graph,
                                     params=visualizer_params, lcm=lcm)
        diagram = builder.Build()
        return diagram

    def _create_point_cloud(self, num_fields):
        if num_fields == 3:
            cloud_fields = Fields(BaseField.kXYZs)
        elif num_fields == 4:
            cloud_fields = Fields(BaseField.kXYZs | BaseField.kRGBs)
        else:
            assert num_fields == 7
            cloud_fields = Fields(
                BaseField.kXYZs | BaseField.kRGBs | BaseField.kNormals
            )

        num_points = 10
        cloud = PointCloud(num_points, cloud_fields)
        xyzs = np.random.uniform(-0.1, 0.1, (3, num_points))
        cloud.mutable_xyzs()[:] = xyzs
        if num_fields > 3:
            rgbs = np.random.randint(0, 255, (3, num_points), dtype=np.uint8)
            cloud.mutable_rgbs()[:] = rgbs
        if num_fields > 4:
            normals = np.random.uniform(-0.1, 0.1, (3, num_points))
            cloud.mutable_normals()[:] = normals
        return cloud

    def _publish_lcm_point_cloud(self, lcm, channel, cloud):
        """Given the complexity of the lcmt_point_cloud message format, the
        easiest way to feed the DUT with a sample point cloud is to use
        PointCloudToLcmSystem for the point cloud conversion and
        LcmPublisherSystem to publish the LCM message onto the DUT's LCM bus.
        """
        builder = DiagramBuilder()
        cloud_to_lcm = builder.AddSystem(PointCloudToLcm(frame_name="world"))
        cloud_lcm_publisher = builder.AddSystem(
            LcmPublisherSystem.Make(
                channel=channel,
                lcm_type=lcmt_point_cloud,
                lcm=lcm,
                publish_period=1.0,
                use_cpp_serializer=True))
        builder.Connect(
            cloud_to_lcm.get_output_port(),
            cloud_lcm_publisher.get_input_port())
        diagram = builder.Build()

        # Set input and publish the point cloud.
        context = diagram.CreateDefaultContext()
        cloud_context = cloud_to_lcm.GetMyContextFromRoot(context)
        cloud_to_lcm.get_input_port().FixValue(cloud_context, cloud)
        diagram.ForcedPublish(context)

    def test_viewer_applet(self):
        """Checks that _ViewerApplet doesn't crash when receiving messages.
        Note that many geometry types are not yet covered by this test.
        """
        # Create the device under test.
        dut = mut.Meldis()
        meshcat = dut.meshcat
        lcm = dut._lcm

        # Polling before any messages doesn't crash.
        dut._invoke_poll()

        # Enqueue the load + draw messages.
        diagram = self._make_diagram(
            resource="drake/multibody/benchmarks/acrobot/acrobot.sdf",
            visualizer_params=DrakeVisualizerParams(),
            lcm=lcm)
        context = diagram.CreateDefaultContext()
        diagram.ForcedPublish(context)

        # The geometry isn't registered until the load is processed.
        self.assertEqual(meshcat.HasPath("/DRAKE_VIEWER"), False)
        link_path = "/DRAKE_VIEWER/2/plant/acrobot/Link2/0"
        self.assertEqual(meshcat.HasPath(link_path), False)

        # Process the load + draw; make sure the geometry exists now.
        lcm.HandleSubscriptions(timeout_millis=0)
        dut._invoke_subscriptions()
        self.assertEqual(meshcat.HasPath("/DRAKE_VIEWER"), True)
        self.assertEqual(meshcat.HasPath(link_path), True)

    def _check_viewer_applet_on_model(self, resource):
        """Checks that _ViewerApplet doesn't crash on the given model file.
        """
        dut = mut.Meldis()
        lcm = dut._lcm
        diagram = self._make_diagram(
            resource=resource,
            visualizer_params=DrakeVisualizerParams(),
            lcm=lcm)
        diagram.ForcedPublish(diagram.CreateDefaultContext())
        lcm.HandleSubscriptions(timeout_millis=0)
        dut._invoke_subscriptions()
        self.assertEqual(dut.meshcat.HasPath("/DRAKE_VIEWER"), True)

    def test_viewer_applet_plain_meshes(self):
        """Checks _ViewerApplet support for untextured meshes.
        """
        self._check_viewer_applet_on_model(
            "drake_models/iiwa_description/urdf/"
            "iiwa14_no_collision.urdf")

    def test_viewer_applet_textured_meshes(self):
        """Checks _ViewerApplet support for textured meshes.
        """
        self._check_viewer_applet_on_model(
            "drake_models/ycb/004_sugar_box.sdf")

    def test_viewer_applet_reload_optimization(self):
        """Checks that loading the identical scene twice is efficient.
        """
        # Create the device under test.
        dut = mut.Meldis()
        meshcat = dut.meshcat

        # Initialize an identical simulation twice in a row. The second time
        # around should be more efficient.
        for i in range(2):
            # To parcel out messages one at a time, we'll have the Diagram send
            # its messages to a temporary location and then forward them along
            # one at a time to Meldis.
            temp_lcm = DrakeLcm()
            temp_lcm_queue = []   # Tuples of (channel, buffer).
            for name in ["DRAKE_VIEWER_LOAD_ROBOT", "DRAKE_VIEWER_DRAW"]:
                def _on_message(channel, data):
                    temp_lcm_queue.append((channel, data))
                temp_lcm.Subscribe(channel=name, handler=functools.partial(
                    _on_message, name))

            # Create the plant + visualizer.
            diagram = self._make_diagram(
                resource="drake/multibody/benchmarks/acrobot/acrobot.sdf",
                visualizer_params=DrakeVisualizerParams(),
                lcm=temp_lcm)

            # Capture the LOAD and DRAW messages via temp_lcm.
            diagram.ForcedPublish(diagram.CreateDefaultContext())
            temp_lcm.HandleSubscriptions(timeout_millis=0)
            load, draw = temp_lcm_queue
            assert load[0] == "DRAKE_VIEWER_LOAD_ROBOT"
            assert draw[0] == "DRAKE_VIEWER_DRAW"

            # Forward the LOAD message to Meldis.
            dut._lcm.Publish(*load)
            dut._lcm.HandleSubscriptions(timeout_millis=0)
            dut._invoke_subscriptions()

            # The first time around, the link won't be added until the DRAW
            # message arrives. The second time around, the link should still
            # be intact from the first time (i.e., no `Delete` occurred).
            link_path = "/DRAKE_VIEWER/2/plant/acrobot/Link2/0"
            if i == 0:
                self.assertFalse(meshcat.HasPath(link_path))
            else:
                self.assertTrue(meshcat.HasPath(link_path))

            # Forward the DRAW message to Meldis.
            dut._lcm.Publish(*draw)
            dut._lcm.HandleSubscriptions(timeout_millis=0)
            dut._invoke_subscriptions()

            # The link always exists after a DRAW.
            self.assertTrue(meshcat.HasPath(link_path))

    def test_reload_detection(self):
        """Tests the mechanism viewer applet uses to determine if it should
        ignore a load message.

        The loading is spread across on_viewer_load and on_viewer_draw. The
        first determines if a redraw is necessary (by setting a flag), and
        the latter actually performs the loading (lazily) while computing
        file checksums.

        We'll successively execute both functions and peek into the applet's
        state to see how the redraw signal and hash values evolve.
        """
        meshcat = Meshcat()
        applet = mut._meldis._ViewerApplet(meshcat=meshcat,
                                           path="/DRAKE_VIEWER",
                                           alpha_slider_name="ReloadTest")

        # The applet starts with no hash history.
        self.assertDictEqual(applet._load_deduplicator._path_hashes, {})

        # We need to create an obj and mtl file so we can test changes to
        # referenced files.
        test_tmpdir = Path(os.environ["TEST_TMPDIR"])
        png_filename = test_tmpdir / "reload_test.png"
        with open(png_filename, "w") as f:
            f.write("Not really a png")

        mtl_filename = test_tmpdir / "reload_test.mtl"
        with open(mtl_filename, "w") as f:
            f.write("""
                newmtl test_mat
                map_Kd reload_test.png
                Kd 1 1 1""")

        obj_filename = test_tmpdir / "reload_test.obj"
        with open(obj_filename, "w") as f:
            f.write("""
                mtllib reload_test.mtl
                v 0 0 0
                v 1 0 0
                v 0 1 0
                usemtl test_mat
                f 1 2 3""")

        # Now we need a load message that references the .obj.
        mesh = lcmt_viewer_geometry_data()
        mesh.float_data = (1, 1, 1)
        mesh.position = (0, 0, 0)
        mesh.quaternion = (1, 0, 0, 0)
        mesh.color = (1, 0, 0, 1)
        mesh.type = lcmt_viewer_geometry_data.MESH
        mesh.string_data = str(obj_filename)
        link = lcmt_viewer_link_data()
        link.name = "link1"
        link.robot_num = 1
        link.geom = [mesh]
        link.num_geom = len(link.geom)
        load_message = lcmt_viewer_load_robot()
        load_message.link = [link]
        load_message.num_links = len(load_message.link)

        # And a draw message that will trigger creation of objects in meshcat.
        draw_message = lcmt_viewer_draw()
        draw_message.link_name = [link.name]
        draw_message.robot_num = [link.robot_num]
        draw_message.position = [(0, 0, 0)]
        draw_message.quaternion = [(1, 0, 0, 0)]

        # The first load message should load and report a need to redraw. No
        # files have been hashed yet, the only thing we did was memorize the
        # load message.
        applet.on_viewer_load(load_message)
        self.assertTrue(applet._waiting_for_first_draw_message)
        self.assertDictEqual(applet._load_deduplicator._path_hashes, {})

        # After drawing, we should have all three files in the checksums.
        applet.on_viewer_draw(draw_message)
        self.assertFalse(applet._waiting_for_first_draw_message)
        self.assertSetEqual(set(applet._load_deduplicator._path_hashes.keys()),
                            {png_filename, mtl_filename, obj_filename})

        # Attempting to load the same load message without changes, means we
        # shouldn't be waiting for a first draw message and checksums are
        # unchanged.
        checksum_copy = applet._load_deduplicator._path_hashes.copy()
        applet.on_viewer_load(load_message)
        self.assertFalse(applet._waiting_for_first_draw_message)
        self.assertDictEqual(applet._load_deduplicator._path_hashes,
                             checksum_copy)

        # Now we'll modify the mtl by removing the texture map.
        with open(mtl_filename, "w") as f:
            f.write("newmtl test_mat\n")
            f.write("Kd 1 0 0")
        old_hashes = applet._load_deduplicator._path_hashes.copy()
        applet.on_viewer_load(load_message)
        self.assertDictEqual(applet._load_deduplicator._path_hashes, {})
        applet.on_viewer_draw(draw_message)
        self.assertSetEqual(set(applet._load_deduplicator._path_hashes.keys()),
                            {mtl_filename, obj_filename})

    def test_viewer_applet_alpha_slider(self):
        # Create the device under test.
        dut = mut.Meldis()
        meshcat = dut.meshcat
        lcm = dut._lcm

        # Create a simple scene with an invisible geometry (alpha == 0.0).
        rgb = [0.0, 0.0, 1.0]
        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
        parser = Parser(plant=plant)
        parser.AddModelsFromString(f"""
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="box">
    <link name="box">
      <visual name="box">
        <geometry>
          <box>
            <size>1 1 1</size>
          </box>
        </geometry>
        <material>
          <diffuse>{rgb[0]} {rgb[1]} {rgb[2]} 0.0</diffuse>
        </material>
      </visual>
    </link>
    <static>1</static>
  </model>
</sdf>
""", "sdf")
        plant.Finalize()
        DrakeVisualizer.AddToBuilder(builder=builder, scene_graph=scene_graph,
                                     params=DrakeVisualizerParams(), lcm=lcm)
        diagram = builder.Build()

        # Process the load + draw messages.
        context = diagram.CreateDefaultContext()
        diagram.ForcedPublish(context)
        lcm.HandleSubscriptions(timeout_millis=0)
        dut._invoke_subscriptions()

        # Request a different alpha.
        new_alpha = 0.5
        meshcat.SetSliderValue("Viewer α", new_alpha)
        dut._invoke_poll()

        # Confirm the new (modulated) opacity of the box. Note: this doesn't
        # actually test the resulting opacity of the box, merely that we
        # called set_property("/DRAKE_VIEWER", "modulated_opacity", new_alpha).
        # We rely on meshcat to do the "right" thing in response.
        path = "/DRAKE_VIEWER"
        self.assertEqual(meshcat.HasPath(path), True)
        message = meshcat._GetPackedProperty(path, "modulated_opacity")
        parsed = umsgpack.unpackb(message)
        self.assertEqual(parsed['value'], new_alpha)

    def test_inertia_geometry(self):
        url = "package://drake_models/manipulation_station/sphere.sdf"
        dut = mut.Meldis()
        lcm = dut._lcm
        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
        parser = Parser(plant=plant)
        parser.AddModels(url=url)
        plant.Finalize()
        config = mut.VisualizationConfig()
        mut.ApplyVisualizationConfig(config=config, builder=builder, lcm=lcm)
        diagram = builder.Build()
        simulator = Simulator(diagram)
        simulator.AdvanceTo(0.1)
        with self.assertRaises(SystemExit):
            dut.serve_forever(idle_timeout=0.001)
        path = "/Inertia Visualizer/0/inertia_visualizer/InertiaVisualizer"
        self.assertTrue(dut.meshcat.HasPath(path))
        self.assertTrue(dut.meshcat.HasPath(f"{path}/sphere/base_link"))

    def test_hydroelastic_geometry(self):
        """Checks that _ViewerApplet doesn't crash when receiving
        hydroelastic geometry.
        """
        dut = mut.Meldis()
        lcm = dut._lcm
        diagram = self._make_diagram(
            resource="drake/examples/hydroelastic/"
                     "spatula_slip_control/spatula.sdf",
            visualizer_params=DrakeVisualizerParams(
                show_hydroelastic=True,
                role=Role.kProximity),
            lcm=lcm)
        context = diagram.CreateDefaultContext()
        diagram.ForcedPublish(context)
        lcm.HandleSubscriptions(timeout_millis=0)
        dut._invoke_subscriptions()

    def test_contact_applet_point_pair(self):
        """Checks that _ContactApplet doesn't crash when receiving point
           contact messages.
        """
        # Create the device under test.
        dut = mut.Meldis()
        meshcat = dut.meshcat
        lcm = dut._lcm

        # Enqueue a point contact result message.
        url = "package://drake_models/manipulation_station/sphere.sdf"
        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.001)
        sphere1_model, = Parser(plant, "sphere1").AddModels(url=url)
        sphere2_model, = Parser(plant, "sphere2").AddModels(url=url)
        body1 = plant.GetBodyByName("base_link", sphere1_model)
        body2 = plant.GetBodyByName("base_link", sphere2_model)
        plant.AddJoint(PrismaticJoint(
            name="sphere1_x", frame_on_parent=plant.world_body().body_frame(),
            frame_on_child=body1.body_frame(), axis=[1, 0, 0]))
        plant.AddJoint(PrismaticJoint(
            name="sphere2_x", frame_on_parent=plant.world_body().body_frame(),
            frame_on_child=body2.body_frame(), axis=[1, 0, 0]))
        plant.Finalize()
        ConnectContactResultsToDrakeVisualizer(
            builder=builder, plant=plant, scene_graph=scene_graph, lcm=lcm)
        diagram = builder.Build()
        context = diagram.CreateDefaultContext()
        plant.SetPositions(plant.GetMyMutableContextFromRoot(context),
                           [-0.03, 0.03])
        diagram.ForcedPublish(context)

        # The geometry isn't registered until the load is processed.
        pair_path = "/CONTACT_RESULTS/point/base_link(2)+base_link(3)"
        self.assertEqual(meshcat.HasPath(pair_path), False)

        # Process the load + draw; make sure the geometry exists now.
        lcm.HandleSubscriptions(timeout_millis=0)
        dut._invoke_subscriptions()

        self.assertEqual(meshcat.HasPath(pair_path), True)

    def test_contact_applet_hydroelastic(self):
        """Checks that _ContactApplet doesn't crash when receiving hydroelastic
           messages.
        """
        # Create the device under test.
        dut = mut.Meldis()
        meshcat = dut.meshcat
        lcm = dut._lcm

        # Enqueue a hydroelastic contact message.
        url = "package://drake/multibody/meshcat/test/hydroelastic.sdf"
        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.001)
        parser = Parser(plant=plant)
        parser.AddModels(url=url)
        body1 = plant.GetBodyByName("body1")
        body2 = plant.GetBodyByName("body2")
        plant.AddJoint(PrismaticJoint(
            name="body1", frame_on_parent=plant.world_body().body_frame(),
            frame_on_child=body1.body_frame(), axis=[0, 0, 1]))
        plant.AddJoint(PrismaticJoint(
            name="body2", frame_on_parent=plant.world_body().body_frame(),
            frame_on_child=body2.body_frame(), axis=[1, 0, 0]))
        plant.Finalize()
        ConnectContactResultsToDrakeVisualizer(
            builder=builder, plant=plant, scene_graph=scene_graph, lcm=lcm)
        diagram = builder.Build()
        context = diagram.CreateDefaultContext()
        plant.SetPositions(plant.GetMyMutableContextFromRoot(context),
                           [0.1, 0.3])
        diagram.ForcedPublish(context)

        # The geometry isn't registered until the load is processed.
        hydro_path = "/CONTACT_RESULTS/hydroelastic/" + \
                     "body1.two_bodies::body1_collision+body2"
        hydro_path2 = "/CONTACT_RESULTS/hydroelastic/" + \
                      "body1.two_bodies::body1_collision2+body2"
        self.assertEqual(meshcat.HasPath(hydro_path), False)
        self.assertEqual(meshcat.HasPath(hydro_path2), False)

        # Process the load + draw; contact results should now exist.
        lcm.HandleSubscriptions(timeout_millis=1)
        dut._invoke_subscriptions()

        self.assertEqual(meshcat.HasPath("/CONTACT_RESULTS/hydroelastic"),
                         True)
        self.assertEqual(meshcat.HasPath(hydro_path), True)
        self.assertEqual(meshcat.HasPath(hydro_path2), True)

    def test_deformable(self):
        """Checks that _ViewerApplet doesn't crash for deformable geometries
        in DRAKE_VIEWER_DEFORMABLE channel.
        """

        # Create the device under test.
        dut = mut.Meldis()

        # Prepare a deformable-geometry message. It is a triangle surface mesh
        # of a tetrahedron.
        geom0 = lcmt_viewer_geometry_data()
        geom0.type = lcmt_viewer_geometry_data.MESH
        geom0.position = [0.0, 0.0, 0.0]
        q_wxyz = [1.0, 0.0, 0.0, 0.0]
        geom0.quaternion = q_wxyz
        geom0.color = [0.9, 0.9, 0.9, 1.0]
        geom0.string_data = "tetrahedron"
        geom0.float_data = [
            # 4 vertices and 4 triangles
            4.0, 4.0,
            # 4 vertices at the origin and on the three axes.
            0.0, 0.0, 0.0,
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
            # 4 triangles, use float for integer vertex indices
            0., 2., 1.,
            0., 1., 3.,
            0., 3., 2.,
            1., 2., 3.]
        geom0.num_float_data = len(geom0.float_data)
        message = lcmt_viewer_link_data()
        message.name = "test_deformable_geometry"
        message.robot_num = 0
        message.num_geom = 1
        message.geom = [geom0]

        dut._lcm.Publish(channel="DRAKE_VIEWER_DEFORMABLE",
                         buffer=message.encode())

        meshcat_path = f"/DRAKE_VIEWER/{message.robot_num}/{message.name}"
        # Before the subscribed handlers are called, there is no meshcat path
        # from the published lcm message.
        self.assertEqual(dut.meshcat.HasPath(meshcat_path), False)

        dut._lcm.HandleSubscriptions(timeout_millis=1)
        dut._invoke_subscriptions()

        # After the handlers are called, we have the expected meshcat path.
        self.assertEqual(dut.meshcat.HasPath(meshcat_path), True)

    def test_point_cloud(self):
        """Checks that _PointCloudApplet doesn't crash when receiving point
        cloud messages.
        """
        # Create the device under test.
        dut = mut.Meldis()

        test_tuples = (
           (3, "DRAKE_POINT_CLOUD", "/POINT_CLOUD/default"),
           (4, "DRAKE_POINT_CLOUD_XYZRGB", "/POINT_CLOUD/XYZRGB"),
           (7, "DRAKE_POINT_CLOUD_12345", "/POINT_CLOUD/12345"),
        )
        for num_fields, channel, meshcat_path in test_tuples:
            with self.subTest(num_fields=num_fields):
                self.assertEqual(dut.meshcat.HasPath(meshcat_path), False)
                cloud = self._create_point_cloud(num_fields=num_fields)
                self._publish_lcm_point_cloud(dut._lcm, channel, cloud)

                dut._lcm.HandleSubscriptions(timeout_millis=1)
                dut._invoke_subscriptions()
                self.assertEqual(dut.meshcat.HasPath(meshcat_path), True)

    def test_draw_frame_applet(self):
        """Checks that _DrawFrameApplet doesn't crash when frames are sent
        in DRAKE_DRAW_FRAMES channel.
        """

        # Create the device under test.
        dut = mut.Meldis()

        # Prepare a frame message
        message = lcmt_viewer_draw()
        message.position = [[0.0, 0.0, 0.1]]
        message.quaternion = [[1.0, 0.0, 0.0, 0.0]]
        message.num_links = 1
        message.link_name = ['0']
        message.robot_num = [1]

        dut._lcm.Publish(channel="DRAKE_DRAW_FRAMES",
                         buffer=message.encode())

        meshcat_path = f"/DRAKE_DRAW_FRAMES/default/{message.link_name[0]}"
        # Before the subscribed handlers are called, there is no meshcat path
        # from the published lcm message.
        self.assertEqual(dut.meshcat.HasPath(meshcat_path), False)

        dut._lcm.HandleSubscriptions(timeout_millis=1)
        dut._invoke_subscriptions()
        # After the handlers are called, we have the expected meshcat path.
        self.assertEqual(dut.meshcat.HasPath(meshcat_path), True)

    def test_args_precedence(self):
        """Checks that the "kwargs wins" part of our API contract is met.
        """
        # When bad MeshcatParams are used Meldis rejects them, but good kwargs
        # can override them and win (no errors).
        bad_host = MeshcatParams(host="8.8.8.8")
        bad_port = MeshcatParams(port=1)
        with self.assertRaises(BaseException):
            mut.Meldis(meshcat_params=bad_host)
        with self.assertRaises(BaseException):
            mut.Meldis(meshcat_params=bad_port)
        mut.Meldis(meshcat_params=bad_host, meshcat_host="localhost")
        mut.Meldis(meshcat_params=bad_port, meshcat_port=0)

    def test_command_line_browser_names(self):
        """Sanity checks our webbrowser names logic. The objective is to return
        some kind of a list, without crashing.
        """
        names = pydrake.visualization.meldis._available_browsers()
        self.assertIsInstance(names, list)

    def test_command_meshcat_params(self):
        """Confirm that the params plumbing works by feeding in bad params and
        seeing a validation error be spit out.
        """
        main = pydrake.visualization.meldis._main
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "meshcat_params.yaml"
            path.write_text("{ port: 1 }", encoding="utf-8")
            args = [f"--meshcat-params={path}"]
            with self.assertRaisesRegex(BaseException, "port.*>.*1024"):
                main(args=args)
