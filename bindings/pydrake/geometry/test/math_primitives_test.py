import pydrake.geometry as mut  # ruff: isort: skip

import copy
import unittest

import numpy as np

from pydrake.common.test_utilities.pickle_compare import assert_pickle


class TestGeometryMathPrimitives(unittest.TestCase):
    def test_plane(self):
        # Test constructor and accessors.
        normal = [1.0, 0.0, 0.0]
        point_on_plane = [0.0, 2.0, 3.0]
        plane = mut.Plane(
            normal=normal,
            point_on_plane=point_on_plane,
            already_normalized=True,
        )
        np.testing.assert_allclose(plane.normal(), normal)
        self.assertAlmostEqual(plane.CalcHeight(plane.point_on_plane()), 0.0)

        plane = mut.Plane(normal=[2, 0, 0], point_on_plane=point_on_plane)
        np.testing.assert_allclose(plane.normal(), normal)

        # Test CalcHeight().
        Q_above = [1.0, 2.0, 3.0]  # 1 unit above the plane.
        Q_below = [-1.0, 2.0, 3.0]  # 1 unit below the plane.
        height_above = plane.CalcHeight(Q_above)
        height_below = plane.CalcHeight(Q_below)
        self.assertAlmostEqual(height_above, 1.0)
        self.assertAlmostEqual(height_below, -1.0)

        # Test copy and deepcopy.
        plane_copy = copy.copy(plane)
        np.testing.assert_allclose(plane_copy.normal(), normal)
        plane_deepcopy = copy.deepcopy(plane)
        np.testing.assert_allclose(plane_deepcopy.normal(), normal)

        # Test pickle.
        assert_pickle(
            self, plane, lambda x: f"Plane({x.normal()}, {x.point_on_plane()})"
        )
