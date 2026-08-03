"""Tests for the planar 2R forward-kinematics simulator."""

import importlib
import importlib.util
import unittest

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


simulator_module = importlib.import_module("planar_2r_simulator")


class SimulatorTestCase(unittest.TestCase):
    """Shared assertions and figure cleanup for simulator tests."""

    def require_attr(self, name):
        self.assertTrue(
            hasattr(simulator_module, name),
            "planar_2r_simulator is missing {!r}".format(name),
        )
        return getattr(simulator_module, name)

    def tearDown(self):
        plt.close("all")


class TestModuleAvailability(unittest.TestCase):
    """Catch a missing runnable simulator module."""

    def test_simulator_module_can_be_imported(self):
        spec = importlib.util.find_spec("planar_2r_simulator")
        self.assertIsNotNone(spec, "planar_2r_simulator.py does not exist")


class TestKinematics(SimulatorTestCase):
    """Verify hand-derived D-H and forward-kinematics results."""

    def test_zero_angle_dh_transform(self):
        dh_transform = self.require_attr("dh_transform")
        actual = dh_transform(0.0, 2.0, 0.0, 0.0)
        expected = np.array(
            [
                [1.0, 0.0, 0.0, 2.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        np.testing.assert_allclose(actual, expected, atol=1e-12)

    def test_reference_forward_kinematics_poses(self):
        forward_kinematics = self.require_attr("forward_kinematics")
        cases = [
            ((0.0, 0.0), (3.5, 0.0)),
            ((np.pi / 2.0, 0.0), (0.0, 3.5)),
            ((0.0, np.pi), (0.5, 0.0)),
        ]

        for angles, expected_o2 in cases:
            with self.subTest(angles=angles):
                result = forward_kinematics(*angles, 2.0, 1.5)
                self.assertEqual(result.origins.shape, (3, 2))
                np.testing.assert_allclose(
                    result.origins[2], expected_o2, atol=1e-12
                )
                np.testing.assert_allclose(
                    result.t02, result.t01 @ result.t12, atol=1e-12
                )

    def test_forward_kinematics_exposes_standard_dh_rows(self):
        dh_parameters_cls = self.require_attr("DHParameters")
        forward_kinematics = self.require_attr("forward_kinematics")

        result = forward_kinematics(0.25, -0.5, 2.0, 1.5)

        self.assertEqual(
            result.dh_parameters,
            (
                dh_parameters_cls(theta=0.25, a=2.0, alpha=0.0, d=0.0),
                dh_parameters_cls(theta=-0.5, a=1.5, alpha=0.0, d=0.0),
            ),
        )

    def test_nonpositive_or_nonfinite_link_lengths_are_rejected(self):
        forward_kinematics = self.require_attr("forward_kinematics")
        invalid_lengths = [
            (0.0, 1.5),
            (-1.0, 1.5),
            (2.0, np.inf),
            (np.nan, 1.5),
        ]

        for lengths in invalid_lengths:
            with self.subTest(lengths=lengths):
                with self.assertRaisesRegex(ValueError, "finite positive"):
                    forward_kinematics(0.0, 0.0, *lengths)


if __name__ == "__main__":
    unittest.main()
