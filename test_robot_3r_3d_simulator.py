"""Tests for the interactive 3D 3R robot simulator."""

import importlib
import importlib.util
import unittest

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


simulator_module = importlib.import_module("robot_3r_3d_simulator")


class SimulatorTestCase(unittest.TestCase):
    """Shared API assertions and Matplotlib cleanup."""

    def require_attr(self, name):
        self.assertTrue(
            hasattr(simulator_module, name),
            "robot_3r_3d_simulator is missing {!r}".format(name),
        )
        return getattr(simulator_module, name)

    def tearDown(self):
        plt.close("all")


class TestModuleAvailability(unittest.TestCase):
    """Catch a missing runnable 3D simulator module."""

    def test_3d_simulator_module_can_be_imported(self):
        spec = importlib.util.find_spec("robot_3r_3d_simulator")
        self.assertIsNotNone(spec, "robot_3r_3d_simulator.py does not exist")


class TestForwardKinematics(SimulatorTestCase):
    """Verify standard D-H rows and the complete 3R transform chain."""

    def test_general_standard_dh_transform(self):
        dh_transform = self.require_attr("dh_transform")
        actual = dh_transform(np.pi / 2.0, 2.0, np.pi / 2.0, 3.0)
        expected = np.array(
            [
                [0.0, 0.0, 1.0, 0.0],
                [1.0, 0.0, 0.0, 2.0],
                [0.0, 1.0, 0.0, 3.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        np.testing.assert_allclose(actual, expected, atol=1e-12)

    def test_fk_chain_and_analytic_endpoint_agree(self):
        forward_kinematics = self.require_attr("forward_kinematics")
        theta1, theta2, theta3 = np.deg2rad([35.0, 20.0, -55.0])
        result = forward_kinematics(theta1, theta2, theta3)
        radial = 2.0 * np.cos(theta2) + 1.5 * np.cos(theta2 + theta3)
        expected = np.array(
            [
                radial * np.cos(theta1),
                radial * np.sin(theta1),
                0.8
                + 2.0 * np.sin(theta2)
                + 1.5 * np.sin(theta2 + theta3),
            ]
        )

        np.testing.assert_allclose(result.origins[3], expected, atol=1e-12)
        np.testing.assert_allclose(
            result.t03,
            result.t01 @ result.t12 @ result.t23,
            atol=1e-12,
        )
        self.assertEqual(result.origins.shape, (4, 3))

    def test_fk_exposes_complete_dh_table(self):
        dh_parameters = self.require_attr("DHParameters")
        forward_kinematics = self.require_attr("forward_kinematics")
        result = forward_kinematics(0.1, 0.2, 0.3)

        self.assertEqual(
            result.dh_parameters,
            (
                dh_parameters(0.1, 0.0, np.pi / 2.0, 0.8),
                dh_parameters(0.2, 2.0, 0.0, 0.0),
                dh_parameters(0.3, 1.5, 0.0, 0.0),
            ),
        )

    def test_invalid_geometry_or_angles_are_rejected(self):
        forward_kinematics = self.require_attr("forward_kinematics")
        invalid_calls = [
            (0.0, 0.0, 0.0, -0.1, 2.0, 1.5),
            (0.0, 0.0, 0.0, 0.8, 0.0, 1.5),
            (0.0, 0.0, 0.0, 0.8, 2.0, np.inf),
            (np.nan, 0.0, 0.0, 0.8, 2.0, 1.5),
        ]

        for args in invalid_calls:
            with self.subTest(args=args), self.assertRaises(ValueError):
                forward_kinematics(*args)


if __name__ == "__main__":
    unittest.main()
