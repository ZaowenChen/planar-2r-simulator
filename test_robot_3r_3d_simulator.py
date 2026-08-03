"""Tests for the interactive 3D 3R robot simulator."""

import importlib
import importlib.util
import unittest

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


simulator_module = importlib.import_module("robot_3r_3d_simulator")


def literal_zyx_rotation(roll, pitch, yaw):
    """Build Rz(yaw) Ry(pitch) Rx(roll) independently of production code."""

    cos_roll, sin_roll = np.cos(roll), np.sin(roll)
    cos_pitch, sin_pitch = np.cos(pitch), np.sin(pitch)
    cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)
    rotate_x = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, cos_roll, -sin_roll],
            [0.0, sin_roll, cos_roll],
        ]
    )
    rotate_y = np.array(
        [
            [cos_pitch, 0.0, sin_pitch],
            [0.0, 1.0, 0.0],
            [-sin_pitch, 0.0, cos_pitch],
        ]
    )
    rotate_z = np.array(
        [
            [cos_yaw, -sin_yaw, 0.0],
            [sin_yaw, cos_yaw, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    return rotate_z @ rotate_y @ rotate_x


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


class TestSpatialTransforms(SimulatorTestCase):
    """Verify fast inverse transforms and stable ZYX RPY extraction."""

    def test_fast_inverse_matches_general_inverse(self):
        forward_kinematics = self.require_attr("forward_kinematics")
        invert_transform = self.require_attr("invert_transform")
        transform = forward_kinematics(
            *np.deg2rad([40.0, -25.0, 70.0])
        ).t03

        actual = invert_transform(transform)

        np.testing.assert_allclose(
            actual, np.linalg.inv(transform), atol=1e-12
        )
        np.testing.assert_allclose(transform @ actual, np.eye(4), atol=1e-12)

    def test_zyx_rpy_reconstructs_regular_and_gimbal_lock_rotations(self):
        rotation_to_rpy = self.require_attr("rotation_to_rpy")
        source_angles = [(0.3, -0.4, 0.8), (0.7, np.pi / 2.0, -0.2)]

        for source in source_angles:
            with self.subTest(source=source):
                rotation = literal_zyx_rotation(*source)
                roll, pitch, yaw = rotation_to_rpy(rotation)
                reconstructed = literal_zyx_rotation(roll, pitch, yaw)
                np.testing.assert_allclose(reconstructed, rotation, atol=1e-10)

    def test_invalid_transform_and_rotation_inputs_are_rejected(self):
        invert_transform = self.require_attr("invert_transform")
        rotation_to_rpy = self.require_attr("rotation_to_rpy")

        with self.assertRaises(ValueError):
            invert_transform(np.eye(3))
        invalid_transform = np.eye(4)
        invalid_transform[0, 0] = 2.0
        with self.assertRaises(ValueError):
            invert_transform(invalid_transform)
        with self.assertRaises(ValueError):
            rotation_to_rpy(np.full((3, 3), np.nan))


class TestInverseKinematics(SimulatorTestCase):
    """Verify closed-form branches, reachability, and workspace samples."""

    def test_both_ik_branches_reproduce_one_target(self):
        inverse_kinematics = self.require_attr("inverse_kinematics")
        forward_kinematics = self.require_attr("forward_kinematics")
        target = np.array([2.5, 0.0, 0.8])

        solutions = inverse_kinematics(target)

        self.assertGreater(solutions.elbow_down[2], 0.0)
        self.assertLess(solutions.elbow_up[2], 0.0)
        for angles in (solutions.elbow_down, solutions.elbow_up):
            np.testing.assert_allclose(
                forward_kinematics(*angles).origins[3], target, atol=1e-10
            )

    def test_ik_reports_unreachable_boundary_and_axis_singularity(self):
        inverse_kinematics = self.require_attr("inverse_kinematics")
        unreachable_error = self.require_attr("UnreachableTargetError")

        with self.assertRaises(unreachable_error):
            inverse_kinematics(np.array([4.0, 0.0, 0.8]))

        boundary = inverse_kinematics(np.array([3.5, 0.0, 0.8]))
        np.testing.assert_allclose(
            boundary.elbow_down, boundary.elbow_up, atol=1e-12
        )

        singular = inverse_kinematics(np.array([0.0, 0.0, 3.8]))
        self.assertTrue(singular.axis_singular)
        self.assertEqual(singular.elbow_down[0], 0.0)

    def test_workspace_sampler_is_finite_and_has_requested_size(self):
        sample_workspace = self.require_attr("sample_workspace")

        points = sample_workspace(counts=(12, 6, 8))

        self.assertEqual(points.shape, (12 * 6 * 8, 3))
        self.assertTrue(np.isfinite(points).all())
        radial_distance = np.linalg.norm(
            points - np.array([0.0, 0.0, 0.8]), axis=1
        )
        self.assertLessEqual(radial_distance.max(), 3.5 + 1e-12)


if __name__ == "__main__":
    unittest.main()
