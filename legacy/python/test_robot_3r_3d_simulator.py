"""Tests for the interactive 3D 3R robot simulator."""

import importlib
import importlib.util
import io
import unittest

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import proj3d


simulator_module = importlib.import_module("legacy.python.robot_3r_3d_simulator")


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

    def require_method(self, instance, name):
        self.assertTrue(
            hasattr(instance, name),
            "{} is missing {!r}".format(type(instance).__name__, name),
        )
        return getattr(instance, name)

    def tearDown(self):
        plt.close("all")


class TestModuleAvailability(unittest.TestCase):
    """Catch a missing runnable 3D simulator module."""

    def test_3d_simulator_module_can_be_imported(self):
        spec = importlib.util.find_spec("legacy.python.robot_3r_3d_simulator")
        self.assertIsNotNone(spec, "legacy.python.robot_3r_3d_simulator does not exist")


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
        source_angles = [
            (0.3, -0.4, 0.8),
            (0.7, np.pi / 2.0, -0.2),
            (-0.4, -np.pi / 2.0, 0.6),
        ]

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

    def test_reference_angles_preserve_folded_radial_family(self):
        inverse_kinematics = self.require_attr("inverse_kinematics")
        forward_kinematics = self.require_attr("forward_kinematics")
        reference = np.deg2rad([-180.0, -80.0, -150.0])
        target = forward_kinematics(*reference).origins[3]

        solutions = inverse_kinematics(
            target, reference_angles=reference
        )
        wrapped_difference = np.arctan2(
            np.sin(solutions.elbow_up - reference),
            np.cos(solutions.elbow_up - reference),
        )

        np.testing.assert_allclose(wrapped_difference, 0.0, atol=1e-10)
        np.testing.assert_allclose(
            forward_kinematics(*solutions.elbow_up).origins[3],
            target,
            atol=1e-10,
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


class TestRobot3DGUI(SimulatorTestCase):
    """Verify the static educational scene, panels, and widget contracts."""

    def test_gui_has_3d_scene_frames_workspace_and_panels(self):
        simulator_cls = self.require_attr("Robot3DSimulator")
        simulator = simulator_cls()

        self.assertEqual(simulator.ax3d.name, "3d")
        self.assertEqual(len(simulator.arm_line.get_xdata()), 4)
        self.assertEqual(len(simulator.frame_artists), 12)
        self.assertEqual(len(simulator.frame_labels), 4)
        self.assertEqual(len(simulator.transform_labels), 3)
        workspace_x, workspace_y, workspace_z = (
            simulator.workspace_scatter._offsets3d
        )
        self.assertGreater(len(workspace_x), 7_000)
        self.assertEqual(len(workspace_x), len(workspace_y))
        self.assertEqual(len(workspace_y), len(workspace_z))
        self.assertIn("T_0^3", simulator.pose_text.get_text())
        self.assertIn("T^{-1}", simulator.formula_text.get_text())

    def test_gui_controls_have_required_defaults_and_ranges(self):
        simulator_cls = self.require_attr("Robot3DSimulator")
        simulator = simulator_cls()
        expected_joint_ranges = [
            (-180.0, 180.0),
            (-90.0, 90.0),
            (-150.0, 150.0),
        ]
        expected_target_ranges = [
            (-3.5, 3.5),
            (-3.5, 3.5),
            (-2.7, 4.3),
        ]

        for slider, expected in zip(
            simulator.joint_sliders, expected_joint_ranges
        ):
            self.assertEqual((slider.valmin, slider.valmax), expected)
        for slider, expected in zip(
            simulator.target_sliders, expected_target_ranges
        ):
            self.assertEqual((slider.valmin, slider.valmax), expected)
            self.assertFalse(slider.active)

        self.assertEqual(simulator.mode, "FK")
        self.assertEqual(simulator.mode_button.label.get_text(), "Mode: FK")
        self.assertEqual(
            [label.get_text() for label in simulator.solution_radio.labels],
            ["Solution 1: Elbow Down", "Solution 2: Elbow Up"],
        )
        self.assertIn(
            "30.0 deg", simulator.dh_table[(1, 4)].get_text().get_text()
        )

    def test_initial_camera_keeps_robot_legible_in_screen_projection(self):
        simulator_cls = self.require_attr("Robot3DSimulator")
        simulator = simulator_cls()
        simulator.fig.canvas.draw()
        origins = simulator.current_result.origins
        projected_x, projected_y, _ = proj3d.proj_transform(
            origins[:, 0],
            origins[:, 1],
            origins[:, 2],
            simulator.ax3d.get_proj(),
        )
        screen_points = simulator.ax3d.transData.transform(
            np.column_stack((projected_x, projected_y))
        )
        pairwise_distances = np.linalg.norm(
            screen_points[:, None, :] - screen_points[None, :, :], axis=2
        )
        span_ratio = pairwise_distances.max() / min(
            simulator.ax3d.bbox.width, simulator.ax3d.bbox.height
        )

        self.assertGreater(span_ratio, 0.20)


class TestRobot3DInteraction(SimulatorTestCase):
    """Exercise real sliders and branch selection across FK/IK modes."""

    def test_fk_sliders_update_endpoint_and_dynamic_dh_table(self):
        simulator_cls = self.require_attr("Robot3DSimulator")
        simulator = simulator_cls()

        for slider, value in zip(
            simulator.joint_sliders, (0.0, 0.0, 0.0)
        ):
            slider.set_val(value)

        np.testing.assert_allclose(
            simulator.current_result.origins[3],
            [3.5, 0.0, 0.8],
            atol=1e-12,
        )
        self.assertIn(
            "0.0 deg", simulator.dh_table[(1, 4)].get_text().get_text()
        )

    def test_fk_mode_reverts_inactive_radio_selection(self):
        simulator_cls = self.require_attr("Robot3DSimulator")
        simulator = simulator_cls()
        selected_before = simulator.solution_radio.value_selected
        internal_before = simulator.selected_solution_index

        simulator.solution_radio.set_active(1)

        self.assertEqual(simulator.solution_radio.value_selected, selected_before)
        self.assertEqual(simulator.selected_solution_index, internal_before)

    def test_switch_to_ik_preserves_pose_and_syncs_target(self):
        simulator_cls = self.require_attr("Robot3DSimulator")
        simulator = simulator_cls()
        toggle_mode = self.require_method(simulator, "_toggle_mode")
        endpoint_before = simulator.current_result.origins[3].copy()

        toggle_mode(None)

        self.assertEqual(simulator.mode, "IK")
        np.testing.assert_allclose(
            [slider.val for slider in simulator.target_sliders],
            endpoint_before,
            atol=0.051,
        )
        np.testing.assert_allclose(
            simulator.current_result.origins[3], endpoint_before, atol=1e-10
        )
        self.assertTrue(all(slider.active for slider in simulator.target_sliders))
        self.assertTrue(
            all(not slider.active for slider in simulator.joint_sliders)
        )

    def test_folded_fk_pose_selects_valid_continuous_ik_family(self):
        simulator_cls = self.require_attr("Robot3DSimulator")
        simulator = simulator_cls()
        toggle_mode = self.require_method(simulator, "_toggle_mode")
        for slider, value in zip(
            simulator.joint_sliders, (-180.0, -80.0, -150.0)
        ):
            slider.set_val(value)
        angles_before = simulator.current_angles.copy()
        origins_before = simulator.current_result.origins.copy()

        toggle_mode(None)

        self.assertEqual(simulator.mode, "IK")
        self.assertEqual(simulator.selected_solution_index, 1)
        self.assertEqual(
            simulator.solution_radio.value_selected,
            "Solution 2: Elbow Up",
        )
        np.testing.assert_allclose(
            simulator.current_angles, angles_before, atol=1e-10
        )
        np.testing.assert_allclose(
            simulator.current_result.origins, origins_before, atol=1e-10
        )
        self.assertTrue(simulator._solution_within_limits(simulator.current_angles))

        simulator.target_sliders[0].set_val(
            simulator.target_sliders[0].val + 0.05
        )
        self.assertNotIn(
            "outside configured joint limits",
            simulator.status_text.get_text().lower(),
        )
        self.assertTrue(simulator._solution_within_limits(simulator.current_angles))

    def test_ik_target_and_radio_switch_between_two_valid_branches(self):
        simulator_cls = self.require_attr("Robot3DSimulator")
        simulator = simulator_cls()
        toggle_mode = self.require_method(simulator, "_toggle_mode")
        toggle_mode(None)

        for slider, value in zip(
            simulator.target_sliders, (2.5, 0.0, 0.8)
        ):
            slider.set_val(value)
        simulator.solution_radio.set_active(0)
        endpoint_down = simulator.current_result.origins[3].copy()
        elbow_down = simulator.current_result.origins[2].copy()

        simulator.solution_radio.set_active(1)
        endpoint_up = simulator.current_result.origins[3].copy()
        elbow_up = simulator.current_result.origins[2].copy()

        np.testing.assert_allclose(endpoint_down, [2.5, 0.0, 0.8], atol=1e-9)
        np.testing.assert_allclose(endpoint_up, [2.5, 0.0, 0.8], atol=1e-9)
        self.assertFalse(np.allclose(elbow_down, elbow_up))

    def test_invalid_ik_target_preserves_last_valid_pose(self):
        simulator_cls = self.require_attr("Robot3DSimulator")
        simulator = simulator_cls()
        toggle_mode = self.require_method(simulator, "_toggle_mode")
        toggle_mode(None)
        last_valid = simulator.current_result.origins.copy()

        simulator.target_sliders[0].set_val(3.5)
        simulator.target_sliders[1].set_val(3.5)

        self.assertIn("unreachable", simulator.status_text.get_text().lower())
        np.testing.assert_allclose(
            simulator.current_result.origins, last_valid, atol=1e-12
        )

        simulator.target_sliders[0].set_val(0.5)
        simulator.target_sliders[1].set_val(0.0)
        simulator.target_sliders[2].set_val(0.8)
        self.assertIn(
            "outside configured joint limits",
            simulator.status_text.get_text().lower(),
        )

    def test_headless_3d_figure_can_render(self):
        simulator_cls = self.require_attr("Robot3DSimulator")
        simulator = simulator_cls()
        output = io.BytesIO()

        simulator.fig.savefig(output, format="png", dpi=100)

        self.assertGreater(len(output.getvalue()), 50_000)


if __name__ == "__main__":
    unittest.main()
