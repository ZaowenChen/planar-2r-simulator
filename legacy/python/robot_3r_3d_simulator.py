"""Interactive 3D 3R robot kinematics and pose-visualization simulator."""

from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.widgets import Button, RadioButtons, Slider


JOINT_LIMITS_DEG = ((-180.0, 180.0), (-90.0, 90.0), (-150.0, 150.0))
TARGET_LIMITS = ((-3.5, 3.5), (-3.5, 3.5), (-2.7, 4.3))


@dataclass(frozen=True)
class DHParameters:
    """One row of standard Denavit-Hartenberg parameters."""

    theta: float
    a: float
    alpha: float
    d: float


@dataclass(frozen=True)
class ForwardKinematicsResult3D:
    """Transforms, origins, and D-H rows for one 3R pose."""

    t01: np.ndarray
    t12: np.ndarray
    t23: np.ndarray
    t02: np.ndarray
    t03: np.ndarray
    origins: np.ndarray
    dh_parameters: tuple


@dataclass(frozen=True)
class IKSolutions:
    """The two geometric position-IK branches for one target."""

    elbow_down: np.ndarray
    elbow_up: np.ndarray
    axis_singular: bool


class UnreachableTargetError(ValueError):
    """Raised when a Cartesian target is outside the geometric workspace."""


def dh_transform(theta, a, alpha, d):
    """Return the standard D-H homogeneous transform ``T_(i-1)^i``."""

    values = np.asarray((theta, a, alpha, d), dtype=float)
    if not np.isfinite(values).all():
        raise ValueError("D-H parameters must be finite numbers")

    cos_theta, sin_theta = np.cos(theta), np.sin(theta)
    cos_alpha, sin_alpha = np.cos(alpha), np.sin(alpha)

    # Standard D-H order: RotZ(theta), TransZ(d), TransX(a), RotX(alpha).
    return np.array(
        [
            [
                cos_theta,
                -sin_theta * cos_alpha,
                sin_theta * sin_alpha,
                a * cos_theta,
            ],
            [
                sin_theta,
                cos_theta * cos_alpha,
                -cos_theta * sin_alpha,
                a * sin_theta,
            ],
            [0.0, sin_alpha, cos_alpha, d],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def _validate_geometry(d1, l2, l3):
    geometry = np.asarray((d1, l2, l3), dtype=float)
    if not np.isfinite(geometry).all():
        raise ValueError("Robot geometry must contain finite numbers")
    if d1 < 0.0:
        raise ValueError("d1 must be finite and nonnegative")
    if l2 <= 0.0 or l3 <= 0.0:
        raise ValueError("l2 and l3 must be finite positive numbers")


def forward_kinematics(theta1, theta2, theta3, d1=0.8, l2=2.0, l3=1.5):
    """Compute the standard D-H chain for a yaw-pitch-pitch 3R arm."""

    angles = np.asarray((theta1, theta2, theta3), dtype=float)
    if not np.isfinite(angles).all():
        raise ValueError("Joint angles must be finite numbers")
    _validate_geometry(d1, l2, l3)

    rows = (
        DHParameters(float(theta1), 0.0, np.pi / 2.0, float(d1)),
        DHParameters(float(theta2), float(l2), 0.0, 0.0),
        DHParameters(float(theta3), float(l3), 0.0, 0.0),
    )
    transforms = tuple(
        dh_transform(row.theta, row.a, row.alpha, row.d) for row in rows
    )
    t01, t12, t23 = transforms
    t02 = t01 @ t12
    t03 = t02 @ t23
    origins = np.vstack(
        (
            np.zeros(3, dtype=float),
            t01[:3, 3],
            t02[:3, 3],
            t03[:3, 3],
        )
    )

    return ForwardKinematicsResult3D(
        t01=t01,
        t12=t12,
        t23=t23,
        t02=t02,
        t03=t03,
        origins=origins,
        dh_parameters=rows,
    )


def _validated_rotation(rotation):
    array = np.asarray(rotation, dtype=float)
    if array.shape != (3, 3):
        raise ValueError("Rotation matrix must have shape (3, 3)")
    if not np.isfinite(array).all():
        raise ValueError("Rotation matrix must contain finite numbers")
    if not np.allclose(array.T @ array, np.eye(3), atol=1e-9):
        raise ValueError("Rotation matrix must be orthogonal")
    if not np.isclose(np.linalg.det(array), 1.0, atol=1e-9):
        raise ValueError("Rotation matrix determinant must equal +1")
    return array


def invert_transform(transform):
    """Invert a rigid transform using ``[R.T, -R.T @ p]``."""

    array = np.asarray(transform, dtype=float)
    if array.shape != (4, 4):
        raise ValueError("Homogeneous transform must have shape (4, 4)")
    if not np.isfinite(array).all():
        raise ValueError("Homogeneous transform must contain finite numbers")
    if not np.allclose(array[3], (0.0, 0.0, 0.0, 1.0), atol=1e-9):
        raise ValueError("Homogeneous transform must end with [0, 0, 0, 1]")

    rotation = _validated_rotation(array[:3, :3])
    translation = array[:3, 3]
    inverse = np.eye(4, dtype=float)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ translation
    return inverse


def rotation_to_rpy(rotation):
    """Return ZYX Roll, Pitch, Yaw angles in radians."""

    matrix = _validated_rotation(rotation)
    horizontal = np.hypot(matrix[0, 0], matrix[1, 0])
    pitch = np.arctan2(-matrix[2, 0], horizontal)

    if horizontal > 1e-10:
        roll = np.arctan2(matrix[2, 1], matrix[2, 2])
        yaw = np.arctan2(matrix[1, 0], matrix[0, 0])
    elif pitch > 0.0:
        roll = np.arctan2(matrix[0, 1], matrix[0, 2])
        yaw = 0.0
    else:
        roll = np.arctan2(-matrix[0, 1], -matrix[0, 2])
        yaw = 0.0

    return np.array((roll, pitch, yaw), dtype=float)


def _wrapped_angle_distance(first, second):
    """Return Euclidean joint-space distance with angles wrapped at 2*pi."""

    difference = np.asarray(first, dtype=float) - np.asarray(second, dtype=float)
    wrapped = np.arctan2(np.sin(difference), np.cos(difference))
    return float(np.linalg.norm(wrapped))


def inverse_kinematics(
    target,
    d1=0.8,
    l2=2.0,
    l3=1.5,
    tolerance=1e-9,
    reference_angles=None,
):
    """Return elbow-down and elbow-up closed-form position IK solutions.

    When ``reference_angles`` is supplied, each elbow branch chooses between
    the equivalent positive- and negative-radial representations.  This keeps
    folded poses continuous while preserving the conventional two-branch API.
    """

    _validate_geometry(d1, l2, l3)
    target_array = np.asarray(target, dtype=float)
    if target_array.shape != (3,):
        raise ValueError("IK target must have shape (3,)")
    if not np.isfinite(target_array).all():
        raise ValueError("IK target must contain finite numbers")
    if not np.isfinite(tolerance) or tolerance < 0.0:
        raise ValueError("IK tolerance must be a finite nonnegative number")
    if reference_angles is None:
        reference = None
    else:
        reference = np.asarray(reference_angles, dtype=float)
        if reference.shape != (3,):
            raise ValueError("IK reference angles must have shape (3,)")
        if not np.isfinite(reference).all():
            raise ValueError("IK reference angles must contain finite numbers")

    x_target, y_target, z_target = target_array
    radial = np.hypot(x_target, y_target)
    vertical = z_target - d1
    cosine_elbow = (
        radial * radial + vertical * vertical - l2 * l2 - l3 * l3
    ) / (2.0 * l2 * l3)

    if cosine_elbow < -1.0 - tolerance or cosine_elbow > 1.0 + tolerance:
        raise UnreachableTargetError(
            "Target is unreachable: cosine-law value is {:.6f}".format(
                cosine_elbow
            )
        )
    cosine_elbow = float(np.clip(cosine_elbow, -1.0, 1.0))

    axis_singular = radial < 1e-10
    theta1 = 0.0 if axis_singular else np.arctan2(y_target, x_target)
    theta3_down = np.arccos(cosine_elbow)
    theta3_up = -theta3_down

    def make_candidate(base_angle, signed_radial, theta3):
        theta2 = np.arctan2(vertical, signed_radial) - np.arctan2(
            l3 * np.sin(theta3), l2 + l3 * np.cos(theta3)
        )
        theta2 = np.arctan2(np.sin(theta2), np.cos(theta2))
        return np.array((base_angle, theta2, theta3), dtype=float)

    def select_radial_family(theta3):
        conventional = make_candidate(theta1, radial, theta3)
        if reference is None or axis_singular:
            return conventional

        opposite_theta1 = np.arctan2(
            np.sin(theta1 + np.pi), np.cos(theta1 + np.pi)
        )
        folded = make_candidate(opposite_theta1, -radial, theta3)
        return min(
            (conventional, folded),
            key=lambda candidate: _wrapped_angle_distance(candidate, reference),
        )

    elbow_down = select_radial_family(theta3_down)
    elbow_up = select_radial_family(theta3_up)
    return IKSolutions(elbow_down, elbow_up, axis_singular)


def sample_workspace(d1=0.8, l2=2.0, l3=1.5, counts=(30, 16, 18)):
    """Return structured workspace samples over the configured joint ranges."""

    _validate_geometry(d1, l2, l3)
    if len(counts) != 3 or any(
        not isinstance(count, (int, np.integer)) or count < 2
        for count in counts
    ):
        raise ValueError("Workspace counts must be three integers >= 2")

    theta1_values = np.linspace(-np.pi, np.pi, counts[0])
    theta2_values = np.linspace(-np.pi / 2.0, np.pi / 2.0, counts[1])
    theta3_values = np.linspace(-5.0 * np.pi / 6.0, 5.0 * np.pi / 6.0, counts[2])
    theta1, theta2, theta3 = np.meshgrid(
        theta1_values, theta2_values, theta3_values, indexing="ij"
    )
    radial = l2 * np.cos(theta2) + l3 * np.cos(theta2 + theta3)
    x_points = radial * np.cos(theta1)
    y_points = radial * np.sin(theta1)
    z_points = d1 + l2 * np.sin(theta2) + l3 * np.sin(theta2 + theta3)
    return np.column_stack(
        (x_points.ravel(), y_points.ravel(), z_points.ravel())
    )


class Robot3DSimulator:
    """Interactive Matplotlib view for the educational 3D 3R model."""

    def __init__(
        self,
        d1=0.8,
        l2=2.0,
        l3=1.5,
        theta1_deg=30.0,
        theta2_deg=25.0,
        theta3_deg=-50.0,
    ):
        _validate_geometry(d1, l2, l3)
        self.d1 = float(d1)
        self.l2 = float(l2)
        self.l3 = float(l3)
        self.reach = self.l2 + self.l3
        self.frame_axis_length = 0.14 * self.reach
        self.mode = "FK"
        self.selected_solution_index = 0
        self._updating_controls = False

        initial_degrees = np.asarray(
            (theta1_deg, theta2_deg, theta3_deg), dtype=float
        )
        if not np.isfinite(initial_degrees).all():
            raise ValueError("Initial joint angles must be finite")
        for value, limits in zip(initial_degrees, JOINT_LIMITS_DEG):
            if value < limits[0] or value > limits[1]:
                raise ValueError("Initial joint angle is outside slider limits")
        self.current_angles = np.deg2rad(initial_degrees)
        self.current_result = forward_kinematics(
            *self.current_angles, self.d1, self.l2, self.l3
        )

        self.fig = plt.figure(figsize=(16.0, 10.0))
        self.fig.suptitle(
            "Interactive 3D 3R Robot — FK, IK, Pose and D-H Visualization",
            fontsize=15,
            fontweight="bold",
        )
        grid = self.fig.add_gridspec(
            2,
            2,
            left=0.04,
            right=0.98,
            bottom=0.31,
            top=0.94,
            width_ratios=(1.65, 1.0),
            height_ratios=(1.12, 1.0),
            wspace=0.18,
            hspace=0.18,
        )
        self.ax3d = self.fig.add_subplot(grid[:, 0], projection="3d")
        self.pose_ax = self.fig.add_subplot(grid[0, 1])
        self.math_ax = self.fig.add_subplot(grid[1, 1])
        self.pose_ax.set_axis_off()
        self.math_ax.set_axis_off()

        self._configure_3d_axes()
        workspace = sample_workspace(self.d1, self.l2, self.l3)
        self.workspace_scatter = self.ax3d.scatter(
            workspace[:, 0],
            workspace[:, 1],
            workspace[:, 2],
            s=2.0,
            color="#78C7DF",
            alpha=0.055,
            depthshade=False,
            label="Reachable workspace",
            zorder=1,
        )

        origins = self.current_result.origins
        (self.arm_line,) = self.ax3d.plot(
            origins[:, 0],
            origins[:, 1],
            origins[:, 2],
            color="#205AA7",
            linewidth=4.0,
            marker="o",
            markersize=8.0,
            markerfacecolor="white",
            markeredgecolor="#123A73",
            markeredgewidth=1.8,
            label="3R robot",
            zorder=8,
        )
        (self.position_vector,) = self.ax3d.plot(
            (origins[0, 0], origins[3, 0]),
            (origins[0, 1], origins[3, 1]),
            (origins[0, 2], origins[3, 2]),
            color="#7A4EAB",
            linestyle="--",
            linewidth=1.4,
            label=r"Position vector $P_B^T$",
            zorder=4,
        )

        self.frame_artists = []
        self.frame_labels = []
        self.transform_labels = []
        self._create_pose_panel()
        self._create_formula_panel(initial_degrees)
        self._create_controls(initial_degrees, origins[3])
        self._style_control_mode()
        self._update_visuals(
            self.current_result,
            "FK mode — move the joint-angle sliders",
            "#1F4E79",
        )
        for slider in self.joint_sliders:
            slider.on_changed(self._on_joint_change)
        for slider in self.target_sliders:
            slider.on_changed(self._on_target_change)
        self.mode_button.on_clicked(self._toggle_mode)
        self.solution_radio.on_clicked(self._on_solution_change)

    def _configure_3d_axes(self):
        horizontal_limit = 1.18 * self.reach
        self.ax3d.set_xlim(-horizontal_limit, horizontal_limit)
        self.ax3d.set_ylim(-horizontal_limit, horizontal_limit)
        self.ax3d.set_zlim(
            self.d1 - horizontal_limit, self.d1 + horizontal_limit
        )
        self.ax3d.set_box_aspect((1.0, 1.0, 1.0))
        self.ax3d.set_xlabel("X")
        self.ax3d.set_ylabel("Y")
        self.ax3d.set_zlabel("Z")
        self.ax3d.set_title("3D robot frames and reachable workspace")
        self.ax3d.grid(True, linestyle=":", alpha=0.5)
        # Keep the default arm plane side-on instead of looking along theta1.
        self.ax3d.view_init(elev=24.0, azim=-60.0)

        legend_handles = [
            Line2D(
                [0],
                [0],
                color="#205AA7",
                linewidth=4.0,
                marker="o",
                markerfacecolor="white",
                label="3R robot",
            ),
            Line2D([0], [0], color="red", linewidth=2.0, label="Local X"),
            Line2D([0], [0], color="green", linewidth=2.0, label="Local Y"),
            Line2D([0], [0], color="blue", linewidth=2.0, label="Local Z"),
            Line2D(
                [0],
                [0],
                color="#78C7DF",
                marker=".",
                linestyle="None",
                label="Workspace",
            ),
        ]
        self.ax3d.legend(handles=legend_handles, loc="upper left", fontsize=8)

    def _create_pose_panel(self):
        self.pose_ax.set_title(
            "Chapter 2 — Spatial Pose and Transform",
            fontsize=12,
            fontweight="bold",
            loc="left",
        )
        self.pose_text = self.pose_ax.text(
            0.0,
            0.96,
            "",
            transform=self.pose_ax.transAxes,
            va="top",
            family="monospace",
            fontsize=8.3,
            linespacing=1.18,
        )

    def _create_formula_panel(self, initial_degrees):
        self.math_ax.set_title(
            "Chapters 3–4 — D-H, FK and Closed-form IK",
            fontsize=12,
            fontweight="bold",
            loc="left",
        )
        column_labels = (
            "i",
            "a_(i-1)",
            "alpha_(i-1)",
            "d_i",
            "theta_i",
        )
        cell_text = (
            ("1", "0.00", "+90.0 deg", "{:.2f}".format(self.d1), ""),
            ("2", "{:.2f}".format(self.l2), "0.0 deg", "0.00", ""),
            ("3", "{:.2f}".format(self.l3), "0.0 deg", "0.00", ""),
        )
        self.dh_table = self.math_ax.table(
            cellText=cell_text,
            colLabels=column_labels,
            cellLoc="center",
            colLoc="center",
            bbox=(0.0, 0.58, 1.0, 0.33),
            colWidths=(0.07, 0.20, 0.25, 0.16, 0.23),
        )
        self.dh_table.auto_set_font_size(False)
        self.dh_table.set_fontsize(8.2)
        for column in range(len(column_labels)):
            self.dh_table[(0, column)].set_facecolor("#DCE6F1")
            self.dh_table[(0, column)].get_text().set_fontweight("bold")
        for row, angle in enumerate(initial_degrees, start=1):
            self.dh_table[(row, 4)].get_text().set_text(
                "{:.1f} deg".format(angle)
            )

        formula = (
            "D-H: T_(i-1)^i = Rz(theta_i) Tz(d_i) "
            "Tx(a_(i-1)) Rx(alpha_(i-1))\n"
            "FK:  T_0^3 = T_0^1 T_1^2 T_2^3\n"
            "Inverse: T^{-1} = [R^T, -R^T P; 0, 1]\n"
            "IK:  D = (r^2 + z'^2 - L2^2 - L3^2)/(2 L2 L3)\n"
            "     theta3 = +/- acos(D)\n"
            "     theta2 = atan2(z',r) - atan2(L3 sin(theta3),\n"
            "              L2 + L3 cos(theta3))"
        )
        self.formula_text = self.math_ax.text(
            0.0,
            0.50,
            formula,
            transform=self.math_ax.transAxes,
            va="top",
            family="monospace",
            fontsize=8.1,
            linespacing=1.30,
        )

    def _create_controls(self, initial_degrees, endpoint):
        joint_x, target_x, slider_width = 0.09, 0.45, 0.27
        slider_height = 0.028
        slider_y = (0.225, 0.175, 0.125)
        joint_labels = (r"$\theta_1$ (deg)", r"$\theta_2$ (deg)", r"$\theta_3$ (deg)")
        target_labels = ("X target", "Y target", "Z target")

        self.joint_sliders = []
        self.target_sliders = []
        for index, y_position in enumerate(slider_y):
            joint_axis = self.fig.add_axes(
                (joint_x, y_position, slider_width, slider_height)
            )
            joint_limits = JOINT_LIMITS_DEG[index]
            joint_slider = Slider(
                joint_axis,
                joint_labels[index],
                joint_limits[0],
                joint_limits[1],
                valinit=float(initial_degrees[index]),
                valstep=1.0,
            )
            self.joint_sliders.append(joint_slider)

            target_axis = self.fig.add_axes(
                (target_x, y_position, slider_width, slider_height)
            )
            target_limits = TARGET_LIMITS[index]
            target_slider = Slider(
                target_axis,
                target_labels[index],
                target_limits[0],
                target_limits[1],
                valinit=float(endpoint[index]),
                valstep=0.05,
            )
            self.target_sliders.append(target_slider)

        mode_axis = self.fig.add_axes((0.79, 0.205, 0.10, 0.05))
        self.mode_button = Button(
            mode_axis, "Mode: FK", color="#DCE6F1", hovercolor="#BDD7EE"
        )
        radio_axis = self.fig.add_axes((0.78, 0.075, 0.20, 0.105))
        self.solution_radio = RadioButtons(
            radio_axis,
            ("Solution 1: Elbow Down", "Solution 2: Elbow Up"),
            active=0,
        )
        self.solution_radio.ax.set_facecolor("#F3F3F3")
        for label in self.solution_radio.labels:
            label.set_fontsize(8.5)

        self.status_text = self.fig.text(
            0.04,
            0.035,
            "",
            fontsize=10,
            fontweight="bold",
            color="#1F4E79",
        )

    def _style_control_mode(self):
        fk_active = self.mode == "FK"
        for slider in self.joint_sliders:
            slider.set_active(fk_active)
            slider.ax.set_alpha(1.0 if fk_active else 0.35)
        for slider in self.target_sliders:
            slider.set_active(not fk_active)
            slider.ax.set_alpha(1.0 if not fk_active else 0.35)
        self.solution_radio.ax.set_alpha(1.0 if not fk_active else 0.35)
        self.mode_button.label.set_text("Mode: {}".format(self.mode))

    @staticmethod
    def _format_matrix(matrix):
        cleaned = np.where(np.abs(matrix) < 5e-13, 0.0, matrix)
        return np.array2string(
            cleaned,
            precision=3,
            suppress_small=True,
            floatmode="fixed",
        )

    def _draw_coordinate_frames(self, result):
        for artist in self.frame_artists:
            artist.remove()
        for label in self.frame_labels + self.transform_labels:
            label.remove()
        self.frame_artists = []
        self.frame_labels = []
        self.transform_labels = []

        transforms = (np.eye(4), result.t01, result.t02, result.t03)
        frame_names = ("{B}", "{1}", "{2}", "{W}/{T}")
        axis_colors = ("red", "green", "blue")
        for origin, transform, frame_name in zip(
            result.origins, transforms, frame_names
        ):
            rotation = transform[:3, :3]
            for column, color in enumerate(axis_colors):
                direction = rotation[:, column]
                arrow = self.ax3d.quiver(
                    origin[0],
                    origin[1],
                    origin[2],
                    direction[0],
                    direction[1],
                    direction[2],
                    length=self.frame_axis_length,
                    normalize=True,
                    color=color,
                    linewidth=1.5,
                    arrow_length_ratio=0.22,
                    zorder=10,
                )
                self.frame_artists.append(arrow)
            label = self.ax3d.text(
                origin[0] + 0.07,
                origin[1] + 0.07,
                origin[2] + 0.07,
                frame_name,
                fontsize=9,
                fontweight="bold",
                zorder=12,
            )
            self.frame_labels.append(label)

        for index, transform_name in enumerate(("T01", "T12", "T23")):
            midpoint = 0.5 * (result.origins[index] + result.origins[index + 1])
            label = self.ax3d.text(
                midpoint[0],
                midpoint[1],
                midpoint[2] + 0.10,
                transform_name,
                fontsize=8,
                color="#704214",
                zorder=11,
            )
            self.transform_labels.append(label)

    def _update_visuals(self, result, status, status_color):
        self.current_result = result
        self.current_angles = np.array(
            [row.theta for row in result.dh_parameters], dtype=float
        )
        origins = result.origins
        self.arm_line.set_data_3d(
            origins[:, 0], origins[:, 1], origins[:, 2]
        )
        self.position_vector.set_data_3d(
            (origins[0, 0], origins[3, 0]),
            (origins[0, 1], origins[3, 1]),
            (origins[0, 2], origins[3, 2]),
        )
        self._draw_coordinate_frames(result)

        rotation = result.t03[:3, :3]
        rpy_degrees = np.rad2deg(rotation_to_rpy(rotation))
        inverse = invert_transform(result.t03)
        inverse_error = np.linalg.norm(result.t03 @ inverse - np.eye(4))
        position = result.origins[3]
        pose = (
            "End position P_B^T\n"
            "x = {0[0]: .3f}   y = {0[1]: .3f}   z = {0[2]: .3f}\n\n"
            "R_0^3 =\n{1}\n\n"
            "T_0^3 =\n{2}\n\n"
            "ZYX RPY (deg)\n"
            "Roll = {3[0]: .2f}  Pitch = {3[1]: .2f}  Yaw = {3[2]: .2f}\n\n"
            "T_3^0 (fast inverse) =\n{4}\n"
            "||T_0^3 T_3^0 - I||_F = {5:.2e}"
        ).format(
            position,
            self._format_matrix(rotation),
            self._format_matrix(result.t03),
            rpy_degrees,
            self._format_matrix(inverse),
            inverse_error,
        )
        self.pose_text.set_text(pose)

        for row_index, angle in enumerate(
            np.rad2deg(self.current_angles), start=1
        ):
            self.dh_table[(row_index, 4)].get_text().set_text(
                "{:.1f} deg".format(angle)
            )
        self.status_text.set_text(status)
        self.status_text.set_color(status_color)
        self.fig.canvas.draw_idle()

    def _on_joint_change(self, _value):
        if self._updating_controls or self.mode != "FK":
            return
        angles = np.deg2rad([slider.val for slider in self.joint_sliders])
        result = forward_kinematics(
            *angles, self.d1, self.l2, self.l3
        )
        self._update_visuals(
            result,
            "FK mode — joint sliders drive T_0^3",
            "#1F4E79",
        )

    def _on_target_change(self, _value):
        if self._updating_controls or self.mode != "IK":
            return
        self._apply_selected_ik_solution()

    def _on_solution_change(self, label):
        if self._updating_controls:
            return
        if self.mode != "IK":
            self._updating_controls = True
            try:
                self.solution_radio.set_active(self.selected_solution_index)
            finally:
                self._updating_controls = False
            return
        labels = [item.get_text() for item in self.solution_radio.labels]
        self.selected_solution_index = labels.index(label)
        self._apply_selected_ik_solution()

    @staticmethod
    def _wrapped_angle_distance(first, second):
        return _wrapped_angle_distance(first, second)

    @staticmethod
    def _solution_within_limits(angles):
        angle_degrees = np.rad2deg(angles)
        return all(
            lower - 1e-9 <= value <= upper + 1e-9
            for value, (lower, upper) in zip(angle_degrees, JOINT_LIMITS_DEG)
        )

    def _apply_selected_ik_solution(self):
        target = np.array(
            [slider.val for slider in self.target_sliders], dtype=float
        )
        try:
            solutions = inverse_kinematics(
                target,
                self.d1,
                self.l2,
                self.l3,
                reference_angles=self.current_angles,
            )
        except UnreachableTargetError as error:
            self.status_text.set_text(
                "IK target unreachable — pose preserved ({})".format(error)
            )
            self.status_text.set_color("#C00000")
            self.fig.canvas.draw_idle()
            return

        selected = (
            solutions.elbow_down
            if self.selected_solution_index == 0
            else solutions.elbow_up
        )
        if not self._solution_within_limits(selected):
            self.status_text.set_text(
                "IK target outside configured joint limits — pose preserved"
            )
            self.status_text.set_color("#C55A11")
            self.fig.canvas.draw_idle()
            return

        result = forward_kinematics(
            *selected, self.d1, self.l2, self.l3
        )
        branch_name = "Elbow Down" if self.selected_solution_index == 0 else "Elbow Up"
        singular_note = " — base-axis singularity: theta1 fixed at 0 deg" if solutions.axis_singular else ""
        self._update_visuals(
            result,
            "IK mode — {}{}".format(branch_name, singular_note),
            "#548235" if not solutions.axis_singular else "#C55A11",
        )

    def _toggle_mode(self, _event):
        if self.mode == "FK":
            endpoint = self.current_result.origins[3].copy()
            solutions = inverse_kinematics(
                endpoint,
                self.d1,
                self.l2,
                self.l3,
                reference_angles=self.current_angles,
            )
            candidates = (solutions.elbow_down, solutions.elbow_up)
            valid_indices = [
                index
                for index, candidate in enumerate(candidates)
                if self._solution_within_limits(candidate)
            ]
            if not valid_indices:
                self.status_text.set_text(
                    "Cannot enter IK mode — no equivalent solution is within "
                    "the configured joint limits"
                )
                self.status_text.set_color("#C55A11")
                self.fig.canvas.draw_idle()
                return
            self.selected_solution_index = min(
                valid_indices,
                key=lambda index: self._wrapped_angle_distance(
                    candidates[index], self.current_angles
                ),
            )

            self._updating_controls = True
            try:
                for slider, coordinate in zip(self.target_sliders, endpoint):
                    slider.set_val(float(coordinate))
                self.solution_radio.set_active(self.selected_solution_index)
            finally:
                self._updating_controls = False
            self.mode = "IK"
            self._style_control_mode()
            branch_name = (
                "Elbow Down"
                if self.selected_solution_index == 0
                else "Elbow Up"
            )
            self._update_visuals(
                self.current_result,
                "IK mode — current pose matched to {}".format(branch_name),
                "#548235",
            )
        else:
            self._updating_controls = True
            try:
                for slider, angle in zip(
                    self.joint_sliders, np.rad2deg(self.current_angles)
                ):
                    slider.set_val(float(angle))
            finally:
                self._updating_controls = False
            self.mode = "FK"
            self._style_control_mode()
            self._update_visuals(
                self.current_result,
                "FK mode — joint sliders synchronized to the current pose",
                "#1F4E79",
            )

    def show(self):
        """Open the interactive Matplotlib window."""

        plt.show()


def main():
    """Launch the default 3D 3R simulator."""

    simulator = Robot3DSimulator()
    simulator.show()


if __name__ == "__main__":
    main()
