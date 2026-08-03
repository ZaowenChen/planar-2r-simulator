"""Interactive planar 2R robot forward-kinematics simulator."""

from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle
from matplotlib.widgets import Slider


@dataclass(frozen=True)
class DHParameters:
    """One row of standard Denavit-Hartenberg parameters."""

    theta: float
    a: float
    alpha: float = 0.0
    d: float = 0.0


@dataclass(frozen=True)
class ForwardKinematicsResult:
    """Transforms and joint origins for one planar 2R pose."""

    t01: np.ndarray
    t12: np.ndarray
    t02: np.ndarray
    origins: np.ndarray
    dh_parameters: tuple


def dh_transform(theta, a, alpha, d):
    """Return the standard D-H homogeneous transform ``T_(i-1)^i``.

    The construction is ``RotZ(theta) @ TransZ(d) @ TransX(a) @
    RotX(alpha)``. Angles are expressed in radians.
    """

    cos_theta, sin_theta = np.cos(theta), np.sin(theta)
    cos_alpha, sin_alpha = np.cos(alpha), np.sin(alpha)

    # Standard D-H order: RotZ(theta) -> TransZ(d) -> TransX(a)
    # -> RotX(alpha). The final column is the translated frame origin.
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


def forward_kinematics(theta1, theta2, l1, l2):
    """Compute standard D-H transforms and origins for a planar 2R arm."""

    for name, length in (("l1", l1), ("l2", l2)):
        if not np.isfinite(length) or length <= 0.0:
            raise ValueError("{} must be a finite positive number".format(name))

    link1 = DHParameters(theta=float(theta1), a=float(l1))
    link2 = DHParameters(theta=float(theta2), a=float(l2))

    t01 = dh_transform(link1.theta, link1.a, link1.alpha, link1.d)
    t12 = dh_transform(link2.theta, link2.a, link2.alpha, link2.d)
    t02 = t01 @ t12

    origins = np.vstack(
        (
            np.zeros(2, dtype=float),
            t01[:2, 3],
            t02[:2, 3],
        )
    )

    return ForwardKinematicsResult(
        t01=t01,
        t12=t12,
        t02=t02,
        origins=origins,
        dh_parameters=(link1, link2),
    )


class Planar2RSimulator:
    """Matplotlib interface for exploring planar 2R forward kinematics."""

    def __init__(self, l1=2.0, l2=1.5, theta1_deg=30.0, theta2_deg=45.0):
        self.l1 = float(l1)
        self.l2 = float(l2)
        self.reach = self.l1 + self.l2
        self.frame_axis_length = 0.16 * self.reach
        self.label_offset = 0.04 * self.reach

        initial_result = forward_kinematics(
            np.deg2rad(theta1_deg),
            np.deg2rad(theta2_deg),
            self.l1,
            self.l2,
        )
        self.current_result = initial_result

        self.fig = plt.figure(figsize=(13.0, 7.5))
        self.fig.suptitle(
            "Planar 2R Robot — Standard D-H Forward Kinematics",
            fontsize=15,
            fontweight="bold",
        )
        grid = self.fig.add_gridspec(
            1,
            2,
            left=0.07,
            right=0.97,
            bottom=0.22,
            top=0.90,
            width_ratios=(3.2, 1.6),
            wspace=0.28,
        )
        self.ax = self.fig.add_subplot(grid[0, 0])
        self.info_ax = self.fig.add_subplot(grid[0, 1])

        self._configure_robot_axes()
        self.workspace_circles = self._create_workspace_circles()

        (self.arm_line,) = self.ax.plot(
            initial_result.origins[:, 0],
            initial_result.origins[:, 1],
            color="#246BCE",
            linewidth=4.0,
            marker="o",
            markersize=10.0,
            markerfacecolor="white",
            markeredgecolor="#123A73",
            markeredgewidth=2.0,
            label="2R robot",
            zorder=5,
        )

        self.frame_lines = []
        for index in range(3):
            x_label = "Local X axis" if index == 0 else "_nolegend_"
            y_label = "Local Y axis" if index == 0 else "_nolegend_"
            (x_line,) = self.ax.plot(
                [], [], color="red", linewidth=2.0, label=x_label, zorder=6
            )
            (y_line,) = self.ax.plot(
                [], [], color="green", linewidth=2.0, label=y_label, zorder=6
            )
            self.frame_lines.append((x_line, y_line))

        rotations = (
            np.eye(2),
            initial_result.t01[:2, :2],
            initial_result.t02[:2, :2],
        )
        for index, (origin, rotation) in enumerate(
            zip(initial_result.origins, rotations)
        ):
            self._set_frame_artist(index, origin, rotation)

        self.frame_labels = []
        for index, origin in enumerate(initial_result.origins):
            label = self.ax.text(
                origin[0] + self.label_offset,
                origin[1] + self.label_offset,
                "O{}".format(index),
                fontsize=10,
                fontweight="bold",
                zorder=7,
            )
            self.frame_labels.append(label)

        self.ax.legend(loc="upper left", fontsize=9)
        self._create_information_panel()
        self._create_sliders(theta1_deg, theta2_deg)
        self.theta1_slider.on_changed(self._update)
        self.theta2_slider.on_changed(self._update)
        self._update()

    def _configure_robot_axes(self):
        limit = 1.15 * self.reach
        self.ax.set_xlim(-limit, limit)
        self.ax.set_ylim(-limit, limit)
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_xlabel("x")
        self.ax.set_ylabel("y")
        self.ax.set_title("Robot configuration and reachable workspace")
        self.ax.grid(True, linestyle=":", linewidth=0.8, alpha=0.6)
        self.ax.axhline(0.0, color="0.75", linewidth=0.8, zorder=0)
        self.ax.axvline(0.0, color="0.75", linewidth=0.8, zorder=0)

    def _create_workspace_circles(self):
        outer = Circle(
            (0.0, 0.0),
            self.reach,
            fill=False,
            color="#666666",
            linestyle="--",
            linewidth=1.4,
            alpha=0.75,
            label="Workspace boundaries",
            zorder=1,
        )
        inner = Circle(
            (0.0, 0.0),
            abs(self.l1 - self.l2),
            fill=False,
            color="#888888",
            linestyle=":",
            linewidth=1.4,
            alpha=0.85,
            zorder=1,
        )
        self.ax.add_patch(outer)
        self.ax.add_patch(inner)
        return (outer, inner)

    def _create_information_panel(self):
        self.info_ax.set_axis_off()
        self.info_ax.text(
            0.0,
            0.98,
            "Forward-kinematics result",
            transform=self.info_ax.transAxes,
            va="top",
            fontsize=13,
            fontweight="bold",
        )
        self.position_text = self.info_ax.text(
            0.0,
            0.83,
            "End-effector position",
            transform=self.info_ax.transAxes,
            va="top",
            family="monospace",
            fontsize=11,
        )
        self.matrix_text = self.info_ax.text(
            0.0,
            0.60,
            "T_0^2",
            transform=self.info_ax.transAxes,
            va="top",
            family="monospace",
            fontsize=10,
            linespacing=1.35,
        )

    def _create_sliders(self, theta1_deg, theta2_deg):
        theta1_ax = self.fig.add_axes((0.14, 0.115, 0.48, 0.035))
        theta2_ax = self.fig.add_axes((0.14, 0.055, 0.48, 0.035))

        self.theta1_slider = Slider(
            theta1_ax,
            r"$\theta_1$ (deg)",
            -180.0,
            180.0,
            valinit=float(theta1_deg),
            valstep=1.0,
        )
        self.theta2_slider = Slider(
            theta2_ax,
            r"$\theta_2$ (deg)",
            -180.0,
            180.0,
            valinit=float(theta2_deg),
            valstep=1.0,
        )

    def _set_frame_artist(self, index, origin, rotation):
        x_endpoint = origin + self.frame_axis_length * rotation[:, 0]
        y_endpoint = origin + self.frame_axis_length * rotation[:, 1]
        x_line, y_line = self.frame_lines[index]
        x_line.set_data(
            (origin[0], x_endpoint[0]), (origin[1], x_endpoint[1])
        )
        y_line.set_data(
            (origin[0], y_endpoint[0]), (origin[1], y_endpoint[1])
        )

    @staticmethod
    def _format_matrix(matrix):
        cleaned = np.where(np.abs(matrix) < 5e-13, 0.0, matrix)
        return np.array2string(
            cleaned,
            precision=3,
            suppress_small=True,
            floatmode="fixed",
        )

    def _update(self, _value=None):
        """Recompute the pose and update existing artists in place."""

        theta1 = np.deg2rad(self.theta1_slider.val)
        theta2 = np.deg2rad(self.theta2_slider.val)
        result = forward_kinematics(theta1, theta2, self.l1, self.l2)
        self.current_result = result

        self.arm_line.set_data(result.origins[:, 0], result.origins[:, 1])

        rotations = (
            np.eye(2),
            result.t01[:2, :2],
            result.t02[:2, :2],
        )
        for index, (origin, rotation) in enumerate(
            zip(result.origins, rotations)
        ):
            self._set_frame_artist(index, origin, rotation)
            self.frame_labels[index].set_position(
                (
                    origin[0] + self.label_offset,
                    origin[1] + self.label_offset,
                )
            )

        endpoint = result.origins[2]
        self.position_text.set_text(
            "End-effector position\n\n"
            "x = {:.3f}\n"
            "y = {:.3f}".format(endpoint[0], endpoint[1])
        )
        self.matrix_text.set_text(
            "T_0^2 =\n\n{}".format(self._format_matrix(result.t02))
        )

        self.fig.canvas.draw_idle()

    def show(self):
        """Open the interactive Matplotlib window."""

        plt.show()


def main():
    """Launch the default planar 2R simulator."""

    simulator = Planar2RSimulator()
    simulator.show()


if __name__ == "__main__":
    main()
