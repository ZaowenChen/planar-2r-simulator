"""Interactive 3D 3R robot kinematics and pose-visualization simulator."""

from dataclasses import dataclass

import numpy as np


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


def inverse_kinematics(
    target, d1=0.8, l2=2.0, l3=1.5, tolerance=1e-9
):
    """Return elbow-down and elbow-up closed-form position IK solutions."""

    _validate_geometry(d1, l2, l3)
    target_array = np.asarray(target, dtype=float)
    if target_array.shape != (3,):
        raise ValueError("IK target must have shape (3,)")
    if not np.isfinite(target_array).all():
        raise ValueError("IK target must contain finite numbers")
    if not np.isfinite(tolerance) or tolerance < 0.0:
        raise ValueError("IK tolerance must be a finite nonnegative number")

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

    def shoulder_angle(theta3):
        return np.arctan2(vertical, radial) - np.arctan2(
            l3 * np.sin(theta3), l2 + l3 * np.cos(theta3)
        )

    elbow_down = np.array(
        (theta1, shoulder_angle(theta3_down), theta3_down), dtype=float
    )
    elbow_up = np.array(
        (theta1, shoulder_angle(theta3_up), theta3_up), dtype=float
    )
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
