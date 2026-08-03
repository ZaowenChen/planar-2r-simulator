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
