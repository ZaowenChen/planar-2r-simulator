"""Interactive planar 2R robot forward-kinematics simulator."""

from dataclasses import dataclass

import numpy as np


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
