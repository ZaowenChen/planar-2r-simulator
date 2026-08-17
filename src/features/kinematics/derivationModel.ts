import { geometricJacobian } from '../../robotics/jacobian'
import {
  forwardKinematics,
  inverseKinematics,
} from '../../robotics/kinematics'
import { cross3, subtract3 } from '../../robotics/linalg'
import { dhTransform, multiply4 } from '../../robotics/transforms'
import type {
  Matrix4,
  Matrix6x3,
  RobotParameters,
  Vector3,
} from '../../robotics/types'

export interface JacobianColumnDerivation {
  axis: Vector3
  origin: Vector3
  offset: Vector3
  linear: Vector3
  angular: Vector3
}

export interface KinematicsDerivation {
  degrees: Vector3
  radians: Vector3
  t01: Matrix4
  t12: Matrix4
  t23: Matrix4
  t02: Matrix4
  t03: Matrix4
  position: Vector3
  radialReach: number
  verticalReach: number
  jacobian: Matrix6x3
  jacobianColumns: readonly [
    JacobianColumnDerivation,
    JacobianColumnDerivation,
    JacobianColumnDerivation,
  ]
  inverse: {
    radial: number
    vertical: number
    cosineElbow: number
    result: ReturnType<typeof inverseKinematics>
  }
}

export function buildKinematicsDerivation(
  q: Vector3,
  parameters: RobotParameters,
  target: Vector3,
): KinematicsDerivation {
  const { d1, l2, l3 } = parameters.geometry
  const t01 = dhTransform(q[0], 0, Math.PI / 2, d1)
  const t12 = dhTransform(q[1], l2, 0, 0)
  const t23 = dhTransform(q[2], l3, 0, 0)
  const t02 = multiply4(t01, t12)
  const t03 = multiply4(t02, t23)
  const forward = forwardKinematics(q, parameters)
  const jacobian = geometricJacobian(q, parameters)

  const columns = forward.jointAxes.map((axis, index) => {
    const origin = forward.origins[index]
    const offset = subtract3(forward.endEffectorPosition, origin)
    return {
      axis,
      origin,
      offset,
      linear: cross3(axis, offset),
      angular: axis,
    }
  }) as unknown as KinematicsDerivation['jacobianColumns']

  const radial = Math.hypot(target[0], target[1])
  const vertical = target[2] - d1
  const cosineElbow = (
    radial ** 2 + vertical ** 2 - l2 ** 2 - l3 ** 2
  ) / (2 * l2 * l3)

  return {
    degrees: q.map((angle) => angle * 180 / Math.PI) as unknown as Vector3,
    radians: q,
    t01,
    t12,
    t23,
    t02,
    t03,
    position: forward.endEffectorPosition,
    radialReach: l2 * Math.cos(q[1]) + l3 * Math.cos(q[1] + q[2]),
    verticalReach: l2 * Math.sin(q[1]) + l3 * Math.sin(q[1] + q[2]),
    jacobian,
    jacobianColumns: columns,
    inverse: {
      radial,
      vertical,
      cosineElbow,
      result: inverseKinematics(target, parameters, q),
    },
  }
}
