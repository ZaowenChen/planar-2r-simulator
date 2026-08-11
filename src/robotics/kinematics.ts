import {
  dhTransform,
  multiply4,
  transformPoint,
  translationOf,
} from './transforms'
import type { Matrix4, RobotParameters, Vector3 } from './types'

const IDENTITY_4: Matrix4 = [
  [1, 0, 0, 0],
  [0, 1, 0, 0],
  [0, 0, 1, 0],
  [0, 0, 0, 1],
]

export interface ForwardKinematicsResult {
  transforms: readonly [Matrix4, Matrix4, Matrix4, Matrix4]
  origins: readonly [Vector3, Vector3, Vector3, Vector3]
  jointAxes: readonly [Vector3, Vector3, Vector3]
  centerOfMassPositions: readonly [Vector3, Vector3, Vector3]
  endEffectorPosition: Vector3
}

function zAxisOf(transform: Matrix4): Vector3 {
  return [transform[0][2], transform[1][2], transform[2][2]]
}

export function forwardKinematics(
  q: Vector3,
  parameters: RobotParameters,
): ForwardKinematicsResult {
  const { d1, l2, l3 } = parameters.geometry
  const t01 = dhTransform(q[0], 0, Math.PI / 2, d1)
  const t12 = dhTransform(q[1], l2, 0, 0)
  const t23 = dhTransform(q[2], l3, 0, 0)
  const t02 = multiply4(t01, t12)
  const t03 = multiply4(t02, t23)
  const t0e = multiply4(t03, IDENTITY_4)

  return {
    transforms: [t01, t02, t03, t0e],
    origins: [[0, 0, 0], translationOf(t01), translationOf(t02), translationOf(t03)],
    jointAxes: [[0, 0, 1], zAxisOf(t01), zAxisOf(t02)],
    centerOfMassPositions: [
      transformPoint(t01, parameters.links[0].centerOfMass),
      transformPoint(t02, parameters.links[1].centerOfMass),
      transformPoint(t03, parameters.links[2].centerOfMass),
    ],
    endEffectorPosition: translationOf(t0e),
  }
}
