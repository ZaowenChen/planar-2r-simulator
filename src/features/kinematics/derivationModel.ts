import { geometricJacobian } from '../../robotics/jacobian'
import {
  forwardKinematics,
  inverseKinematics,
  inverseKinematicsCandidates,
} from '../../robotics/kinematics'
import type { InverseKinematicsSolution } from '../../robotics/kinematics'
import { add3, cross3, norm3, scale3, subtract3 } from '../../robotics/linalg'
import { dhTransform, multiply4, rotationOf } from '../../robotics/transforms'
import type {
  Matrix3,
  Matrix4,
  Matrix6x3,
  RobotParameters,
  Vector3,
} from '../../robotics/types'
import {
  jacobianForMillimetresAndDegrees,
  metresToMillimetres,
  transformInMillimetres,
} from './presentation'

export interface JacobianColumnDerivation {
  axis: Vector3
  origin: Vector3
  offset: Vector3
  linear: Vector3
  angular: Vector3
  originMm: Vector3
  offsetMm: Vector3
  linearMmPerDegree: Vector3
}

export interface JointVelocityContribution {
  jointIndex: 0 | 1 | 2
  linearMetresPerSecond: Vector3
  linearMillimetresPerSecond: Vector3
  angularRadiansPerSecond: Vector3
  angularDegreesPerSecond: Vector3
}

export interface InverseSolutionDerivation {
  solution: InverseKinematicsSolution
  qDegrees: Vector3
  signedRadialMm: number
  targetDirectionDegrees: number
  triangleCorrectionDegrees: number
  triangleProjectionMm: number
  triangleHeightMm: number
  elbowPointMm: readonly [number, number]
  achievedPositionMm: Vector3
  positionResidualMm: Vector3
  positionErrorMm: number
  orientation: Matrix3
  toolElevationDegrees: number
}

export interface KinematicsDerivation {
  degrees: Vector3
  radians: Vector3
  qDegrees: Vector3
  qdRadiansPerSecond: Vector3
  qdDegreesPerSecond: Vector3
  t01: Matrix4
  t12: Matrix4
  t23: Matrix4
  t02: Matrix4
  t03: Matrix4
  position: Vector3
  positionMm: Vector3
  orientation: Matrix3
  azimuthDegrees: number
  toolElevationDegrees: number
  displayTransforms: {
    t01: Matrix4
    t12: Matrix4
    t23: Matrix4
    t02: Matrix4
    t03: Matrix4
  }
  radialReach: number
  verticalReach: number
  radialReachMm: number
  verticalReachMm: number
  jacobian: Matrix6x3
  displayJacobian: Matrix6x3
  linearVelocityMetresPerSecond: Vector3
  linearVelocityMillimetresPerSecond: Vector3
  angularVelocityRadiansPerSecond: Vector3
  angularVelocityDegreesPerSecond: Vector3
  velocityContributions: readonly [
    JointVelocityContribution,
    JointVelocityContribution,
    JointVelocityContribution,
  ]
  jacobianColumns: readonly [
    JacobianColumnDerivation,
    JacobianColumnDerivation,
    JacobianColumnDerivation,
  ]
  inverse: {
    radial: number
    vertical: number
    cosineElbow: number
    radialMm: number
    verticalMm: number
    shoulderToTargetMm: number
    pythagoreanDistanceMm: number
    targetMm: Vector3
    baseAngleDegrees: number
    reachability: {
      minimumMm: number
      maximumMm: number
      isReachable: boolean
    }
    conventionalBranches: readonly InverseSolutionDerivation[]
    candidateDetails: readonly InverseSolutionDerivation[]
    result: ReturnType<typeof inverseKinematics>
    solutionDetails: readonly InverseSolutionDerivation[]
  }
}

function vectorInMillimetres(vector: Vector3): Vector3 {
  return vector.map(metresToMillimetres) as unknown as Vector3
}

function vectorInDegrees(vector: Vector3): Vector3 {
  return vector.map((angle) => angle * 180 / Math.PI) as unknown as Vector3
}

export function buildKinematicsDerivation(
  q: Vector3,
  parameters: RobotParameters,
  target: Vector3,
  qd: Vector3 = [0, 0, 0],
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
    const linear = cross3(axis, offset)
    return {
      axis,
      origin,
      offset,
      linear,
      angular: axis,
      originMm: vectorInMillimetres(origin),
      offsetMm: vectorInMillimetres(offset),
      linearMmPerDegree: linear.map((value) => (
        value * 1000 * Math.PI / 180
      )) as unknown as Vector3,
    }
  }) as unknown as KinematicsDerivation['jacobianColumns']
  const velocityContributions = columns.map((column, index) => {
    const jointIndex = index as 0 | 1 | 2
    const linearMetresPerSecond = scale3(column.linear, qd[index])
    const angularRadiansPerSecond = scale3(column.angular, qd[index])
    return {
      jointIndex,
      linearMetresPerSecond,
      linearMillimetresPerSecond: vectorInMillimetres(linearMetresPerSecond),
      angularRadiansPerSecond,
      angularDegreesPerSecond: vectorInDegrees(angularRadiansPerSecond),
    }
  }) as unknown as KinematicsDerivation['velocityContributions']
  const linearVelocityMetresPerSecond = velocityContributions.reduce(
    (sum, contribution) => add3(sum, contribution.linearMetresPerSecond),
    [0, 0, 0] as Vector3,
  )
  const angularVelocityRadiansPerSecond = velocityContributions.reduce(
    (sum, contribution) => add3(sum, contribution.angularRadiansPerSecond),
    [0, 0, 0] as Vector3,
  )

  const radial = Math.hypot(target[0], target[1])
  const vertical = target[2] - d1
  const cosineElbow = (
    radial ** 2 + vertical ** 2 - l2 ** 2 - l3 ** 2
  ) / (2 * l2 * l3)
  const inverseResult = inverseKinematics(target, parameters, q)
  const analyticCandidates = inverseKinematicsCandidates(target, parameters)
  const qDegrees = vectorInDegrees(q)
  const buildSolutionDetail = (
    solution: InverseKinematicsSolution,
  ): InverseSolutionDerivation => {
    const signedRadial = solution.radialFamily === 'conventional' ? radial : -radial
    const targetDirection = Math.atan2(vertical, signedRadial)
    const triangleProjection = l2 + l3 * Math.cos(solution.q[2])
    const triangleHeight = l3 * Math.sin(solution.q[2])
    const triangleCorrection = Math.atan2(
      triangleHeight,
      triangleProjection,
    )
    const achieved = forwardKinematics(solution.q, parameters)
    const achievedPositionMm = vectorInMillimetres(achieved.endEffectorPosition)
    const targetMm = vectorInMillimetres(target)
    const positionResidualMm = subtract3(achievedPositionMm, targetMm)
    return {
      solution,
      qDegrees: vectorInDegrees(solution.q),
      signedRadialMm: metresToMillimetres(signedRadial),
      targetDirectionDegrees: targetDirection * 180 / Math.PI,
      triangleCorrectionDegrees: triangleCorrection * 180 / Math.PI,
      triangleProjectionMm: metresToMillimetres(triangleProjection),
      triangleHeightMm: metresToMillimetres(triangleHeight),
      elbowPointMm: [
        metresToMillimetres(l2 * Math.cos(solution.q[1])),
        metresToMillimetres(l2 * Math.sin(solution.q[1])),
      ],
      achievedPositionMm,
      positionResidualMm,
      positionErrorMm: norm3(positionResidualMm),
      orientation: rotationOf(achieved.transforms[2]),
      toolElevationDegrees: (solution.q[1] + solution.q[2]) * 180 / Math.PI,
    }
  }
  const solutionDetails = inverseResult.solutions.map(buildSolutionDetail)
  const candidateDetails = analyticCandidates.map(buildSolutionDetail)
  const conventionalBranches = analyticCandidates
    .filter((candidate) => candidate.radialFamily === 'conventional')
    .map(buildSolutionDetail)
  const minimumReachMm = metresToMillimetres(Math.abs(l2 - l3))
  const maximumReachMm = metresToMillimetres(l2 + l3)
  const shoulderToTargetMm = metresToMillimetres(Math.hypot(radial, vertical))

  return {
    degrees: qDegrees,
    radians: q,
    qDegrees,
    qdRadiansPerSecond: qd,
    qdDegreesPerSecond: vectorInDegrees(qd),
    t01,
    t12,
    t23,
    t02,
    t03,
    position: forward.endEffectorPosition,
    positionMm: vectorInMillimetres(forward.endEffectorPosition),
    orientation: rotationOf(t03),
    azimuthDegrees: qDegrees[0],
    toolElevationDegrees: qDegrees[1] + qDegrees[2],
    displayTransforms: {
      t01: transformInMillimetres(t01),
      t12: transformInMillimetres(t12),
      t23: transformInMillimetres(t23),
      t02: transformInMillimetres(t02),
      t03: transformInMillimetres(t03),
    },
    radialReach: l2 * Math.cos(q[1]) + l3 * Math.cos(q[1] + q[2]),
    verticalReach: l2 * Math.sin(q[1]) + l3 * Math.sin(q[1] + q[2]),
    radialReachMm: metresToMillimetres(
      l2 * Math.cos(q[1]) + l3 * Math.cos(q[1] + q[2]),
    ),
    verticalReachMm: metresToMillimetres(
      l2 * Math.sin(q[1]) + l3 * Math.sin(q[1] + q[2]),
    ),
    jacobian,
    displayJacobian: jacobianForMillimetresAndDegrees(jacobian),
    linearVelocityMetresPerSecond,
    linearVelocityMillimetresPerSecond: vectorInMillimetres(linearVelocityMetresPerSecond),
    angularVelocityRadiansPerSecond,
    angularVelocityDegreesPerSecond: vectorInDegrees(angularVelocityRadiansPerSecond),
    velocityContributions,
    jacobianColumns: columns,
    inverse: {
      radial,
      vertical,
      cosineElbow,
      radialMm: metresToMillimetres(radial),
      verticalMm: metresToMillimetres(vertical),
      shoulderToTargetMm,
      pythagoreanDistanceMm: shoulderToTargetMm,
      targetMm: vectorInMillimetres(target),
      baseAngleDegrees: Math.atan2(target[1], target[0]) * 180 / Math.PI,
      reachability: {
        minimumMm: minimumReachMm,
        maximumMm: maximumReachMm,
        isReachable: shoulderToTargetMm >= minimumReachMm - 1e-7
          && shoulderToTargetMm <= maximumReachMm + 1e-7,
      },
      conventionalBranches,
      candidateDetails,
      result: inverseResult,
      solutionDetails,
    },
  }
}
