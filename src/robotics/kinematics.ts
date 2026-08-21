import {
  dhTransform,
  multiply4,
  transformPoint,
  translationOf,
} from './transforms'
import type { Matrix4, RobotParameters, Vector3 } from './types'

export type InverseKinematicsStatus =
  | 'reachable'
  | 'unreachable'
  | 'axis-singular'
  | 'joint-limit'

export interface InverseKinematicsSolution {
  q: Vector3
  branch: 'elbow-down' | 'elbow-up'
  radialFamily: 'conventional' | 'folded'
}

export interface InverseKinematicsCandidate extends InverseKinematicsSolution {
  withinJointLimits: boolean
}

export interface InverseKinematicsResult {
  status: InverseKinematicsStatus
  solutions: readonly InverseKinematicsSolution[]
}

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

const IK_ROUNDOFF_TOLERANCE = 1e-10
const AXIS_SINGULARITY_TOLERANCE = 1e-10

function requireFiniteVector(vector: Vector3, name: string): void {
  if (!Array.isArray(vector) || vector.length !== 3) {
    throw new RangeError(`${name} must contain exactly three values`)
  }
  if (vector.some((value) => !Number.isFinite(value))) {
    throw new RangeError(`${name} must contain only finite values`)
  }
}

function requireValidIkGeometry(parameters: RobotParameters): void {
  const { d1, l2, l3 } = parameters.geometry
  if (!Number.isFinite(d1) || !Number.isFinite(l2) || !Number.isFinite(l3)) {
    throw new RangeError('IK geometry must contain only finite values')
  }
  if (d1 < 0 || l2 <= 0 || l3 <= 0) {
    throw new RangeError('IK geometry requires d1 >= 0 and positive link lengths')
  }
}

function normalizeAngle(angle: number): number {
  return Math.atan2(Math.sin(angle), Math.cos(angle))
}

function wrappedJointDistance(left: Vector3, right: Vector3): number {
  return Math.hypot(
    normalizeAngle(left[0] - right[0]),
    normalizeAngle(left[1] - right[1]),
    normalizeAngle(left[2] - right[2]),
  )
}

function withinJointLimits(q: Vector3, parameters: RobotParameters): boolean {
  return q.every((angle, index) => {
    const [minimum, maximum] = parameters.jointLimits[index]
    return angle >= minimum - IK_ROUNDOFF_TOLERANCE
      && angle <= maximum + IK_ROUNDOFF_TOLERANCE
  })
}

export function inverseKinematicsCandidates(
  target: Vector3,
  parameters: RobotParameters,
): readonly InverseKinematicsCandidate[] {
  requireFiniteVector(target, 'IK target')
  requireValidIkGeometry(parameters)

  const { d1, l2, l3 } = parameters.geometry
  const [x, y, z] = target
  const radial = Math.hypot(x, y)
  const vertical = z - d1
  const cosineElbow = (
    radial * radial + vertical * vertical - l2 * l2 - l3 * l3
  ) / (2 * l2 * l3)

  if (
    cosineElbow < -1 - IK_ROUNDOFF_TOLERANCE
    || cosineElbow > 1 + IK_ROUNDOFF_TOLERANCE
  ) {
    return []
  }

  const baseAngle = radial < AXIS_SINGULARITY_TOLERANCE ? 0 : Math.atan2(y, x)
  const elbowMagnitude = Math.acos(Math.max(-1, Math.min(1, cosineElbow)))
  const branchDefinitions = [
    { branch: 'elbow-down' as const, theta3: elbowMagnitude },
    { branch: 'elbow-up' as const, theta3: -elbowMagnitude },
  ]
  const radialFamilies = ['conventional', 'folded'] as const

  return radialFamilies.flatMap((radialFamily) => branchDefinitions.map(({ branch, theta3 }) => {
    const signedRadial = radialFamily === 'conventional' ? radial : -radial
    const theta1 = radialFamily === 'conventional'
      ? baseAngle
      : normalizeAngle(baseAngle + Math.PI)
    const theta2 = normalizeAngle(
      Math.atan2(vertical, signedRadial)
      - Math.atan2(l3 * Math.sin(theta3), l2 + l3 * Math.cos(theta3)),
    )
    const q: Vector3 = [theta1, theta2, theta3]
    return {
      q,
      branch,
      radialFamily,
      withinJointLimits: withinJointLimits(q, parameters),
    }
  }))
}

export function inverseKinematics(
  target: Vector3,
  parameters: RobotParameters,
  referenceQ?: Vector3,
): InverseKinematicsResult {
  requireFiniteVector(target, 'IK target')
  requireValidIkGeometry(parameters)
  if (referenceQ !== undefined) {
    requireFiniteVector(referenceQ, 'IK reference')
  }

  const { d1, l2, l3 } = parameters.geometry
  const [x, y, z] = target
  const radial = Math.hypot(x, y)
  const vertical = z - d1
  const cosineElbow = (
    radial * radial + vertical * vertical - l2 * l2 - l3 * l3
  ) / (2 * l2 * l3)

  if (
    cosineElbow < -1 - IK_ROUNDOFF_TOLERANCE
    || cosineElbow > 1 + IK_ROUNDOFF_TOLERANCE
  ) {
    return { status: 'unreachable', solutions: [] }
  }

  const axisSingular = radial < AXIS_SINGULARITY_TOLERANCE
  const analyticCandidates = inverseKinematicsCandidates(target, parameters)

  function selectRadialFamily(
    branch: InverseKinematicsSolution['branch'],
  ): InverseKinematicsSolution {
    const branchCandidates = analyticCandidates
      .filter((candidate) => candidate.branch === branch)
    const conventional = branchCandidates.find((candidate) => (
      candidate.radialFamily === 'conventional'
    ))!
    if (referenceQ === undefined || axisSingular) {
      return conventional
    }

    const folded = branchCandidates.find((candidate) => candidate.radialFamily === 'folded')!
    const validFamilies = [conventional, folded]
      .filter((candidate) => candidate.withinJointLimits)
    const selectableFamilies = validFamilies.length > 0
      ? validFamilies
      : [conventional, folded]
    return selectableFamilies.reduce((nearest, candidate) => (
      wrappedJointDistance(candidate.q, referenceQ)
        < wrappedJointDistance(nearest.q, referenceQ)
        ? candidate
        : nearest
    ))
  }

  const candidates = [
    selectRadialFamily('elbow-down'),
    selectRadialFamily('elbow-up'),
  ]
  const solutions = candidates
    .filter((candidate) => withinJointLimits(candidate.q, parameters))
    .map(({ q, branch, radialFamily }) => ({ q, branch, radialFamily }))

  if (solutions.length === 0) {
    return { status: 'joint-limit', solutions: [] }
  }
  return {
    status: axisSingular ? 'axis-singular' : 'reachable',
    solutions,
  }
}
