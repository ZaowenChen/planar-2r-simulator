import { forwardKinematics } from './kinematics'
import {
  cross3,
  dot3,
  inverse3,
  matVec,
  subtract3,
  symmetricEigenvalues3,
} from './linalg'
import { rotationOf } from './transforms'
import type {
  JointState,
  Matrix3,
  RobotParameters,
  Vector3,
} from './types'

const DEFAULT_DERIVATIVE_STEP = 1e-5
const MINIMUM_MASS_EIGENVALUE = 1e-9
const MAXIMUM_MASS_CONDITION_NUMBER = 1e10
const ZERO_VECTOR: Vector3 = [0, 0, 0]
const ZERO_MATRIX: Matrix3 = [ZERO_VECTOR, ZERO_VECTOR, ZERO_VECTOR]

export interface EnergyResult {
  kinetic: number
  potential: number
  total: number
  jointPower: Vector3
}

export interface DynamicsResult {
  tau: Vector3
  massMatrix: Matrix3
  coriolisMatrix: Matrix3
  coriolisTorque: Vector3
  gravityTorque: Vector3
  frictionTorque: Vector3
  conditionNumber: number
}

export type DynamicsDiagnosticKey =
  | '惯性矩阵最小特征值'
  | '惯性矩阵条件数'

export class DynamicsError extends Error {
  constructor(
    public readonly diagnosticKey: DynamicsDiagnosticKey,
    public readonly value: number,
  ) {
    super(`${diagnosticKey}：${value}`)
    this.name = 'DynamicsError'
  }
}

interface CenterOfMassJacobians {
  translational: Matrix3
  angular: Matrix3
}

function matrixFromColumns(columns: readonly Vector3[]): Matrix3 {
  return [
    [columns[0][0], columns[1][0], columns[2][0]],
    [columns[0][1], columns[1][1], columns[2][1]],
    [columns[0][2], columns[1][2], columns[2][2]],
  ]
}

function column(matrix: Matrix3, index: number): Vector3 {
  return [matrix[0][index], matrix[1][index], matrix[2][index]]
}

function addVectors(...vectors: readonly Vector3[]): Vector3 {
  return vectors.reduce<Vector3>((sum, vector) => [
    sum[0] + vector[0],
    sum[1] + vector[1],
    sum[2] + vector[2],
  ], ZERO_VECTOR)
}

function scaleVector(vector: Vector3, scalar: number): Vector3 {
  return [vector[0] * scalar, vector[1] * scalar, vector[2] * scalar]
}

function centerOfMassJacobians(
  q: Vector3,
  parameters: RobotParameters,
): readonly [
  CenterOfMassJacobians,
  CenterOfMassJacobians,
  CenterOfMassJacobians,
] {
  const kinematics = forwardKinematics(q, parameters)

  return parameters.links.map((_, linkIndex) => {
    const translationalColumns = kinematics.jointAxes.map((axis, jointIndex) => (
      jointIndex <= linkIndex
        ? cross3(
          axis,
          subtract3(
            kinematics.centerOfMassPositions[linkIndex],
            kinematics.origins[jointIndex],
          ),
        )
        : ZERO_VECTOR
    ))
    const angularColumns = kinematics.jointAxes.map((axis, jointIndex) => (
      jointIndex <= linkIndex ? axis : ZERO_VECTOR
    ))

    return {
      translational: matrixFromColumns(translationalColumns),
      angular: matrixFromColumns(angularColumns),
    }
  }) as unknown as readonly [
    CenterOfMassJacobians,
    CenterOfMassJacobians,
    CenterOfMassJacobians,
  ]
}

function rotatedInertia(
  rotation: Matrix3,
  inertia: Matrix3,
): Matrix3 {
  const valueAt = (row: number, matrixColumn: number): number => {
    let value = 0
    for (let first = 0; first < 3; first += 1) {
      for (let second = 0; second < 3; second += 1) {
        value += rotation[row][first]
          * inertia[first][second]
          * rotation[matrixColumn][second]
      }
    }
    return value
  }

  return [
    [valueAt(0, 0), valueAt(0, 1), valueAt(0, 2)],
    [valueAt(1, 0), valueAt(1, 1), valueAt(1, 2)],
    [valueAt(2, 0), valueAt(2, 1), valueAt(2, 2)],
  ]
}

function matrixDifferenceQuotient(
  plus: Matrix3,
  minus: Matrix3,
  denominator: number,
): Matrix3 {
  const valueAt = (row: number, matrixColumn: number): number => (
    (plus[row][matrixColumn] - minus[row][matrixColumn]) / denominator
  )
  return [
    [valueAt(0, 0), valueAt(0, 1), valueAt(0, 2)],
    [valueAt(1, 0), valueAt(1, 1), valueAt(1, 2)],
    [valueAt(2, 0), valueAt(2, 1), valueAt(2, 2)],
  ]
}

function requireDerivativeStep(step: number): void {
  if (!Number.isFinite(step) || step <= 0) {
    throw new RangeError('Mass-matrix derivative step must be finite and positive')
  }
}

function massMatrixPartials(
  q: Vector3,
  parameters: RobotParameters,
  derivativeStep: number,
): readonly [Matrix3, Matrix3, Matrix3] {
  requireDerivativeStep(derivativeStep)
  return q.map((_, jointIndex) => {
    const plus = [...q] as [number, number, number]
    const minus = [...q] as [number, number, number]
    plus[jointIndex] += derivativeStep
    minus[jointIndex] -= derivativeStep
    return matrixDifferenceQuotient(
      massMatrix(plus, parameters),
      massMatrix(minus, parameters),
      2 * derivativeStep,
    )
  }) as unknown as readonly [Matrix3, Matrix3, Matrix3]
}

function potentialEnergy(q: Vector3, parameters: RobotParameters): number {
  const positions = forwardKinematics(q, parameters).centerOfMassPositions
  return parameters.links.reduce((potential, link, linkIndex) => (
    potential - link.mass * dot3(parameters.gravity, positions[linkIndex])
  ), 0)
}

function massConditionNumber(matrix: Matrix3): number {
  const eigenvalues = symmetricEigenvalues3(matrix)
  const minimum = eigenvalues[0]
  return minimum <= 0
    ? Number.POSITIVE_INFINITY
    : eigenvalues[2] / minimum
}

function frictionVector(
  qd: Vector3,
  parameters: RobotParameters,
): Vector3 {
  return parameters.frictionEnabled
    ? [
      parameters.viscousFriction[0] * qd[0],
      parameters.viscousFriction[1] * qd[1],
      parameters.viscousFriction[2] * qd[2],
    ]
    : ZERO_VECTOR
}

export function massMatrix(
  q: Vector3,
  parameters: RobotParameters,
): Matrix3 {
  const kinematics = forwardKinematics(q, parameters)
  const jacobians = centerOfMassJacobians(q, parameters)
  const result = ZERO_MATRIX.map((row) => [...row]) as [
    [number, number, number],
    [number, number, number],
    [number, number, number],
  ]

  parameters.links.forEach((link, linkIndex) => {
    const translational = jacobians[linkIndex].translational
    const angular = jacobians[linkIndex].angular
    const worldInertia = rotatedInertia(
      rotationOf(kinematics.transforms[linkIndex]),
      link.inertia,
    )

    for (let row = 0; row < 3; row += 1) {
      for (let matrixColumn = 0; matrixColumn < 3; matrixColumn += 1) {
        const linearContribution = link.mass * dot3(
          column(translational, row),
          column(translational, matrixColumn),
        )
        const angularContribution = dot3(
          column(angular, row),
          matVec(worldInertia, column(angular, matrixColumn)),
        )
        result[row][matrixColumn] += linearContribution + angularContribution
      }
    }
  })

  return result
}

export function directionalMassDerivative(
  q: Vector3,
  qd: Vector3,
  parameters: RobotParameters,
  derivativeStep = DEFAULT_DERIVATIVE_STEP,
): Matrix3 {
  const partials = massMatrixPartials(q, parameters, derivativeStep)
  const valueAt = (row: number, matrixColumn: number): number => (
    partials[0][row][matrixColumn] * qd[0]
    + partials[1][row][matrixColumn] * qd[1]
    + partials[2][row][matrixColumn] * qd[2]
  )
  return [
    [valueAt(0, 0), valueAt(0, 1), valueAt(0, 2)],
    [valueAt(1, 0), valueAt(1, 1), valueAt(1, 2)],
    [valueAt(2, 0), valueAt(2, 1), valueAt(2, 2)],
  ]
}

export function coriolisMatrix(
  q: Vector3,
  qd: Vector3,
  parameters: RobotParameters,
  derivativeStep = DEFAULT_DERIVATIVE_STEP,
): Matrix3 {
  const partials = massMatrixPartials(q, parameters, derivativeStep)
  const valueAt = (row: number, matrixColumn: number): number => {
    let value = 0
    for (let velocityIndex = 0; velocityIndex < 3; velocityIndex += 1) {
      const christoffel = 0.5 * (
        partials[velocityIndex][row][matrixColumn]
        + partials[matrixColumn][row][velocityIndex]
        - partials[row][matrixColumn][velocityIndex]
      )
      value += christoffel * qd[velocityIndex]
    }
    return value
  }

  return [
    [valueAt(0, 0), valueAt(0, 1), valueAt(0, 2)],
    [valueAt(1, 0), valueAt(1, 1), valueAt(1, 2)],
    [valueAt(2, 0), valueAt(2, 1), valueAt(2, 2)],
  ]
}

export function gravityVector(
  q: Vector3,
  parameters: RobotParameters,
): Vector3 {
  const jacobians = centerOfMassJacobians(q, parameters)
  return parameters.links.reduce<Vector3>((gravityTorque, link, linkIndex) => {
    const translational = jacobians[linkIndex].translational
    return addVectors(
      gravityTorque,
      [
        -link.mass * dot3(column(translational, 0), parameters.gravity),
        -link.mass * dot3(column(translational, 1), parameters.gravity),
        -link.mass * dot3(column(translational, 2), parameters.gravity),
      ],
    )
  }, ZERO_VECTOR)
}

export function inverseDynamics(
  state: JointState,
  parameters: RobotParameters,
): DynamicsResult {
  const inertia = massMatrix(state.q, parameters)
  const coriolis = coriolisMatrix(state.q, state.qd, parameters)
  const inertiaTorque = matVec(inertia, state.qdd)
  const coriolisTorque = matVec(coriolis, state.qd)
  const gravityTorque = gravityVector(state.q, parameters)
  const frictionTorque = frictionVector(state.qd, parameters)
  const tau = addVectors(
    inertiaTorque,
    coriolisTorque,
    gravityTorque,
    frictionTorque,
  )

  return {
    tau,
    massMatrix: inertia,
    coriolisMatrix: coriolis,
    coriolisTorque,
    gravityTorque,
    frictionTorque,
    conditionNumber: massConditionNumber(inertia),
  }
}

export function energy(
  state: JointState,
  parameters: RobotParameters,
): EnergyResult {
  const inertia = massMatrix(state.q, parameters)
  const kinetic = 0.5 * dot3(state.qd, matVec(inertia, state.qd))
  const potential = potentialEnergy(state.q, parameters)
  const tau = inverseDynamics(state, parameters).tau
  return {
    kinetic,
    potential,
    total: kinetic + potential,
    jointPower: [
      tau[0] * state.qd[0],
      tau[1] * state.qd[1],
      tau[2] * state.qd[2],
    ],
  }
}

export function forwardDynamics(
  q: Vector3,
  qd: Vector3,
  tau: Vector3,
  parameters: RobotParameters,
): Vector3 {
  const inertia = massMatrix(q, parameters)
  const eigenvalues = symmetricEigenvalues3(inertia)
  const minimumEigenvalue = eigenvalues[0]
  if (minimumEigenvalue <= MINIMUM_MASS_EIGENVALUE) {
    throw new DynamicsError('惯性矩阵最小特征值', minimumEigenvalue)
  }

  const conditionNumber = eigenvalues[2] / minimumEigenvalue
  if (conditionNumber > MAXIMUM_MASS_CONDITION_NUMBER) {
    throw new DynamicsError('惯性矩阵条件数', conditionNumber)
  }

  const coriolisTorque = matVec(coriolisMatrix(q, qd, parameters), qd)
  const gravityTorque = gravityVector(q, parameters)
  const frictionTorque = frictionVector(qd, parameters)
  const opposingTorque = addVectors(
    coriolisTorque,
    gravityTorque,
    frictionTorque,
  )
  const netTorque = addVectors(tau, scaleVector(opposingTorque, -1))
  return matVec(inverse3(inertia), netTorque)
}
