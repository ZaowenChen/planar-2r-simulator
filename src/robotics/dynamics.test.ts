import { describe, expect, it } from 'vitest'
import { DEFAULT_ROBOT_PARAMETERS } from './defaults'
import {
  DynamicsError,
  coriolisMatrix,
  directionalMassDerivative,
  energy,
  forwardDynamics,
  gravityVector,
  inverseDynamics,
  massMatrix,
} from './dynamics'
import {
  frobeniusNorm,
  symmetricEigenvalues3,
  transpose,
} from './linalg'
import type {
  JointState,
  Matrix3,
  RobotParameters,
  Vector3,
} from './types'

function expectVectorClose(
  actual: Vector3,
  expected: Vector3,
  tolerance: number,
): void {
  actual.forEach((value, index) => {
    expect(Math.abs(value - expected[index])).toBeLessThanOrEqual(tolerance)
  })
}

function expectMatrixClose(
  actual: Matrix3,
  expected: Matrix3,
  tolerance: number,
): void {
  actual.forEach((row, rowIndex) => {
    row.forEach((value, columnIndex) => {
      expect(Math.abs(value - expected[rowIndex][columnIndex]))
        .toBeLessThanOrEqual(tolerance)
    })
  })
}

function addMatrices(left: Matrix3, right: Matrix3): Matrix3 {
  return left.map((row, rowIndex) => row.map((value, columnIndex) => (
    value + right[rowIndex][columnIndex]
  ))) as unknown as Matrix3
}

function subtractMatrices(left: Matrix3, right: Matrix3): Matrix3 {
  return left.map((row, rowIndex) => row.map((value, columnIndex) => (
    value - right[rowIndex][columnIndex]
  ))) as unknown as Matrix3
}

function scaleMatrix(matrix: Matrix3, scalar: number): Matrix3 {
  return matrix.map((row) => row.map((value) => value * scalar)) as unknown as Matrix3
}

function withParameters(
  changes: Partial<RobotParameters>,
): RobotParameters {
  return {
    ...DEFAULT_ROBOT_PARAMETERS,
    ...changes,
  }
}

describe('rigid-body dynamics', () => {
  it('builds a symmetric positive-definite mass matrix', () => {
    const matrix = massMatrix([0.3, -0.4, 0.8], DEFAULT_ROBOT_PARAMETERS)

    expectMatrixClose(matrix, transpose(matrix), 1e-10)
    expect(Math.min(...symmetricEigenvalues3(matrix))).toBeGreaterThan(1e-9)
  })

  it('includes the exact translational and rotated-inertia terms for one active link', () => {
    const parameters = withParameters({
      links: [
        {
          mass: 2,
          centerOfMass: [0.5, 0, 0],
          inertia: [[3, 0, 0], [0, 5, 0], [0, 0, 7]],
        },
        {
          mass: 0,
          centerOfMass: [0, 0, 0],
          inertia: [[0, 0, 0], [0, 0, 0], [0, 0, 0]],
        },
        {
          mass: 0,
          centerOfMass: [0, 0, 0],
          inertia: [[0, 0, 0], [0, 0, 0], [0, 0, 0]],
        },
      ],
    })

    expectMatrixClose(
      massMatrix([0.7, -0.2, 0.4], parameters),
      [[5.5, 0, 0], [0, 0, 0], [0, 0, 0]],
      1e-12,
    )
  })

  it('satisfies the skew-symmetry identity for M-dot minus twice C', () => {
    const q: Vector3 = [0.2, -0.3, 0.6]
    const qd: Vector3 = [0.4, -0.2, 0.3]
    const mDot = directionalMassDerivative(
      q,
      qd,
      DEFAULT_ROBOT_PARAMETERS,
      1e-5,
    )
    const c = coriolisMatrix(q, qd, DEFAULT_ROBOT_PARAMETERS)
    const residual = subtractMatrices(mDot, scaleMatrix(c, 2))
    const symmetricResidual = addMatrices(residual, transpose(residual))

    expect(frobeniusNorm(symmetricResidual)).toBeLessThan(1e-6)
  })

  it('returns an exactly zero Coriolis matrix at zero velocity', () => {
    expect(coriolisMatrix(
      [0.2, -0.3, 0.6],
      [0, 0, 0],
      DEFAULT_ROBOT_PARAMETERS,
    )).toEqual([[0, 0, 0], [0, 0, 0], [0, 0, 0]])
  })

  it('rejects a nonpositive central-difference step', () => {
    expect(() => coriolisMatrix(
      [0.2, -0.3, 0.6],
      [0.4, -0.2, 0.3],
      DEFAULT_ROBOT_PARAMETERS,
      0,
    )).toThrowError(RangeError)
  })

  it('returns zero gravity torque and zero potential when gravity is zero', () => {
    const parameters = withParameters({ gravity: [0, 0, 0] })
    const state: JointState = {
      q: [0.3, -0.2, 0.5],
      qd: [0.1, 0.2, -0.1],
      qdd: [0.4, -0.3, 0.2],
    }

    expect(gravityVector(state.q, parameters)).toEqual([0, 0, 0])
    expect(energy(state, parameters).potential).toBe(0)
  })

  it('matches the potential-energy gradient with the gravity vector', () => {
    const q: Vector3 = [0.3, -0.4, 0.7]
    const step = 1e-6
    const gradient = q.map((_, jointIndex) => {
      const plus = [...q] as [number, number, number]
      const minus = [...q] as [number, number, number]
      plus[jointIndex] += step
      minus[jointIndex] -= step
      const stationary = { q: plus, qd: [0, 0, 0], qdd: [0, 0, 0] } as JointState
      const potentialPlus = energy(stationary, DEFAULT_ROBOT_PARAMETERS).potential
      const potentialMinus = energy(
        { ...stationary, q: minus },
        DEFAULT_ROBOT_PARAMETERS,
      ).potential
      return (potentialPlus - potentialMinus) / (2 * step)
    }) as unknown as Vector3

    expectVectorClose(
      gradient,
      gravityVector(q, DEFAULT_ROBOT_PARAMETERS),
      1e-7,
    )
  })

  it('computes positive kinetic energy and a consistent total', () => {
    const result = energy({
      q: [0.2, -0.5, 0.4],
      qd: [0.4, -0.3, 0.2],
      qdd: [0, 0, 0],
    }, DEFAULT_ROBOT_PARAMETERS)

    expect(result.kinetic).toBeGreaterThan(0)
    expect(result.total).toBeCloseTo(result.kinetic + result.potential, 12)
  })

  it('disables friction exactly when the parameter switch is off', () => {
    const parameters = withParameters({ frictionEnabled: false })
    const result = inverseDynamics({
      q: [0.2, -0.5, 0.4],
      qd: [0.4, -0.3, 0.2],
      qdd: [0.1, 0.2, -0.1],
    }, parameters)

    expect(result.frictionTorque).toEqual([0, 0, 0])
  })

  it('reports exact per-joint actuator power tau-i times qd-i', () => {
    const state: JointState = {
      q: [0.2, -0.5, 0.4],
      qd: [0.4, -0.3, 0.2],
      qdd: [0.1, 0.2, -0.1],
    }
    const tau = inverseDynamics(state, DEFAULT_ROBOT_PARAMETERS).tau

    expect(energy(state, DEFAULT_ROBOT_PARAMETERS).jointPower).toEqual([
      tau[0] * state.qd[0],
      tau[1] * state.qd[1],
      tau[2] * state.qd[2],
    ])
  })

  it('round-trips acceleration through inverse and forward dynamics', () => {
    const state: JointState = {
      q: [0.3, -0.2, 0.5],
      qd: [0.1, 0.2, -0.1],
      qdd: [0.4, -0.3, 0.2],
    }
    const tau = inverseDynamics(state, DEFAULT_ROBOT_PARAMETERS).tau

    expectVectorClose(
      forwardDynamics(state.q, state.qd, tau, DEFAULT_ROBOT_PARAMETERS),
      state.qdd,
      1e-7,
    )
  })

  it('rejects a mass matrix whose minimum eigenvalue is too small', () => {
    const zeroInertia = [[0, 0, 0], [0, 0, 0], [0, 0, 0]] as Matrix3
    const parameters = withParameters({
      links: DEFAULT_ROBOT_PARAMETERS.links.map((link) => ({
        ...link,
        mass: 0,
        inertia: zeroInertia,
      })) as unknown as RobotParameters['links'],
    })

    expect(() => forwardDynamics(
      [0, 0, 0],
      [0, 0, 0],
      [0, 0, 0],
      parameters,
    )).toThrowError(DynamicsError)

    try {
      forwardDynamics([0, 0, 0], [0, 0, 0], [0, 0, 0], parameters)
      throw new Error('Expected forward dynamics to reject a singular mass matrix')
    } catch (error) {
      expect(error).toBeInstanceOf(DynamicsError)
      expect((error as DynamicsError).diagnosticKey).toBe('惯性矩阵最小特征值')
      expect((error as DynamicsError).value).toBe(0)
    }
  })

  it('rejects an otherwise positive mass matrix with excessive conditioning', () => {
    const parameters = withParameters({
      links: [
        {
          mass: 0,
          centerOfMass: [0, 0, 0],
          inertia: [[1, 0, 0], [0, 1e6, 0], [0, 0, 1]],
        },
        {
          mass: 0,
          centerOfMass: [0, 0, 0],
          inertia: [[1, 0, 0], [0, 0, 0], [0, 0, 1]],
        },
        {
          mass: 0,
          centerOfMass: [0, 0, 0],
          inertia: [[1, 0, 0], [0, 0, 0], [0, 0, 2e-9]],
        },
      ],
    })

    try {
      forwardDynamics([0, 0, 0], [0, 0, 0], [0, 0, 0], parameters)
      throw new Error('Expected forward dynamics to reject an ill-conditioned mass matrix')
    } catch (error) {
      expect(error).toBeInstanceOf(DynamicsError)
      expect((error as DynamicsError).diagnosticKey).toBe('惯性矩阵条件数')
      expect((error as DynamicsError).value).toBeGreaterThan(1e10)
    }
  })
})
