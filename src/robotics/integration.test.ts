import { describe, expect, it } from 'vitest'
import { DEFAULT_ROBOT_PARAMETERS } from './defaults'
import { forwardDynamics } from './dynamics'
import {
  integrateScalarRk4,
  simulateForwardDynamics,
  simulateInverseDynamics,
} from './integration'
import type { RobotParameters, Vector3 } from './types'

function expectVectorClose(
  actual: Vector3,
  expected: Vector3,
  tolerance: number,
): void {
  actual.forEach((value, index) => {
    expect(Math.abs(value - expected[index])).toBeLessThanOrEqual(tolerance)
  })
}

function withParameters(changes: Partial<RobotParameters>): RobotParameters {
  return { ...DEFAULT_ROBOT_PARAMETERS, ...changes }
}

describe('RK4 integration', () => {
  it('shows fourth-order convergence on y-prime equals y', () => {
    const errors = [0.2, 0.1, 0.05].map((stepSize) => Math.abs(
      integrateScalarRk4(1, 0, 1, stepSize, (value) => value) - Math.E,
    ))

    expect(errors[0] / errors[1]).toBeGreaterThan(12)
    expect(errors[1] / errors[2]).toBeGreaterThan(12)
  })
})

describe('inverse-dynamics simulation', () => {
  it('includes the exact final time for a non-divisible step size', () => {
    const samples = simulateInverseDynamics({
      trajectory: {
        type: 'quintic',
        q0: [0, 0, 0],
        qf: [0.2, -0.1, 0.3],
        duration: 1,
      },
      duration: 1,
      stepSize: 0.3,
    }, DEFAULT_ROBOT_PARAMETERS)

    expect(samples.map(({ time }) => time)).toEqual([0, 0.3, 0.6, 0.9, 1])
  })

  it('produces torque that reproduces prescribed acceleration at every tenth sample', () => {
    const samples = simulateInverseDynamics({
      trajectory: {
        type: 'quintic',
        q0: [0, 0.2, -0.3],
        qf: [0.8, -0.4, 0.5],
        duration: 1,
      },
      duration: 1,
      stepSize: 0.01,
    }, DEFAULT_ROBOT_PARAMETERS)

    samples.filter((_, index) => index % 10 === 0).forEach((sample) => {
      expectVectorClose(
        forwardDynamics(
          sample.q,
          sample.qd,
          sample.tau,
          DEFAULT_ROBOT_PARAMETERS,
        ),
        sample.qdd,
        1e-7,
      )
      expect(sample.totalEnergy).toBeCloseTo(sample.kinetic + sample.potential, 12)
      expect(sample.jointPower).toEqual([
        sample.tau[0] * sample.qd[0],
        sample.tau[1] * sample.qd[1],
        sample.tau[2] * sample.qd[2],
      ])
    })
  })
})

describe('forward-dynamics simulation', () => {
  it('preserves mechanical energy during two unforced frictionless seconds', () => {
    const parameters = withParameters({
      frictionEnabled: false,
      jointLimits: [
        [-2 * Math.PI, 2 * Math.PI],
        [-2 * Math.PI, 2 * Math.PI],
        [-2 * Math.PI, 2 * Math.PI],
      ],
    })
    const samples = simulateForwardDynamics({
      initialState: { q: [0.2, -0.3, 0.4], qd: [0.1, 0.05, -0.08] },
      torqueProfile: { type: 'constant', value: [0, 0, 0], duration: 2 },
      duration: 2,
      stepSize: 0.001,
    }, parameters)
    const initialEnergy = samples[0].totalEnergy
    const maximumDrift = Math.max(...samples.map(({ totalEnergy }) => (
      Math.abs(totalEnergy - initialEnergy) / Math.max(1, Math.abs(initialEnergy))
    )))

    expect(samples).toHaveLength(2001)
    expect(maximumDrift).toBeLessThan(0.005)
  }, 15_000)

  it('includes the exact final time for a non-divisible integration step', () => {
    const samples = simulateForwardDynamics({
      initialState: { q: [0, 0, 0], qd: [0, 0, 0] },
      torqueProfile: { type: 'constant', value: [0, 0, 0], duration: 0.25 },
      duration: 0.25,
      stepSize: 0.1,
    }, DEFAULT_ROBOT_PARAMETERS)

    expect(samples.map(({ time }) => time)).toEqual([0, 0.1, 0.2, 0.25])
  })

  it('stops before an out-of-limit sample and returns its diagnostic', () => {
    const samples = simulateForwardDynamics({
      initialState: { q: [Math.PI - 0.001, 0, 0], qd: [1, 0, 0] },
      torqueProfile: { type: 'constant', value: [0, 0, 0], duration: 0.1 },
      duration: 0.1,
      stepSize: 0.1,
    }, withParameters({ gravity: [0, 0, 0], frictionEnabled: false }))

    expect(samples).toHaveLength(1)
    expect(samples.diagnostic).toMatchObject({
      kind: 'joint-limit',
      time: 0.1,
      jointIndex: 0,
    })
    expect(samples.every(({ q }) => q[0] <= Math.PI)).toBe(true)
  })

  it('returns a non-finite diagnostic without appending an invalid sample', () => {
    const samples = simulateForwardDynamics({
      initialState: { q: [0, 0, 0], qd: [0, 0, 0] },
      torqueProfile: { type: 'constant', value: [Number.NaN, 0, 0], duration: 0.1 },
      duration: 0.1,
      stepSize: 0.1,
    }, DEFAULT_ROBOT_PARAMETERS)

    expect(samples).toHaveLength(0)
    expect(samples.diagnostic).toMatchObject({ kind: 'non-finite', time: 0 })
  })
})
