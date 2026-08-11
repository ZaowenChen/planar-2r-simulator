import { describe, expect, it } from 'vitest'
import { DEFAULT_ROBOT_PARAMETERS } from './defaults'
import { forwardKinematics, inverseKinematics } from './kinematics'
import type { Vector3 } from './types'

function expectVectorClose(actual: Vector3, expected: Vector3, tolerance: number): void {
  actual.forEach((value, index) => {
    expect(Math.abs(value - expected[index])).toBeLessThanOrEqual(tolerance)
  })
}

function wrappedDistance(left: Vector3, right: Vector3): number {
  return Math.hypot(...left.map((value, index) => (
    Math.atan2(Math.sin(value - right[index]), Math.cos(value - right[index]))
  )))
}

describe('closed-form inverse kinematics', () => {
  it.each([
    [1.8, 0.8, 1.4],
    [2.2, -0.7, 0.2],
    [0.5, 0.5, 3.0],
  ] as const)('returns every valid position branch for target %j', (...target) => {
    const desiredPosition: Vector3 = target
    const result = inverseKinematics(desiredPosition, DEFAULT_ROBOT_PARAMETERS)

    expect(result.status).toBe('reachable')
    expect(result.solutions.length).toBeGreaterThan(0)
    for (const solution of result.solutions) {
      expectVectorClose(
        forwardKinematics(solution.q, DEFAULT_ROBOT_PARAMETERS).endEffectorPosition,
        desiredPosition,
        1e-8,
      )
    }
  })

  it('returns a structured unreachable result without candidate poses', () => {
    const result = inverseKinematics([4, 0, 0.8], DEFAULT_ROBOT_PARAMETERS)

    expect(result).toEqual({ status: 'unreachable', solutions: [] })
  })

  it('clamps cosine-law roundoff at the reach boundary and preserves both coincident branches', () => {
    const result = inverseKinematics([3.5 + 1e-12, 0, 0.8], DEFAULT_ROBOT_PARAMETERS)

    expect(result.status).toBe('reachable')
    expect(result.solutions).toHaveLength(2)
    expectVectorClose(result.solutions[0].q, result.solutions[1].q, 1e-12)
    expect(result.solutions.map((solution) => solution.branch)).toEqual([
      'elbow-down',
      'elbow-up',
    ])
  })

  it('reports a base-axis singular target and chooses the documented zero base angle', () => {
    const result = inverseKinematics([0, 0, 3.8], DEFAULT_ROBOT_PARAMETERS)

    expect(result.status).toBe('axis-singular')
    expect(result.solutions.length).toBeGreaterThan(0)
    expect(result.solutions.every((solution) => solution.q[0] === 0)).toBe(true)
  })

  it('reports a geometrically reachable target when every branch violates a joint limit', () => {
    const result = inverseKinematics([0.75, 0, 0.8], DEFAULT_ROBOT_PARAMETERS)

    expect(result).toEqual({ status: 'joint-limit', solutions: [] })
  })

  it('selects the folded radial family nearest a folded reference pose', () => {
    const reference: Vector3 = [
      -Math.PI,
      -80 * Math.PI / 180,
      -150 * Math.PI / 180,
    ]
    const target = forwardKinematics(reference, DEFAULT_ROBOT_PARAMETERS).endEffectorPosition

    const result = inverseKinematics(target, DEFAULT_ROBOT_PARAMETERS, reference)
    const elbowUp = result.solutions.find((solution) => solution.branch === 'elbow-up')

    expect(result.status).toBe('reachable')
    expect(elbowUp?.radialFamily).toBe('folded')
    expect(wrappedDistance(elbowUp!.q, reference)).toBeLessThanOrEqual(1e-10)
  })

  it('preserves every elbow branch when the nearest radial family violates joint limits', () => {
    const target: Vector3 = [-0.9058666579, 0, -2.5807403920]
    const reference: Vector3 = [
      -60 * Math.PI / 180,
      -90 * Math.PI / 180,
      -150 * Math.PI / 180,
    ]

    const result = inverseKinematics(target, DEFAULT_ROBOT_PARAMETERS, reference)

    expect(result.status).toBe('reachable')
    expect(result.solutions.map((solution) => solution.branch)).toEqual([
      'elbow-down',
      'elbow-up',
    ])
    expect(result.solutions.every((solution) => solution.radialFamily === 'conventional'))
      .toBe(true)
    for (const solution of result.solutions) {
      expectVectorClose(
        forwardKinematics(solution.q, DEFAULT_ROBOT_PARAMETERS).endEffectorPosition,
        target,
        1e-8,
      )
      solution.q.forEach((angle, joint) => {
        const [minimum, maximum] = DEFAULT_ROBOT_PARAMETERS.jointLimits[joint]
        expect(angle).toBeGreaterThanOrEqual(minimum - 1e-10)
        expect(angle).toBeLessThanOrEqual(maximum + 1e-10)
      })
    }
  })

  it('rejects malformed target tuples rather than misclassifying them as user states', () => {
    expect(() => inverseKinematics(
      [1, 2] as unknown as Vector3,
      DEFAULT_ROBOT_PARAMETERS,
    )).toThrow(RangeError)
  })
})
