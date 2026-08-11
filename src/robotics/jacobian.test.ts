import { describe, expect, it } from 'vitest'
import { DEFAULT_ROBOT_PARAMETERS } from './defaults'
import { geometricJacobian, singularityMetrics } from './jacobian'
import { forwardKinematics } from './kinematics'
import type { Matrix6x3, Vector3 } from './types'

function expectJacobianToMatchFiniteDifference(
  jacobian: Matrix6x3,
  q: Vector3,
  step: number,
  tolerance: number,
): void {
  for (let joint = 0; joint < 3; joint += 1) {
    const plus = [...q] as [number, number, number]
    const minus = [...q] as [number, number, number]
    plus[joint] += step
    minus[joint] -= step
    const plusPosition = forwardKinematics(plus, DEFAULT_ROBOT_PARAMETERS).endEffectorPosition
    const minusPosition = forwardKinematics(minus, DEFAULT_ROBOT_PARAMETERS).endEffectorPosition

    for (let coordinate = 0; coordinate < 3; coordinate += 1) {
      const finiteDifference = (plusPosition[coordinate] - minusPosition[coordinate]) / (2 * step)
      expect(Math.abs(jacobian[coordinate][joint] - finiteDifference))
        .toBeLessThanOrEqual(tolerance)
    }
  }
}

describe('geometric Jacobian', () => {
  it('matches translational columns to position finite differences', () => {
    const q: Vector3 = [0.4, -0.2, 0.7]
    const jacobian = geometricJacobian(q, DEFAULT_ROBOT_PARAMETERS)

    expectJacobianToMatchFiniteDifference(jacobian, q, 1e-6, 2e-6)
  })

  it('uses each preceding-frame axis for its angular column', () => {
    const q: Vector3 = [Math.PI / 6, Math.PI / 4, -Math.PI / 3]
    const jacobian = geometricJacobian(q, DEFAULT_ROBOT_PARAMETERS)

    const expected = [
      [0, 0.5, 0.5],
      [0, -Math.sqrt(3) / 2, -Math.sqrt(3) / 2],
      [1, 0, 0],
    ]

    jacobian.slice(3).forEach((row, rowIndex) => row.forEach((value, columnIndex) => {
      expect(Math.abs(value - expected[rowIndex][columnIndex])).toBeLessThanOrEqual(1e-12)
    }))
  })

  it('detects a straight-arm position singularity with an infinite condition number', () => {
    const metrics = singularityMetrics(
      geometricJacobian([0, 0, 0], DEFAULT_ROBOT_PARAMETERS),
      1e-3,
    )

    expect(metrics.singularValues[0]).toBeGreaterThanOrEqual(metrics.singularValues[1])
    expect(metrics.singularValues[1]).toBeGreaterThanOrEqual(metrics.singularValues[2])
    expect(metrics.minimumSingularValue).toBeLessThanOrEqual(1e-12)
    expect(metrics.conditionNumber).toBe(Number.POSITIVE_INFINITY)
    expect(metrics.isSingular).toBe(true)
  })

  it('reports a nonsingular position Jacobian above the configured threshold', () => {
    const metrics = singularityMetrics(
      geometricJacobian([0.4, -0.2, 0.7], DEFAULT_ROBOT_PARAMETERS),
      1e-3,
    )

    expect(metrics.minimumSingularValue).toBeGreaterThan(1e-3)
    expect(metrics.conditionNumber).toBeGreaterThan(1)
    expect(Number.isFinite(metrics.conditionNumber)).toBe(true)
    expect(metrics.isSingular).toBe(false)
  })
})
