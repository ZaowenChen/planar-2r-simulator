import { describe, expect, expectTypeOf, it } from 'vitest'
import { DEFAULT_ROBOT_PARAMETERS } from './defaults'
import { forwardKinematics } from './kinematics'
import { rotationOf } from './transforms'
import type { Matrix3, Vector3 } from './types'

function expectVectorClose(actual: Vector3, expected: Vector3, tolerance: number): void {
  actual.forEach((value, index) => {
    expect(Math.abs(value - expected[index])).toBeLessThanOrEqual(tolerance)
  })
}

function determinant3(matrix: Matrix3): number {
  return matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
    - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
    + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
}

const ENDPOINT_FIXTURES: readonly (readonly [Vector3, Vector3])[] = [
  [[0, 0, 0], [3.5, 0, 0.8]],
  [[Math.PI / 2, 0, 0], [0, 3.5, 0.8]],
  [[-Math.PI / 2, 0, 0], [0, -3.5, 0.8]],
  [[Math.PI, 0, 0], [-3.5, 0, 0.8]],
  [[0, Math.PI / 2, 0], [0, 0, 4.3]],
  [[0, -Math.PI / 2, 0], [0, 0, -2.7]],
  [[0, 0, Math.PI / 2], [2, 0, 2.3]],
  [[0, 0, -Math.PI / 2], [2, 0, -0.7]],
  [[Math.PI / 4, Math.PI / 4, -Math.PI / 4], [2.0606601717798214, 2.060660171779821, 2.214213562373095]],
  [[-Math.PI / 3, Math.PI / 6, Math.PI / 3], [0.8660254037844388, -1.5, 3.3]],
  [[Math.PI / 6, -Math.PI / 6, -Math.PI / 3], [1.5, 0.8660254037844385, -1.7]],
  [[Math.PI / 3, Math.PI / 3, -2 * Math.PI / 3], [0.8750000000000002, 1.5155444566227676, 1.2330127018922195]],
]

describe('3R forward kinematics', () => {
  it('places the straight arm at the analytical endpoint and exposes the full chain', () => {
    const result = forwardKinematics([0, 0, 0], DEFAULT_ROBOT_PARAMETERS)

    expectVectorClose(result.endEffectorPosition, [3.5, 0, 0.8], 1e-12)
    expect(result.transforms).toHaveLength(4)
    expect(result.origins).toEqual([
      [0, 0, 0],
      [0, 0, 0.8],
      [2, 0, 0.8],
      [3.5, 0, 0.8],
    ])
    expectTypeOf(result.endEffectorPosition).toEqualTypeOf<Vector3>()
  })

  it.each(ENDPOINT_FIXTURES)('matches analytical endpoint fixture %#', (q, expected) => {
    expectVectorClose(
      forwardKinematics(q, DEFAULT_ROBOT_PARAMETERS).endEffectorPosition,
      expected,
      1e-12,
    )
  })

  it('expresses joint axes in the base frame from each preceding frame', () => {
    const result = forwardKinematics([Math.PI / 6, Math.PI / 4, -Math.PI / 3], DEFAULT_ROBOT_PARAMETERS)

    expectVectorClose(result.jointAxes[0], [0, 0, 1], 1e-12)
    expectVectorClose(result.jointAxes[1], [0.5, -Math.sqrt(3) / 2, 0], 1e-12)
    expectVectorClose(result.jointAxes[2], [0.5, -Math.sqrt(3) / 2, 0], 1e-12)
  })

  it('transforms each local center of mass with its link frame', () => {
    const result = forwardKinematics([0, 0, 0], DEFAULT_ROBOT_PARAMETERS)

    expectVectorClose(result.centerOfMassPositions[0], [0, 0, 0.4], 1e-12)
    expectVectorClose(result.centerOfMassPositions[1], [1, 0, 0.8], 1e-12)
    expectVectorClose(result.centerOfMassPositions[2], [2.75, 0, 0.8], 1e-12)
  })

  it('keeps every cumulative rotation proper and orthonormal', () => {
    const result = forwardKinematics([0.37, -0.61, 1.13], DEFAULT_ROBOT_PARAMETERS)

    result.transforms.forEach((transform) => {
      const rotation = rotationOf(transform)
      for (let row = 0; row < 3; row += 1) {
        for (let column = 0; column < 3; column += 1) {
          const dot = rotation[0][row] * rotation[0][column]
            + rotation[1][row] * rotation[1][column]
            + rotation[2][row] * rotation[2][column]
          expect(Math.abs(dot - Number(row === column))).toBeLessThanOrEqual(1e-10)
        }
      }
      expect(Math.abs(determinant3(rotation) - 1)).toBeLessThanOrEqual(1e-10)
    })
  })
})
