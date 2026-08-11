import { describe, expect, it } from 'vitest'
import {
  dhTransform,
  multiply4,
  rotationOf,
  rotationX,
  rotationZ,
  transformPoint,
  translation,
  translationOf,
} from './transforms'
import type { Matrix4, Vector3 } from './types'

function expectMatrixClose(actual: Matrix4, expected: Matrix4, tolerance: number): void {
  actual.forEach((row, rowIndex) => row.forEach((value, columnIndex) => {
    expect(Math.abs(value - expected[rowIndex][columnIndex])).toBeLessThanOrEqual(tolerance)
  }))
}

function expectVectorClose(actual: Vector3, expected: Vector3, tolerance: number): void {
  actual.forEach((value, index) => {
    expect(Math.abs(value - expected[index])).toBeLessThanOrEqual(tolerance)
  })
}

describe('homogeneous transforms', () => {
  it('reproduces the general nonzero D–H reference transform', () => {
    expectMatrixClose(dhTransform(Math.PI / 2, 2, Math.PI / 2, 3), [
      [0, 0, 1, 0],
      [1, 0, 0, 2],
      [0, 1, 0, 3],
      [0, 0, 0, 1],
    ], 1e-12)
  })

  it('composes rotations and translations when transforming a point', () => {
    const transform = multiply4(
      translation(1, 2, 3),
      multiply4(rotationZ(Math.PI / 2), rotationX(Math.PI / 2)),
    )

    expectVectorClose(transformPoint(transform, [0, 1, 0]), [1, 2, 4], 1e-12)
    expectVectorClose(translationOf(transform), [1, 2, 3], 1e-12)
    expect(rotationOf(transform)).toEqual([
      [transform[0][0], transform[0][1], transform[0][2]],
      [transform[1][0], transform[1][1], transform[1][2]],
      [transform[2][0], transform[2][1], transform[2][2]],
    ])
  })

  it('does not clean tiny values during calculations', () => {
    const tinyAngle = 1e-15

    expect(rotationZ(tinyAngle)[1][0]).toBe(Math.sin(tinyAngle))
    expect(rotationZ(tinyAngle)[1][0]).not.toBe(0)
  })
})
