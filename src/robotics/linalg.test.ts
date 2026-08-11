import { describe, expect, expectTypeOf, it } from 'vitest'
import {
  add3,
  cross3,
  determinant3,
  dot3,
  frobeniusNorm,
  inverse3,
  matMul,
  matVec,
  nearlyEqual,
  norm3,
  scale3,
  subtract3,
  symmetricEigenvalues3,
  transpose,
} from './linalg'
import type { Matrix3, Matrix4, Vector3 } from './types'

const IDENTITY_3: Matrix3 = [
  [1, 0, 0],
  [0, 1, 0],
  [0, 0, 1],
]

const IDENTITY_4: Matrix4 = [
  [1, 0, 0, 0],
  [0, 1, 0, 0],
  [0, 0, 1, 0],
  [0, 0, 0, 1],
]

describe('small linear algebra', () => {
  it('keeps Vector3 arithmetic immutable and tuple-typed', () => {
    const left: Vector3 = [1, 2, 3]
    const right: Vector3 = [4, -2, 1]

    expect(add3(left, right)).toEqual([5, 0, 4])
    expect(subtract3(left, right)).toEqual([-3, 4, 2])
    expect(scale3(left, 2)).toEqual([2, 4, 6])
    expect(left).toEqual([1, 2, 3])
    expectTypeOf(add3(left, right)).toEqualTypeOf<Vector3>()
  })

  it('computes dot products and Euclidean norms', () => {
    expect(dot3([1, 2, 3], [4, -2, 1])).toBe(3)
    expect(norm3([2, -3, 6])).toBe(7)
  })

  it('computes a right-handed cross product', () => {
    expect(cross3([1, 0, 0], [0, 1, 0])).toEqual([0, 0, 1])
  })

  it('multiplies matrices by the identity without changing the input', () => {
    const matrix: Matrix3 = [
      [2, -1, 4],
      [0, 3, 5],
      [7, 2, 1],
    ]

    expect(matMul(IDENTITY_3, matrix)).toEqual(matrix)
    expect(matMul(matrix, IDENTITY_3)).toEqual(matrix)
    expect(matrix[0]).toEqual([2, -1, 4])
    expectTypeOf(matMul(IDENTITY_3, matrix)).toEqualTypeOf<Matrix3>()
    expectTypeOf(matMul(IDENTITY_4, IDENTITY_4)).toEqualTypeOf<Matrix4>()
  })

  it('multiplies a matrix by a vector', () => {
    expect(matVec([[1, 2, 3], [4, 5, 6]], [2, 0, -1])).toEqual([-1, 2])
    expectTypeOf(matVec(IDENTITY_3, [1, 2, 3])).toEqualTypeOf<Vector3>()
  })

  it('transposes rectangular matrices', () => {
    expect(transpose([[1, 2, 3], [4, 5, 6]])).toEqual([
      [1, 4],
      [2, 5],
      [3, 6],
    ])
    expectTypeOf(transpose(IDENTITY_3)).toEqualTypeOf<Matrix3>()
    expectTypeOf(transpose(IDENTITY_4)).toEqualTypeOf<Matrix4>()
  })

  it('computes a 3x3 determinant', () => {
    expect(determinant3([[6, 1, 1], [4, -2, 5], [2, 8, 7]])).toBe(-306)
  })

  it('inverts a nonsingular 3x3 matrix', () => {
    const matrix: Matrix3 = [[4, 7, 2], [3, 6, 1], [2, 5, 1]]
    const inverse = inverse3(matrix)
    const expected: Matrix3 = [
      [1 / 3, 1, -5 / 3],
      [-1 / 3, 0, 2 / 3],
      [1, -2, 1],
    ]

    inverse.forEach((row, rowIndex) => row.forEach((value, columnIndex) => {
      expect(value).toBeCloseTo(expected[rowIndex][columnIndex], 12)
    }))
  })

  it('returns sorted eigenvalues for a symmetric 3x3 matrix', () => {
    const eigenvalues = symmetricEigenvalues3([[2, 1, 0], [1, 2, 0], [0, 0, 5]])

    expect(eigenvalues[0]).toBeCloseTo(1, 12)
    expect(eigenvalues[1]).toBeCloseTo(3, 12)
    expect(eigenvalues[2]).toBeCloseTo(5, 12)
  })

  it('computes the Frobenius norm', () => {
    expect(frobeniusNorm([[1, 2], [2, 4]])).toBe(5)
  })

  it('compares floating-point values with an explicit tolerance', () => {
    expect(nearlyEqual(0.1 + 0.2, 0.3)).toBe(true)
    expect(nearlyEqual(1, 1.01, 1e-3)).toBe(false)
  })

  it('rejects incompatible or malformed matrix dimensions descriptively', () => {
    expect(() => matMul([[1, 2]], [[1, 2]])).toThrow(/dimensions/i)
    expect(() => transpose([[1, 2], [3]])).toThrow(/rectangular/i)
  })

  it('rejects singular inversion descriptively', () => {
    expect(() => inverse3([[1, 2, 3], [2, 4, 6], [0, 1, 1]])).toThrow(/singular/i)
  })

  it('does not mistake a uniformly small invertible matrix for a singular one', () => {
    const inverse = inverse3([[1e-12, 0, 0], [0, 2e-12, 0], [0, 0, 4e-12]])

    expect(inverse[0][0]).toBeCloseTo(1e12, 2)
    expect(inverse[1][1]).toBeCloseTo(5e11, 2)
    expect(inverse[2][2]).toBeCloseTo(2.5e11, 2)
  })
})
