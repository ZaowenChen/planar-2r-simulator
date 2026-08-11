import { determinant, EigenvalueDecomposition, inverse, Matrix } from 'ml-matrix'
import type { Matrix3, Matrix4, Vector3 } from './types'

export type NumericMatrix = readonly (readonly number[])[]

interface MatrixDimensions {
  rows: number
  columns: number
}

function matrixDimensions(matrix: NumericMatrix, name: string): MatrixDimensions {
  if (matrix.length === 0) {
    throw new RangeError(`${name} dimensions must include at least one row`)
  }

  const columns = matrix[0].length
  if (columns === 0) {
    throw new RangeError(`${name} dimensions must include at least one column`)
  }
  if (matrix.some((row) => row.length !== columns)) {
    throw new RangeError(`${name} must be rectangular`)
  }

  return { rows: matrix.length, columns }
}

function requireFiniteMatrix(matrix: NumericMatrix, name: string): MatrixDimensions {
  const dimensions = matrixDimensions(matrix, name)
  if (matrix.some((row) => row.some((value) => !Number.isFinite(value)))) {
    throw new RangeError(`${name} must contain only finite values`)
  }
  return dimensions
}

function requireMatrix3(matrix: NumericMatrix, name: string): asserts matrix is Matrix3 {
  const { rows, columns } = requireFiniteMatrix(matrix, name)
  if (rows !== 3 || columns !== 3) {
    throw new RangeError(`${name} dimensions must be 3x3`)
  }
}

function toMatrix(matrix: NumericMatrix, name: string): Matrix {
  requireFiniteMatrix(matrix, name)
  return new Matrix(matrix.map((row) => Array.from(row)))
}

function toMatrix3(matrix: Matrix): Matrix3 {
  const values = matrix.to2DArray()
  return [
    [values[0][0], values[0][1], values[0][2]],
    [values[1][0], values[1][1], values[1][2]],
    [values[2][0], values[2][1], values[2][2]],
  ]
}

export function add3(left: Vector3, right: Vector3): Vector3 {
  return [left[0] + right[0], left[1] + right[1], left[2] + right[2]]
}

export function subtract3(left: Vector3, right: Vector3): Vector3 {
  return [left[0] - right[0], left[1] - right[1], left[2] - right[2]]
}

export function scale3(vector: Vector3, scalar: number): Vector3 {
  return [vector[0] * scalar, vector[1] * scalar, vector[2] * scalar]
}

export function dot3(left: Vector3, right: Vector3): number {
  return left[0] * right[0] + left[1] * right[1] + left[2] * right[2]
}

export function cross3(left: Vector3, right: Vector3): Vector3 {
  return [
    left[1] * right[2] - left[2] * right[1],
    left[2] * right[0] - left[0] * right[2],
    left[0] * right[1] - left[1] * right[0],
  ]
}

export function norm3(vector: Vector3): number {
  return Math.hypot(vector[0], vector[1], vector[2])
}

export function matMul(left: Matrix3, right: Matrix3): Matrix3
export function matMul(left: Matrix4, right: Matrix4): Matrix4
export function matMul(left: NumericMatrix, right: NumericMatrix): readonly (readonly number[])[]
export function matMul(left: NumericMatrix, right: NumericMatrix): readonly (readonly number[])[] {
  const leftDimensions = requireFiniteMatrix(left, 'left matrix')
  const rightDimensions = requireFiniteMatrix(right, 'right matrix')
  if (leftDimensions.columns !== rightDimensions.rows) {
    throw new RangeError(
      `Matrix dimensions are incompatible for multiplication: `
      + `${leftDimensions.rows}x${leftDimensions.columns} and `
      + `${rightDimensions.rows}x${rightDimensions.columns}`,
    )
  }

  return new Matrix(left.map((row) => Array.from(row)))
    .mmul(new Matrix(right.map((row) => Array.from(row))))
    .to2DArray()
}

export function matVec(matrix: Matrix3, vector: Vector3): Vector3
export function matVec(matrix: NumericMatrix, vector: readonly number[]): readonly number[]
export function matVec(matrix: NumericMatrix, vector: readonly number[]): readonly number[] {
  const dimensions = requireFiniteMatrix(matrix, 'matrix')
  if (vector.length !== dimensions.columns) {
    throw new RangeError(
      `Matrix and vector dimensions are incompatible: `
      + `${dimensions.rows}x${dimensions.columns} and ${vector.length}`,
    )
  }
  if (vector.some((value) => !Number.isFinite(value))) {
    throw new RangeError('vector must contain only finite values')
  }

  return new Matrix(matrix.map((row) => Array.from(row)))
    .mmul(Matrix.columnVector(Array.from(vector)))
    .getColumn(0)
}

export function transpose(matrix: Matrix3): Matrix3
export function transpose(matrix: Matrix4): Matrix4
export function transpose(matrix: NumericMatrix): readonly (readonly number[])[]
export function transpose(matrix: NumericMatrix): readonly (readonly number[])[] {
  return toMatrix(matrix, 'matrix').transpose().to2DArray()
}

export function determinant3(matrix: Matrix3): number {
  requireMatrix3(matrix, 'matrix')
  return determinant(new Matrix(matrix.map((row) => Array.from(row))))
}

export function inverse3(matrix: Matrix3): Matrix3 {
  requireMatrix3(matrix, 'matrix')
  const matrixNorm = frobeniusNorm(matrix)
  const singularityThreshold = Number.EPSILON * matrixNorm ** 3
  if (Math.abs(determinant3(matrix)) <= singularityThreshold) {
    throw new RangeError('Cannot invert a singular 3x3 matrix')
  }

  return toMatrix3(inverse(new Matrix(matrix.map((row) => Array.from(row)))))
}

export function symmetricEigenvalues3(matrix: Matrix3): Vector3 {
  requireMatrix3(matrix, 'matrix')
  const symmetryTolerance = 1e-10
  for (let row = 0; row < 3; row += 1) {
    for (let column = row + 1; column < 3; column += 1) {
      if (Math.abs(matrix[row][column] - matrix[column][row]) > symmetryTolerance) {
        throw new RangeError('Symmetric eigenvalues require a symmetric 3x3 matrix')
      }
    }
  }

  const decomposition = new EigenvalueDecomposition(
    new Matrix(matrix.map((row) => Array.from(row))),
    { assumeSymmetric: true },
  )
  const eigenvalues = decomposition.realEigenvalues.sort((left, right) => left - right)
  return [eigenvalues[0], eigenvalues[1], eigenvalues[2]]
}

export function frobeniusNorm(matrix: NumericMatrix): number {
  return toMatrix(matrix, 'matrix').norm('frobenius')
}

export function nearlyEqual(left: number, right: number, tolerance = 1e-9): boolean {
  if (!Number.isFinite(tolerance) || tolerance < 0) {
    throw new RangeError('Tolerance must be a finite nonnegative number')
  }
  if (left === right) {
    return true
  }
  return Number.isFinite(left) && Number.isFinite(right) && Math.abs(left - right) <= tolerance
}
