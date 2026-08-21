import { cross3, singularValues, subtract3 } from './linalg'
import { forwardKinematics } from './kinematics'
import type { Matrix6x3, RobotParameters, Vector3 } from './types'

/** Position-manipulation singularity metrics computed from the Jv block. */
export interface SingularityMetrics {
  singularValues: Vector3
  minimumSingularValue: number
  conditionNumber: number
  inverseConditionNumber: number
  yoshikawaManipulability: number
  isSingular: boolean
}

export function geometricJacobian(
  q: Vector3,
  parameters: RobotParameters,
): Matrix6x3 {
  const kinematics = forwardKinematics(q, parameters)
  const endEffector = kinematics.endEffectorPosition
  const jointOrigins = kinematics.origins.slice(0, 3)
  const translationalColumns = kinematics.jointAxes.map((axis, index) => (
    cross3(axis, subtract3(endEffector, jointOrigins[index]))
  ))

  return [
    [translationalColumns[0][0], translationalColumns[1][0], translationalColumns[2][0]],
    [translationalColumns[0][1], translationalColumns[1][1], translationalColumns[2][1]],
    [translationalColumns[0][2], translationalColumns[1][2], translationalColumns[2][2]],
    [kinematics.jointAxes[0][0], kinematics.jointAxes[1][0], kinematics.jointAxes[2][0]],
    [kinematics.jointAxes[0][1], kinematics.jointAxes[1][1], kinematics.jointAxes[2][1]],
    [kinematics.jointAxes[0][2], kinematics.jointAxes[1][2], kinematics.jointAxes[2][2]],
  ]
}

export function singularityMetrics(
  jacobian: Matrix6x3,
  threshold: number,
): SingularityMetrics {
  if (!Number.isFinite(threshold) || threshold < 0) {
    throw new RangeError('Singularity threshold must be a finite nonnegative number')
  }
  if (jacobian.some((row) => row.some((value) => !Number.isFinite(value)))) {
    throw new RangeError('Jacobian must contain only finite values')
  }

  // Position IK uses Jv; including Jw would hide straight-arm and base-axis
  // position singularities because the angular columns remain independent.
  const values = singularValues(jacobian.slice(0, 3))
  const sortedValues: Vector3 = [values[0], values[1], values[2]]
  const maximumSingularValue = sortedValues[0]
  const minimumSingularValue = sortedValues[2]
  const numericalZero = Number.EPSILON * 3 * maximumSingularValue
  const numericallySingular = maximumSingularValue === 0
    || minimumSingularValue <= numericalZero

  return {
    singularValues: sortedValues,
    minimumSingularValue,
    conditionNumber: numericallySingular
      ? Number.POSITIVE_INFINITY
      : maximumSingularValue / minimumSingularValue,
    inverseConditionNumber: numericallySingular
      ? 0
      : minimumSingularValue / maximumSingularValue,
    // For the square translational Jacobian this is equivalent to
    // sqrt(det(Jv Jv^T)), but the singular-value product is more stable.
    yoshikawaManipulability: sortedValues.reduce(
      (product, value) => product * value,
      1,
    ),
    isSingular: minimumSingularValue <= threshold,
  }
}
