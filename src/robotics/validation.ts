import { norm3, symmetricEigenvalues3 } from './linalg'
import type { Matrix3, RobotParameters, ValidationIssue } from './types'

const SYMMETRY_TOLERANCE = 1e-10
const MINIMUM_INERTIA = 1e-9
const TRIANGLE_TOLERANCE = 1e-9

function issue(path: string, code: string, message: string): ValidationIssue {
  return { path, code, message }
}

function isFiniteMatrix3(matrix: Matrix3): boolean {
  return matrix.every((row) => row.every(Number.isFinite))
}

function isSymmetric(matrix: Matrix3): boolean {
  return Math.abs(matrix[0][1] - matrix[1][0]) <= SYMMETRY_TOLERANCE
    && Math.abs(matrix[0][2] - matrix[2][0]) <= SYMMETRY_TOLERANCE
    && Math.abs(matrix[1][2] - matrix[2][1]) <= SYMMETRY_TOLERANCE
}

export function validateRobotParameters(parameters: RobotParameters): ValidationIssue[] {
  const issues: ValidationIssue[] = []
  const { d1, l2, l3 } = parameters.geometry

  if (!Number.isFinite(d1) || d1 < 0) {
    issues.push(issue('geometry.d1', 'BASE_HEIGHT_RANGE', 'Base height must be finite and nonnegative.'))
  }
  for (const [name, length] of [['l2', l2], ['l3', l3]] as const) {
    if (!Number.isFinite(length) || length <= 0) {
      issues.push(issue(`geometry.${name}`, 'LINK_LENGTH_RANGE', 'Link length must be finite and positive.'))
    }
  }

  const centerOfMassBounds = [d1, l2, l3] as const
  parameters.links.forEach((link, linkIndex) => {
    const massPath = `links.${linkIndex}.mass`
    if (!Number.isFinite(link.mass) || link.mass <= 0) {
      issues.push(issue(massPath, 'MASS_RANGE', 'Link mass must be finite and positive.'))
    }

    const centerPath = `links.${linkIndex}.centerOfMass`
    if (link.centerOfMass.some((component) => !Number.isFinite(component))) {
      issues.push(issue(centerPath, 'CENTER_OF_MASS_NONFINITE', 'Center of mass must contain only finite values.'))
    } else {
      const bound = centerOfMassBounds[linkIndex]
      if (Number.isFinite(bound) && bound >= 0 && norm3(link.centerOfMass) > bound) {
        issues.push(issue(centerPath, 'CENTER_OF_MASS_RANGE', 'Center of mass must lie within the nominal link sphere.'))
      }
    }

    const inertiaPath = `links.${linkIndex}.inertia`
    if (!isFiniteMatrix3(link.inertia)) {
      issues.push(issue(inertiaPath, 'INERTIA_NONFINITE', 'Inertia tensor must contain only finite values.'))
      return
    }
    if (!isSymmetric(link.inertia)) {
      issues.push(issue(inertiaPath, 'INERTIA_ASYMMETRIC', 'Inertia tensor must be symmetric within 1e-10.'))
      return
    }

    const principalInertias = symmetricEigenvalues3(link.inertia)
    if (principalInertias[0] <= MINIMUM_INERTIA) {
      issues.push(issue(
        inertiaPath,
        'INERTIA_NOT_POSITIVE_DEFINITE',
        'Every principal inertia must be greater than 1e-9.',
      ))
    }
    if (principalInertias[2] > principalInertias[0] + principalInertias[1] + TRIANGLE_TOLERANCE) {
      issues.push(issue(
        inertiaPath,
        'INERTIA_TRIANGLE',
        'Principal inertias must satisfy the triangle inequalities within 1e-9.',
      ))
    }
  })

  if (parameters.gravity.some((component) => !Number.isFinite(component))) {
    issues.push(issue('gravity', 'GRAVITY_NONFINITE', 'Gravity must contain only finite values.'))
  }

  parameters.viscousFriction.forEach((friction, jointIndex) => {
    if (!Number.isFinite(friction) || friction < 0) {
      issues.push(issue(
        `viscousFriction.${jointIndex}`,
        'FRICTION_RANGE',
        'Viscous friction must be finite and nonnegative.',
      ))
    }
  })

  return issues
}
