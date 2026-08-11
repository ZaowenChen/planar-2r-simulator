import { describe, expect, expectTypeOf, it } from 'vitest'
import { DEFAULT_JOINT_STATE, DEFAULT_ROBOT_PARAMETERS } from './defaults'
import type { JointState, RobotParameters } from './types'
import { validateRobotParameters } from './validation'

describe('documented scientific defaults', () => {
  it('encodes the approved geometry, rigid-body, environment, and limit values', () => {
    expect(DEFAULT_ROBOT_PARAMETERS).toMatchObject({
      geometry: { d1: 0.8, l2: 2, l3: 1.5 },
      gravity: [0, 0, -9.81],
      viscousFriction: [0.08, 0.08, 0.05],
      frictionEnabled: true,
      jointLimits: [
        [-Math.PI, Math.PI],
        [-Math.PI / 2, Math.PI / 2],
        [-5 * Math.PI / 6, 5 * Math.PI / 6],
      ],
      links: [
        { mass: 4, centerOfMass: [0, -0.4, 0], inertia: [[0.2133, 0, 0], [0, 0.02, 0], [0, 0, 0.2133]] },
        { mass: 3, centerOfMass: [-1, 0, 0], inertia: [[0.03, 0, 0], [0, 1, 0], [0, 0, 1]] },
        { mass: 2, centerOfMass: [-0.75, 0, 0], inertia: [[0.02, 0, 0], [0, 0.375, 0], [0, 0, 0.375]] },
      ],
    })
    expectTypeOf(DEFAULT_ROBOT_PARAMETERS).toMatchTypeOf<RobotParameters>()
  })

  it('starts at the documented pose in radians with zero rates', () => {
    expect(DEFAULT_JOINT_STATE.q).toEqual([Math.PI / 6, 5 * Math.PI / 36, -5 * Math.PI / 18])
    expect(DEFAULT_JOINT_STATE.qd).toEqual([0, 0, 0])
    expect(DEFAULT_JOINT_STATE.qdd).toEqual([0, 0, 0])
    expectTypeOf(DEFAULT_JOINT_STATE).toMatchTypeOf<JointState>()
  })

  it('deep-freezes exported defaults in development', () => {
    expect(Object.isFrozen(DEFAULT_ROBOT_PARAMETERS)).toBe(true)
    expect(Object.isFrozen(DEFAULT_ROBOT_PARAMETERS.links[0].inertia[0])).toBe(true)
    expect(Object.isFrozen(DEFAULT_JOINT_STATE.q)).toBe(true)
  })
})

describe('robot parameter validation', () => {
  it('accepts the documented teaching parameters', () => {
    expect(validateRobotParameters(DEFAULT_ROBOT_PARAMETERS)).toEqual([])
  })

  it('identifies a nonphysical inertia tensor by field and link', () => {
    const invalid = structuredClone(DEFAULT_ROBOT_PARAMETERS)
    invalid.links[1].inertia = [[0.01, 0, 0], [0, 5, 0], [0, 0, 0.01]]
    expect(validateRobotParameters(invalid)).toContainEqual(expect.objectContaining({
      path: 'links.1.inertia', code: 'INERTIA_TRIANGLE',
    }))
  })

  it('rejects a center of mass outside its nominal link sphere', () => {
    const invalid = structuredClone(DEFAULT_ROBOT_PARAMETERS)
    invalid.links[2].centerOfMass = [-2, 0, 0]
    expect(validateRobotParameters(invalid)).toContainEqual(expect.objectContaining({
      path: 'links.2.centerOfMass', code: 'CENTER_OF_MASS_RANGE',
    }))
  })

  it('validates base height, link lengths, and masses independently', () => {
    const invalid = structuredClone(DEFAULT_ROBOT_PARAMETERS)
    invalid.geometry.d1 = -1
    invalid.geometry.l2 = 0
    invalid.geometry.l3 = Number.POSITIVE_INFINITY
    invalid.links[0].mass = Number.NaN
    invalid.links[1].mass = -2

    expect(validateRobotParameters(invalid)).toEqual(expect.arrayContaining([
      expect.objectContaining({ path: 'geometry.d1', code: 'BASE_HEIGHT_RANGE' }),
      expect.objectContaining({ path: 'geometry.l2', code: 'LINK_LENGTH_RANGE' }),
      expect.objectContaining({ path: 'geometry.l3', code: 'LINK_LENGTH_RANGE' }),
      expect.objectContaining({ path: 'links.0.mass', code: 'MASS_RANGE' }),
      expect.objectContaining({ path: 'links.1.mass', code: 'MASS_RANGE' }),
    ]))
  })

  it('rejects nonfinite centers, gravity, and inertia components', () => {
    const invalid = structuredClone(DEFAULT_ROBOT_PARAMETERS)
    invalid.links[0].centerOfMass = [Number.NaN, 0, 0]
    invalid.links[1].inertia = [[Number.POSITIVE_INFINITY, 0, 0], [0, 1, 0], [0, 0, 1]]
    invalid.gravity = [0, Number.NEGATIVE_INFINITY, -9.81]

    expect(validateRobotParameters(invalid)).toEqual(expect.arrayContaining([
      expect.objectContaining({ path: 'links.0.centerOfMass', code: 'CENTER_OF_MASS_NONFINITE' }),
      expect.objectContaining({ path: 'links.1.inertia', code: 'INERTIA_NONFINITE' }),
      expect.objectContaining({ path: 'gravity', code: 'GRAVITY_NONFINITE' }),
    ]))
  })

  it('rejects asymmetric and non-positive-definite inertia tensors', () => {
    const invalid = structuredClone(DEFAULT_ROBOT_PARAMETERS)
    invalid.links[0].inertia = [[1, 1e-8, 0], [0, 1, 0], [0, 0, 1]]
    invalid.links[2].inertia = [[1, 0, 0], [0, 1, 0], [0, 0, 1e-9]]

    expect(validateRobotParameters(invalid)).toEqual(expect.arrayContaining([
      expect.objectContaining({ path: 'links.0.inertia', code: 'INERTIA_ASYMMETRIC' }),
      expect.objectContaining({ path: 'links.2.inertia', code: 'INERTIA_NOT_POSITIVE_DEFINITE' }),
    ]))
  })

  it('rejects negative or nonfinite viscous friction and reports every component', () => {
    const invalid = structuredClone(DEFAULT_ROBOT_PARAMETERS)
    invalid.viscousFriction = [-0.1, Number.NaN, Number.POSITIVE_INFINITY]

    expect(validateRobotParameters(invalid)).toEqual(expect.arrayContaining([
      expect.objectContaining({ path: 'viscousFriction.0', code: 'FRICTION_RANGE' }),
      expect.objectContaining({ path: 'viscousFriction.1', code: 'FRICTION_RANGE' }),
      expect.objectContaining({ path: 'viscousFriction.2', code: 'FRICTION_RANGE' }),
    ]))
  })

  it('accepts inertia symmetry at 1e-10 and rejects values just above it', () => {
    const atTolerance = structuredClone(DEFAULT_ROBOT_PARAMETERS)
    atTolerance.links[0].inertia = [[1, 1e-10, 0], [0, 1, 0], [0, 0, 1]]
    const aboveTolerance = structuredClone(atTolerance)
    aboveTolerance.links[0].inertia = [[1, 1.0001e-10, 0], [0, 1, 0], [0, 0, 1]]

    expect(validateRobotParameters(atTolerance)).not.toContainEqual(expect.objectContaining({
      path: 'links.0.inertia', code: 'INERTIA_ASYMMETRIC',
    }))
    expect(validateRobotParameters(aboveTolerance)).toContainEqual(expect.objectContaining({
      path: 'links.0.inertia', code: 'INERTIA_ASYMMETRIC',
    }))
  })

  it('requires the minimum principal inertia to be strictly above 1e-9', () => {
    const atThreshold = structuredClone(DEFAULT_ROBOT_PARAMETERS)
    atThreshold.links[0].inertia = [[1e-9, 0, 0], [0, 1, 0], [0, 0, 1]]
    const aboveThreshold = structuredClone(atThreshold)
    aboveThreshold.links[0].inertia = [[1.001e-9, 0, 0], [0, 1, 0], [0, 0, 1]]

    expect(validateRobotParameters(atThreshold)).toContainEqual(expect.objectContaining({
      path: 'links.0.inertia', code: 'INERTIA_NOT_POSITIVE_DEFINITE',
    }))
    expect(validateRobotParameters(aboveThreshold)).not.toContainEqual(expect.objectContaining({
      path: 'links.0.inertia', code: 'INERTIA_NOT_POSITIVE_DEFINITE',
    }))
  })

  it('allows triangle excess at 1e-9 and rejects values just above it', () => {
    const atTolerance = structuredClone(DEFAULT_ROBOT_PARAMETERS)
    atTolerance.links[0].inertia = [[1, 0, 0], [0, 1, 0], [0, 0, 2 + 1e-9]]
    const aboveTolerance = structuredClone(atTolerance)
    aboveTolerance.links[0].inertia = [[1, 0, 0], [0, 1, 0], [0, 0, 2 + 1.0001e-9]]

    expect(validateRobotParameters(atTolerance)).not.toContainEqual(expect.objectContaining({
      path: 'links.0.inertia', code: 'INERTIA_TRIANGLE',
    }))
    expect(validateRobotParameters(aboveTolerance)).toContainEqual(expect.objectContaining({
      path: 'links.0.inertia', code: 'INERTIA_TRIANGLE',
    }))
  })
})
