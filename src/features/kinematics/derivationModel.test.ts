import { describe, expect, it } from 'vitest'
import { DEFAULT_JOINT_STATE, DEFAULT_ROBOT_PARAMETERS } from '../../robotics/defaults'
import { geometricJacobian } from '../../robotics/jacobian'
import { forwardKinematics } from '../../robotics/kinematics'
import { buildKinematicsDerivation } from './derivationModel'

describe('buildKinematicsDerivation', () => {
  it('exposes intermediate transforms without changing the authoritative FK and Jacobian results', () => {
    const { q } = DEFAULT_JOINT_STATE
    const target = forwardKinematics(q, DEFAULT_ROBOT_PARAMETERS).endEffectorPosition
    const derivation = buildKinematicsDerivation(q, DEFAULT_ROBOT_PARAMETERS, target)

    expect(derivation.t03).toEqual(forwardKinematics(q, DEFAULT_ROBOT_PARAMETERS).transforms[2])
    expect(derivation.position).toEqual(target)
    expect(derivation.jacobian).toEqual(geometricJacobian(q, DEFAULT_ROBOT_PARAMETERS))
  })

  it('records each Jacobian cross product and both inverse-kinematics branches', () => {
    const { q } = DEFAULT_JOINT_STATE
    const target = forwardKinematics(q, DEFAULT_ROBOT_PARAMETERS).endEffectorPosition
    const derivation = buildKinematicsDerivation(q, DEFAULT_ROBOT_PARAMETERS, target)

    derivation.jacobianColumns.forEach((column, index) => {
      expect(column.linear[0]).toBeCloseTo(derivation.jacobian[0][index], 12)
      expect(column.linear[1]).toBeCloseTo(derivation.jacobian[1][index], 12)
      expect(column.linear[2]).toBeCloseTo(derivation.jacobian[2][index], 12)
    })
    expect(derivation.inverse.cosineElbow).toBeCloseTo(Math.cos(q[2]), 12)
    expect(derivation.inverse.result.solutions.map((solution) => solution.branch)).toEqual([
      'elbow-down',
      'elbow-up',
    ])
  })
})
