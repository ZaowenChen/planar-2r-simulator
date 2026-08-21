import { describe, expect, it } from 'vitest'
import { DEFAULT_JOINT_STATE, DEFAULT_ROBOT_PARAMETERS } from '../../robotics/defaults'
import { geometricJacobian } from '../../robotics/jacobian'
import { forwardKinematics } from '../../robotics/kinematics'
import { buildKinematicsDerivation } from './derivationModel'
import {
  jacobianForMillimetresAndDegrees,
  metresToMillimetres,
  transformInMillimetres,
} from './presentation'

describe('buildKinematicsDerivation', () => {
  it('converts only presentation values to millimetres and degrees', () => {
    expect(metresToMillimetres(1.25)).toBe(1250)

    const transform = forwardKinematics(
      DEFAULT_JOINT_STATE.q,
      DEFAULT_ROBOT_PARAMETERS,
    ).transforms[2]
    const displayTransform = transformInMillimetres(transform)

    displayTransform.forEach((row, rowIndex) => row.forEach((value, columnIndex) => {
      const expected = columnIndex === 3 && rowIndex < 3
        ? transform[rowIndex][columnIndex] * 1000
        : transform[rowIndex][columnIndex]
      expect(value).toBeCloseTo(expected, 12)
    }))

    const displayJacobian = jacobianForMillimetresAndDegrees([
      [1, 2, 3],
      [4, 5, 6],
      [7, 8, 9],
      [10, 11, 12],
      [13, 14, 15],
      [16, 17, 18],
    ])
    expect(displayJacobian[0][0]).toBeCloseTo(1000 * Math.PI / 180, 12)
    expect(displayJacobian[2][2]).toBeCloseTo(9 * 1000 * Math.PI / 180, 12)
    expect(displayJacobian.slice(3)).toEqual([
      [10, 11, 12],
      [13, 14, 15],
      [16, 17, 18],
    ])
  })

  it('exposes intermediate transforms without changing the authoritative FK and Jacobian results', () => {
    const { q } = DEFAULT_JOINT_STATE
    const target = forwardKinematics(q, DEFAULT_ROBOT_PARAMETERS).endEffectorPosition
    const derivation = buildKinematicsDerivation(q, DEFAULT_ROBOT_PARAMETERS, target)

    expect(derivation.t03).toEqual(forwardKinematics(q, DEFAULT_ROBOT_PARAMETERS).transforms[2])
    expect(derivation.position).toEqual(target)
    expect(derivation.jacobian).toEqual(geometricJacobian(q, DEFAULT_ROBOT_PARAMETERS))
  })

  it('prepares the default end-effector pose for millimetre and degree presentation', () => {
    const { q } = DEFAULT_JOINT_STATE
    const target = forwardKinematics(q, DEFAULT_ROBOT_PARAMETERS).endEffectorPosition
    const derivation = buildKinematicsDerivation(q, DEFAULT_ROBOT_PARAMETERS, target)

    derivation.qDegrees.forEach((value, index) => {
      expect(value).toBeCloseTo([30, 25, -50][index], 12)
    })
    expect(derivation.toolElevationDegrees).toBeCloseTo(-25, 12)
    expect(derivation.azimuthDegrees).toBeCloseTo(30, 12)
    expect(derivation.positionMm).toEqual([
      expect.closeTo(2747.099485, 6),
      expect.closeTo(1586.038627, 6),
      expect.closeTo(1011.309131, 6),
    ])
    expect(derivation.orientation).toEqual([
      [expect.closeTo(0.784885567, 9), expect.closeTo(0.365998151, 9), expect.closeTo(0.5, 12)],
      [expect.closeTo(0.453153894, 9), expect.closeTo(0.211309131, 9), expect.closeTo(-0.866025404, 9)],
      [expect.closeTo(-0.422618262, 9), expect.closeTo(0.906307787, 9), expect.closeTo(0, 12)],
    ])
    expect(derivation.displayTransforms.t03[0][3]).toBeCloseTo(2747.099485, 6)
    expect(derivation.displayTransforms.t03[0][0]).toBeCloseTo(0.784885567, 9)
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
    expect(derivation.inverse.pythagoreanDistanceMm).toBeCloseTo(
      Math.hypot(derivation.inverse.radialMm, derivation.inverse.verticalMm),
      12,
    )
    expect(derivation.inverse.reachability).toEqual({
      minimumMm: expect.closeTo(500, 12),
      maximumMm: expect.closeTo(3500, 12),
      isReachable: true,
    })
    expect(derivation.inverse.conventionalBranches).toHaveLength(2)
    expect(derivation.inverse.candidateDetails.map((detail) => (
      `${detail.solution.radialFamily}:${detail.solution.branch}`
    ))).toEqual([
      'conventional:elbow-down',
      'conventional:elbow-up',
      'folded:elbow-down',
      'folded:elbow-up',
    ])
    derivation.inverse.conventionalBranches.forEach((detail) => {
      const [elbowR, elbowH] = detail.elbowPointMm
      const l2Mm = DEFAULT_ROBOT_PARAMETERS.geometry.l2 * 1000
      expect(elbowR).toBeCloseTo(l2Mm * Math.cos(detail.qDegrees[1] * Math.PI / 180), 9)
      expect(elbowH).toBeCloseTo(l2Mm * Math.sin(detail.qDegrees[1] * Math.PI / 180), 9)
      expect(detail.targetDirectionDegrees - detail.triangleCorrectionDegrees)
        .toBeCloseTo(detail.qDegrees[1], 10)
      expect(detail.triangleCorrectionDegrees).toBeCloseTo(
        Math.atan2(detail.triangleHeightMm, detail.triangleProjectionMm) * 180 / Math.PI,
        10,
      )
      expect(detail.positionErrorMm).toBeLessThan(1e-6)
    })
    expect(derivation.inverse.result.solutions.map((solution) => solution.branch)).toEqual([
      'elbow-down',
      'elbow-up',
    ])
    expect(derivation.inverse.shoulderToTargetMm).toBeGreaterThan(0)
    expect(derivation.inverse.solutionDetails).toHaveLength(2)
    derivation.inverse.solutionDetails.forEach((detail) => {
      expect(detail.positionErrorMm).toBeLessThan(1e-6)
      expect(detail.achievedPositionMm).toHaveLength(3)
      detail.positionResidualMm.forEach((component, index) => {
        expect(component).toBeCloseTo(
          detail.achievedPositionMm[index] - derivation.inverse.targetMm[index],
          12,
        )
      })
      expect(detail.qDegrees).toHaveLength(3)
      expect(detail.targetDirectionDegrees - detail.triangleCorrectionDegrees)
        .toBeCloseTo(detail.qDegrees[1], 10)
      expect(detail.orientation).toHaveLength(3)
      expect(detail.toolElevationDegrees).toBeCloseTo(
        detail.qDegrees[1] + detail.qDegrees[2],
        12,
      )
    })
  })

  it('maps joint velocity to a display-unit twist and preserves per-joint contributions', () => {
    const { q } = DEFAULT_JOINT_STATE
    const qd = [Math.PI / 6, -Math.PI / 9, Math.PI / 12] as const
    const target = forwardKinematics(q, DEFAULT_ROBOT_PARAMETERS).endEffectorPosition
    const derivation = buildKinematicsDerivation(q, DEFAULT_ROBOT_PARAMETERS, target, qd)

    expect(derivation.qdDegreesPerSecond).toEqual([
      expect.closeTo(30, 12),
      expect.closeTo(-20, 12),
      expect.closeTo(15, 12),
    ])

    const expectedLinear = [0, 1, 2].map((row) => (
      derivation.jacobian[row][0] * qd[0]
      + derivation.jacobian[row][1] * qd[1]
      + derivation.jacobian[row][2] * qd[2]
    ))
    const expectedAngular = [3, 4, 5].map((row) => (
      derivation.jacobian[row][0] * qd[0]
      + derivation.jacobian[row][1] * qd[1]
      + derivation.jacobian[row][2] * qd[2]
    ))

    derivation.linearVelocityMetresPerSecond.forEach((value, index) => {
      expect(value).toBeCloseTo(expectedLinear[index], 12)
      expect(derivation.linearVelocityMillimetresPerSecond[index])
        .toBeCloseTo(expectedLinear[index] * 1000, 9)
    })
    derivation.angularVelocityRadiansPerSecond.forEach((value, index) => {
      expect(value).toBeCloseTo(expectedAngular[index], 12)
      expect(derivation.angularVelocityDegreesPerSecond[index])
        .toBeCloseTo(expectedAngular[index] * 180 / Math.PI, 9)
    })

    for (const field of [
      'linearMillimetresPerSecond',
      'angularDegreesPerSecond',
    ] as const) {
      const total = derivation.velocityContributions.reduce(
        (sum, contribution) => sum.map((value, index) => (
          value + contribution[field][index]
        )),
        [0, 0, 0],
      )
      const expected = field === 'linearMillimetresPerSecond'
        ? derivation.linearVelocityMillimetresPerSecond
        : derivation.angularVelocityDegreesPerSecond
      total.forEach((value, index) => expect(value).toBeCloseTo(expected[index], 9))
    }
  })
})
