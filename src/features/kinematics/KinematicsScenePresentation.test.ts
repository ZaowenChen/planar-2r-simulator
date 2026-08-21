import { describe, expect, it } from 'vitest'
import { DEFAULT_JOINT_STATE, DEFAULT_ROBOT_PARAMETERS } from '../../robotics/defaults'
import { forwardKinematics } from '../../robotics/kinematics'
import { buildKinematicsDerivation } from './derivationModel'
import { buildKinematicsScenePresentation } from './KinematicsScenePresentation'
import {
  INITIAL_KINEMATICS_TEACHING_STATE,
  type KinematicsTeachingState,
} from './teachingState'

function makePresentation(patch: Partial<KinematicsTeachingState>) {
  const forward = forwardKinematics(
    DEFAULT_JOINT_STATE.q,
    DEFAULT_ROBOT_PARAMETERS,
  )
  const target = forward.endEffectorPosition
  return buildKinematicsScenePresentation({
    derivation: buildKinematicsDerivation(
      DEFAULT_JOINT_STATE.q,
      DEFAULT_ROBOT_PARAMETERS,
      target,
    ),
    forward,
    parameters: DEFAULT_ROBOT_PARAMETERS,
    target,
    teaching: { ...INITIAL_KINEMATICS_TEACHING_STATE, ...patch },
  })
}

describe('buildKinematicsScenePresentation', () => {
  it('merges coincident world/base and frame-3/tool coordinate systems', () => {
    const presentation = makePresentation({ frameMode: 'all', stepIndex: 1 })

    expect(presentation.frames.map((frame) => frame.label)).toEqual([
      '{W}/{0}', '{1}', '{2}', '{3}/{e}',
    ])
    expect(presentation.frames.every((frame) => frame.visible)).toBe(true)
    expect(presentation.note).toContain('世界坐标系 {W} 与基座坐标系 {0} 重合')
  })

  it('builds the work-plane projection with r, h, target, and teaching camera', () => {
    const presentation = makePresentation({
      cameraPreset: 'work-plane',
      stepIndex: 6,
      symbolFocus: 'r',
    })

    expect(presentation.camera.id).toBe('work-plane')
    expect(presentation.workPlane).not.toBeNull()
    expect(presentation.points.map((point) => point.id)).toContain('target-position')
    expect(presentation.dimensions.map((item) => item.id)).toEqual(['r', 'h'])
    expect(presentation.dimensions.find((item) => item.id === 'r')?.emphasized).toBe(true)
    expect(presentation.dimensions.find((item) => item.id === 'h')?.emphasized).toBe(false)
  })

  it('shows only the selected solid pose and the elbow-angle geometry at the elbow step', () => {
    const presentation = makePresentation({
      activeConfigurationId: 'conventional:elbow-down',
      stepIndex: 9,
    })

    expect(presentation.hideBaseRobot).toBe(true)
    expect(presentation.primaryRobot).not.toBeNull()
    expect(presentation.primaryRobot?.label).toBeUndefined()
    expect(presentation.ghostRobots).toHaveLength(0)
    expect(presentation.dimensions.map((item) => item.id)).toEqual(['l2', 'l3'])
    expect(presentation.arcs.map((arc) => arc.id)).toContain('theta3')
  })

  it('visualizes the selected D–H operation and keeps only adjacent frames emphasized', () => {
    const presentation = makePresentation({
      dhOperation: 'tx',
      frameMode: 'current',
      selectedDhRow: 1,
      stepIndex: 2,
    })

    expect(presentation.dimensions.map((item) => item.id)).toContain('dh-a-2')
    expect(presentation.frames.find((frame) => frame.id === 'frame-1')?.visible).toBe(false)
    expect(presentation.frames.find((frame) => frame.id === 'frame-2')?.label).toContain('构造中')
    expect(presentation.frames.find((frame) => frame.id === 'frame-2')?.opacity).toBe(1)
    expect(presentation.frames.find((frame) => frame.id === 'frame-2')?.visibleAxes).toEqual(['x'])
    expect(presentation.frames.find((frame) => frame.id === 'frame-w0')?.visible).toBe(false)
    expect(presentation.frames.find((frame) => frame.id === 'dh-stage-previous-2')).toMatchObject({
      detail: 'name',
      opacity: 0.24,
      visible: true,
      visibleAxes: [],
    })
  })

  it('keeps D–H construction labels to the active frame and one weak related frame', () => {
    const presentation = makePresentation({ selectedDhRow: 0, stepIndex: 1 })

    expect(presentation.dimensions.every((item) => item.showLabel === false)).toBe(true)
    expect(presentation.points.every((point) => point.label === undefined)).toBe(true)
    expect(presentation.frames.filter((frame) => frame.visible)).toHaveLength(2)
  })

  it('adds target, FK point, and true residual label for back-substitution', () => {
    const presentation = makePresentation({ stepIndex: 14 })

    expect(presentation.points.map((point) => point.id)).toEqual([
      'target-position', 'fk-position',
    ])
    expect(presentation.dimensions.find((item) => item.id === 'position-error')?.label)
      .toContain('已放大，仅用于观察')
  })

  it('shows only the selected tool frame when validating inverse-solution attitude', () => {
    const presentation = makePresentation({ stepIndex: 15 })

    expect(presentation.primaryRobot).not.toBeNull()
    expect(presentation.ghostRobots).toHaveLength(0)
    expect(presentation.frames.filter((frame) => frame.visible)).toHaveLength(1)
    expect(presentation.frames.find((frame) => frame.visible)?.detail).toBe('axes')
  })

  it('shows l2, l3, and s without angle clutter for the reachability triangle', () => {
    const presentation = makePresentation({ stepIndex: 8 })

    expect(presentation.dimensions.map((item) => item.id)).toEqual(['l2', 'l3', 's'])
    expect(presentation.arcs).toHaveLength(0)
    expect(presentation.ghostRobots).toHaveLength(0)
  })

  it('compares one solid radial family against one unlabeled ghost only at the configuration step', () => {
    const presentation = makePresentation({ stepIndex: 13 })

    expect(presentation.primaryRobot?.label).toContain('肘下')
    expect(presentation.ghostRobots).toHaveLength(1)
    expect(presentation.ghostRobots[0].label).toBeUndefined()
    expect(presentation.dimensions.map((item) => item.id)).toEqual(['r'])
  })

  it('renders one selected Jacobian column at a time', () => {
    const presentation = makePresentation({ selectedJacobianColumn: 1, stepIndex: 17 })

    expect(presentation.dimensions.map((item) => item.id)).toEqual([
      'jacobian-axis-1',
      'jacobian-offset-1',
      'jacobian-tangent-1',
    ])
    expect(presentation.points.map((point) => point.id)).toEqual(['jacobian-origin-1'])
  })

  it('maps current joint motion to normalized teaching vectors with true displayed values', () => {
    const presentation = makePresentation({ selectedJacobianColumn: 1, stepIndex: 18 })

    expect(presentation.vectors.map((vector) => vector.id)).toEqual([
      'end-linear-velocity',
      'end-angular-velocity',
      'joint-2-linear-contribution',
    ])
    expect(presentation.note).toContain('真实量值请查看右侧公式')
  })
})
