import { describe, expect, it } from 'vitest'
import type { ForwardKinematicsResult } from '../robotics/kinematics'
import type { Matrix6x3 } from '../robotics/types'
import { buildSceneModel } from './sceneModel'

const forward: ForwardKinematicsResult = {
  transforms: [
    [[0, -1, 0, 1], [1, 0, 0, 2], [0, 0, 1, 3], [0, 0, 0, 1]],
    [[1, 0, 0, 4], [0, 0, -1, 6], [0, 1, 0, 3], [0, 0, 0, 1]],
    [[0, 0, 1, 4], [1, 0, 0, 6], [0, 1, 0, 8], [0, 0, 0, 1]],
    [[0, 0, 1, 4], [1, 0, 0, 6], [0, 1, 0, 8], [0, 0, 0, 1]],
  ],
  origins: [[0, 0, 0], [1, 2, 3], [4, 6, 3], [4, 6, 8]],
  jointAxes: [[0, 0, 1], [0, -1, 0], [1, 0, 0]],
  centerOfMassPositions: [[0.5, 1, 1.5], [2.5, 4, 3], [4, 6, 5.5]],
  endEffectorPosition: [4, 6, 8],
}

const jacobian: Matrix6x3 = [
  [1, 0, 0],
  [0, 2, 0],
  [0, 0, 3],
  [0, 0, 0],
  [0, 0, 0],
  [0, 0, 0],
]

function makeModel() {
  return buildSceneModel({
    forward,
    jointState: { q: [0, 0, 0], qd: [2, 3, 4], qdd: [-1, 2, 0.5] },
    jacobian,
    torque: [5, -7, 0],
    gravity: [0, 0, -9.81],
    workspaceSamples: [[1, 1, 1], [2, 2, 2]],
    trail: [[3, 5, 7], [4, 6, 8]],
    overlays: {
      coordinateFrames: true,
      centerOfMass: true,
      workspace: true,
      trail: true,
      linearVelocity: true,
      acceleration: true,
      gravity: true,
      torque: true,
      grid: true,
    },
  })
}

describe('buildSceneModel', () => {
  it('places the four joint markers and three links from adjacent FK origins', () => {
    const model = makeModel()

    expect(model.joints.map((joint) => joint.position)).toEqual(forward.origins)
    expect(model.links.map((link) => link.midpoint)).toEqual([
      [0.5, 1, 1.5], [2.5, 4, 3], [4, 6, 5.5],
    ])
    expect(model.links[0].length).toBeCloseTo(Math.sqrt(14), 12)
    expect(model.links[1].length).toBe(5)
    expect(model.links[2].length).toBe(5)
  })

  it('derives each displayed coordinate axis from transform rotation columns', () => {
    const frames = makeModel().coordinateFrames

    expect(frames[0]).toMatchObject({
      label: '{0}',
      position: [0, 0, 0],
      axes: { x: [1, 0, 0], y: [0, 1, 0], z: [0, 0, 1] },
    })
    expect(frames[1]).toMatchObject({
      label: '{1}',
      position: [1, 2, 3],
      axes: { x: [0, 1, 0], y: [-1, 0, 0], z: [0, 0, 1] },
    })
    expect(frames[3]).toMatchObject({
      label: '{3}',
      position: [4, 6, 8],
      axes: { x: [0, 1, 0], y: [0, 0, 1], z: [1, 0, 0] },
    })
    expect(frames[4].label).toBe('{e}')
  })

  it('uses dynamics center-of-mass positions without recomputing them', () => {
    expect(makeModel().centerOfMassMarkers).toEqual([
      expect.objectContaining({ label: 'c₁', position: [0.5, 1, 1.5] }),
      expect.objectContaining({ label: 'c₂', position: [2.5, 4, 3] }),
      expect.objectContaining({ label: 'c₃', position: [4, 6, 5.5] }),
    ])
  })

  it('orients a negative torque along the signed joint axis while preserving its true value', () => {
    const torque = makeModel().vectors.find((vector) => vector.id === 'torque-2')

    expect(torque).toMatchObject({
      origin: [1, 2, 3],
      direction: [0, 1, 0],
      magnitude: 7,
      signedMagnitude: -7,
      unit: 'N·m',
      normalized: true,
    })
    expect(torque?.displayLength).toBeGreaterThan(0)
    expect(torque?.displayLength).toBeLessThanOrEqual(1.35)
  })

  it('normalizes scientific vectors for display and keeps literal magnitudes and units', () => {
    const model = makeModel()
    const velocity = model.vectors.find((vector) => vector.id === 'linear-velocity')
    const acceleration = model.vectors.find((vector) => vector.id === 'acceleration')
    const gravity = model.vectors.find((vector) => vector.id === 'gravity')

    expect(velocity).toMatchObject({ vector: [2, 6, 12], unit: 'm/s' })
    expect(velocity?.magnitude).toBeCloseTo(Math.sqrt(184), 12)
    expect(acceleration).toMatchObject({ vector: [-1, 4, 1.5], unit: 'm/s²' })
    expect(acceleration?.magnitude).toBeCloseTo(Math.sqrt(19.25), 12)
    expect(gravity).toMatchObject({ vector: [0, 0, -9.81], magnitude: 9.81, unit: 'm/s²' })
    expect(model.vectors.every((vector) => vector.displayLength <= 1.35)).toBe(true)
    expect(JSON.parse(JSON.stringify(model))).toEqual(model)
  })
})
