import { cleanup, fireEvent, render, screen } from '@testing-library/react'
import { afterEach, describe, expect, it, vi } from 'vitest'
import { DEFAULT_JOINT_STATE, DEFAULT_ROBOT_PARAMETERS } from '../robotics/defaults'
import { geometricJacobian } from '../robotics/jacobian'
import { forwardKinematics } from '../robotics/kinematics'
import { resetSceneCamera, RobotScene } from './RobotScene'

vi.mock('@react-three/fiber', async () => {
  const actual = await vi.importActual<typeof import('@react-three/fiber')>('@react-three/fiber')
  const React = await import('react')
  return {
    ...actual,
    Canvas: ({ children, ...props }: React.PropsWithChildren<Record<string, unknown>>) => {
      const child = React.Children.toArray(children).find(React.isValidElement)
      const sceneModel = child && React.isValidElement<{ sceneModel?: unknown }>(child)
        ? child.props.sceneModel
        : undefined
      return (
        <div
          data-camera={JSON.stringify(props.camera)}
          data-scene-model={JSON.stringify(sceneModel)}
          data-testid="canvas-boundary"
        />
      )
    },
  }
})

afterEach(cleanup)

function renderScene() {
  const forward = forwardKinematics(DEFAULT_JOINT_STATE.q, DEFAULT_ROBOT_PARAMETERS)
  return render(
    <RobotScene
      calculation={{
        forward,
        jacobian: geometricJacobian(DEFAULT_JOINT_STATE.q, DEFAULT_ROBOT_PARAMETERS),
        jointState: DEFAULT_JOINT_STATE,
        torque: [1, -2, 3],
        gravity: DEFAULT_ROBOT_PARAMETERS.gravity,
      }}
      trail={[[0, 0, 0], forward.endEffectorPosition]}
      workspaceSamples={[[1, 2, 3], [2, 3, 4]]}
    />,
  )
}

describe('RobotScene', () => {
  it('passes real serialized geometry to the child renderer across the Canvas boundary', () => {
    renderScene()

    const canvas = screen.getByTestId('canvas-boundary')
    const model = JSON.parse(canvas.dataset.sceneModel ?? '{}')
    expect(model.joints).toHaveLength(4)
    expect(model.links).toHaveLength(3)
    expect(model.joints[3].position).toEqual(
      forwardKinematics(DEFAULT_JOINT_STATE.q, DEFAULT_ROBOT_PARAMETERS).endEffectorPosition,
    )
    expect(model.workspace.points).toEqual([[1, 2, 3], [2, 3, 4]])
    expect(model.trail.points).toHaveLength(2)
    expect(canvas.dataset.camera).toBe(JSON.stringify({ position: [6, -7, 5], fov: 38, near: 0.1, far: 100 }))
  })

  it('offers independent scientific overlays and reports their true units outside WebGL', () => {
    renderScene()

    for (const label of ['线速度', '加速度', '重力', '关节力矩']) {
      expect(screen.getByRole('checkbox', { name: label })).not.toBeChecked()
    }

    fireEvent.click(screen.getByRole('checkbox', { name: '关节力矩' }))

    expect(screen.getByRole('checkbox', { name: '关节力矩' })).toBeChecked()
    expect(screen.getByText(/τ₂ = −2\.000 N·m/)).toBeInTheDocument()
    expect(screen.getByText('箭头长度已归一化，仅用于辨识方向。')).toBeInTheDocument()
  })

  it('restores the documented camera position, Z-up axis, and orbit target', () => {
    const cameraState = { position: [0, 0, 0], up: [1, 0, 0] }
    const orbitState = { target: [9, 9, 9], updates: 0 }

    resetSceneCamera(
      {
        position: { set: (x, y, z) => { cameraState.position = [x, y, z] } },
        up: { set: (x, y, z) => { cameraState.up = [x, y, z] } },
      },
      {
        target: { set: (x, y, z) => { orbitState.target = [x, y, z] } },
        update: () => { orbitState.updates += 1 },
      },
    )

    expect(cameraState).toEqual({ position: [6, -7, 5], up: [0, 0, 1] })
    expect(orbitState).toEqual({ target: [0, 0, 1], updates: 1 })
  })

  it('supports geometry toggles and exposes the camera-reset action', () => {
    renderScene()

    expect(screen.getByRole('checkbox', { name: '局部坐标系' })).toBeChecked()
    expect(screen.getByRole('checkbox', { name: '质心' })).toBeChecked()
    expect(screen.getByRole('checkbox', { name: '工作空间' })).not.toBeChecked()
    expect(screen.getByRole('button', { name: '复位三维视角' })).toBeInTheDocument()

    fireEvent.click(screen.getByRole('checkbox', { name: '工作空间' }))
    const model = JSON.parse(screen.getByTestId('canvas-boundary').dataset.sceneModel ?? '{}')
    expect(model.workspace.visible).toBe(true)
  })
})
