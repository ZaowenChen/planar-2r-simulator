import { OrbitControls } from '@react-three/drei'
import { Canvas, useThree } from '@react-three/fiber'
import { useEffect, useMemo, useRef, useState } from 'react'
import type { ElementRef } from 'react'
import type { JointState, Matrix6x3, Vector3 } from '../robotics/types'
import type { ForwardKinematicsResult } from '../robotics/kinematics'
import { SceneOverlays } from './SceneOverlays'
import {
  buildSceneModel,
  DEFAULT_SCENE_OVERLAYS,
  type SceneModel,
  type SceneOverlayFlags,
} from './sceneModel'
import './robotScene.css'

export const DEFAULT_SCENE_CAMERA = {
  position: [6, -7, 5] as [number, number, number],
  fov: 38,
  near: 0.1,
  far: 100,
}

const CAMERA_TARGET: [number, number, number] = [0, 0, 1]

export interface RobotSceneCalculation {
  forward: ForwardKinematicsResult
  jointState: JointState
  jacobian: Matrix6x3
  torque: Vector3
  gravity: Vector3
}

export interface RobotSceneProps {
  calculation: RobotSceneCalculation
  workspaceSamples?: readonly Vector3[]
  trail?: readonly Vector3[]
  initialOverlays?: Partial<SceneOverlayFlags>
}

export function RobotSceneContents({ sceneModel }: { sceneModel: SceneModel }) {
  return (
    <>
      <color attach="background" args={['#142638']} />
      <ambientLight intensity={1.1} />
      <hemisphereLight args={['#dcecea', '#0b3035', 1.2]} />
      <directionalLight intensity={1.6} position={[5, -4, 8]} />
      <directionalLight color="#7bc5c1" intensity={0.55} position={[-4, 5, 2]} />
      <SceneOverlays sceneModel={sceneModel} />
    </>
  )
}

function CameraRig({ resetRevision }: { resetRevision: number }) {
  const controls = useRef<ElementRef<typeof OrbitControls>>(null)
  const camera = useThree((state) => state.camera)

  useEffect(() => {
    camera.position.set(...DEFAULT_SCENE_CAMERA.position)
    camera.up.set(0, 0, 1)
    controls.current?.target.set(...CAMERA_TARGET)
    controls.current?.update()
  }, [camera, resetRevision])

  return (
    <OrbitControls
      ref={controls}
      enableDamping
      enablePan
      enableZoom
      maxDistance={20}
      minDistance={2.5}
      target={CAMERA_TARGET}
    />
  )
}

const OVERLAY_CONTROLS: readonly [keyof SceneOverlayFlags, string][] = [
  ['coordinateFrames', '局部坐标系'],
  ['centerOfMass', '质心'],
  ['workspace', '工作空间'],
  ['trail', '末端轨迹'],
  ['grid', '网格'],
  ['linearVelocity', '线速度'],
  ['acceleration', '加速度'],
  ['gravity', '重力'],
  ['torque', '关节力矩'],
]

function signedValue(value: number): string {
  return value < 0 ? `−${Math.abs(value).toFixed(3)}` : value.toFixed(3)
}

export function RobotScene({
  calculation,
  workspaceSamples = [],
  trail = [],
  initialOverlays,
}: RobotSceneProps) {
  const [overlays, setOverlays] = useState<SceneOverlayFlags>({
    ...DEFAULT_SCENE_OVERLAYS,
    ...initialOverlays,
  })
  const [resetRevision, setResetRevision] = useState(0)
  const sceneModel = useMemo(() => buildSceneModel({
    ...calculation,
    workspaceSamples,
    trail,
    overlays,
  }), [calculation, overlays, trail, workspaceSamples])
  const visibleVectors = sceneModel.vectors.filter((vector) => vector.visible)

  return (
    <div className="robot-scene">
      <div className="robot-scene__viewport">
        <Canvas camera={DEFAULT_SCENE_CAMERA} dpr={[1, 1.75]} gl={{ antialias: true, alpha: false }}>
          <RobotSceneContents sceneModel={sceneModel} />
          <CameraRig resetRevision={resetRevision} />
        </Canvas>
        <button
          aria-label="复位三维视角"
          className="robot-scene__reset"
          onClick={() => setResetRevision((revision) => revision + 1)}
          type="button"
        >
          视角复位
        </button>
      </div>

      <fieldset className="robot-scene__controls">
        <legend>三维图层</legend>
        {OVERLAY_CONTROLS.map(([key, label]) => (
          <label key={key}>
            <input
              checked={overlays[key]}
              onChange={(event) => setOverlays((current) => ({
                ...current,
                [key]: event.target.checked,
              }))}
              type="checkbox"
            />
            <span>{label}</span>
          </label>
        ))}
      </fieldset>

      {visibleVectors.length > 0 && (
        <aside aria-label="矢量真实量值" className="robot-scene__legend">
          <p>箭头长度已归一化，仅用于辨识方向。</p>
          <ul>
            {visibleVectors.map((vector) => (
              <li key={vector.id}>
                <i aria-hidden="true" style={{ background: vector.color }} />
                <span>{vector.label} = {signedValue(vector.signedMagnitude ?? vector.magnitude)} {vector.unit}</span>
              </li>
            ))}
          </ul>
        </aside>
      )}
    </div>
  )
}
