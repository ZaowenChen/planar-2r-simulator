import { OrbitControls } from '@react-three/drei'
import { Canvas, useFrame, useThree } from '@react-three/fiber'
import { useEffect, useMemo, useRef, useState } from 'react'
import type { ElementRef } from 'react'
import { Vector3 as ThreeVector3 } from 'three'
import type { JointState, Matrix6x3, Vector3 } from '../robotics/types'
import type { ForwardKinematicsResult } from '../robotics/kinematics'
import { SceneOverlays } from './SceneOverlays'
import {
  buildSceneModel,
  DEFAULT_SCENE_OVERLAYS,
  type SceneModel,
  type SceneOverlayFlags,
  type ScenePresentationModel,
  type ScenePointModel,
} from './sceneModel'
import './robotScene.css'

export const DEFAULT_SCENE_CAMERA = {
  position: [6, -7, 5] as [number, number, number],
  fov: 38,
  near: 0.1,
  far: 100,
}

const CAMERA_TARGET: [number, number, number] = [0, 0, 1]

interface VectorSetter {
  set: (x: number, y: number, z: number) => unknown
}

interface ResettableCamera {
  position: VectorSetter
  up: VectorSetter
}

interface ResettableOrbitControls {
  target: VectorSetter
  update: () => unknown
}

export function resetSceneCamera(
  camera: ResettableCamera,
  controls: ResettableOrbitControls | null,
): void {
  camera.position.set(...DEFAULT_SCENE_CAMERA.position)
  camera.up.set(0, 0, 1)
  if (controls !== null) {
    controls.target.set(...CAMERA_TARGET)
    controls.update()
  }
}

export interface RobotSceneCalculation {
  forward: ForwardKinematicsResult
  jointState: JointState
  jacobian: Matrix6x3
  torque?: Vector3
  gravity?: Vector3
}

export interface RobotSceneProps {
  calculation: RobotSceneCalculation
  workspaceSamples?: readonly Vector3[]
  trail?: readonly Vector3[]
  markers?: readonly ScenePointModel[]
  initialOverlays?: Partial<SceneOverlayFlags>
  visibleOverlayControls?: readonly (keyof SceneOverlayFlags)[]
  presentation?: ScenePresentationModel
  cameraResetRevision?: number
  followPresentationCamera?: boolean
  onCameraInteraction?: () => void
  onSceneObjectSelect?: (id: string) => void
}

export function RobotSceneContents({
  sceneModel,
  presentation,
  onObjectSelect,
}: {
  sceneModel: SceneModel
  presentation?: ScenePresentationModel
  onObjectSelect?: (id: string) => void
}) {
  return (
    <>
      <color attach="background" args={['#142638']} />
      <ambientLight intensity={1.1} />
      <hemisphereLight args={['#dcecea', '#0b3035', 1.2]} />
      <directionalLight intensity={1.6} position={[5, -4, 8]} />
      <directionalLight color="#7bc5c1" intensity={0.55} position={[-4, 5, 2]} />
      <SceneOverlays
        onObjectSelect={onObjectSelect}
        presentation={presentation}
        sceneModel={sceneModel}
      />
    </>
  )
}

function CameraRig({
  resetRevision,
  presentation,
  followPresentationCamera,
  onCameraInteraction,
}: {
  resetRevision: number
  presentation?: ScenePresentationModel
  followPresentationCamera: boolean
  onCameraInteraction?: () => void
}) {
  const controls = useRef<ElementRef<typeof OrbitControls>>(null)
  const camera = useThree((state) => state.camera)
  const transition = useRef<{
    position: ThreeVector3
    target: ThreeVector3
    up: ThreeVector3
  } | null>(null)

  useEffect(() => {
    if (presentation === undefined) {
      resetSceneCamera(camera, controls.current)
      transition.current = null
      return
    }
    if (!followPresentationCamera) {
      transition.current = null
      return
    }
    const next = {
      position: new ThreeVector3(...presentation.camera.position),
      target: new ThreeVector3(...presentation.camera.target),
      up: new ThreeVector3(...presentation.camera.up),
    }
    if (window.matchMedia?.('(prefers-reduced-motion: reduce)').matches) {
      camera.position.copy(next.position)
      camera.up.copy(next.up)
      controls.current?.target.copy(next.target)
      controls.current?.update()
      transition.current = null
      return
    }
    transition.current = next
  }, [camera, followPresentationCamera, presentation?.camera.id, presentation?.camera.position, presentation?.camera.target, resetRevision])

  useFrame((_, delta) => {
    const next = transition.current
    const orbitControls = controls.current
    if (next === null || orbitControls === null) return
    const alpha = 1 - Math.exp(-6 * delta)
    camera.position.lerp(next.position, alpha)
    camera.up.lerp(next.up, alpha).normalize()
    orbitControls.target.lerp(next.target, alpha)
    orbitControls.update()
    if (
      camera.position.distanceTo(next.position) < 0.005
      && orbitControls.target.distanceTo(next.target) < 0.005
    ) {
      camera.position.copy(next.position)
      orbitControls.target.copy(next.target)
      orbitControls.update()
      transition.current = null
    }
  })

  return (
    <OrbitControls
      ref={controls}
      enableDamping
      enablePan
      enableZoom
      maxDistance={20}
      minDistance={2.5}
      onStart={() => {
        transition.current = null
        onCameraInteraction?.()
      }}
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
  markers = [],
  initialOverlays,
  visibleOverlayControls,
  presentation,
  cameraResetRevision = 0,
  followPresentationCamera = true,
  onCameraInteraction,
  onSceneObjectSelect,
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
    markers,
    overlays,
  }), [calculation, markers, overlays, trail, workspaceSamples])
  const visibleVectors = sceneModel.vectors.filter((vector) => vector.visible)
  const overlayControls = visibleOverlayControls === undefined
    ? OVERLAY_CONTROLS
    : OVERLAY_CONTROLS.filter(([key]) => visibleOverlayControls.includes(key))

  return (
    <div className="robot-scene">
      <div
        className="robot-scene__viewport"
        onPointerDown={presentation === undefined ? undefined : onCameraInteraction}
        onWheel={presentation === undefined ? undefined : onCameraInteraction}
      >
        <Canvas camera={DEFAULT_SCENE_CAMERA} dpr={[1, 1.75]} gl={{ antialias: true, alpha: false }}>
          <RobotSceneContents
            onObjectSelect={onSceneObjectSelect}
            presentation={presentation}
            sceneModel={sceneModel}
          />
          <CameraRig
            followPresentationCamera={followPresentationCamera}
            onCameraInteraction={onCameraInteraction}
            presentation={presentation}
            resetRevision={resetRevision + cameraResetRevision}
          />
        </Canvas>
        {presentation === undefined && (
          <button
            aria-label="复位三维视角"
            className="robot-scene__reset"
            onClick={() => setResetRevision((revision) => revision + 1)}
            type="button"
          >
            视角复位
          </button>
        )}
      </div>

      {overlayControls.length > 0 && (
        <fieldset className="robot-scene__controls">
          <legend>三维图层</legend>
          {overlayControls.map(([key, label]) => (
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
      )}

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
