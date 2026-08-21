import { Html, Line } from '@react-three/drei'
import { CoordinateFrame } from './CoordinateFrame'
import { KinematicsTeachingOverlays } from './KinematicsTeachingOverlays'
import { RobotChain } from './RobotChain'
import type {
  SceneModel,
  ScenePresentationModel,
  VectorOverlayModel,
} from './sceneModel'

export interface SceneOverlaysProps {
  sceneModel: SceneModel
  presentation?: ScenePresentationModel
  onObjectSelect?: (id: string) => void
}

function ScientificArrow({ vector }: { vector: VectorOverlayModel }) {
  if (!vector.visible || vector.displayLength === 0) return null
  const coneLength = Math.min(0.22, vector.displayLength * 0.28)
  const shaftLength = Math.max(0.01, vector.displayLength - coneLength)
  const coneCenter: [number, number, number] = [
    vector.end[0] - vector.direction[0] * coneLength / 2,
    vector.end[1] - vector.direction[1] * coneLength / 2,
    vector.end[2] - vector.direction[2] * coneLength / 2,
  ]
  const shaftCenter: [number, number, number] = [
    vector.origin[0] + vector.direction[0] * shaftLength / 2,
    vector.origin[1] + vector.direction[1] * shaftLength / 2,
    vector.origin[2] + vector.direction[2] * shaftLength / 2,
  ]

  return (
    <group name={vector.id}>
      <mesh position={shaftCenter} quaternion={[...vector.quaternion]}>
        <cylinderGeometry args={[0.025, 0.025, shaftLength, 10]} />
        <meshBasicMaterial color={vector.color} />
      </mesh>
      <mesh position={coneCenter} quaternion={[...vector.quaternion]}>
        <coneGeometry args={[0.075, coneLength, 12]} />
        <meshBasicMaterial color={vector.color} />
      </mesh>
    </group>
  )
}

export function SceneOverlays({
  sceneModel,
  presentation,
  onObjectSelect,
}: SceneOverlaysProps) {
  const workspacePositions = new Float32Array(sceneModel.workspace.points.flatMap((point) => point))

  return (
    <group name="robot-scientific-scene">
      {sceneModel.gridVisible && (
        <gridHelper
          args={[10, 20, '#34556c', '#203950']}
          position={[0, 0, -0.01]}
          rotation={[Math.PI / 2, 0, 0]}
        />
      )}

      {!presentation?.hideBaseRobot && (
        <RobotChain
          id="robot-base"
          joints={sceneModel.joints}
          links={sceneModel.links}
          onObjectSelect={onObjectSelect}
        />
      )}

      {presentation === undefined && sceneModel.coordinateFrames.map((frame) => (
        <CoordinateFrame frame={frame} key={frame.id} onSelect={onObjectSelect} />
      ))}

      {sceneModel.centerOfMassMarkers.map((marker) => marker.visible && (
        <group key={marker.id} name={marker.id} position={[...marker.position]}>
          <mesh>
            <sphereGeometry args={[0.085, 16, 12]} />
            <meshBasicMaterial color={marker.color} />
          </mesh>
          <Html center position={[0, 0, 0.15]} transform sprite>
            <span className="robot-scene__com-label">{marker.label}</span>
          </Html>
        </group>
      ))}

      {sceneModel.workspace.visible && workspacePositions.length > 0 && (
        <points name="workspace-point-cloud">
          <bufferGeometry>
            <bufferAttribute attach="attributes-position" args={[workspacePositions, 3]} />
          </bufferGeometry>
          <pointsMaterial
            color={sceneModel.workspace.color}
            opacity={sceneModel.workspace.opacity}
            size={0.035}
            sizeAttenuation
            transparent
          />
        </points>
      )}

      {sceneModel.trail.visible && sceneModel.trail.points.length > 1 && (
        <Line
          color={sceneModel.trail.color}
          lineWidth={2}
          points={sceneModel.trail.points.map((point) => [...point] as [number, number, number])}
        />
      )}

      {sceneModel.markers.map((marker) => (
        <group key={marker.id} name={marker.id} position={[...marker.position]}>
          <mesh>
            <sphereGeometry args={[0.1, 18, 14]} />
            <meshBasicMaterial color={marker.color} />
          </mesh>
          {marker.label !== undefined && (
            <Html center position={[0, 0, 0.2]} transform sprite>
              <span className="robot-scene__marker-label">{marker.label}</span>
            </Html>
          )}
        </group>
      ))}

      {sceneModel.vectors.map((vector) => (
        <ScientificArrow key={vector.id} vector={vector} />
      ))}

      {presentation !== undefined && (
        <KinematicsTeachingOverlays
          onObjectSelect={onObjectSelect}
          presentation={presentation}
        />
      )}
    </group>
  )
}
