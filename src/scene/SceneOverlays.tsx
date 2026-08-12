import { Html, Line } from '@react-three/drei'
import { CoordinateFrame } from './CoordinateFrame'
import type { SceneModel, VectorOverlayModel } from './sceneModel'

export interface SceneOverlaysProps {
  sceneModel: SceneModel
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

export function SceneOverlays({ sceneModel }: SceneOverlaysProps) {
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

      {sceneModel.links.map((link) => link.length > 0 && (
        <mesh
          key={link.id}
          name={link.id}
          position={[...link.midpoint]}
          quaternion={[...link.quaternion]}
        >
          <cylinderGeometry args={[link.radius, link.radius, link.length, 18]} />
          <meshStandardMaterial color={link.color} metalness={0.08} roughness={0.72} />
        </mesh>
      ))}

      {sceneModel.joints.map((joint) => (
        <mesh key={joint.id} name={joint.id} position={[...joint.position]}>
          <sphereGeometry args={[joint.radius, 20, 14]} />
          <meshStandardMaterial color={joint.color} metalness={0.12} roughness={0.56} />
        </mesh>
      ))}

      {sceneModel.coordinateFrames.map((frame) => (
        <CoordinateFrame frame={frame} key={frame.id} />
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

      {sceneModel.vectors.map((vector) => (
        <ScientificArrow key={vector.id} vector={vector} />
      ))}
    </group>
  )
}
