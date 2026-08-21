import { Html } from '@react-three/drei'
import type { JointMarkerModel, LinkModel } from './sceneModel'

export interface RobotChainProps {
  id: string
  label?: string
  joints: readonly JointMarkerModel[]
  links: readonly LinkModel[]
  opacity?: number
  style?: 'solid' | 'ghost'
  onObjectSelect?: (id: string) => void
}

export function RobotChain({
  id,
  label,
  joints,
  links,
  opacity = 1,
  style = 'solid',
  onObjectSelect,
}: RobotChainProps) {
  const selectable = onObjectSelect !== undefined
  const endpoint = joints.at(-1)?.position
  return (
    <group name={id}>
      {links.map((link) => link.length > 0 && (
        <mesh
          key={link.id}
          name={link.id}
          onClick={selectable ? (event) => {
            event.stopPropagation()
            onObjectSelect(link.id)
          } : undefined}
          position={[...link.midpoint]}
          quaternion={[...link.quaternion]}
        >
          <cylinderGeometry args={[link.radius, link.radius, link.length, 18]} />
          <meshStandardMaterial
            color={link.color}
            depthWrite={opacity >= 1}
            metalness={style === 'ghost' ? 0 : 0.08}
            opacity={opacity}
            roughness={0.72}
            transparent={opacity < 1}
          />
        </mesh>
      ))}

      {joints.map((joint) => (
        <mesh
          key={joint.id}
          name={joint.id}
          onClick={selectable ? (event) => {
            event.stopPropagation()
            onObjectSelect(joint.id)
          } : undefined}
          position={[...joint.position]}
        >
          <sphereGeometry args={[joint.radius, 20, 14]} />
          <meshStandardMaterial
            color={joint.color}
            depthWrite={opacity >= 1}
            metalness={style === 'ghost' ? 0 : 0.12}
            opacity={opacity}
            roughness={0.56}
            transparent={opacity < 1}
          />
        </mesh>
      ))}

      {label !== undefined && endpoint !== undefined && (
        <Html center distanceFactor={6} position={[endpoint[0], endpoint[1], endpoint[2] + 0.22]} transform sprite>
          <span className={`robot-scene__robot-label robot-scene__robot-label--${style}`}>
            {label}
          </span>
        </Html>
      )}
    </group>
  )
}
