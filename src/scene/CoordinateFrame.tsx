import { Html, Line } from '@react-three/drei'
import { Quaternion, Vector3 as ThreeVector3 } from 'three'
import type { CoordinateFrameModel } from './sceneModel'

export interface CoordinateFrameProps {
  frame: CoordinateFrameModel
  onSelect?: (id: string) => void
}

function endpoint(
  position: CoordinateFrameModel['position'],
  direction: CoordinateFrameModel['position'],
  length: number,
): [number, number, number] {
  return [
    position[0] + direction[0] * length,
    position[1] + direction[1] * length,
    position[2] + direction[2] * length,
  ]
}

export function CoordinateFrame({ frame, onSelect }: CoordinateFrameProps) {
  if (!frame.visible) return null
  const position = [...frame.position] as [number, number, number]
  const labelPosition: [number, number, number] = [
    position[0] - frame.axisLength * 0.16,
    position[1],
    position[2] + frame.axisLength * 0.22,
  ]
  const opacity = frame.opacity ?? 1
  const axisSubscript = frame.label.includes('{W}/{0}')
    ? '0'
    : frame.label.match(/\{([0-3])\}/)?.[1] ?? 'e'
  const axes = [
    { key: 'x', color: '#ef4444', direction: frame.axes.x, label: 'x' },
    { key: 'y', color: '#22c55e', direction: frame.axes.y, label: 'y' },
    { key: 'z', color: '#3b82f6', direction: frame.axes.z, label: 'z' },
  ] as const

  return (
    <group
      name={frame.id}
      onClick={onSelect === undefined ? undefined : (event) => {
        event.stopPropagation()
        onSelect(frame.id)
      }}
    >
      {axes.filter((axis) => frame.visibleAxes?.includes(axis.key) ?? true).map((axis) => {
        const end = endpoint(frame.position, axis.direction, frame.axisLength)
        const labelEnd = endpoint(frame.position, axis.direction, frame.axisLength * 1.12)
        const direction = new ThreeVector3(...axis.direction).normalize()
        const quaternion = new Quaternion().setFromUnitVectors(
          new ThreeVector3(0, 1, 0),
          direction,
        )
        const headLength = frame.axisLength * 0.18
        const headCenter: [number, number, number] = [
          end[0] - direction.x * headLength / 2,
          end[1] - direction.y * headLength / 2,
          end[2] - direction.z * headLength / 2,
        ]
        return (
          <group key={axis.key}>
            <Line
              color={axis.color}
              lineWidth={frame.lineWidth ?? 2}
              opacity={opacity}
              points={[position, end]}
              transparent={opacity < 1}
            />
            <mesh position={headCenter} quaternion={quaternion}>
              <coneGeometry args={[frame.axisLength * 0.055, headLength, 10]} />
              <meshBasicMaterial color={axis.color} opacity={opacity} transparent={opacity < 1} />
            </mesh>
            {frame.detail === 'axes' && (
              <Html center distanceFactor={9} position={labelEnd} transform sprite>
                <span className="robot-scene__axis-label" style={{ opacity }}>
                  {axis.label}<sub>{axisSubscript}</sub>
                </span>
              </Html>
            )}
          </group>
        )
      })}
      {frame.showFrameLabel !== false && (
        <Html center distanceFactor={9} position={labelPosition} transform sprite>
          <span className="robot-scene__frame-label" style={{ opacity }}>
            {frame.originLabel !== undefined && <small>{frame.originLabel}</small>}
            {frame.label}
          </span>
        </Html>
      )}
    </group>
  )
}
