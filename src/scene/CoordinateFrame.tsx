import { Html, Line } from '@react-three/drei'
import type { CoordinateFrameModel } from './sceneModel'

export interface CoordinateFrameProps {
  frame: CoordinateFrameModel
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

export function CoordinateFrame({ frame }: CoordinateFrameProps) {
  if (!frame.visible) return null
  const position = [...frame.position] as [number, number, number]

  return (
    <group name={frame.id}>
      <Line
        color="#ef4444"
        lineWidth={2}
        points={[position, endpoint(frame.position, frame.axes.x, frame.axisLength)]}
      />
      <Line
        color="#22c55e"
        lineWidth={2}
        points={[position, endpoint(frame.position, frame.axes.y, frame.axisLength)]}
      />
      <Line
        color="#3b82f6"
        lineWidth={2}
        points={[position, endpoint(frame.position, frame.axes.z, frame.axisLength)]}
      />
      <Html center position={position} transform sprite>
        <span className="robot-scene__frame-label">{frame.label}</span>
      </Html>
    </group>
  )
}
