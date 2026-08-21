import { Html, Line } from '@react-three/drei'
import { Quaternion, Vector3 as ThreeVector3 } from 'three'
import type { Vector3 } from '../robotics/types'
import { CoordinateFrame } from './CoordinateFrame'
import { RobotChain } from './RobotChain'
import type { ScenePresentationModel } from './sceneModel'

export interface KinematicsTeachingOverlaysProps {
  presentation: ScenePresentationModel
  onObjectSelect?: (id: string) => void
}

function midpoint(left: Vector3, right: Vector3): [number, number, number] {
  return [
    (left[0] + right[0]) / 2,
    (left[1] + right[1]) / 2,
    (left[2] + right[2]) / 2,
  ]
}

function arcPoints(
  arc: ScenePresentationModel['arcs'][number],
): [number, number, number][] {
  const samples = 32
  return Array.from({ length: samples + 1 }, (_, index) => {
    const angle = arc.startAngle
      + (arc.endAngle - arc.startAngle) * index / samples
    return [
      arc.center[0] + arc.radius * (
        arc.basisX[0] * Math.cos(angle) + arc.basisY[0] * Math.sin(angle)
      ),
      arc.center[1] + arc.radius * (
        arc.basisX[1] * Math.cos(angle) + arc.basisY[1] * Math.sin(angle)
      ),
      arc.center[2] + arc.radius * (
        arc.basisX[2] * Math.cos(angle) + arc.basisY[2] * Math.sin(angle)
      ),
    ]
  })
}

export function KinematicsTeachingOverlays({
  presentation,
  onObjectSelect,
}: KinematicsTeachingOverlaysProps) {
  const planeQuaternion = presentation.workPlane === null
    ? new Quaternion()
    : new Quaternion().setFromUnitVectors(
        new ThreeVector3(0, 0, 1),
        new ThreeVector3(...presentation.workPlane.normal).normalize(),
      )

  return (
    <group name="kinematics-teaching-overlays">
      {presentation.workPlane !== null && (
        <mesh
          name="kinematics-work-plane"
          position={[...presentation.workPlane.origin]}
          quaternion={planeQuaternion}
        >
          <planeGeometry args={[presentation.workPlane.width, presentation.workPlane.height]} />
          <meshBasicMaterial
            color={presentation.workPlane.color}
            depthWrite={false}
            opacity={presentation.workPlane.opacity}
            side={2}
            transparent
          />
        </mesh>
      )}

      {presentation.primaryRobot !== null && (
        <RobotChain
          id={presentation.primaryRobot.id}
          joints={presentation.primaryRobot.joints}
          label={presentation.primaryRobot.label}
          links={presentation.primaryRobot.links}
          onObjectSelect={onObjectSelect}
          opacity={presentation.primaryRobot.opacity}
          style={presentation.primaryRobot.style}
        />
      )}

      {presentation.ghostRobots.map((robot) => (
        <RobotChain
          id={robot.id}
          joints={robot.joints}
          key={robot.id}
          label={robot.label}
          links={robot.links}
          opacity={robot.opacity}
          style="ghost"
        />
      ))}

      {presentation.frames.map((frame) => (
        <CoordinateFrame frame={frame} key={frame.id} onSelect={onObjectSelect} />
      ))}

      {presentation.points.map((point) => (
        <group key={point.id} name={point.id} position={[...point.position]}>
          <mesh>
            <sphereGeometry args={[0.085, 18, 14]} />
            <meshBasicMaterial
              color={point.color}
              opacity={point.opacity ?? 1}
              transparent={(point.opacity ?? 1) < 1}
            />
          </mesh>
          {point.label !== undefined && (
            <Html
              center
              distanceFactor={6}
              position={[...(point.labelOffset ?? [0, 0, 0.16])]}
              transform
              sprite
            >
              <span className="robot-scene__annotation-label">{point.label}</span>
            </Html>
          )}
        </group>
      ))}

      {presentation.dimensions.map((dimension) => {
        const labelPosition = dimension.labelPosition ?? midpoint(dimension.start, dimension.end)
        return (
          <group
            key={dimension.id}
            name={dimension.id}
            onClick={(event) => {
              event.stopPropagation()
              onObjectSelect?.(dimension.id)
            }}
          >
            <Line
              color={dimension.color}
              dashed={dimension.style === 'dashed'}
              dashScale={22}
              lineWidth={dimension.emphasized ? 4 : 2}
              opacity={dimension.emphasized ? 1 : 0.45}
              points={[[...dimension.start], [...dimension.end]]}
              transparent={!dimension.emphasized}
            />
            {dimension.showLabel !== false && (
              <Html center distanceFactor={6} position={labelPosition} transform sprite>
                <span className={`robot-scene__dimension-label${dimension.emphasized ? ' is-active' : ''}`}>
                  {dimension.label}
                </span>
              </Html>
            )}
          </group>
        )
      })}

      {presentation.arcs.map((arc) => {
        const points = arcPoints(arc)
        const labelPosition = arc.labelPosition ?? points[Math.floor(points.length / 2)]
        return (
          <group
            key={arc.id}
            name={arc.id}
            onClick={(event) => {
              event.stopPropagation()
              onObjectSelect?.(arc.id)
            }}
          >
            <Line
              color={arc.color}
              lineWidth={arc.emphasized ? 4 : 2}
              opacity={arc.emphasized ? 1 : 0.35}
              points={points}
              transparent={!arc.emphasized}
            />
            <Html center distanceFactor={6} position={labelPosition} transform sprite>
              <span className={`robot-scene__angle-label${arc.emphasized ? ' is-active' : ''}`}>
                {arc.label}
              </span>
            </Html>
          </group>
        )
      })}

      {presentation.vectors.map((vector) => {
        const magnitude = Math.hypot(...vector.vector)
        if (magnitude < 1e-12) return null
        const direction = new ThreeVector3(...vector.vector).normalize()
        const displayLength = Math.min(1.4, 0.72 + 0.68 * magnitude / (1 + magnitude))
        const end: [number, number, number] = [
          vector.origin[0] + direction.x * displayLength,
          vector.origin[1] + direction.y * displayLength,
          vector.origin[2] + direction.z * displayLength,
        ]
        const labelPositionFactor = vector.labelPositionFactor ?? 1
        const labelPosition: [number, number, number] = [
          vector.origin[0] + direction.x * displayLength * labelPositionFactor + (vector.labelOffset?.[0] ?? 0),
          vector.origin[1] + direction.y * displayLength * labelPositionFactor + (vector.labelOffset?.[1] ?? 0),
          vector.origin[2] + direction.z * displayLength * labelPositionFactor + (vector.labelOffset?.[2] ?? 0),
        ]
        const coneLength = Math.min(0.2, displayLength * 0.28)
        const coneCenter: [number, number, number] = [
          end[0] - direction.x * coneLength / 2,
          end[1] - direction.y * coneLength / 2,
          end[2] - direction.z * coneLength / 2,
        ]
        const quaternion = new Quaternion().setFromUnitVectors(
          new ThreeVector3(0, 1, 0),
          direction,
        )
        return (
          <group key={vector.id} name={vector.id}>
            <Line
              color={vector.color}
              lineWidth={4}
              opacity={vector.opacity ?? 1}
              points={[[...vector.origin], end]}
              transparent={(vector.opacity ?? 1) < 1}
            />
            <mesh position={coneCenter} quaternion={quaternion}>
              <coneGeometry args={[0.075, coneLength, 12]} />
              <meshBasicMaterial
                color={vector.color}
                opacity={vector.opacity ?? 1}
                transparent={(vector.opacity ?? 1) < 1}
              />
            </mesh>
            <Html center distanceFactor={10} position={labelPosition} transform sprite>
              <span
                className="robot-scene__dimension-label robot-scene__vector-label is-active"
                title={`${vector.label} = ${magnitude.toFixed(2)} ${vector.unit}；画布长度已归一化`}
              >
                {vector.label} = {magnitude.toFixed(1)} {vector.unit}
              </span>
            </Html>
          </group>
        )
      })}
    </group>
  )
}
