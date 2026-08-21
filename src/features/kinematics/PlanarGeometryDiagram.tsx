import type { InverseSolutionDerivation } from './derivationModel'
import type { KinematicsSymbol } from './teachingState'

export type GeometryDiagramFocus =
  | 'projection'
  | 'distance'
  | 'elbow'
  | 'gamma'
  | 'delta'
  | 'solution'

interface PlanarGeometryDiagramProps {
  branches: readonly InverseSolutionDerivation[]
  radialMm: number
  verticalMm: number
  focus: GeometryDiagramFocus
  activeBranch: 'elbow-down' | 'elbow-up'
  onBranchChange: (branch: 'elbow-down' | 'elbow-up') => void
  symbolFocus: KinematicsSymbol | null
}

type Point2 = readonly [number, number]

const VIEWBOX_WIDTH = 520
const VIEWBOX_HEIGHT = 330
const MARGIN = 52

function format(value: number, precision = 1): string {
  return Number.isFinite(value) ? value.toFixed(precision) : '—'
}

function midpoint(left: Point2, right: Point2): Point2 {
  return [(left[0] + right[0]) / 2, (left[1] + right[1]) / 2]
}

function branchLabel(branch: InverseSolutionDerivation['solution']['branch']): string {
  return branch === 'elbow-down' ? '肘下' : '肘上'
}

export function PlanarGeometryDiagram({
  branches,
  radialMm,
  verticalMm,
  focus,
  activeBranch,
  onBranchChange,
  symbolFocus,
}: PlanarGeometryDiagramProps) {
  if (branches.length === 0) {
    return (
      <div
        aria-label="解析几何工作平面：目标不可达，无法构成连杆三角形"
        className="geometry-diagram geometry-diagram--empty"
        role="img"
      >
        目标不可达，当前参数不能构成肩—肘—目标三角形。
      </div>
    )
  }

  const active = branches.find((branch) => branch.solution.branch === activeBranch)
    ?? branches[0]
  const alternate = branches.find((branch) => branch !== active)
  const shoulder: Point2 = [0, 0]
  const target: Point2 = [radialMm, verticalMm]
  const foot: Point2 = [radialMm, 0]
  const activeElbow = active.elbowPointMm
  const alternateElbow = alternate?.elbowPointMm
  const points = [shoulder, target, foot, activeElbow]
  if (alternateElbow !== undefined) points.push(alternateElbow)

  const xValues = points.map((point) => point[0])
  const yValues = points.map((point) => point[1])
  const rawMinX = Math.min(...xValues)
  const rawMaxX = Math.max(...xValues)
  const rawMinY = Math.min(...yValues)
  const rawMaxY = Math.max(...yValues)
  const span = Math.max(rawMaxX - rawMinX, rawMaxY - rawMinY, 1)
  const padding = span * 0.18
  const minX = rawMinX - padding
  const maxX = rawMaxX + padding
  const minY = rawMinY - padding
  const maxY = rawMaxY + padding
  const scale = Math.min(
    (VIEWBOX_WIDTH - MARGIN * 2) / (maxX - minX),
    (VIEWBOX_HEIGHT - MARGIN * 2) / (maxY - minY),
  )
  const usedWidth = (maxX - minX) * scale
  const usedHeight = (maxY - minY) * scale
  const offsetX = (VIEWBOX_WIDTH - usedWidth) / 2
  const offsetY = (VIEWBOX_HEIGHT - usedHeight) / 2
  const toScreen = ([x, y]: Point2): Point2 => [
    offsetX + (x - minX) * scale,
    VIEWBOX_HEIGHT - offsetY - (y - minY) * scale,
  ]
  const [shoulderX, shoulderY] = toScreen(shoulder)
  const [targetX, targetY] = toScreen(target)
  const [footX, footY] = toScreen(foot)
  const [elbowX, elbowY] = toScreen(activeElbow)
  const activeL2Mid = toScreen(midpoint(shoulder, activeElbow))
  const activeL3Mid = toScreen(midpoint(activeElbow, target))

  const anglePoints = (
    center: Point2,
    radiusPixels: number,
    startDegrees: number,
    endDegrees: number,
  ): string => {
    const samples = 18
    return Array.from({ length: samples + 1 }, (_, index) => {
      const angle = (
        startDegrees + (endDegrees - startDegrees) * index / samples
      ) * Math.PI / 180
      const worldRadius = radiusPixels / scale
      return toScreen([
        center[0] + worldRadius * Math.cos(angle),
        center[1] + worldRadius * Math.sin(angle),
      ]).join(',')
    }).join(' ')
  }
  const angleLabelPoint = (
    center: Point2,
    radiusPixels: number,
    startDegrees: number,
    endDegrees: number,
  ): Point2 => {
    const angle = (startDegrees + endDegrees) * Math.PI / 360
    const worldRadius = radiusPixels / scale
    return toScreen([
      center[0] + worldRadius * Math.cos(angle),
      center[1] + worldRadius * Math.sin(angle),
    ])
  }
  const theta2 = active.qDegrees[1]
  const theta3 = active.qDegrees[2]
  const gamma = active.targetDirectionDegrees
  const theta2Label = angleLabelPoint(shoulder, 62, 0, theta2)
  const gammaLabel = angleLabelPoint(shoulder, 102, 0, gamma)
  const deltaLabel = angleLabelPoint(shoulder, 142, theta2, gamma)
  const theta3Label = angleLabelPoint(activeElbow, 58, theta2, theta2 + theta3)

  return (
    <figure
      className="geometry-diagram"
      data-focus={focus}
      data-focus-symbol={symbolFocus ?? undefined}
    >
      <svg
        aria-label={`解析几何工作平面：${branchLabel(active.solution.branch)}构型，标出 r、h、s、l₂、l₃、γ、δ、θ₂ 和 θ₃`}
        data-branch={active.solution.branch}
        role="img"
        viewBox={`0 0 ${VIEWBOX_WIDTH} ${VIEWBOX_HEIGHT}`}
      >
        <defs>
          <marker id="geometry-arrow" markerHeight="7" markerWidth="7" orient="auto" refX="6" refY="3.5">
            <path d="M0,0 L7,3.5 L0,7 z" />
          </marker>
        </defs>

        <line className="geometry-axis" markerEnd="url(#geometry-arrow)" x1={toScreen([minX, 0])[0]} x2={toScreen([maxX, 0])[0]} y1={shoulderY} y2={shoulderY} />
        <line className="geometry-axis" markerEnd="url(#geometry-arrow)" x1={shoulderX} x2={shoulderX} y1={toScreen([0, minY])[1]} y2={toScreen([0, maxY])[1]} />
        <text className="geometry-axis-label" x={toScreen([maxX, 0])[0] - 16} y={shoulderY - 8}>r</text>
        <text className="geometry-axis-label" x={shoulderX + 8} y={toScreen([0, maxY])[1] + 18}>h</text>

        <line className="geometry-measure geometry-measure--r" x1={shoulderX} x2={footX} y1={footY} y2={footY} />
        <line className="geometry-measure geometry-measure--h" x1={footX} x2={targetX} y1={footY} y2={targetY} />
        <polyline className="geometry-right-angle" points={`${footX - 10},${footY} ${footX - 10},${footY - Math.sign(verticalMm || 1) * 10} ${footX},${footY - Math.sign(verticalMm || 1) * 10}`} />
        <line className="geometry-target-line" x1={shoulderX} x2={targetX} y1={shoulderY} y2={targetY} />

        {alternate !== undefined && alternateElbow !== undefined && (() => {
          const [alternateX, alternateY] = toScreen(alternateElbow)
          return (
            <g className="geometry-branch geometry-branch--alternate">
              <polyline points={`${shoulderX},${shoulderY} ${alternateX},${alternateY} ${targetX},${targetY}`} />
              <circle cx={alternateX} cy={alternateY} r="5" />
              <text x={alternateX + 8} y={alternateY - 8}>E{alternate.solution.branch === 'elbow-down' ? '↓' : '↑'}</text>
            </g>
          )
        })()}

        <g className="geometry-branch geometry-branch--active">
          <line x1={shoulderX} x2={elbowX} y1={shoulderY} y2={elbowY} />
          <line x1={elbowX} x2={targetX} y1={elbowY} y2={targetY} />
          <circle cx={elbowX} cy={elbowY} r="6" />
          <text x={elbowX + 8} y={elbowY - 9}>E{active.solution.branch === 'elbow-down' ? '↓' : '↑'}</text>
        </g>

        <circle className="geometry-joint" cx={shoulderX} cy={shoulderY} r="7" />
        <circle className="geometry-target" cx={targetX} cy={targetY} r="7" />
        <text className="geometry-point-label" x={shoulderX - 24} y={shoulderY + 22}>S 肩</text>
        <text className="geometry-point-label" x={targetX + 9} y={targetY - 10}>P 目标</text>

        <text className="geometry-length-label geometry-length-label--r" x={(shoulderX + footX) / 2} y={footY + 20}>r = {format(radialMm)} mm</text>
        <text className="geometry-length-label geometry-length-label--h" x={footX + 8} y={(footY + targetY) / 2}>h = {format(verticalMm)} mm</text>
        <text className="geometry-length-label geometry-length-label--s" x={(shoulderX + targetX) / 2 + 5} y={(shoulderY + targetY) / 2 - 9}>s</text>
        <text className="geometry-length-label geometry-length-label--l2" x={activeL2Mid[0] - 16} y={activeL2Mid[1] - 10}>l₂</text>
        <text className="geometry-length-label geometry-length-label--l3" x={activeL3Mid[0] + 5} y={activeL3Mid[1] - 10}>l₃</text>

        <polyline className="geometry-angle geometry-angle--theta2" points={anglePoints(shoulder, 62, 0, theta2)} />
        <polyline className="geometry-angle geometry-angle--gamma" points={anglePoints(shoulder, 102, 0, gamma)} />
        <polyline className="geometry-angle geometry-angle--delta" points={anglePoints(shoulder, 142, theta2, gamma)} />
        <polyline className="geometry-angle geometry-angle--theta3" points={anglePoints(activeElbow, 58, theta2, theta2 + theta3)} />
        <text className="geometry-angle-label geometry-angle-label--theta2" x={theta2Label[0] - 9} y={theta2Label[1] + (theta2 < 0 ? 18 : -10)}>θ₂</text>
        <text className="geometry-angle-label geometry-angle-label--gamma" x={gammaLabel[0]} y={gammaLabel[1] - 15}>γ</text>
        <text className="geometry-angle-label geometry-angle-label--delta" x={deltaLabel[0]} y={deltaLabel[1] + (active.triangleCorrectionDegrees > 0 ? 22 : -12)}>δ</text>
        <text className="geometry-angle-label geometry-angle-label--theta3" x={theta3Label[0] + 5} y={theta3Label[1] + (theta3 > 0 ? -10 : 20)}>θ₃</text>
      </svg>

      <figcaption>
        <span>实线：当前查看构型</span>
        <span>虚线：另一肘部构型</span>
      </figcaption>
      <div aria-label="几何图构型" className="geometry-diagram__branches" role="group">
        {branches.map((branch) => (
          <button
            aria-pressed={branch.solution.branch === active.solution.branch}
            key={branch.solution.branch}
            onClick={() => onBranchChange(branch.solution.branch)}
            type="button"
          >
            显示{branchLabel(branch.solution.branch)}构型
          </button>
        ))}
      </div>
    </figure>
  )
}
