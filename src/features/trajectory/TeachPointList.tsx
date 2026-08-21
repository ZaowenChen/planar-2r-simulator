import type { RobotParameters } from '../../robotics/types'
import { vectorRadiansToDegrees } from './trajectoryModel'
import { useTrajectoryStore } from './trajectoryStore'

function angleVectorText(q: readonly number[]): string {
  const degrees = vectorRadiansToDegrees(q as [number, number, number])
  return `[${degrees.map((value) => `${value.toFixed(1)}°`).join(', ')}]`
}

export function TeachPointList({
  jointLimits,
}: {
  jointLimits: RobotParameters['jointLimits']
}) {
  const points = useTrajectoryStore((state) => state.teachPoints)
  const updatePoint = useTrajectoryStore((state) => state.updateTeachPoint)
  const renamePoint = useTrajectoryStore((state) => state.renameTeachPoint)
  const deletePoint = useTrajectoryStore((state) => state.deleteTeachPoint)

  return (
    <section className="teach-point-section" aria-labelledby="teach-point-title">
      <div className="trajectory-section-heading">
        <div>
          <span>示教点</span>
          <h4 id="teach-point-title">已记录姿态</h4>
        </div>
        <output aria-label="示教点数量">{points.length} 点</output>
      </div>
      {points.length === 0 ? (
        <p className="trajectory-empty-state">使用 JOG 调整姿态，再记录当前位置。</p>
      ) : (
        <ol className="teach-point-list">
          {points.map((point) => (
            <li key={point.id}>
              <input
                aria-label={`${point.name} 名称`}
                onBlur={(event) => renamePoint(point.id, event.target.value)}
                onChange={(event) => renamePoint(point.id, event.target.value)}
                value={point.name}
              />
              <output>{angleVectorText(point.q)}</output>
              <div>
                <button onClick={() => updatePoint(point.id, jointLimits)} type="button">更新</button>
                <button
                  className="danger-button"
                  onClick={() => {
                    if (window.confirm(`确定删除示教点 ${point.name} 吗？`)) deletePoint(point.id)
                  }}
                  type="button"
                >删除</button>
              </div>
            </li>
          ))}
        </ol>
      )}
    </section>
  )
}

