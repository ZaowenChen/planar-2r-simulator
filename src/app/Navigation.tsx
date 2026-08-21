import type { LabModule } from '../state/labStore'

export const LAB_MODULES = [
  { id: 'model', label: '机器人模型', index: '01' },
  { id: 'kinematics', label: '运动学', index: '02' },
  { id: 'dynamics', label: '动力学', index: '03' },
  { id: 'experiments', label: '轨迹示教', index: '04' },
] as const satisfies readonly { id: LabModule; label: string; index: string }[]

export interface NavigationProps {
  activeModule: LabModule
  onSelect: (module: LabModule) => void
}

export function Navigation({ activeModule, onSelect }: NavigationProps) {
  return (
    <nav className="lab-navigation" aria-label="学习模块">
      {LAB_MODULES.map(({ id, label, index }) => (
        <button
          aria-label={label}
          aria-current={activeModule === id ? 'page' : undefined}
          key={id}
          onClick={() => onSelect(id)}
          type="button"
        >
          <span aria-hidden="true">{index}</span>
          {label}
        </button>
      ))}
    </nav>
  )
}
