import { ExperimentPage } from '../features/experiments/ExperimentPage'
import { StatusBanner } from '../components/StatusBanner'
import { DynamicsPage } from '../features/dynamics/DynamicsPage'
import { KinematicsPage } from '../features/kinematics/KinematicsPage'
import { RobotModelPage } from '../features/model/RobotModelPage'
import { useLabStore } from '../state/labStore'
import type { LabModule } from '../state/labStore'
import { Navigation, LAB_MODULES } from './Navigation'

const MODULE_COPY: Record<LabModule, { kicker: string; title: string; summary: string }> = {
  model: {
    kicker: '几何 · 坐标系 · 刚体参数',
    title: '机器人模型',
    summary: '检查标准 D–H 约定、连杆几何与教学参数。',
  },
  kinematics: {
    kicker: '正解 · 逆解 · 雅可比',
    title: '运动学',
    summary: '从同一关节姿态观察末端位置、速度映射与奇异性。',
  },
  dynamics: {
    kicker: '惯性 · 科氏项 · 重力',
    title: '动力学',
    summary: '用完整刚体模型分析力矩、能量与功率。',
  },
  experiments: {
    kicker: '轨迹 · 力矩 · 时间响应',
    title: '动态实验',
    summary: '让动画、曲线与公式共享一个仿真时刻。',
  },
}

export function App() {
  const activeModule = useLabStore((state) => state.activeModule)
  const setActiveModule = useLabStore((state) => state.setActiveModule)
  const calculation = useLabStore((state) => state.calculation)
  const moduleCopy = MODULE_COPY[activeModule]
  const moduleIndex = LAB_MODULES.find((module) => module.id === activeModule)?.index ?? '01'

  return (
    <main className="lab-shell">
      <header className="lab-masthead">
        <div>
          <p className="eyebrow">ROBOTICS LAB · 3R</p>
          <h1 aria-label="空间 3R 机器人学交互实验室">空间 3R 机器人学交互实验室</h1>
        </div>
        <p className="lab-masthead__note">
          偏航–俯仰–俯仰教学模型<br />
          SI 制 · 内部角度使用弧度
        </p>
      </header>

      <Navigation activeModule={activeModule} onSelect={setActiveModule} />

      <section className="module-heading" aria-labelledby="module-title">
        <span>{moduleIndex}</span>
        <div>
          <p>{moduleCopy.kicker}</p>
          <h2 id="module-title">{moduleCopy.title}</h2>
          <p>{moduleCopy.summary}</p>
        </div>
        <StatusBanner
          tone={calculation.singularity.isSingular ? 'warning' : 'success'}
          title={calculation.singularity.isSingular ? '接近奇异位形' : '计算状态正常'}
        >
          结果图修订 {calculation.revision}
        </StatusBanner>
      </section>

      {activeModule === 'model' && <RobotModelPage />}
      {activeModule === 'kinematics' && <KinematicsPage />}
      {activeModule === 'dynamics' && <DynamicsPage />}
      {activeModule === 'experiments' && <ExperimentPage />}
    </main>
  )
}
