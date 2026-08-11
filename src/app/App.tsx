const modules = ['机器人模型', '运动学', '动力学', '动态实验'] as const

export function App() {
  return (
    <main>
      <header><p>ROBOTICS LAB · 3R</p><h1>空间 3R 机器人学交互实验室</h1></header>
      <nav aria-label="学习模块">
        {modules.map((label) => <button key={label} type="button">{label}</button>)}
      </nav>
    </main>
  )
}
