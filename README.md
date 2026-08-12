# 空间 3R 机器人学交互实验室

一个无需登录、完全在浏览器本地运行的中文机器人学教学应用。它围绕偏航–俯仰–俯仰空间 3R 串联机器人，将 3D 机构、公式、矩阵与时间响应放在同一个可编辑工作台中。

在线演示：[GitHub Pages](https://zaowenchen.github.io/planar-2r-simulator/)（仓库启用 Pages 后可匿名访问）

![空间 3R 机器人学交互实验室预览](docs/robotics-lab-preview.png)

## 学习模块

- **机器人模型**：编辑连杆几何、质量与重力，查看标准 D–H 表和坐标系约定。
- **运动学**：联动关节角、正运动学末端位姿、几何雅可比、奇异性指标和解析逆解分支。
- **动力学**：编辑质量、质心、惯量、重力与粘性摩擦，检查惯性矩阵、科氏项、重力项、力矩、能量和功率。
- **动态实验**：用五次多项式或正弦轨迹做逆动力学，或以常值、阶跃、正弦、分段常值力矩做正动力学；动画、曲线与公式共享时间轴，可导出 CSV。

## 公式速览

标准 D–H 变换与整机变换为

$$ {}^{i-1}T_i=R_z(\theta_i)T_z(d_i)T_x(a_i)R_x(\alpha_i), \qquad {}^0T_3={}^0T_1{}^1T_2{}^2T_3. $$

速度与刚体动力学模型为

$$ \begin{bmatrix}{}^0v_e\\{}^0\omega_e\end{bmatrix}=J(q)\dot q, \qquad
\tau=M(q)\ddot q+C(q,\dot q)\dot q+g(q)+B\dot q. $$

完整推导和实现对应关系见 [数学模型](docs/mathematics.md)，界面符号与 SI 单位见 [符号表](docs/symbols.md)。

## 本地运行与验证

需要 Node.js 20.19 或更高版本，以及支持 WebGL 2 的桌面浏览器。

```bash
npm ci
npm run dev
```

浏览器访问 `http://localhost:5173`。完整验证：

```bash
npm test
npm run typecheck
npm run build
npx playwright install chromium
npm run e2e
python3 -m unittest discover -s legacy/python -p 'test_*.py'
```

推荐当前稳定版 Chrome、Edge 或 Firefox；Safari 需要支持 WebGL 2。CI 使用 Chromium 执行真实浏览器验收，并在 `main` 分支发布静态产物到 GitHub Pages。

## 数值方法与适用范围

- 三角函数和动力学内部统一使用弧度，长度、质量、时间、力和力矩采用 SI 制；运动学面板可用角度输入。
- 惯性矩阵由各连杆质心线速度/角速度雅可比构造；Christoffel 符号按精确定义组织，质量矩阵偏导用中心差分、默认步长 `1e-5 rad` 求值。
- 正动力学采用固定步长四阶 Runge–Kutta（RK4），允许步长 `0.001–0.02 s`；不提供自适应误差估计，不应用于实时控制或安全关键计算。
- 奇异性以位置雅可比块的奇异值判断；3D 箭头为便于辨认会归一化，真实量值在旁侧图例给出。
- 逆运动学是该偏航–俯仰–俯仰结构的解析解，并按关节限位过滤；不是任意机械臂求解器。

`legacy/` 保留早期 Python/Matplotlib 平面 2R 与空间 3R 演示及其测试，仅用于历史对照；当前产品入口是 React 应用。

## 许可与引用

当前仓库未附带开源许可证；默认保留所有权利。公开复用、分发或改编前请先取得作者许可。

课程作业、文章或演示可建议引用为：

> Chen, Zaowen. *空间 3R 机器人学交互实验室（Interactive 3R Robotics Lab）*. GitHub repository, 2026. https://github.com/ZaowenChen/planar-2r-simulator
