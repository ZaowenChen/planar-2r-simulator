# Robotics Toolbox for Python 参考仓库评估

本文记录当前项目对 `petercorke/robotics-toolbox-python` 的定向参考，避免把“参考一个大型工具箱”变成无边界的功能搬运。

## 对照基线

- 仓库：<https://github.com/petercorke/robotics-toolbox-python>
- 本次检查提交：`9487d203d38dfc88eeacf6cce38994be37e9fd31`
- 提交日期：2026-08-18
- 参考项目许可证：MIT License
- 重点阅读范围：`robot/Robot.py`、`robot/RobotKinematics.py`、`tools/trajectory.py`、`models/`、`tests/`

参考仓库是面向研究与教学的通用 Python 机器人学工具箱；当前项目则是固定偏航–俯仰–俯仰空间 3R 机构的浏览器教学应用。因此适合借鉴的是数学能力分层、指标定义、轨迹类型和验证思路，不适合直接复制通用机器人对象体系或 Python 后端。

## 本次已经采用

| 参考能力 | 当前项目落地 | 取舍 |
|---|---|---|
| `trapezoidal` / `mtraj` | 动态实验新增三关节同步梯形速度轨迹 | 固定自动平台速度，不增加容易误用的逐轴速度参数 |
| `manipulability(..., method="yoshikawa")` | 对位置雅可比显示 $\mu=\prod_i\sigma_i$ | 只使用 $J_v$，不混合线速度与角速度单位 |
| `manipulability(..., method="invcondition")` | 显示 $\eta=\sigma_{\min}/\sigma_{\max}$ | 与现有条件数和奇异阈值并列解释 |
| 轨迹与算法的分层测试 | 增加端点、分段、钳位和界面入口测试 | 延续 Vitest 与 Playwright 技术栈 |

这些算法按公开数学定义重新写成 TypeScript，并适配现有 SI 核心、中文教学界面和三关节元组类型；没有把参考仓库作为运行时依赖。

## 后续建议

1. **数值逆运动学对照模式**：加入 Levenberg–Marquardt 或 Gauss–Newton 求解器，并与当前解析 3R 解并排比较收敛过程、初值敏感性、关节限位和残差。教学价值高，但应保留解析法为主线。
2. **笛卡尔位姿轨迹**：实现位置插值与 SO(3) 姿态插值，再通过数值 IK 生成关节轨迹。当前 3R 不能跟踪任意六维位姿，需要先设计任务维度或 mask。
3. **速度椭球可视化**：把 $J_vJ_v^T$ 的特征向量和奇异值映射为 3D 椭球，与本次新增的 $\mu$、$\eta$ 数值联动。
4. **微分运动控制**：参考 resolved-rate motion control 示例，展示 $\dot q=J^+v_d$、阻尼伪逆以及接近奇异位形时的速度放大。
5. **模型导入层**：只有在项目计划扩展到多种机械臂时，再考虑 URDF 或通用 D–H 模型目录；对当前单一 3R 教学目标，过早引入会显著增加状态、渲染与验证复杂度。

## 不建议直接搬入

- 移动机器人、SLAM、路径规划和碰撞后端与当前串联机械臂教学主线无关。
- Python/C++ 加速扩展不适合浏览器静态部署目标。
- 参考仓库的通用 `Robot` / `ERobot` / `DHRobot` 类层级远大于当前固定 3R 模型所需；应在确认多机器人需求后再抽象。
- 参考仓库源码即使采用 MIT License，复制实质性代码时仍需要保留其版权与许可文本；当前实现选择数学重写以保持依赖和版权边界清晰。
