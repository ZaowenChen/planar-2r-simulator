# 3D 3R 机械臂正逆运动学与位姿可视化仿真器设计

## 目标

用 Python、NumPy 和 Matplotlib 实现一个单文件、可直接运行的交互式 3D 机械臂仿真器。仿真器把《机器人学导论》第 2、3、4 章的空间描述、坐标变换、D-H 正运动学、闭式逆运动学、多解与工作空间融入同一界面，形成“参数 → 变换 → 位姿 → 图形”的教学闭环。

新功能作为独立文件加入项目，不修改已有的平面 2R 仿真器。

## 机械臂几何

采用偏置底座上的 yaw–pitch–pitch 3R 机构：

- 关节 1：绕基坐标系 Z 轴旋转，关节变量为 `theta1`。
- 关节 2：肩部抬起，关节变量为 `theta2`。
- 关节 3：肘部弯曲，关节变量为 `theta3`。
- 基座高度：`d1 = 0.8`。
- 上臂长度：`L2 = 2.0`。
- 前臂长度：`L3 = 1.5`。

坐标系原点为：地面基座 `O0`、肩部 `O1`、肘部 `O2` 和末端 `O3`。`{B}` 位于 `O0`，`{W}` 与 `{T}` 均位于 `O3`。默认工具标定变换为 `T_W^T = I`，因此腕部和工具坐标系重合，在图中标记为 `{W}/{T}`。

## 架构

采用“纯数学核心 + GUI 类”的单文件结构：

- `DHParameters`：不可变的标准 D-H 参数行。
- `ForwardKinematicsResult3D`：保存 `T01/T12/T23/T02/T03`、各原点和 D-H 参数。
- `IKSolutions`：保存肘下和肘上两组关节角。
- `dh_transform(theta, a, alpha, d)`：构建单连杆标准 D-H 变换。
- `forward_kinematics(theta1, theta2, theta3, d1, l2, l3)`：矩阵连乘并提取各原点。
- `inverse_kinematics(target, d1, l2, l3)`：返回两组闭式位置逆解。
- `invert_transform(transform)`：按旋转矩阵转置公式快速求逆。
- `rotation_to_rpy(rotation)`：按 ZYX 约定提取 Roll、Pitch、Yaw。
- `Robot3DSimulator`：只负责 Matplotlib 图形对象、控件、状态与事件回调。
- `main()`：创建默认仿真器并进入事件循环。

数学函数不依赖 Matplotlib 状态，可以独立测试。GUI 复用线条与文本对象；3D quiver 箭头在更新时按小规模集合重建。

## 标准 D-H 模型与正运动学

单连杆标准 D-H 变换为：

```text
T(theta, a, alpha, d) =
[[cos(theta), -sin(theta)cos(alpha),  sin(theta)sin(alpha), a cos(theta)],
 [sin(theta),  cos(theta)cos(alpha), -cos(theta)sin(alpha), a sin(theta)],
 [0,           sin(alpha),            cos(alpha),           d           ],
 [0,           0,                     0,                    1           ]]
```

完整 D-H 表：

| i | `a_(i-1)` | `alpha_(i-1)` | `d_i` | `theta_i` |
| --- | ---: | ---: | ---: | ---: |
| 1 | `0` | `+pi/2` | `d1=0.8` | `theta1` |
| 2 | `L2=2.0` | `0` | `0` | `theta2` |
| 3 | `L3=1.5` | `0` | `0` | `theta3` |

计算链为：

```text
T01 = DH(theta1, 0,  pi/2, d1)
T12 = DH(theta2, L2, 0,    0)
T23 = DH(theta3, L3, 0,    0)
T02 = T01 @ T12
T03 = T02 @ T23
```

原点由累计变换的平移列提取：`O0=(0,0,0)`，`O1=T01[:3,3]`，`O2=T02[:3,3]`，`O3=T03[:3,3]`。局部坐标轴方向由相应累计变换的旋转列取得。

用于独立核验的解析位置公式为：

```text
rho = L2 cos(theta2) + L3 cos(theta2 + theta3)
x = rho cos(theta1)
y = rho sin(theta1)
z = d1 + L2 sin(theta2) + L3 sin(theta2 + theta3)
```

## 空间描述、位姿和快速逆变换

实时面板显示：

- 末端位置 `P = (x, y, z)`。
- `R03 = T03[:3,:3]`。
- 完整 `T03`。
- ZYX Roll–Pitch–Yaw，满足 `R = Rz(yaw) @ Ry(pitch) @ Rx(roll)`。
- 快速逆变换 `T30`。
- 验证误差 `||T03 @ T30 - I||_F`。

快速逆变换实现：

```text
T^-1 = [[R^T, -R^T P],
        [  0,       1]]
```

RPY 提取在 `cos(pitch)` 远离零时采用常规 `atan2` 公式；接近万向节锁时固定 Yaw 为零，并从剩余旋转元素求 Roll，确保返回有限稳定值。

## 闭式逆运动学与多解

令：

```text
r = sqrt(x_target^2 + y_target^2)
z_offset = z_target - d1
D = (r^2 + z_offset^2 - L2^2 - L3^2) / (2 L2 L3)
```

两组解为：

```text
theta1 = atan2(y_target, x_target)
theta3_down = +acos(D)
theta3_up   = -acos(D)
theta2 = atan2(z_offset, r)
         - atan2(L3 sin(theta3), L2 + L3 cos(theta3))
```

`Solution 1` 对应肘下解，`Solution 2` 对应肘上解。边界处两解允许重合。若 `r` 接近零，则底座旋转不唯一，纯函数约定 `theta1=0` 并在结果中标记轴向奇异性，GUI 显示提示。

若 `|D| > 1 + tolerance`，抛出 `UnreachableTargetError`。若只因浮点误差轻微越界，则先裁剪至 `[-1,1]`。GUI 捕获不可达异常、保留当前有效姿态，并显示红色状态文本。

## 3D 工作空间

工作空间点云在 GUI 初始化时一次性生成，之后保持静态。采样范围与关节滑块一致：

- `theta1`：`[-180°, 180°]`。
- `theta2`：`[-90°, 90°]`。
- `theta3`：`[-150°, 150°]`。

使用约 8,000 个结构化角度样本，通过向量化解析 FK 得到末端坐标，绘制为尺寸小、透明度低的浅青色散点。点云展示实际关节范围下的可达包络，不遮挡当前机械臂。

## 3D 可视化

主视图占窗口左侧约 65%，包含：

- 粗线与球形标记表示 `O0 → O1 → O2 → O3`。
- 在 `{B}`、`{1}`、`{2}` 和 `{W}/{T}` 处绘制局部坐标轴：X 红、Y 绿、Z 蓝。
- 坐标系文本标签与 `T01/T12/T23` 变换链标签。
- 从 `{B}` 指向 `{T}` 的虚线位置向量。
- 浅色工作空间点云。
- 等比例 XYZ 范围、网格和轴标签。

Matplotlib 的 3D 坐标轴原生支持鼠标拖拽旋转和滚轮缩放。

## 公式与 D-H 表可视化

右下数学面板包含：

- 标准 D-H 乘积公式 `Rz(theta) Tz(d) Tx(a) Rx(alpha)`。
- `T03 = T01 T12 T23`。
- 快速逆变换公式。
- 闭式 IK 的 `D`、`theta3` 和 `theta2` 公式。
- 三行动态 D-H 表。

D-H 表中的三个关节角显示当前角度值，随着 FK 滑块或 IK 解实时更新。右上数值面板同步显示位置、`R03`、`T03`、RPY、`T30` 和逆变换误差，从而把公式、数值矩阵与 3D 姿态连接起来。

## 控件与状态流

底部控制区包含六个 Slider、一个 Button 和一个 RadioButtons：

- FK 滑块：`theta1 [-180°,180°]`、`theta2 [-90°,90°]`、`theta3 [-150°,150°]`。
- IK 滑块：`X/Y [-3.5,3.5]`、`Z [-2.7,4.3]`。
- 模式按钮：在 FK 与 IK 之间切换。
- IK 解单选：`Solution 1: Elbow Down`、`Solution 2: Elbow Up`。

初始模式为 FK，初始角度为 `theta1=30°`、`theta2=25°`、`theta3=-50°`。

模式行为：

1. FK 模式中，关节角滑块直接驱动正运动学；目标滑块与解选择被停用并变暗。
2. 切换到 IK 时，目标滑块先同步到当前末端坐标，再启用目标和解选择控件。
3. IK 模式中，目标坐标或解选择变化会调用闭式 IK，再用 FK 更新图形与位姿面板。
4. 切回 FK 时，关节角滑块同步到当前 IK 解，再重新启用。
5. 内部 `_updating_controls` 标志防止程序性滑块同步引起递归回调。

状态栏显示当前模式、IK 分支、不可达目标或轴向奇异性提示。

## 输入验证与错误处理

- `d1` 必须为有限非负数。
- `L2`、`L3` 必须为有限正数。
- 角度、目标坐标和变换矩阵元素必须有限。
- `invert_transform` 要求输入为 `4×4` 齐次变换；验证末行和旋转矩阵正交性。
- `rotation_to_rpy` 要求输入为有限 `3×3` 矩阵。
- GUI 中不可达 IK 不覆盖最后一个有效 FK 结果。

## 测试与验收

使用 Python 标准库 `unittest`，不增加运行依赖。覆盖：

- 通用非零 `theta/a/alpha/d` D-H 变换。
- `T03 = T01 @ T12 @ T23`。
- 矩阵 FK 位置与独立解析公式一致。
- 各原点和旋转坐标轴形状正确。
- 快速逆变换与 NumPy 通用求逆一致，且双向乘积为单位阵。
- 普通姿态及万向节锁附近的 RPY 重建。
- 肘下和肘上 IK 经 FK 后都返回目标点。
- 不可达目标、边界重合解与轴向奇异性。
- GUI 默认控件范围、工作空间点云、四组局部坐标系和动态 D-H 表。
- FK/IK 模式切换、滑块同步、解分支切换、不可达状态保持。
- Matplotlib `Agg` 后端可完成无界面渲染。

最终运行完整测试、Python 语法编译、无界面渲染并保存预览图。人工检查预览中的 3D 机械臂、点云、四组 XYZ 坐标轴、姿态矩阵、公式、D-H 表和控件是否完整且无裁切。

## 交付物

- `robot_3r_3d_simulator.py`：单文件交互式仿真器，仅依赖 NumPy 和 Matplotlib。
- `test_robot_3r_3d_simulator.py`：标准库单元与无界面 GUI 测试。
- `artifacts/robot_3r_3d_preview.png`：视觉验收预览。

运行方式：

```bash
python3 robot_3r_3d_simulator.py
```
