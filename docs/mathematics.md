# 数学模型与数值实现

本文档描述当前 TypeScript 实现使用的公式。向量均为列向量。运动学教学界面显示毫米和度；核心计算仍以米、弧度和其他 SI 单位运行，显示转换不进入动力学模型。

运动学教学链明确分工：正运动学使用 D–H 矩阵法，逆运动学使用解析几何法，解析解使用正运动学回代验证，雅可比使用关节轴叉乘的几何构造法。“代入当前参数”只表示把具体值代入解析公式，不表示迭代求解。

## 1. 标准 D–H 模型与正运动学

第 $i$ 个标准 D–H 变换为

$$
{}^{i-1}T_i=R_z(\theta_i)T_z(d_i)T_x(a_i)R_x(\alpha_i).
$$

本机构的参数行为

| $i$ | $\theta_i$ | $d_i$ | $a_i$ | $\alpha_i$ |
|---:|---:|---:|---:|---:|
| 1 | $\theta_1$ | $d_1$ | $0$ | $\pi/2$ |
| 2 | $\theta_2$ | $0$ | $l_2$ | $0$ |
| 3 | $\theta_3$ | $0$ | $l_3$ | $0$ |

于是

$$ {}^0T_3={}^0T_1{}^1T_2{}^2T_3,\qquad {}^0p_e=\operatorname{trans}({}^0T_3). $$

界面显示数值齐次矩阵时，只将前三行第四列乘以 $1000$；左上角旋转块保持无量纲。

三维教学场景采用

$$
{}^WT_0=I_4,
$$

所以世界坐标系 $\{W\}$ 与基座坐标系 $\{0\}$ 当前重合。$\{1\},\{2\},\{3\}$ 的位置和方向直接取自同一套正运动学累计变换；当前没有额外工具偏置，因此 $\{e\}\equiv\{3\}$，场景合并显示为 $\{3\}/\{e\}$。

D–H 教学播放器依次展示 $R_z(\theta_i)$、$T_z(d_i)$、$T_x(a_i)$、$R_x(\alpha_i)$ 的累计结果。这是相邻坐标系的构造分解，不是机器人真实运动轨迹。点击 D–H 表的行或参数只改变当前教学焦点，矩阵数值仍来自上述标准 D–H 变换。

## 2. 末端姿态

令 $\beta=\theta_2+\theta_3$、$c_{23}=\cos\beta$、$s_{23}=\sin\beta$，则

$$
{}^0R_3=
\begin{bmatrix}
c_1c_{23} & -c_1s_{23} & s_1\\
s_1c_{23} & -s_1s_{23} & -c_1\\
s_{23} & c_{23} & 0
\end{bmatrix}.
$$

末端坐标系与第三连杆坐标系重合，即 $\{e\}\equiv\{3\}$。机构自然姿态参数为基座方位角 $\theta_1$ 和末端连杆仰角 $\beta$。这个 3R 机构不能独立满足任意六维目标位姿，因此界面展示每个位置逆解实际达到的姿态，而不提供任意目标旋转输入。

## 3. 解析几何逆运动学

对目标 ${}^0p_d=[x,y,z]^T$，令

$$
r=\sqrt{x^2+y^2},\qquad h=z-d_1,\qquad s=\sqrt{r^2+h^2}.
$$

$r$ 是目标的水平投影半径，$h$ 是相对肩部高度，$s$ 是肩部到目标的距离。由 $l_2,l_3,s$ 构成的三角形可得

$$
s^2=r^2+h^2.
$$

构成连杆三角形前先检查

$$
|l_2-l_3|\le s\le l_2+l_3.
$$

随后对边长为 $l_2,l_3,s$ 的连杆三角形使用余弦定理：

$$
s^2=l_2^2+l_3^2+2l_2l_3\cos\theta_3,
$$

所以

$$
D=\cos\theta_3=\frac{s^2-l_2^2-l_3^2}{2l_2l_3}
=\frac{r^2+h^2-l_2^2-l_3^2}{2l_2l_3},
\qquad \theta_3=\pm\arccos D.
$$

若 $|D|>1$，目标不可达（仅将 $10^{-10}$ 内的浮点越界钳位）。对常规或折叠径向族，令

$$
r_s\in\{r,-r\},\qquad
\theta_1=\begin{cases}
\operatorname{atan2}(y,x),&r_s=r,\\
\operatorname{wrap}(\operatorname{atan2}(y,x)+\pi),&r_s=-r.
\end{cases}
$$

常规径向族先取 $r_s=r$。目标方向角是肩部指向目标的方向：

$$
\gamma=\operatorname{atan2}(h,r).
$$

令连杆三角形沿第二连杆方向的两个合成分量为

$$
k_1=l_2+l_3\cos\theta_3,\qquad k_2=l_3\sin\theta_3.
$$

第二连杆相对肩部—目标连线的补偿角和肩角为

$$
\delta=\operatorname{atan2}(k_2,k_1),
\qquad \boxed{\theta_2=\gamma-\delta}.
$$

折叠径向族再作为扩展情况取 $r_s=-r$，并使用 $\theta_1'=\operatorname{wrap}(\theta_1+\pi)$、$\gamma'=\operatorname{atan2}(h,-r)$。实现按当前参考构型选择较近的径向族并过滤关节限位。$r\approx0$ 时采用 $\theta_1=0$ 约定并报告轴线奇异。肘上、肘下由 $\theta_3$ 的正负决定；常规、折叠由 $r_s$ 的正负决定，二者是独立分类。

## 4. 正运动学回代验证

对第 $i$ 组解析解，重新计算

$$
{}^0T_{3,\mathrm{FK}}^{(i)}
=A_1(q_1^{(i)})A_2(q_2^{(i)})A_3(q_3^{(i)}),
\qquad
p_{\mathrm{FK}}^{(i)}=\operatorname{trans}({}^0T_{3,\mathrm{FK}}^{(i)}).
$$

界面逐项展示目标位置、回代位置、三个方向的分量误差及其范数：

$$
\Delta p^{(i)}=p_{\mathrm{FK}}^{(i)}-p_d,
\qquad e_p^{(i)}=\|\Delta p^{(i)}\|_2.
$$

解析回代误差接近浮点精度时，三维场景会把误差标记放大到可见长度，并明确注明“已放大，仅用于观察”；表格中的 $\Delta p$ 和 $e_p$ 始终保留真实数值。

姿态验证链为

$$
q_{\mathrm{IK}}^{(i)}\longrightarrow
\beta^{(i)}=\theta_2^{(i)}+\theta_3^{(i)}
\longrightarrow{}^0R_{3,\mathrm{FK}}^{(i)}.
$$

两组解析解可以到达同一位置，但 $\beta$ 和实际旋转矩阵可能不同；当前求解的是目标位置，而不是任意六维目标位姿。

## 5. 几何雅可比与奇异性

对第 $j$ 个转动关节，令 ${}^0z_{j-1}$ 为关节轴，${}^0p_{j-1}$ 为关节原点，则

$$
J_{v,j}={}^0z_{j-1}\times({}^0p_e-{}^0p_{j-1}),\qquad
J_{\omega,j}={}^0z_{j-1}.
$$

$$
\begin{bmatrix}{}^0v_e\\{}^0\omega_e\end{bmatrix}
=J(q)\dot q,
\qquad J=\begin{bmatrix}J_v\\J_\omega\end{bmatrix}.
$$

位置奇异性由 $J_v$ 的奇异值判断：$\sigma_{\min}$ 低于阈值时报警；条件数为 $\kappa=\sigma_{\max}/\sigma_{\min}$，数值零时为无穷大。界面还给出两种互补的位置灵巧度指标：

$$
\mu=\sqrt{\det(J_vJ_v^T)}=\sigma_1\sigma_2\sigma_3,
\qquad
\eta=\frac{1}{\kappa}=\frac{\sigma_{\min}}{\sigma_{\max}}.
$$

$\mu$ 是 Yoshikawa 位置可操作度，反映速度椭球体积；$\eta\in[0,1]$ 是逆条件数，反映各方向的均衡程度。两者在位置奇异位形均趋近于零。这里只评价 $J_v$，避免把线速度和角速度的不同单位混在同一个指标中。

运动学界面以毫米和度展示位置雅可比：

$$
J_{v,\mathrm{mm}/{}^\circ}=1000\frac{\pi}{180}J_{v,\mathrm{m}},
$$

角速度块保持无量纲。位置奇异值使用相同缩放，条件数、逆条件数和奇异判定不变。界面显示的可操作度按缩放因子的三次方换算为 $(\mathrm{mm}/{}^\circ)^3$。

## 6. 惯性、科氏与重力

刚体动力学方程为

$$
\tau=M(q)\ddot q+C(q,\dot q)\dot q+g(q)+\tau_f,
\qquad \tau_f=B\dot q.
$$

连杆质心线速度和角速度雅可比分别为 $J_{v_i}$、$J_{\omega_i}$，连杆坐标系惯量为 ${}^{c_i}I_i$。惯性矩阵按

$$
M(q)=\sum_{i=1}^{3}\left[
m_iJ_{v_i}^TJ_{v_i}+J_{\omega_i}^T
{}^0R_i{}^{c_i}I_i{}^0R_i^TJ_{\omega_i}
\right]
$$

构造。第一类 Christoffel 系数与科氏矩阵为

$$
c_{ijk}=\frac12\left(
\frac{\partial M_{ij}}{\partial q_k}+
\frac{\partial M_{ik}}{\partial q_j}-
\frac{\partial M_{jk}}{\partial q_i}
\right),\qquad
C_{ij}=\sum_{k=1}^{3}c_{ijk}\dot q_k.
$$

Christoffel 关系按上式精确定义；实现以中心差分

$$
\frac{\partial M}{\partial q_k}\approx
\frac{M(q+h e_k)-M(q-h e_k)}{2h},\qquad h=10^{-5}\ \mathrm{rad}
$$

求质量矩阵偏导。重力势能及广义重力项为

$$
V(q)=-\sum_{i=1}^{3}m_i\,{}^0g^T{}^0p_{c_i}(q),
$$

$$
g(q)=\frac{\partial V}{\partial q}
=-\sum_{i=1}^{3}m_iJ_{v_i}^T{}^0g.
$$

## 7. 能量与功率

$$
K=\frac12\dot q^TM(q)\dot q,\qquad E=K+V,\qquad P_i=\tau_i\dot q_i.
$$

界面同时显示 $K$、$V$、$E$ 和各关节功率 $P_i$。

## 8. 轨迹

五次点到点轨迹令 $u=t/T$、$s(u)=10u^3-15u^4+6u^5$：

$$
q(t)=q_0+(q_f-q_0)s(u),
$$

$$
\dot q(t)=(q_f-q_0)\frac{30u^2-60u^3+30u^4}{T},
$$

$$
\ddot q(t)=(q_f-q_0)\frac{60u-180u^2+120u^3}{T^2}.
$$

梯形速度轨迹采用抛物线过渡。令归一化加速段时长 $u_b=1/3$、归一化平台速度 $v_c=3/2$、归一化加速度 $a=9/2$，则

$$
s(u)=
\begin{cases}
\frac12au^2,&0\le u\le u_b,\\
v_cu-\frac14,&u_b<u\le1-u_b,\\
1-\frac12a(1-u)^2,&1-u_b<u\le1,
\end{cases}
$$

$$
q(t)=q_0+(q_f-q_0)s(u),\qquad
\dot q(t)=\frac{q_f-q_0}{T}s'(u),\qquad
\ddot q(t)=\frac{q_f-q_0}{T^2}s''(u).
$$

三个关节共享同一个 $s(u)$，因此无论各自位移大小如何，都会同时开始并同时到达终点。加速度在三段交界处不连续，这是梯形速度轮廓的预期性质。

正弦轨迹逐关节定义为

$$
q_i(t)=\bar q_i+A_i\sin(2\pi f_it+\phi_i),
$$

$$
\dot q_i(t)=A_i(2\pi f_i)\cos(2\pi f_it+\phi_i),\qquad
\ddot q_i(t)=-A_i(2\pi f_i)^2\sin(2\pi f_it+\phi_i).
$$

逆动力学实验把上述 $q,\dot q,\ddot q$ 代入完整动力学方程。

## 9. 正动力学与 RK4

正动力学状态 $y=[q^T,\dot q^T]^T$ 满足

$$
\dot y=f(t,y)=\begin{bmatrix}
\dot q\\M(q)^{-1}[\tau(t)-C(q,\dot q)\dot q-g(q)-B\dot q]
\end{bmatrix}.
$$

每个固定步长 $h$ 使用经典四阶 Runge–Kutta：

$$
k_1=f(t,y),\quad
k_2=f(t+h/2,y+hk_1/2),\quad
k_3=f(t+h/2,y+hk_2/2),\quad
k_4=f(t+h,y+hk_3),
$$

$$ y_{n+1}=y_n+\frac{h}{6}(k_1+2k_2+2k_3+k_4). $$

力矩允许常值、阶跃、正弦和分段常值。积分器在力矩不连续点拆分步长，并确保样本包含精确终止时刻。当前实现不采用自适应步长；若出现非有限数值、病态惯性矩阵或越过关节限位，实验会给出诊断并停止。
