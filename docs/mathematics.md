# 数学模型与数值实现

本文档描述当前 TypeScript 实现使用的公式。向量均为列向量，内部角度单位为弧度，所有物理量采用 SI 制。

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

## 2. 解析逆运动学

对目标 ${}^0p_d=[x,y,z]^T$，令

$$ r=\sqrt{x^2+y^2},\qquad z'=z-d_1,\qquad
D=\frac{r^2+z'^2-l_2^2-l_3^2}{2l_2l_3}. $$

若 $|D|>1$，目标不可达（仅将 $10^{-10}$ 内的浮点越界钳位）。两种肘部分支为

$$
\theta_3=\pm\arccos D,
$$

$$
\theta_1=\operatorname{atan2}(y,x),\qquad
\theta_2=\operatorname{atan2}(z',r)-
\operatorname{atan2}(l_3\sin\theta_3,l_2+l_3\cos\theta_3).
$$

实现还保留 $r\mapsto-r$、$\theta_1\mapsto\theta_1+\pi$ 的折叠径向族，按当前参考姿态选取较近解，并过滤关节限位。$r\approx0$ 时采用 $\theta_1=0$ 约定并报告轴线奇异。

## 3. 几何雅可比与奇异性

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

位置奇异性由 $J_v$ 的奇异值判断：$\sigma_{\min}$ 低于阈值时报警；条件数为 $\kappa=\sigma_{\max}/\sigma_{\min}$，数值零时为无穷大。

## 4. 惯性、科氏与重力

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

## 5. 能量与功率

$$
K=\frac12\dot q^TM(q)\dot q,\qquad E=K+V,\qquad P_i=\tau_i\dot q_i.
$$

界面同时显示 $K$、$V$、$E$ 和各关节功率 $P_i$。

## 6. 轨迹

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

正弦轨迹逐关节定义为

$$
q_i(t)=\bar q_i+A_i\sin(2\pi f_it+\phi_i),
$$

$$
\dot q_i(t)=A_i(2\pi f_i)\cos(2\pi f_it+\phi_i),\qquad
\ddot q_i(t)=-A_i(2\pi f_i)^2\sin(2\pi f_it+\phi_i).
$$

逆动力学实验把上述 $q,\dot q,\ddot q$ 代入完整动力学方程。

## 7. 正动力学与 RK4

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
