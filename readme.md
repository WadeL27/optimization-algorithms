# Control Parameterization in Python

Python implementation of control parameterization for optimal control problems.

控制参数化方法的 Python 实现，用于求解最优控制问题。

---

## Problem Formulation / 问题形式

Consider the optimal control problem:

$$
\min_u \; J = \int_0^T u(t)^\top u(t) \, dt
$$

subject to the dynamics, boundary conditions, and state constraints:

$$
\dot{x} = f(x, u), \quad x(0) = x_0, \quad x(T) = x_f, \quad g(x(t)) \geq 0
$$

**Control parameterization** discretizes the time horizon into $N$ intervals and
optimizes the control values $u_0, \dots, u_{N-1}$ at the nodes only. Within each
interval, the control is held constant (zero-order hold). This reduces the
infinite-dimensional optimal control problem to a finite-dimensional nonlinear
program.

**控制参数化**将时间区间离散为 $N$ 段，只在节点处优化控制值
$u_0, \dots, u_{N-1}$，区间内控制量保持不变（零阶保持）。由此把无穷维最优
控制问题转化为有限维非线性规划。

---

## Algorithm / 算法流程

1. **Parameterize** the control by node values $u_0, \dots, u_{N-1}$.
   将控制量参数化为节点值。

2. **Simulate** the state trajectory via fixed-step RK4 with zero-order hold.
   用固定步长 RK4 和零阶保持仿真状态轨迹。

3. **Evaluate** the cost $J = \int u^\top u \, dt + \rho \|x(T) - x_f\|^2$.
   计算目标函数：控制能量 + 终端惩罚。

4. **Optimize** over $u$ using `scipy.optimize.minimize`.
   用 `scipy.optimize.minimize` 对 $u$ 做优化。

State constraints are handled by either:
- **Hard constraints**: SLSQP with inequality constraints $g(x) \geq 0$.
- **Quadratic penalty**: BFGS on $J + \rho \sum \max(0, -g(x))^2$.

状态约束有两种处理方式：
- **硬约束**：用 SLSQP 直接处理不等式约束 $g(x) \geq 0$。
- **惩罚法**：在目标函数上加 $\rho \sum \max(0, -g(x))^2$，用 BFGS 求解。

---

## Repository Structure / 目录结构
control_param/
├── problem_setting/ # problem definitions / 问题定义
│ ├── double_integrator.py
│ ├── vehicle.py
│ └── vehicle_obstacle.py
├── solver/ # solver core / 求解器核心
│ ├── integrator.py # RK4, zero-order hold
│ └── optimize.py # objective, constraints, wrappers
└── examples/ # runnable scripts / 可运行示例
├── run_double_integrator.py
├── run_vehicle.py
└── run_vehicle_obstacle.py


---

## Examples / 示例

### 1. Double integrator (minimum energy)

### 2. Vehicle trajectory planning (bicycle model)
python -m control_param.examples.run_vehicle
State [x,y,θ,v][x,y,θ,v], control [a,ω][a,ω]. Terminal error ∼2×10−4∼2×10−4
at N=20N=20.

状态 [x,y,θ,v][x,y,θ,v]，控制 [a,ω][a,ω]。N=20N=20 时终端误差约 2×10−42×10−4。

### 3. Vehicle obstacle avoidance (state constraint)
python -m control_param.examples.run_vehicle_obstacle

Compares three strategies:
Method	terminal error	min clearance	running cost	nfev
Unconstrained BFGS	2.0e-4	-0.7916	4.7604	36885
SLSQP (two-stage)	1.6e-4	0.0000	4.8668	2939
Penalty BFGS (ρ=104ρ=104)	1.6e-4	-6e-6	4.8668	30918

Key findings / 主要结论:

    The unconstrained optimum violates the safety constraint by 0.79 units,
    confirming that state constraints fundamentally change the solution.
    无约束最优解违反安全约束 0.79 个单位，说明状态约束确实改变了最优解。

    SLSQP and penalty BFGS converge to nearly identical solutions, providing
    mutual validation of the result.
    SLSQP 与惩罚法收敛到几乎相同的解，两种方法互相验证。

    SLSQP is 10x more efficient (2939 vs 30918 function evaluations).
    SLSQP 效率高 10 倍（2939 vs 30918 次函数评估）。

    SLSQP stuck at u=0u=0 when started from zero initial guess; a two-stage
    strategy (BFGS seed then SLSQP) fixes this.
    SLSQP 从零初值会卡住；两阶段策略（先用 BFGS 得初值，再用 SLSQP）解决。

## SLSQP 从零初值卡住：现象与已验证事实

**已验证**：
1. 目标函数在 u=0 处的梯度为 -1.69e5（有限差分，步长 1e-2 ~ 1e-8 稳定）。
2. SLSQP 第一次迭代 J 从 7.4e5 跳到 2.0e13。
3. SLSQP 第二次迭代回到接近起点（g_min = 16.55）。
4. SLSQP 只做了 2 次梯度评估，6 次迭代后判定收敛。
5. 两阶段法（BFGS 预热）能正常收敛。

**待验证的假设**：
- 约束雅可比在 u=0 处是否异常小？（数据 2 可验证）
- SLSQP 第一次迭代的实际步长有多大？（数据 1 或 3 可验证）
- QP 子问题的搜索方向是否合理？（需读 scipy 源码）

**当前结论**：
SLSQP 对“零初值 + 非线性约束”敏感，两阶段法是有效修复。
具体原因仍需进一步验证。


## Requirements / 依赖
numpy
scipy
matplotlib

## Related Work / 相关工作
Original MATLAB implementation of the author's control parameterization
method (PhD research, Shanghai University).

## Author / 作者

Shanghai University, Department of Mathematics
Optimal Control Group LWD