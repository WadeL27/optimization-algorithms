# Day 4 Notes: State Constraints in Control Parameterization
# Day 4 笔记：控制参数化中的状态约束

## Problem / 问题

Vehicle obstacle avoidance with a circular safety region.
车辆在圆形安全区域内避障。

    minimize  J = ∫ u^T u dt
    subject to  ẋ = f(x, u)
                x(0) = x0,  x(T) = xf
                ||pos(t) - center||² ≥ r_safe²   for all t

Parameters / 参数:
    center  = (3, 3)
    r_safe  = 1.2
    T       = 6.0
    N       = 25
    x0      = (0, 0, 0, 0)
    xf      = (6, 6, π/2, 0)

State / 状态:   [x, y, θ, v]
Control / 控制: [a, ω]

---

## Three Methods Compared / 三种方法对比

### Method 1: Two-stage (BFGS seed + SLSQP hard constraint)
### 方法 1：两阶段（BFGS 预热 + SLSQP 硬约束）

Stage 1: unconstrained BFGS from u=0
Stage 2: SLSQP with inequality constraint, initialized from Stage 1

阶段 1：从 u=0 出发的无约束 BFGS
阶段 2：带不等式约束的 SLSQP，从阶段 1 结果出发

### Method 2: Quadratic penalty + BFGS
### 方法 2：二次惩罚 + BFGS

Minimize J + ρ · Σ max(0, -g(x))² with ρ = 1e4
最小化 J + ρ · Σ max(0, -g(x))²，取 ρ = 1e4

### Method 3: Baseline — SLSQP from u=0 (failed)
### 方法 3：基线——从 u=0 出发的 SLSQP（失败）

Kept here only as a diagnostic case.
仅作为诊断案例保留。

---

## Numerical Results / 数值结果

| Method / 方法                | terminal error 终端误差 | min clearance 最小裕度 | running cost 控制能量 | nfev 评估次数 |
|------------------------------|------------------------|------------------------|----------------------|--------------|
| Stage 1: unconstrained BFGS  | 2.0e-4                 | **-0.7916**            | 4.7604               | 36885        |
| Stage 2: SLSQP (hard)        | 1.6e-4                 | **0.0000**             | 4.8668               | 2939         |
| Penalty BFGS (ρ = 1e4)       | 1.6e-4                 | -6e-6                  | 4.8668               | 30918        |
| SLSQP from u=0 (baseline)    | 8.48                   | 16.55                  | 0.4113               | 107          |

### Key observations / 关键观察

1. **Unconstrained optimum violates the constraint by 0.79 units.**
   无约束最优解违反约束 0.79 个单位。
   The state constraint is not decorative — it fundamentally changes
   the optimal trajectory.
   状态约束不是装饰性的——它从根本上改变了最优轨迹。

2. **SLSQP and penalty BFGS converge to nearly identical solutions.**
   SLSQP 与惩罚法 BFGS 收敛到几乎相同的解。
   running cost / 控制能量:  4.866814 (SLSQP) vs 4.866810 (penalty)
   terminal error / 终端误差: 1.61e-4 vs 1.59e-4
   Two independent methods reaching the same solution is strong
   mutual validation of the result.
   两种独立方法得到同一解，是结果正确性的强有力互验。

3. **SLSQP is 10× more efficient.**
   SLSQP 效率高 10 倍。
   2939 evaluations vs 30918. Hard constraints avoid the need to
   tune ρ and use linearization of the constraints instead of
   repeatedly probing the objective.
   2939 次 vs 30918 次。硬约束无需调 ρ，用约束的线性化代替
   反复探测目标函数。

4. **Constraint enforcement has a cost.**
   约束执行有代价。
   Running cost increases from 4.76 (unconstrained) to 4.87 (+2.2%)
   to avoid the obstacle.
   为避障，控制能量从 4.76（无约束）增加到 4.87（+2.2%）。

---

## SLSQP from Zero Initial Guess: Diagnosis
## SLSQP 从零初值卡住：诊断

### Setup / 设置

Run SLSQP with `u_init = zeros(N*m)` and a callback that prints
J and g_min at each iteration.
用 `u_init = zeros(N*m)` 运行 SLSQP，并加 callback 打印每次迭代的 J 和 g_min。

### Callback output / Callback 输出

    iter 1: J = 2.0465e+13 | g_min = +7.735e+00
    iter 2: J = 7.1983e+05 | g_min = +1.655e+01
    Iterations: 6
    Function evaluations: 107
    Gradient evaluations: 2
    ||u_final|| = 1.309

### Verified facts (measured) / 已验证事实（有数据）

1. **Gradient at u=0 is significantly nonzero.**
   u=0 处梯度显著非零。
   Manual finite-difference check / 手动有限差分检查:
       h = 1e-02 → dJ/du_0 ≈ -1.691e+05
       h = 1e-04 → dJ/du_0 ≈ -1.693e+05
       h = 1e-06 → dJ/du_0 ≈ -1.693e+05
       h = 1e-08 → dJ/du_0 ≈ -1.693e+05
   Stable across 6 orders of magnitude. Rules out:
   六个数量级下稳定。排除：
       - "gradient too small" hypothesis / “梯度太小”假设
       - "finite-difference noise" hypothesis / “有限差分噪声”假设

2. **SLSQP's first iteration jumps to J = 2.05e13.**
   SLSQP 第一次迭代跳到 J = 2.05e13。
   From the initial J = 7.45e5, one iteration increases J by ~7
   orders of magnitude.
   从初始 J = 7.45e5，一次迭代让 J 增加约 7 个数量级。

3. **SLSQP's second iteration returns to near the starting point.**
   SLSQP 第二次迭代回到接近起点。
   g_min = 16.55 matches the clearance at (0, 0), i.e., vehicle
   did not move.
   g_min = 16.55 与 (0,0) 处的安全裕度吻合，即车辆没有移动。

4. **SLSQP terminates after only 2 gradient evaluations.**
   SLSQP 只做了 2 次梯度评估就终止。
   It did not iterate extensively — it declared convergence quickly.
   它没有大量迭代——很快判定了收敛。

5. **Two-stage strategy (BFGS seed then SLSQP) works.**
   两阶段策略（BFGS 预热 + SLSQP）有效。
   Stage 2 converges in 2939 evaluations with min clearance = 0.
   阶段 2 在 2939 次评估内收敛，最小裕度 = 0。

### Unverified hypotheses (not yet tested) / 未验证假设（尚未测试）

- *"Constraint Jacobian degenerates at u = 0."*
  *“约束雅可比在 u=0 处退化”。*
  Reasoning: position is the double integral of acceleration, so
  ∂g/∂u may be O(dt²). Not measured.
  推理：位置是加速度的两次积分，所以 ∂g/∂u 可能是 O(dt²)。未测量。
  To test: compute finite-difference Jacobian of g at u=0, check
  its Frobenius norm.
  验证方法：计算 g 在 u=0 处的有限差分雅可比，检查其 Frobenius 范数。

- *"Line search shrinks step size repeatedly and returns to start."*
  *“线搜索反复缩小步长并回到起点”。*
  Plausible narrative, but the callback only fires after each
  iteration, not inside the line search. Not directly observed.
  合理的叙事，但 callback 只在每次迭代后触发，不在线搜索内部。未直接观测。

- *"QP subproblem gives a bad direction at u=0."*
  *“QP 子问题在 u=0 处给出错误方向”。*
  Not inspected. Would require printing the search direction d
  inside scipy's `_slsqp_py.py`.
  未检查。需要在 scipy 的 `_slsqp_py.py` 里打印搜索方向 d。

### Working conclusion (for now) / 当前工作结论

SLSQP is sensitive to zero initial guess when constraints are
nonlinear functions of the state. A two-stage strategy — solve
unconstrained first to get a moving trajectory, then refine with
SLSQP — reliably fixes the issue. The exact internal mechanism
remains to be verified.
当约束是状态的非线性函数时，SLSQP 对零初值敏感。两阶段策略
（先无约束求得移动轨迹，再用 SLSQP 精化）能可靠修复。具体
内部机制仍待验证。

---

## New Tool: Callback-based Iteration Display
## 新工具：基于 callback 的迭代显示

Added to `solver/optimize.py`:
已添加到 `solver/optimize.py`：

- `make_callback(objective, constraint, print_every, tag)`:
  returns a function that prints J, J_best, g_min, elapsed time.
  返回一个函数，打印 J、J_best、g_min、已用时间。
- `solve_parameterized(..., verbose, print_every, tag)`:
  attaches the callback automatically when `verbose=True`.
  当 `verbose=True` 时自动附加 callback。

Usage / 用法:

    res = solve_parameterized(
        objective, u_init, method='SLSQP',
        constraints={'type': 'ineq', 'fun': constraint},
        constraint_func=constraint,      # for display only / 仅用于显示
        verbose=True, print_every=1, tag='SLSQP',
        maxiter=500, ftol=1e-6,
    )

This is the Python equivalent of MATLAB's `fmincon('Display','iter')`.
这是 MATLAB `fmincon('Display','iter')` 的 Python 等价物。

---

## Next Steps / 下一步

1. **Verify the constraint Jacobian hypothesis (data 2).**
   验证约束雅可比假设（数据 2）。
   Compute ‖∂g/∂u‖_F at u=0 and at u≠0 (e.g., at Stage 1's
   solution). If the norm at u=0 is dramatically smaller, the
   hypothesis is supported.
   计算 u=0 和 u≠0（如阶段 1 的解）处 ‖∂g/∂u‖_F。如果 u=0 处
   范数显著更小，假设得到支持。

2. **Add `method='trust-constr'` comparison.**
   加入 `method='trust-constr'` 对比。
   Does trust-constr succeed from u=0 without a two-stage warm-up?
   If yes, the issue is specific to SLSQP's QP-based line search.
   trust-constr 能否不用两阶段直接从 u=0 成功？如果能，说明问题
   是 SLSQP 基于 QP 的线搜索特有的。

3. **Compare warm-start sensitivity.**
   比较预热敏感性。
   How good does the seed need to be? Try:
   初值需要多好？尝试：
   - u_init = 0.1 * ones (constant small) / 常数小量
   - u_init = random small / 小随机数
   - u_init = Stage 1 solution / 阶段 1 的解

4. **Add unit tests.**
   加入单元测试。
   Test `rk4_step` against a known analytical solution.
   用已知解析解测试 `rk4_step`。
   Test `make_objective` with a trivial dynamics (identity).
   用平凡动力学（恒等）测试 `make_objective`。

---

## Reproducibility / 可复现性

All results generated with / 所有结果由以下命令生成:

    python -m control_para.examples.run_vehicle_obs

Files touched / 涉及文件:
    control_para/solver/optimize.py                (added callback, verbose / 加 callback 和 verbose)
    control_para/problem_setting/vehicle_obs.py    (new / 新建)
    control_para/examples/run_vehicle_obs.py
    control_para/examples/debug_slsqp.py           (new, diagnostic / 新建，诊断)
    README.md
    requirements.txt