# Day 5 Notes: Adjoint Method and Open Problems
# Day 5 笔记：伴随法与待解决问题

## 1. Completed Today / 今日已完成

### 1.1 Adjoint method for objective gradient
### 1.1 目标梯度的伴随法

Implemented in `solver/adjoint.py`:
已实现于 `solver/adjoint.py`：

- `rk4_step_with_jacobians(x, u, dt, f_jac)`:
  RK4 step with exact Jacobians A = dg/dx, B = dg/du.
  RK4 步进，附精确雅可比 A = dg/dx，B = dg/du。

- `gradient_adjoint(u_flat, dt, x0, xf, f_jac, m, penalty)`:
  Backward adjoint propagation for dJ/du.
  反向伴随传播，计算 dJ/du。

### 1.2 Gradient verification / 梯度验证

Vehicle example:
车辆例子：

    max rel diff  = 2.469290e-07
    cosine sim    = 1.0000000000

**Adjoint implementation is correct.**
**伴随法实现正确。**

### 1.3 Numerical comparison: finite diff vs adjoint
### 1.3 数值对比：有限差分 vs 伴随法

| Method / 方法 | nfev | terminal error 终端误差 | min clearance 最小裕度 | running cost 控制能量 |
|---|---|---|---|---|
| Finite diff + SLSQP | 2939 | 1.61e-4 | 0.0000 | 4.866814 |
| Adjoint grad + SLSQP | **88** | 1.62e-4 | -0.0000 | 4.866825 |

**33× speedup, same optimal solution.**
**加速 33 倍，最优解一致。**

---

## 2. Open Problems / 待解决问题

### 2.1 Constraint Jacobian is still finite-difference
### 2.1 约束雅可比仍是有限差分

**Status / 状态**:
- Objective gradient: analytical (adjoint). 
  目标梯度：解析（伴随法）。
- Constraint Jacobian: computed by SLSQP internally via finite differences. 
  约束雅可比：SLSQP 内部用有限差分。

**Why it matters / 为什么重要**:
- In the 88 nfev, a large fraction comes from SLSQP's internal finite-diff of the constraints. 
  88 次评估里，相当一部分来自 SLSQP 内部对约束的有限差分。
- For a fully analytical solver, the constraint Jacobian must also be provided. 
  要做“全解析”求解器，必须也提供约束雅可比。

**How to fix / 如何修复**:
- Use forward sensitivity propagation: S_k = dx_k / du in R^{n × N_c m}. 
  用前向灵敏度传播：S_k = dx_k/du ∈ R^{n × N_c m}。
- Compute dg_i/dx_i (analytical for obstacle constraint). 
  计算 dg_i/dx_i（对障碍约束是解析的）。
- Assemble G = dg/du via G_i = (dg_i/dx_i) @ S_i. 
  通过 G_i = (dg_i/dx_i) @ S_i 组装 G = dg/du。
- Pass as `constraints={'type': 'ineq', 'fun': ..., 'jac': ...}`. 
  作为 `jac` 参数传给 SLSQP。

### 2.2 Penalty gradient currently uses forward sensitivity
### 2.2 惩罚梯度目前用前向灵敏度

**Better approach / 更好的做法**: adjoint method.
用伴随法。

**Key idea / 核心思路**:
- Treat penalty as a distributed cost: L_k = ρ · max(0, -g_k)². 
  把惩罚项看作分布在各时刻的瞬时代价：L_k = ρ · max(0, -g_k)²。
- Add dL_k/dx_k to the costate at each step: 
  在每步的协态上叠加 dL_k/dx_k：

    λ_N = 2ρ(x_N - x_f) + dL_N/dx_N
    λ_k = A_k^T λ_{k+1} + dL_k/dx_k,   k = N-1, ..., 0

- Gradient: dJ/du_k = 2 u_k dt + B_k^T λ_{k+1}. 
  梯度：dJ/du_k = 2 u_k dt + B_k^T λ_{k+1}。

**Advantage / 优势**:
- Same cost as objective gradient: N_int · n² (matrix-vector). 
  和目标梯度同价：N_int · n²（矩阵-向量乘）。
- No need to store S_k ∈ R^{n × N_c m}. 
  无需存储 S_k ∈ R^{n × N_c m}。
- Memory: O(N_int · n²) vs O(N_int · n · N_c m). 
  内存：O(N_int · n²) vs O(N_int · n · N_c m)。

### 2.3 Control segmentation and RK sub-steps are coupled
### 2.3 控制分段与 RK 子步耦合

**Current code / 现状**:
- N_c = N_int (control intervals = integration steps). 
  N_c = N_int（控制分段数 = 积分步数）。
- Each control interval uses exactly one RK4 step. 
  每个控制区间恰好用一次 RK4。

**Decoupling / 解耦**:
- N_c: control intervals (decision variables = N_c · m). 
  N_c：控制分段数（决策变量数 = N_c · m）。
- N_sub: RK4 sub-steps per control interval. 
  N_sub：每个控制区间的 RK4 子步数。
- N_int = N_c · N_sub: total integration steps. 
  N_int = N_c · N_sub：总积分步数。

**Benefit / 收益**:
- Precision improves as O(dt_sub⁴) without increasing decision variable dimension. 
  精度以 O(dt_sub⁴) 提升，而决策变量维度不变。
- Key advantage of control parameterization over direct collocation. 
  这是控制参数化相对直接配点法的核心优势。

**Implementation / 实现**:
- Add n_sub parameter to simulate and gradient_adjoint. 
  给 simulate 和 gradient_adjoint 加 n_sub 参数。
- B_k only appears at the first sub-step of each control interval. 
  B_k 只在每个控制区间的第一个子步出现。
- Approximately 30 lines of code. 
  约 30 行代码。

### 2.4 Adjoint vs forward for constraint Jacobian
### 2.4 约束雅可比的伴随 vs 前向

**Critical insight from today's discussion**:
**今日讨论的关键洞察**：

When N_sub grows, the number of continuous state constraints
grows as N_int ≈ N_c · N_sub. The adjoint method needs one
backward pass per constraint, so its cost grows as O(N_int²).
Forward sensitivity computes all S_k in one pass, so its cost
grows as O(N_int).

当 N_sub 增加，连续状态约束个数 ≈ N_int = N_c · N_sub 也增加。
伴随法每个约束要一次反向传播，代价 O(N_int²)。
前向灵敏度一次传播得到所有 S_k，代价 O(N_int)。

**Cost ratio / 代价比**:

    T_adj / T_fwd = N_sub / (2m)

**Threshold / 临界点**:
- N_sub < 2m: adjoint is faster. 
  N_sub < 2m：伴随法更快。
- N_sub > 2m: forward sensitivity is faster. 
  N_sub > 2m：前向灵敏度更快。

**For vehicle problem (m = 2) / 车辆问题 (m = 2)**:

| N_sub | Ratio | Winner |
|---|---|---|
| 1 | 0.25 | Adjoint (4× faster) |
| 4 | 1.00 | Tie |
| 10 | 2.50 | Forward (2.5×) |
| 100 | 25.0 | Forward (25×) |

**General rule / 一般规律**:

| Task / 任务 | Best method / 最优方法 |
|---|---|
| Objective gradient (scalar) | Adjoint (always N_c m × faster) |
| Penalty gradient (scalar) | Adjoint (always) |
| Constraint Jacobian (N_int rows) | Depends on N_sub / (2m) |

**Mixed strategy / 混合策略**:
Choose per-task based on N_sub:
- Objective/penalty: adjoint.
- Constraints: adjoint if N_sub < 2m, else forward.

**Memory caveat / 内存注意**:
- Adjoint memory: O(N_int · n²).
- Forward memory: O(N_int · n · N_c m).
- Ratio: N_c m / n.
- For vehicle: 50/4 = 12.5× more memory for forward.
- Still acceptable for moderate N_sub.

### 2.5 Recover finite-difference objective function
### 2.5 恢复有限差分目标函数

**Current state / 现状**:
- Finite-difference version is commented out. 
  有限差分版被注释掉了。

**Recommended fix / 推荐修复**:
- Restore as `make_objective_finite_diff(f, x0, xf, dt, m, penalty)`. 
  恢复为 `make_objective_finite_diff(f, x0, xf, dt, m, penalty)`。
- Return (objective, None) — pass jac=None to minimize. 
  返回 (objective, None)——传 jac=None 给 minimize。
- Keep both versions side by side for comparison. 
  两个版本并存，便于对比。
- Do not leave commented-out code in the repository. 
  不要在仓库里留下注释掉的代码。

### 2.6 Stage 1 BFGS with adjoint gradient
### 2.6 Stage 1 BFGS 用伴随梯度

**Not yet tested / 尚未测试**:
- Stage 1 currently uses finite-diff (nfev = 36885). 
  Stage 1 目前用有限差分（nfev = 36885）。
- Expected with adjoint: nfev drops to a few hundred, Success becomes True. 
  用伴随法预期：nfev 降到几百，Success 变 True。

**To test / 待测试**:

    objective, gradient = make_objective_with_grad(
        vo.dynamics_jac, x0, xf, dt, m=2)
    res = solve_parameterized(
        objective, u_init, method='BFGS',
        jac=gradient,
        gtol=1e-8, maxiter=2000,
        verbose=True, print_every=50, tag='BFGS-adjoint')

**Why it matters for notes / 为什么值得记录**:
- Second data point showing adjoint speedup over finite diff. 
  第二个数据点，展示伴随法相对有限差分的加速。
- Also shows that Success: True becomes achievable at gtol=1e-8. 
  也说明 gtol=1e-8 下 Success: True 变得可达。

---

## 3. Theoretical Summary / 理论小结

### Cost per method / 各方法代价

Variables:
- n: state dimension
- m: control dimension
- N_c: control intervals
- N_sub: RK4 sub-steps per control interval
- N_int = N_c · N_sub: total integration steps
- N_c · m: total decision variables

| Method / 方法 | Per-step cost | Total cost | Memory |
|---|---|---|---|
| Forward sensitivity | n² · N_c m | N_int · n² · N_c m | N_int · n · N_c m |
| Adjoint (one backward) | n² | N_int · n² | N_int · n² |
| Adjoint (per constraint) | n² | N_int² · n² / 2 | N_int · n² |

### Key ratios / 关键比值

- Objective gradient: T_fwd / T_adj = N_c m.
- Constraint Jacobian: T_adj / T_fwd = N_sub / (2m).
- Memory: M_fwd / M_adj = N_c m / n.

### Takeaway / 核心结论

For objective gradients, adjoint is always superior.
For constraint Jacobians, the choice depends on N_sub relative to 2m.

目标梯度：伴随法永远最优。
约束雅可比：取决于 N_sub 与 2m 的大小关系。

This "regime-dependent algorithm selection" is itself a form
of algorithmic improvement and is worth documenting for
future work.

这种“按参数区间选择算法”的策略本身就是一种算法改进，
值得记录以备后续工作。

---

## 4. Next Steps / 下一步

Priority order / 优先级:

1. **Restore finite-diff objective as a function** (5 min). 
   恢复有限差分目标函数（5 分钟）。
2. **Stage 1 BFGS with jac=gradient** (5 min, expected nfev 36885 → < 500). 
   Stage 1 BFGS 用解析梯度（5 分钟，预期 nfev 从 36885 降到 < 500）。
3. **Adjoint-based penalty gradient** (~30 min). 
   基于伴随法的惩罚梯度（约 30 分钟）。
4. **Forward sensitivity + constraint Jacobian** (~40 min). 
   前向灵敏度 + 约束雅可比（约 40 分钟）。
5. **Decouple N_c and N_sub** (~30 min). 
   解耦 N_c 和 N_sub（约 30 分钟）。
6. **Mixed strategy for constraint Jacobian** (~20 min). 
   约束雅可比的混合策略（约 20 分钟）。

Items 1-2 are quick wins; do them today.
Items 3-6 are larger and can be scheduled for the next 1-2 days.

第 1-2 项是快速收益，今天可做。
第 3-6 项较大，可安排到接下来 1-2 天。

---

## 5. Reproducibility / 可复现性

Files touched today / 今日涉及文件:

    control_para/solver/adjoint.py               (new / 新建)
    control_para/solver/optimize.py              (added make_objective_with_grad, jac param)
    control_para/problem_setting/vehicle_obs.py  (added dynamics_jac)
    control_para/examples/debug_gradient.py      (new / 新建)
    control_para/examples/run_vehicle_obstacle.py

Run / 运行:

    python -m control_para.examples.debug_gradient
    python -m control_para.examples.run_vehicle_obstacle