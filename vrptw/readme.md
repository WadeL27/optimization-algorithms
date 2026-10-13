## Parameter Sensitivity Analysis / 参数敏感性分析

Instance: R101-100 (Solomon benchmark). All runs use COPT 8.0.7.

### A. TimeLimit (RelGap=0.01)

| TimeLimit | objective | best bound | gap(%) | time(s) | vehicles |
|---|---|---|---|---|---|
| 30 | 1642.88 | 1616.97 | 1.577 | 30.01 | 20 |
| 60 | 1642.88 | 1627.07 | 0.962 | 57.03 | 20 |
| 120 | 1642.88 | 1627.07 | 0.962 | 57.71 | 20 |

**Finding**: The feasible solution 1642.88 is found within 30 seconds.
Extending the time limit only tightens the lower bound, not the
incumbent. This shows the solver's bottleneck is proving optimality,
not finding good solutions.

**结论**：可行解在 30 秒内找到，延长时间只收紧下界，不改进可行解。
这说明求解器的瓶颈在证明最优性，而非找解。

### B. RelGap (TimeLimit=120)

| RelGap | objective | best bound | gap(%) | time(s) | vehicles |
|---|---|---|---|---|---|
| 0.001 | 1642.88 | 1631.45 | 0.696 | 120.06 | 20 |
| 0.01 | 1642.88 | 1627.07 | 0.962 | 62.55 | 20 |
| 0.05 | 1684.61 | 1616.97 | 4.015 | 17.80 | 21 |

**Finding**: Relaxing RelGap from 0.01 to 0.05 cuts solve time
from 63s to 18s, but degrades the solution by 2.5% and increases
the vehicle count by 1. This quantifies the quality-vs-time
trade-off.

**结论**：RelGap 从 0.01 放宽到 0.05，求解时间从 63 秒降到 18 秒，
但解质量下降 2.5%，车辆数多 1。这量化了“质量 vs 时间”的权衡。

### C. Threads (TimeLimit=60, RelGap=0.01)

| Threads | objective | best bound | gap(%) | time(s) | vehicles |
|---|---|---|---|---|---|
| 1 | 1642.88 | 1616.97 | 1.577 | 60.01 | 20 |
| 4 | 1642.88 | 1617.80 | 1.527 | 60.02 | 20 |
| 20 | 1642.88 | 1617.13 | 1.567 | 60.18 | 20 |

**Finding**: Parallelism has negligible impact on this instance.
The lower bound differs by only 0.05% across 1 and 20 threads.
MILP branch-and-bound parallelism is limited by synchronization
overhead and the structure of the search tree.

**结论**：线程数对本实例影响极小，1 线程与 20 线程的下界差 0.05%。
MILP 分支定界的并行效率受同步开销和搜索树结构限制。