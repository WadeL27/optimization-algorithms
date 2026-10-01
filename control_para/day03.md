# Day 3 Notes

## 目录结构

control_param/
├── problem_setting/   # 问题定义：f(x,u), x0, xf, T, N, 解析解
├── solver/            # 求解器：rk4_step, simulate, make_objective, solve
└── examples/          # 示例脚本

## 结果

### 双积分器
- N=20, J=1.5029, 解析解 1.5, 相对误差 0.19%
- Function evals: 294

### 车辆轨迹规划（自行车模型）
- N=20, 40 维决策变量
- Terminal error: 1.93e-4
- Function evals: 19856

## 观察

- 决策变量从 20 维增加到 40 维，函数评估次数从 294 涨到 19856（约 67 倍）
- 有限差分梯度是主要原因：每次梯度估计要调用目标函数 (n+1) 次

## 待改进

- 用解析梯度替代有限差分，大幅减少函数评估
- 加入状态约束（Day 4）
- 写中英文 README（Day 4）